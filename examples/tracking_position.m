% =========================================================================
% 1. 파라미터 초기화 및 설정 (Parameters Setup)
% =========================================================================

% 2-D constant velocity
% 3-D constant velocity
% 2-D constant acceleration
% 3-D constant acceleration
motionModel = "2-D Constant Velocity";       % 선형 운동 모델 (2차원 등속도)

dfMethod = "AoA";                       % 방향 탐지 방법 (AoA: 도착각 방식)
dfPacketType = "ConnectionCTE";         % 방향 탐지용 패킷 유형 (연결 기반 CTE)
phyMode = "LE1M";                       % PHY 전송 모드 설정: ConnectionCTE의 경우 LE1M 또는 LE2M, ConnectionlessCTE는 LE1M 사용

arraySize = 16;                         % 안테나 어레이 크기: 2차원 위치 추정은 스칼라(ULA: 선형 배열), 3차원은 벡터(URA: 평면 배열)
elementSpacing = 0.5;                   % 안테나 소자 간의 규격화된 간격 (파장 대비 비율)

switchingPattern = 1:prod(arraySize);   % 안테나 스위칭 패턴: 1xM 행 벡터, M 범위는 [2, 74/slotDuration+1]
slotDuration = 2;                       % 슬롯 지속 시간 (마이크로초 단위)
sps = 4;                                % 심볼당 샘플 수(나이퀴스트 샘플링), 보통 4 혹은 8(고해상도)
channelIndex = 17;                      % 데이터 채널 인덱스
crcInit = '555551';                     % CRC(순환 중복 검사) 초기화 값. 이진수 변환시 01이 반복되는 값.
accessAddress = '01234567';             % 액세스 주소
payloadLength = 1;                      % 페이로드 길이 (바이트 단위). ConnnectionCTE는 1바이트, Connectionless의 경우 송신자 주소를 담아야해서 3바이트 이상

% CTE(Constant Tone Extension) 길이: [16, 160] 마이크로초 범위 (8단위)
cteLength = 160;

% 샘플 오프셋: LE1M은 [sps/8, 7*sps/8], LE2M은 [sps/4, 7*sps/4] 범위의 정수
sampleOffset =  2;                % 안테나 슬롯 변화 후 신호가 안정될때 까지 기다리는 시간 (인덱스)
EbNo = 18;                        % 비트 에너지 대비 잡음 밀도 (dB). 5~10(생존) | 15~20(안정) | 25(이상)
environment = "Outdoor";          % 통신 환경 "Indoor", "Outdoor", "Free Space"
txPower = 0;                      % BLE 송신 전력 0~4 (dBm) 0dBm = 1mW

% Bluetooth 경로 손실 및 범위 설정 (물리적 환경 정보)
rangeConfig = bluetoothRangeConfig;
rangeConfig.SignalPowerType = "ReceivedSignalPower";    % 수신전력을 계산. "PathLoss"의 경우 에너지 손실량 계산
rangeConfig.Environment = environment;
rangeConfig.TransmitterPower = txPower;
rangeConfig.TransmitterCableLoss = 0;               % 송신측 케이블 손실
rangeConfig.ReceiverCableLoss = 0;                  % 수신측 케이블 손실

% =========================================================================
% 2. 설정 값 유효성 검사 (Validation)
% =========================================================================

% 운동 모델에 따른 차원 수 계산 (3D 여부 확인)
numDimensions = 2 + (strcmp(motionModel,"3-D Constant Velocity") || strcmp(motionModel,"3-D Constant Acceleration"));

% 설정 유효성 검사 (차원과 안테나 배열 크기 일치 확인)
if numDimensions == 2 && size(arraySize,2) ~= 1
    error("2차원 위치 추정을 위해서는 arraySize가 스칼라여야 합니다.");
elseif numDimensions == 3 && size(arraySize,2) ~= 2
    error("3차원 위치 추정을 위해서는 arraySize가 1x2 벡터여야 합니다.");
end

% 패킷 타입에 따른 페이로드 길이 제한 확인
if strcmp(dfPacketType,"ConnectionCTE") && payloadLength ~= 1
    error("ConnectionCTE 타입의 경우 페이로드 길이는 1바이트여야 합니다.");
elseif strcmp(dfPacketType,"ConnectionlessCTE") && payloadLength < 3
    error("ConnectionlessCTE 타입의 경우 페이로드 길이는 3바이트 이상이어야 합니다.");
end

% 차원에 다른 안테나 간 간격 자동 변환
if numDimensions == 3 && isscalar(elementSpacing)
    elementSpacing = [elementSpacing elementSpacing];
end

% BLE 각도 추정용 설정 객체(앵커) 생성 및 규격 검증
cfg = bleAngleEstimateConfig("ArraySize",arraySize, ...
    "SlotDuration",slotDuration, ...
    "SwitchingPattern",switchingPattern, ...
    "ElementSpacing",elementSpacing);
validateConfig(cfg);                            % 구성 유효성 검사
pos = getElementPosition(cfg);                  % 안테나 어레이의 소자별 위치 계산

% CTE 타입 설정 (AoA/AoD 및 슬롯 지속 시간에 따라 결정)
% 패킷 생성에 쓰이는 값으로. AoA는 기본 CTE를 생성하지만, AoD는 안테나를 스위칭하며 생성. 
if strcmp(dfMethod,"AoA")
    cteType = [0;0];
else
    cteType = [0;1];
    if slotDuration == 1
        cteType = [1;0];
    end
end

% =========================================================================
% 3. 수신기 System Object 구성
% =========================================================================-
% 프리앰블 탐지기 설정(Preamble과 Address를 모두 사용)
accessAddBitsLen = 32;
accessAddBits = int2bit(hex2dec(accessAddress),accessAddBitsLen,false);
refSamples = helperBLEReferenceWaveform(phyMode,accessAddBits,sps);
prbDet = comm.PreambleDetector(refSamples,"Detections","First");

% 거친 주파수 보정기(Coarse Frequency Compensator) 설정
phyFactor = 1+strcmp(phyMode,"LE2M"); % LE2M이 LE1M에 비해 심볼당 절반의 시간 사용
sampleRate = 1e6*phyFactor*sps;
coarsesync = comm.CoarseFrequencyCompensator("Modulation","OQPSK",...
    "SampleRate",sampleRate,...
    "SamplesPerSymbol",2*sps,...
    "FrequencyResolution",100);

% CRC 탐지기 설정
crcLen = 24;        % CRC 길이
crcDet = comm.CRCDetector("x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1","DirectMethod",true,...
    "InitialConditions",int2bit(hex2dec(crcInit),crcLen).'); % 블루투스 표준 CRC 수식

rng('default'); % 랜덤 생성기 초기화. awgn, randsrc 등에서 동일한 시퀀스의 난수 생성 보장
snr = EbNo - 10*log10(sps);                             % EbNo -> 신호 대 잡음비(SNR) 계산
headerLen = 16+8*strcmp(dfPacketType,"ConnectionCTE");  % 헤더 길이 계산
preambleLen = 8*phyFactor;                              % 프리앰블 길이

% 채널 인덱스 기반 데이터 화이트닝 초기 상태 유도
% 0과 1이 고르게 나타나도록 데이터(데이터+CRC)를 약속된 비트열과 XOR 연산
% DC 편향 및 동기화 상실을 방지
dewhitenStateLen = 6;
chanIdxBin = int2bit(channelIndex,dewhitenStateLen).';  % 채널에 따라 비트열 결정
initState = [1 chanIdxBin];                             % 블루투스 표준 (1 + 채널비트) = 7비트
dewhiten = bluetoothWhiten(InitialConditions=initState');   % 화이트닝/디화이트닝 처리 객체 생성

% =========================================================================
% 4. 네트워크 배치 및 이동 시뮬레이션 준비
% =========================================================================-
% 1개의 이동 노드와 15개의 로케이터(수신기) 생성
numLocators = 15;           % 로케이터 수
numNodePositions = 30;      % 노드 이동 경로 상의 지점 수
[posNode,posLocators] = helperBLEPositions(motionModel,numNodePositions,numLocators);

% 노드 위치 추정값을 저장할 변수
posNodeEst = zeros(numDimensions,numNodePositions);
S = RandStream('mt19937ar','Seed',5489); % 시드 초기화

% 오실레이터 불안정성 반영
% 위상/주파수 오프셋 및 위상 잡음 객체 초기화
pfo = comm.PhaseFrequencyOffset(SampleRate=sampleRate); % 위상 주파수 오프셋. coarsesync로 보정되는 값
phaseNoise = comm.PhaseNoise(Level=[-130 -136],FrequencyOffset=[1e4 1e5],SampleRate=sampleRate); % 미세한 위상 오차로 AoA 측정에 영향

% =========================================================================
% 5. 메인 루프: 노드 이동 시뮬레이션
% =========================================================================-
for inumNode = 1:numNodePositions
    % 노드로부터 80m 이내에 있는 활성 로케이터 찾기
    % [활성 로케이터, 실제 각도, 실제 거리]
    [posActiveLocators,angleActive,distanceActive] = helperBLEActiveLocators(posNode(:,inumNode),posLocators);
    posLocatorBuffer{:,inumNode} = posActiveLocators;   %#ok<SAGROW>. 루프별 활성 로케이터 임시 저장
    numActiveLocators = size(posActiveLocators,2);      % 활성 로케이터 수
    
    % 거리와 환경에 따른 경로 손실(Pathloss) 추정
    plLin = helperBluetoothEstimatePathLoss(rangeConfig.Environment,distanceActive); % 선형배수: 100이 나오면 1/100으로 신호가 줄어듬
    angleEst = zeros(numActiveLocators,numDimensions-1);            % 추정된 각도 저장
    [pathLossdB,linkFailFlag] = deal(zeros(numActiveLocators,1));   % 경로 손실 및 링크 실패 플래그
    
    % 각 활성 로케이터에 대해 반복
    for i = 1:numActiveLocators
        % --- 송신기 시뮬레이션 구성 ---
        % CTE가 붙은 방향 탐지용 패킷(PDU) 비트열 생성
        data = helperBLEGenerateDFPDU(dfPacketType,cteLength,cteType,payloadLength,crcInit);
        
        % 디지털 비트로 IQ샘플 Bluetooth LE 웨이브폼으로 변형
        bleWaveform = bleWaveformGenerator(data,"Mode",phyMode,"SamplesPerSymbol",sps,...
            "ChannelIndex",channelIndex,"DFPacketType",dfPacketType,"AccessAddress",accessAddBits);
        
        % AoD 방식인 경우 송신단 안테나 스위칭(Steering) 수행
        if strcmp(dfMethod,"AoD")
            bleWaveform = helperBLESteerSwitchAntenna(bleWaveform,angleActive(i,:),...
                phyMode,sps,dfPacketType,payloadLength,cfg);
        end
        
        % RF 채널 임페어먼트(왜곡) 추가
        % 주파수 오프셋 추가
        freqOffset = randsrc(1,1,-10e3:100:10e3,S); % -10kHz ~ 10kHz 사이의 랜덤 값 선택
        pfo.FrequencyOffset = freqOffset;
        freqWaveform = pfo(bleWaveform);            % 신호에 적용 (IQ 평면에서 회전 발생)
        release(pfo)                                % 객체 초기화 (다음 루프 준비)
        
        % 타이밍 오프셋 추가
        timingoff = randsrc(1,1,1:0.2:20,S);        % 1~20 샘플 사이의 랜덤한 지연 시간 발생
        timingOffWaveform = helperBLEDelaySignal(freqWaveform,timingoff);
        
        % DC 오프셋 추가
        dcMaxValue = max(max(real(timingOffWaveform)),max(imag(timingOffWaveform))); % 신호의 최대 진폭 계산
        dcValue = (5/100)*randsrc(1,1,(-1:0.05:1)*dcMaxValue)*(sqrt(0.5)+sqrt(0.5)*1i); % 최대 진폭의 5% 수준으로 DC 값 생성
        dcWaveform = timingOffWaveform + dcValue; % 신호의 중심이 IQ 평면 상에서 (0,0)에서 치우쳐짐
   
        % 미세한 위상 잡음 추가
        noisyWaveform = phaseNoise(dcWaveform);
        release(phaseNoise);
        
        % --- 송신 전력 및 경로 손실 적용 ---
        dBmConverter = 30;  % dBm -> dBW : 10log(1000) = 30
        txImpairedWaveform = 10^((rangeConfig.TransmitterPower-dBmConverter)/20)*noisyWaveform/plLin(i);
        
        % AoA 방식인 경우 수신단 안테나 조향(Steering Vector) 적용
        % 기하학적 정보를 토대로 각 안테나 별 신호 생성 
        if strcmp(dfMethod,"AoA")
            steerVec = helperBLESteeringVector(angleActive(i,:),pos);
            steeredWaveform = txImpairedWaveform .* steerVec.';
        else
            steeredWaveform = txImpairedWaveform;
        end
        
        % --- 수신기 처리 (Receiver Processing) ---
        % 수신기 상태 변수 초기화
        packetDetect = 0;            % 패킷 시작(프리엠블)을 찾았는지 나타내는 플래그                                   
        headerFlag = 1;              % 지금 읽는 구간이 헤더인지 여부 (패킷을 찾았으면 자동으로 1)
        pduCRCFlag = 0;              % PDU 데이터 전송이 끝나고 CRC 체크를 했는지 여부                    
        samplingFlag = 0;            % CTE 샘플링 구간에 진입했는지 여부
        crcError = 1;                % 데이터가 깨졌는지 여부 (유죄추정의 원칙으로 1)                         
        samplesPerFrame = 8*sps;     % 1바이트(8bit) 단위로 데이터 처리                               
        samplesPerModule = samplesPerFrame+sps;                         
        numTimes = ceil(length(timingOffWaveform)/samplesPerFrame)+1;   % 전체 신호를 처리하기 위한 회전수
        countpacketDetect = 0;            % 패킷 검출을 시도한 횟수
        moduleStartInd = 0;               % 현재 처리 중인 모듈의 시작 인덱스     
        
        % 웨이브폼 버퍼링 (제로 패딩)
        rxWaveformBuffer = [steeredWaveform;zeros(samplesPerFrame*numTimes-length(steeredWaveform)+samplesPerFrame,size(steeredWaveform,2))];
        
        % 청크 단위 수신 처리
        for j = 1:numTimes
            % 청크 생성
            if (j-1)*samplesPerFrame + moduleStartInd + samplesPerModule <= length(rxWaveformBuffer)
                rxChunkWaveform = rxWaveformBuffer((j-1)*samplesPerFrame+moduleStartInd+(1:samplesPerModule),:);
            else
                rxChunkWaveform = rxWaveformBuffer((j-1)*samplesPerFrame+moduleStartInd+1:end,:);
            end
            
            if ~samplingFlag
                % 일반 패킷(프리엠블, 주소, 헤더, 데이터) 부분 수신 및 잡음 추가
                % 첫번째 안테나만 사용
                rxNoisyWaveform = awgn(rxChunkWaveform(:,1),snr,"measured"); 
            else
                % (3)
                % CTE 부분: AoA일 경우 안테나 스위칭 수행
                if strcmp(dfMethod,"AoA")
                    rxSwitchWaveform = helperBLESwitchAntenna(rxChunkWaveform,phyMode,sps,slotDuration,switchingPattern);
                else
                    rxSwitchWaveform = rxChunkWaveform;
                end
                rxNoisyWaveform = awgn(rxSwitchWaveform,snr,"measured"); 
        
                % CTE 구간에서 IQ 샘플링 수행 (각도 추정용 핵심 데이터)
                cteSamples = rxNoisyWaveform(1:cteTime*8*sps*phyFactor);
                iqSamples = bleCTEIQSample(cteSamples,"Mode",phyMode,...
                    "SamplesPerSymbol",sps,"SlotDuration",slotDuration,"SampleOffset",sampleOffset);
                samplingFlag = 0;
                break;      % 각도 추정 단계로 넘어감
            end
            
            % (1) 패킷 탐지 및 동기화 과정
            if packetDetect == 0
                countpacketDetect = countpacketDetect+1;    % 패킷탐지 시도 횟수
                rcvSigBuffer((countpacketDetect-1)*(samplesPerModule-sps)+(1:samplesPerModule)) = rxNoisyWaveform;  % 청크 단위의 신호를 rcvSigBuffer에 이어 붙힘
                if countpacketDetect >= (preambleLen+accessAddBitsLen+headerLen)*sps/samplesPerFrame    % (프리앰블 + 주소 + 헤더)이 다 들어왔을 법한 충분한 양의 신호가 모였을 때 탐지 시작
                    % 타이밍 동기화
                    [~, dtMt] = prbDet(rcvSigBuffer.'); % 수신된 신호와 프리앰플 패턴 사이 상관관계
                    release(prbDet)
                    prbDet.Threshold = max(dtMt);
                    prbIdx = prbDet(rcvSigBuffer.');    % 가장 강력한 일치가 나타난 지점으로 인덱스
                    release(prbDet)
                    
                    % 찾아낸 시작점을 기준으로 앞뒤 공백을 잘라내고 순수한 신호
                    if prbIdx >=length(refSamples)
                        rcvTrim = rcvSigBuffer(1+prbIdx-length(refSamples):end).';
                    else
                        rcvTrim = rcvSigBuffer.';
                    end
                    
                    % DC 오프셋 추정 및 보상
                    estimatedDCOffset = mean(rcvTrim(1:length(refSamples)))-mean(refSamples)*sqrt(var(rcvTrim));
                    rcvDCFree = rcvTrim-estimatedDCOffset;
                    
                    % 주파수 오프셋 추정 및 보상
                    [~,freqoff] = coarsesync(rcvDCFree);
                    release(coarsesync)
                    [rcvFreqOffsetFree,iniFreqState] = helperBLEFrequencyOffset(rcvDCFree,sampleRate,-freqoff);
                    
                    % GMSK 복조
                    % 물리적인 파형을 다시 디지털 비트(0, 1)로 변환
                    x = rem(length(rcvFreqOffsetFree),sps);
                    if x, remsad = sps - x; else, remsad = x; end
                    remNumSamples = x;
                    remSamples = rcvFreqOffsetFree(end-remNumSamples+1:end);
                    [demodSoftBits,demodInitPhase] = helperBLEGMSKDemod(rcvFreqOffsetFree(1:end-remNumSamples),phyMode,sps,0);
                    demodBits = demodSoftBits>0;
                    
                    % 액세스 주소 확인
                    if length(demodBits) >= preambleLen+accessAddBitsLen
                        accessAddress = int8(demodBits(preambleLen+(1:accessAddBitsLen))>0);
                        decodeData = int8(demodBits(accessAddBitsLen+preambleLen+1:end)>0);
                        if isequal(accessAddBits,accessAddress) % 주소가 일치할 경우
                            packetDetect = 1;
                            samplesPerModule = headerLen*sps-length(decodeData)*sps+remsad+sps;     % 남은 헤더와 데이터 처리를 위해 데이터 크기 업데이트
                            rcvSigBuffer = [];
                        end
                    end
                end
            end
            
            % (2) 패킷 탐지 성공 후 헤더 및 PDU 처리
            if packetDetect
                % (2)-1
                if headerFlag && (length(rxNoisyWaveform) ~= samplesPerFrame+sps || samplesPerModule <= sps)
                    % 헤더 복구 및 복조
                    rxDCFreeWaveform = rxNoisyWaveform-estimatedDCOffset;   % 이전에 추정한 DC 오프셋 재사용
                    [rxNoisyWaveform,iniFreqState] = helperBLEFrequencyOffset(rxDCFreeWaveform,sampleRate,-freqoff,iniFreqState);   % 이전에 추정한 iniFreqState 이어서 사용
                    
                    if (length(rxNoisyWaveform) ~= samplesPerFrame)
                        rcvSigHeaderBuffer = [remSamples;rxNoisyWaveform];
                        remSamp = rem(length(rcvSigHeaderBuffer),sps);
                        if remSamp, appSamp = sps - remSamp; else, appSamp = remSamp; end
                        rcvSigHeaderBuffer = [rcvSigHeaderBuffer;zeros(appSamp,1)]; %#ok<AGROW>
                        [headerSoftBits,demodInitPhase] = helperBLEGMSKDemod(rcvSigHeaderBuffer,phyMode,sps,demodInitPhase);
                        decodeDataHeader = [decodeData;headerSoftBits>0];
                    elseif samplesPerModule <= 0
                        decodeDataHeader = decodeData;
                    end
                    
                    % 디화이트닝 및 PDU 정보 추출
                    dewhitenedBits = dewhiten(decodeDataHeader);
                    pduLenField = double(dewhitenedBits(9:16)); % 데이터 길이 정보
                    pduLenInBits = bi2de(pduLenField')*8;       % 바이트 단위를 비트 단위로 변경
                    
                    % 연결형 CTE 패킷인 경우, CTE의 지속 시간과 슬롯 간격정보 추출
                    if strcmp(dfPacketType,"ConnectionCTE")
                        [cteTime,cteTypeDec] = helperBLECTEInfoExtract(dewhitenedBits,dfPacketType);
                        if cteTypeDec == 1 || cteTypeDec == 2
                            slotDuration = cteTypeDec;
                        else
                            slotDuration = cfg.SlotDuration;
                        end
                    end
                    
                    % CRC 체크를 위한 파라미터 업데이트
                    % 다음 청크의 크기를 남은 데이터 + CRC 전체 길이에 맞춤
                    if samplesPerModule, moduleStartInd = samplesPerModule-samplesPerFrame-sps; else, moduleStartInd = -sps; end
                    samplesPerModule = (pduLenInBits+crcLen-length(dewhitenedBits)+headerLen+1)*sps;
                    pduCRCFlag = 1; headerFlag = 0;
                    rcvSigHeaderVar = rxNoisyWaveform;
                    rxNoisyWaveform = [];
                end
                
                % (2)-2
                % PDU 및 CRC 검사
                if pduCRCFlag && ~isempty(rxNoisyWaveform)
                    % DC 오프셋 및 주파수 오프셋 제거
                    rxDCFreeWaveform = rxNoisyWaveform-estimatedDCOffset;
                    [rxNoisyWaveform,iniFreqState] = helperBLEFrequencyOffset(rxDCFreeWaveform,sampleRate,-freqoff,iniFreqState);
                    
                    x = rem(length(rxNoisyWaveform),sps);
                    if x, remNumSamples = sps - x; else, remNumSamples = x; end
                    
                    % GMSK 복조 수행
                    demodPDUCRC = helperBLEGMSKDemod([rxNoisyWaveform;zeros(remNumSamples,1)],phyMode,sps,demodInitPhase);
                    
                    % 디화이트닝 및 PDU & CRC 정보 추출
                    dewhitenedPDUCRC = dewhiten(demodPDUCRC);
                    reset(dewhiten)
                    headerPDUCRC = [double(dewhitenedBits);dewhitenedPDUCRC];

                    if strcmp(dfPacketType,"ConnectionlessCTE")
                        [cteTime,cteTypeDec] = helperBLECTEInfoExtract(headerPDUCRC,dfPacketType);
                        if cteTypeDec == 1 || cteTypeDec == 2, slotDuration = cteTypeDec; else, slotDuration = cfg.SlotDuration; end
                    end
                    
                    % CRC 오류가 없다면 다음 단계(IQ 샘플링)로 진행
                    [dfPDU,crcError] = crcDet(headerPDUCRC);
                    if crcError, break; end
                    
                    % 각도 추정 단계를 위한 파라미터 업데이트
                    moduleStartInd = samplesPerModule-samplesPerFrame+moduleStartInd-sps;
                    samplesPerModule = cteTime*8*sps*phyFactor;
                    samplingFlag = 1; pduCRCFlag = 0;
                    rcvSigPDUVar = rxNoisyWaveform;
                end
            end
        end
        
        % --- 각도 추정 및 위치 추정 준비 ---
        refSampleLength = 8;                                            
        minIQSamples = refSampleLength+getNumElements(cfg)-1;
        
        % 패킷 성공 시 각도 추정 실행
        if ~crcError && length(nonzeros(iqSamples)) >= minIQSamples     
            angleEst(i,:) = bleAngleEstimate(iqSamples,cfg);
            % 수신 전력 측정 및 경로 손실 계산
            rangeConfig.ReceivedSignalPower = 10*log10(var([rcvFreqOffsetFree;rcvSigHeaderVar;rcvSigPDUVar]))+30;
            pathLossdB(i) = pathLoss(rangeConfig);
        else
            linkFailFlag(i) = 1; % 링크 실패 표시
        end
    end
    
    % 수신 실패한 로케이터 데이터 제외
    if any(linkFailFlag == 1)
        idx = find(linkFailFlag == 1);
        posActiveLocators(:,idx) = [];
        angleEst(idx,:) = [];
        pathLossdB(idx) = [];
    end
    
    % ---  최종 위치 추정 ---
    % 2개 이상의 로케이터 데이터가 있으면 삼각측량(Angulation) 수행
    if (numActiveLocators-nnz(linkFailFlag)) >= 2 && ~all(angleEst(:,1) == angleEst(1,1))
        posNodeEst(:,inumNode) = blePositionEstimate(posActiveLocators,"angulation",angleEst.');
    % 1개의 로케이터만 있으면 거리-각도 기반 위치 추정 수행
    elseif (numActiveLocators-nnz(linkFailFlag)) == 1 || (~all(linkFailFlag == 1) && all(angleEst(:,1) == angleEst(1,1)))
        range = bluetoothRange(rangeConfig);
        distanceValues = min(range):max(range);
        idx = randi(length(distanceValues),1);
        distanceEst = distanceValues(idx);
        posNodeEst(:,inumNode) = blePositionEstimate(posActiveLocators,"distance-angle",distanceEst.',angleEst.');
    else
        posNodeEst(:,inumNode) = NaN(numDimensions,1);
    end
end

% =========================================================================
% 6. 결과 출력 및 시각화
% =========================================================================-
posErr = sqrt(sum((posNodeEst-posNode).^2)); % 위치 오차(RMSE) 계산
disp(["Positioning error in meters = ", num2str(posErr)])

% 추정된 궤적 시각화
if ~all(isnan(posNodeEst(1,:)))
    helperBLEVisualizeNodeTracking(posLocators,posNode,posLocatorBuffer,posNodeEst)
end

% (선택 사항) 칼만 필터를 이용한 경로 평활화 코드 (주석 처리됨)
% if ~all(isnan(posNodeEst(1,:)))
%     posNodeTrackEst = helperBLEKalmanFilter(motionModel,posNodeEst);
%     ...
% end