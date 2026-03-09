% =========================================================================
% 1. 파라미터 초기화 및 설정 (Parameters Setup)
% =========================================================================
% 시뮬레이션 공간 설정 (가로 x 세로 x 높이, 단위: m)
roomSize = [10, 8, 3];

% 로케이터(앵커) 위치 초기 파라미터 (4개의 앵커 배치)
% 각 열은 [x; y; z] 좌표를 나타내며, 전치(')를 통해 3xN 행렬로 구성
locatorPos = [5, 0, 1.5; 0, 4, 1.5; 5, 8, 1.5; 5, 4, 3]';

% 로케이터 방향 초기 파라미터 (단위 벡터 기반)
% nor: Elevation 0도 평면의 수직(Normal) 벡터 / azi: Azimuth 90도 방향 벡터
% 이를 통해 안테나 배열이 바라보는 방향(Orientation)을 정의함
locator_ori_nor = [0, 1, 0; 1, 0, 0; 0, -1, 0; 0, 0, -1]; 
locator_ori_azi = [-1, 0, 0; 0, 1, 0; 1, 0, 0; -1, 0 , 0];

% 이동 노드(태그)의 주요 경유지(Waypoints) 설정
nodeWaypoints = [2, 6, 1.5; 2, 2, 1.5; 5, 2, 1.5; 5, 6, 1.5; 8, 6, 1.5; 8, 2, 1.5];
interpInterval = 0.5; % 경로 보간 간격 (0.5m 단위로 위치 생성)

% 위치 추정 방식 설정 (Linear: 선형 삼각측량, Non-Linear: 비선형 최적화)
estimationMethod = "Linear";        

% BLE AoA 물리 계층(PHY) 및 하드웨어 파라미터 설정
numDimensions = 3;                      % 3차원 추정
dfPacketType = "ConnectionCTE";         % 방향 탐지용 CTE(Constant Tone Extension) 포함 패킷
phyMode = "LE1M";                       % Bluetooth LE 1Mbps 모드
arraySize = [4 4];                      % 4x4 평면 안테나 배열 (URA, 총 16개 소자)
elementSpacing = [0.5 0.5];             % 안테나 소자 간 간격 (파장 $\lambda$ 대비 비율)
switchingPattern = 1:prod(arraySize);   % 안테나 스위칭 순서
slotDuration = 2;                       % 스위칭/샘플링 슬롯 지속 시간 ($\mu s$)
sps = 4;                                % 심볼당 샘플 수
channelIndex = 17;                      % 사용 채널 번호
crcInit = '555551';                     % CRC 초기화 값
accessAddress = '01234567';             % 접속 주소
payloadLength = 1;                      % 페이로드 길이
cteLength = 160;                        % CTE 구간 길이 ($\mu s$)
sampleOffset = 2;                       % 샘플링 시작 오프셋
EbNo = 15;                              % SNR 결정을 위한 비트 에너지 대비 잡음 밀도 (dB)
environment = "Outdoor";                % 경로 손실 모델 환경 (실외)
txPower = 0;                            % BLE 송신 전력 (dBm)

% Bluetooth 경로 손실(Path Loss) 모델 설정 객체 생성
rangeConfig = bluetoothRangeConfig;
rangeConfig.SignalPowerType = "ReceivedSignalPower";
rangeConfig.Environment = environment;
rangeConfig.TransmitterPower = txPower;
rangeConfig.TransmitterCableLoss = 0;
rangeConfig.ReceiverCableLoss = 0;

% =========================================================================
% 2. 쿼터니언을 이용한 로케이터 방향(설치 각도) 및 궤적 보간 처리
% =========================================================================
numLocators = size(locatorPos, 2);
rotMatArray = zeros(3, 3, numLocators); % 각 로케이터의 회전 행렬 저장용

% 로케이터의 로컬 좌표계를 글로벌 좌표계로 변환하기 위한 회전 행렬 계산
for i = 1:numLocators
    x_vec = locator_ori_nor(i, :);          % Normal 벡터를 x축으로 할당
    y_vec = locator_ori_azi(i, :);          % Azimuth 벡터를 y축으로 할당
    z_vec = cross(x_vec, y_vec);            % 두 벡터의 외적으로 z축 계산
    rotMatArray(:, :, i) = [x_vec', y_vec', z_vec']; % 3x3 회전 행렬 생성
end

% 회전 행렬을 쿼터니언(Quaternion) 객체로 변환 (좌표 회전 연산의 효율성 증대)
locatorQuat = quaternion(rotMatArray, 'rotmat', 'point');

% 노드 궤적 선형 보간 (경유지 사이를 interpInterval 간격으로 채움)
dists = sqrt(sum(diff(nodeWaypoints).^2, 2)); % 각 구간 거리 계산
cumDists = [0; cumsum(dists)];                % 누적 거리
queryPoints = 0:interpInterval:cumDists(end); % 샘플링 포인트 생성
posNode = interp1(cumDists, nodeWaypoints, queryPoints, 'linear')'; % 전체 경로 생성
numNodePositions = size(posNode, 2);

% =========================================================================
% 3. 수신기 및 BLE 객체 구성
% =========================================================================
% AoA 추정 알고리즘 설정 객체 생성
cfg = bleAngleEstimateConfig("ArraySize", arraySize, ...
    "SlotDuration", slotDuration, ...
    "SwitchingPattern", switchingPattern, ...
    "ElementSpacing", elementSpacing);
validateConfig(cfg); % 설정 유효성 검사

pos = getElementPosition(cfg); % 안테나 소자별 상대 위치 획득
cteType = [0;0]; % AoA 모드 설정

% 패킷 감지를 위한 프리앰블 및 참조 신호 생성
accessAddBitsLen = 32;
accessAddBits = int2bit(hex2dec(accessAddress), accessAddBitsLen, false);
refSamples = helperBLEReferenceWaveform(phyMode, accessAddBits, sps);
prbDet = comm.PreambleDetector(refSamples, "Detections", "First");

% 샘플링 레이트 및 주파수 보정 객체 설정
phyFactor = 1 + strcmp(phyMode, "LE2M");
sampleRate = 1e6 * phyFactor * sps;
coarsesync = comm.CoarseFrequencyCompensator("Modulation", "OQPSK", ...
    "SampleRate", sampleRate, "SamplesPerSymbol", 2*sps, "FrequencyResolution", 100);

% CRC 체크 및 화이트닝(Whitening) 해제 설정
crcLen = 24;
crcDet = comm.CRCDetector("x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1", "DirectMethod", true, ...
    "InitialConditions", int2bit(hex2dec(crcInit), crcLen).');
rng('default'); % 난수 초기화
snr = EbNo - 10*log10(sps); % SNR 계산

% 패킷 구조 파라미터
headerLen = 16 + 8 * strcmp(dfPacketType, "ConnectionCTE");
preambleLen = 8 * phyFactor;
dewhitenStateLen = 6;
chanIdxBin = int2bit(channelIndex, dewhitenStateLen).';
initState = [1 chanIdxBin];
dewhiten = bluetoothWhiten(InitialConditions=initState');

% 결과 저장용 버퍼 초기화
posNodeEst = zeros(numDimensions, numNodePositions);
posLocatorBuffer = cell(1, numNodePositions);
S = RandStream('mt19937ar', 'Seed', 5489);
pfo = comm.PhaseFrequencyOffset(SampleRate=sampleRate);
phaseNoise = comm.PhaseNoise(Level=[-130 -136], FrequencyOffset=[1e4 1e5], SampleRate=sampleRate);

% =========================================================================
% 4. 메인 루프: 노드 이동 시뮬레이션
% =========================================================================
for inumNode = 1:numNodePositions
    % 1) 현재 노드 위치에서 각 로케이터까지의 거리 계산 및 유효 로케이터(80m 이내) 선별
    distanceAll = vecnorm(locatorPos - posNode(:, inumNode));
    activeIdx = distanceAll <= 80;
    
    posActiveLocators = locatorPos(:, activeIdx);
    distanceActive = distanceAll(activeIdx)';
    activeLocIndices = find(activeIdx);
    numActiveLocators = sum(activeIdx);
    
    posLocatorBuffer{1, inumNode} = posActiveLocators;
    % 경로 손실(Path Loss) 선형 값 계산
    plLin = helperBluetoothEstimatePathLoss(rangeConfig.Environment, distanceActive);
    
    angleEst = zeros(numActiveLocators, numDimensions-1);
    [pathLossdB, linkFailFlag] = deal(zeros(numActiveLocators, 1));
    angleActiveLocal = zeros(numActiveLocators, numDimensions-1);
    
    % 2) 각 로케이터별 신호 송수신 및 각도 측정 시뮬레이션
    for i = 1:numActiveLocators
        locIdx = activeLocIndices(i);
        
        % [좌표 변환] 글로벌 방향 벡터를 로케이터의 로컬 좌표계로 회전 (역회전 conj 사용)
        dirVecGlobal = (posNode(:, inumNode) - posActiveLocators(:, i))'; 
        dirVecLocal = rotatepoint(conj(locatorQuat(locIdx)), dirVecGlobal);
        
        % 로컬 좌표에서의 실제 방위각(Azimuth) 및 고도각(Elevation) 계산
        [az_true, el_true, ~] = cart2sph(dirVecLocal(1), dirVecLocal(2), dirVecLocal(3));
        angleActiveLocal(i, :) = [rad2deg(az_true), rad2deg(el_true)];
        
        % BLE 데이터 패킷(PDU) 및 파형 생성
        data = helperBLEGenerateDFPDU(dfPacketType, cteLength, cteType, payloadLength, crcInit);
        bleWaveform = bleWaveformGenerator(data, "Mode", phyMode, "SamplesPerSymbol", sps, ...
            "ChannelIndex", channelIndex, "DFPacketType", dfPacketType, "AccessAddress", accessAddBits);
        
        % [무선 채널 모델링] 주파수 오프셋, 타이밍 오프셋, DC 오프셋, 위상 잡음 추가
        freqOffset = randsrc(1, 1, -10e3:100:10e3, S);
        pfo.FrequencyOffset = freqOffset;
        freqWaveform = pfo(bleWaveform);
        release(pfo);
        
        timingoff = randsrc(1, 1, 1:0.2:20, S);
        timingOffWaveform = helperBLEDelaySignal(freqWaveform, timingoff);
        
        dcMaxValue = max(max(real(timingOffWaveform)), max(imag(timingOffWaveform)));
        dcValue = (5/100) * randsrc(1, 1, (-1:0.05:1)*dcMaxValue) * (sqrt(0.5)+sqrt(0.5)*1i);
        dcWaveform = timingOffWaveform + dcValue;
   
        noisyWaveform = phaseNoise(dcWaveform);
        release(phaseNoise);
        
        % 송신 전력 및 경로 손실(거리 감쇠) 적용
        dBmConverter = 30;
        txImpairedWaveform = 10^((rangeConfig.TransmitterPower-dBmConverter)/20)*noisyWaveform/plLin(i);
        
        % 로컬 벡터 기반 수신단 안테나별 위상차(Steering Vector) 적용 (AoA 핵심)
        steerVec = helperBLESteeringVector(angleActiveLocal(i,:), pos);
        steeredWaveform = txImpairedWaveform .* steerVec.';
        
        % 3) 수신기 신호 처리 (디모듈레이션 및 동기화)
        packetDetect = 0; headerFlag = 1; pduCRCFlag = 0; samplingFlag = 0; crcError = 1;
        samplesPerFrame = 8*sps; samplesPerModule = samplesPerFrame+sps;
        numTimes = ceil(length(timingOffWaveform)/samplesPerFrame)+1;
        countpacketDetect = 0; moduleStartInd = 0;
        
        rxWaveformBuffer = [steeredWaveform; zeros(samplesPerFrame*numTimes-length(steeredWaveform)+samplesPerFrame, size(steeredWaveform,2))];
        rcvSigBuffer = [];
        
        for j = 1:numTimes
            % 프레임 단위로 수신 신호 처리
            if (j-1)*samplesPerFrame + moduleStartInd + samplesPerModule <= length(rxWaveformBuffer)
                rxChunkWaveform = rxWaveformBuffer((j-1)*samplesPerFrame+moduleStartInd+(1:samplesPerModule), :);
            else
                rxChunkWaveform = rxWaveformBuffer((j-1)*samplesPerFrame+moduleStartInd+1:end, :);
            end
            
            if ~samplingFlag
                rxNoisyWaveform = awgn(rxChunkWaveform(:,1), snr, "measured"); 
            else
                % CTE 구간에서 안테나 스위칭 시뮬레이션 및 IQ 샘플 추출
                rxSwitchWaveform = helperBLESwitchAntenna(rxChunkWaveform, phyMode, sps, slotDuration, switchingPattern);
                rxNoisyWaveform = awgn(rxSwitchWaveform, snr, "measured"); 
        
                cteSamples = rxNoisyWaveform(1:cteTime*8*sps*phyFactor);
                iqSamples = bleCTEIQSample(cteSamples, "Mode", phyMode, ...
                    "SamplesPerSymbol", sps, "SlotDuration", slotDuration, "SampleOffset", sampleOffset);
                samplingFlag = 0;
                break;
            end
            
            % 패킷 동기화 및 헤더 디코딩 과정
            if packetDetect == 0
                countpacketDetect = countpacketDetect + 1;
                rcvSigBuffer((countpacketDetect-1)*(samplesPerModule-sps)+(1:samplesPerModule)) = rxNoisyWaveform;
                
                if countpacketDetect >= (preambleLen+accessAddBitsLen+headerLen)*sps/samplesPerFrame
                    % 프리앰블 검출
                    [~, dtMt] = prbDet(rcvSigBuffer.');
                    release(prbDet);
                    prbDet.Threshold = max(dtMt);
                    prbIdx = prbDet(rcvSigBuffer.');
                    release(prbDet);
                    
                    if prbIdx >= length(refSamples)
                        rcvTrim = rcvSigBuffer(1+prbIdx-length(refSamples):end).';
                    else
                        rcvTrim = rcvSigBuffer.';
                    end
                    
                    % DC 오프셋 및 주파수 오프셋 보정
                    estimatedDCOffset = mean(rcvTrim(1:length(refSamples))) - mean(refSamples)*sqrt(var(rcvTrim));
                    rcvDCFree = rcvTrim - estimatedDCOffset;
                    
                    [~, freqoff] = coarsesync(rcvDCFree);
                    release(coarsesync);
                    [rcvFreqOffsetFree, iniFreqState] = helperBLEFrequencyOffset(rcvDCFree, sampleRate, -freqoff);
                    
                    % GMSK 복조 및 액세스 주소 확인
                    x = rem(length(rcvFreqOffsetFree), sps);
                    if x, remsad = sps - x; else, remsad = x; end
                    remNumSamples = x;
                    remSamples = rcvFreqOffsetFree(end-remNumSamples+1:end);
                    
                    [demodSoftBits, demodInitPhase] = helperBLEGMSKDemod(rcvFreqOffsetFree(1:end-remNumSamples), phyMode, sps, 0);
                    demodBits = demodSoftBits > 0;
                    
                    if length(demodBits) >= preambleLen + accessAddBitsLen
                        packetAddress = int8(demodBits(preambleLen+(1:accessAddBitsLen)) > 0);
                        decodeData = int8(demodBits(accessAddBitsLen+preambleLen+1:end) > 0);
                        if isequal(accessAddBits, packetAddress)
                            packetDetect = 1; % 패킷 검출 성공
                            samplesPerModule = headerLen*sps - length(decodeData)*sps + remsad + sps;
                            rcvSigBuffer = [];
                        end
                    end
                end
            end
            
            % PDU 헤더 분석 및 CTE 정보 추출
            if packetDetect
                if headerFlag && (length(rxNoisyWaveform) ~= samplesPerFrame+sps || samplesPerModule <= sps)
                    rxDCFreeWaveform = rxNoisyWaveform - estimatedDCOffset;
                    [rxNoisyWaveform, iniFreqState] = helperBLEFrequencyOffset(rxDCFreeWaveform, sampleRate, -freqoff, iniFreqState);
                    
                    if (length(rxNoisyWaveform) ~= samplesPerFrame)
                        rcvSigHeaderBuffer = [remSamples; rxNoisyWaveform];
                        remSamp = rem(length(rcvSigHeaderBuffer), sps);
                        if remSamp, appSamp = sps - remSamp; else, appSamp = remSamp; end
                        rcvSigHeaderBuffer = [rcvSigHeaderBuffer; zeros(appSamp,1)];
                        [headerSoftBits, demodInitPhase] = helperBLEGMSKDemod(rcvSigHeaderBuffer, phyMode, sps, demodInitPhase);
                        decodeDataHeader = [decodeData; headerSoftBits > 0];
                    elseif samplesPerModule <= 0
                        decodeDataHeader = decodeData;
                    end
                    
                    % 데이터 화이트닝 해제 및 CTE 타입 확인
                    dewhitenedBits = dewhiten(decodeDataHeader);
                    pduLenField = double(dewhitenedBits(9:16));
                    pduLenInBits = bi2de(pduLenField') * 8;
                    
                    [cteTime, cteTypeDec] = helperBLECTEInfoExtract(dewhitenedBits, dfPacketType);
                    if cteTypeDec == 1 || cteTypeDec == 2
                        slotDuration = cteTypeDec;
                    else
                        slotDuration = cfg.SlotDuration;
                    end
                    
                    if samplesPerModule, moduleStartInd = samplesPerModule-samplesPerFrame-sps; else, moduleStartInd = -sps; end
                    samplesPerModule = (pduLenInBits+crcLen-length(dewhitenedBits)+headerLen+1)*sps;
                    pduCRCFlag = 1; headerFlag = 0;
                    rcvSigHeaderVar = rxNoisyWaveform;
                    rxNoisyWaveform = [];
                end
                
                % CRC 검사 수행
                if pduCRCFlag && ~isempty(rxNoisyWaveform)
                    rxDCFreeWaveform = rxNoisyWaveform - estimatedDCOffset;
                    [rxNoisyWaveform, iniFreqState] = helperBLEFrequencyOffset(rxDCFreeWaveform, sampleRate, -freqoff, iniFreqState);
                    
                    x = rem(length(rxNoisyWaveform), sps);
                    if x, remNumSamples = sps - x; else, remNumSamples = x; end
                    
                    demodPDUCRC = helperBLEGMSKDemod([rxNoisyWaveform; zeros(remNumSamples,1)], phyMode, sps, demodInitPhase);
                    dewhitenedPDUCRC = dewhiten(demodPDUCRC);
                    reset(dewhiten);
                    headerPDUCRC = [double(dewhitenedBits); dewhitenedPDUCRC];
                    
                    [dfPDU, crcError] = crcDet(headerPDUCRC);
                    if crcError, break; end % CRC 오류 시 루프 탈출
                    
                    moduleStartInd = samplesPerModule - samplesPerFrame + moduleStartInd - sps;
                    samplesPerModule = cteTime * 8 * sps * phyFactor;
                    samplingFlag = 1; pduCRCFlag = 0;
                    rcvSigPDUVar = rxNoisyWaveform;
                end
            end
        end
        
        % 4) [각도 추정] 획득한 IQ 샘플을 바탕으로 AoA 계산
        refSampleLength = 8;                                            
        minIQSamples = refSampleLength + getNumElements(cfg) - 1;
        
        if ~crcError && length(nonzeros(iqSamples)) >= minIQSamples     
            % 수신기 로컬 좌표계 기준의 도래각(AoA) 추정
            angleEstLocal = bleAngleEstimate(iqSamples, cfg);
            
            % [좌표 변환] 추정된 로컬 각도를 글로벌 좌표계로 다시 변환 (정방향 회전)
            [lx, ly, lz] = sph2cart(deg2rad(angleEstLocal(1)), deg2rad(angleEstLocal(2)), 1);
            dirVecGlobalEst = rotatepoint(locatorQuat(locIdx), [lx, ly, lz]);
            
            % 최종 글로벌 방위각 및 고도각 저장
            [gaz, gel, ~] = cart2sph(dirVecGlobalEst(1), dirVecGlobalEst(2), dirVecGlobalEst(3));
            angleEst(i, :) = [rad2deg(gaz), rad2deg(gel)];
            
            % RSSI를 기반으로 경로 손실 역산
            rangeConfig.ReceivedSignalPower = 10*log10(var([rcvFreqOffsetFree; rcvSigHeaderVar; rcvSigPDUVar])) + 30;
            pathLossdB(i) = pathLoss(rangeConfig);
        else
            linkFailFlag(i) = 1; % 통신 실패 표시
        end
    end
    
    % 수신에 실패한 로케이터 데이터 제외
    if any(linkFailFlag == 1)
        idx = find(linkFailFlag == 1);
        posActiveLocators(:,idx) = [];
        angleEst(idx,:) = [];
        pathLossdB(idx) = [];
    end
    
    % 5) [최종 위치 추정] 삼각측량 알고리즘 수행
    validIdx = find(~linkFailFlag);
    validLocatorsCount = length(validIdx);
    
    % 최소 2개 이상의 로케이터로부터 유효한 각도를 얻었을 때 위치 계산
    if validLocatorsCount >= 2 && ~all(angleEst(:,1) == angleEst(1,1))
        % 선형 삼각측량법으로 기초 위치 계산
        posLin = blePositionEstimate(posActiveLocators(:, validIdx), "angulation", angleEst(validIdx, :).');
        
        if strcmp(estimationMethod, "Non-linear")
            % 비선형 정밀화(가우스-뉴턴 등)를 통해 좌표 보정
            posNodeEst(:, inumNode) = nonLinearTriangulation(...
                posActiveLocators(:, validIdx), ...
                angleEst(validIdx, :), ...
                posLin, ...
                roomSize);
        else
            % 선형 측정 결과 그대로 사용
            posNodeEst(:, inumNode) = posLin;
        end
    else
        % 추정 불가능한 경우 NaN(Not a Number) 처리
        posNodeEst(:, inumNode) = NaN(numDimensions, 1);
    end
end

% =========================================================================
% 5. 결과 출력 및 시각화
% =========================================================================
% 실제 위치와 추정 위치 간의 평균 제곱근 오차(RMSE) 계산
posErr = sqrt(sum((posNodeEst - posNode).^2)); 
disp(["Positioning error (RMSE) in meters = ", num2str(mean(posErr, 'omitnan'))])

% 시뮬레이션 결과 3D 시각화
if ~all(isnan(posNodeEst(1,:)))
    helperBLEVisualizeNodeTracking(locatorPos, posNode, posLocatorBuffer, posNodeEst)
    
    % 시각화 범위 고정 및 시점 조정
    axis([0 roomSize(1) 0 roomSize(2) 0 roomSize(3)]); 
    view(45, 30); 
    title('BLE 3D Tracking Simulation Space');
end