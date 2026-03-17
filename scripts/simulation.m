function [posNodeEst, fim] = simulation(posNode, locatorsPos, locatorsUp, locatorsFront, varargin)
    % [필수 인자]
    % posNode, locatorsPos, locatorsUp, locatorsFront
    % 
    % [선택 인자 (Name-Value Pair)]
    % 'enableMultipath'  : (default) false
    % 'material'         : (default) 'concrete'
    % 'enableWeighted'   : (default) true
    % 'estimationMethod' : (default) 'Non-linear'
    % 'visibility'       : (default) 80 
    % 'stlFileName'      : (default) '' (레이트레이싱용 3D 맵 데이터)

    %% 1. 입력 인자 파싱 (Input Parser)
    p = inputParser;
    
    % 필수 인자 확인
    addRequired(p, 'posNode');
    addRequired(p, 'locatorsPos');
    addRequired(p, 'locatorsUp');
    addRequired(p, 'locatorsFront');
    
    % 선택 인자 및 기본값 설정
    addParameter(p, 'enableMultipath', false);
    addParameter(p, 'enableWeighted', true);
    addParameter(p, 'estimationMethod', 'Non-linear');
    addParameter(p, 'visibility', 80);
    addParameter(p, 'stlFileName', '');
    addParameter(p, 'material', 'concrete');
    
    % 파싱 실행
    parse(p, posNode, locatorsPos, locatorsUp, locatorsFront, varargin{:});
    
    % 변수 할당
    enableMultipath = p.Results.enableMultipath;
    enableWeighted = p.Results.enableWeighted;
    estimationMethod = p.Results.estimationMethod;
    visibility = p.Results.visibility;
    stlFileName = p.Results.stlFileName;
    material = p.Results.material;

    %% 2. 입력 인자 파싱 (Input Parser)

    % BLE AoA 물리 계층(PHY) 및 하드웨어 파라미터 설정
    numDimensions = 3;                      % 3차원 추정
    dfPacketType = "ConnectionCTE";         % 방향 탐지용 CTE(Constant Tone Extension) 포함 패킷
    phyMode = "LE1M";                       % Bluetooth LE 1Mbps 모드
    arraySize = [3 3];                      % 4x4 평면 안테나 배열 (URA, 총 16개 소자)
    elementSpacing = [0.5 0.5];             % 안테나 소자 간 간격 (파장 대비 비율)
    switchingPattern = 1:prod(arraySize);   % 안테나 스위칭 순서
    slotDuration = 2;                       % 스위칭/샘플링 슬롯 지속 시간 (ms)
    sps = 4;                                % 심볼당 샘플 수
    channelIndex = 17;                      % 사용 채널 번호 (고정채널)
    crcInit = '555551';                     % CRC 초기화 값
    accessAddress = '01234567';             % 접속 주소
    payloadLength = 1;                      % 페이로드 길이 (바이트)
    cteLength = 160;                        % CTE 구간 길이 (ms)
    sampleOffset = 2;                       % 샘플링 시작 오프셋
    EbNo = 15;                              % SNR 결정을 위한 비트 에너지 대비 잡음 밀도 (dB) 5~10(생존) | 15~20(안정) | 25(이상)
    environment = "Industrial";             % 경로 손실 모델 환경 'Outdoor', 'Industrial', 'Home', 'Office'
    txPower = 0;                            % BLE 송신 전력 0~4 (dBm) 0dBm = 1mW

    % 파장(Lambda) 계산 (BLE 중심 주파수 2402MHz + 2MHz * channelIndex 기준)
    fc = 2402e6 + channelIndex * 2e6; 
    lambda = 3e8 / fc; % 빛의 속도 / 주파수
    
    % Bluetooth 경로 손실(Path Loss) 모델 설정 객체 생성
    rangeConfig = bluetoothRangeConfig;
    rangeConfig.SignalPowerType = "ReceivedSignalPower";
    rangeConfig.Environment = environment;
    rangeConfig.TransmitterPower = txPower;
    rangeConfig.TransmitterCableLoss = 0;
    rangeConfig.ReceiverCableLoss = 0;


    %% 3. 쿼터니언을 이용한 로케이터 방향(설치 각도) 및 궤적 보간 처리
    numLocators = size(locatorsPos, 2);      % 행렬의 칼럼(2)의 갯수
    rotMatArray = zeros(3, 3, numLocators); % 각 로케이터의 회전 행렬 저장용
    
    % 로케이터의 로컬 좌표계를 글로벌 좌표계로 변환하기 위한 회전 행렬 계산
    for i = 1:numLocators
        x_vec = locatorsFront(i, :);
        x_vec = x_vec / norm(x_vec);
        
        z_vec = locatorsUp(i, :);
        z_vec = z_vec / norm(z_vec);
        
        % 오른손 법칙에 따라 Side(Y축) 계산: Y = Z x X
        y_vec = cross(z_vec, x_vec);
        y_vec = y_vec / norm(y_vec);
    
        rotMatArray(:, :, i) = [x_vec', y_vec', z_vec'];
    end
    
    % 회전 행렬을 쿼터니언(Quaternion) 객체로 변환 (좌표 회전 연산의 효율성)
    locatorQuat = quaternion(rotMatArray, 'rotmat', 'point');
    
    % 측위 갯수
    numNodePositions = size(posNode, 2);    % 행렬의 칼럼(2)의 갯수

    %% 4. 수신기 및 BLE 객체 구성
    
    % BLE 멀티 안테나 객체(Locator) 생성
    cfg = bleAngleEstimateConfig("ArraySize", arraySize, ...
        "SlotDuration", slotDuration, ...
        "SwitchingPattern", switchingPattern, ...
        "ElementSpacing", elementSpacing);
    validateConfig(cfg);                                        % 설정 유효성 검사
    pos = getElementPosition(cfg);                              % 안테나 소자별 상대 위치 획득
    cteType = [0;0];                                            % AoA 모드 설정
    
    % 패킷 감지를 위한 프리앰블 & 프리앰블 검출기 생성
    accessAddBitsLen = 32;
    accessAddBits = int2bit(hex2dec(accessAddress), accessAddBitsLen, false);
    refSamples = helperBLEReferenceWaveform(phyMode, accessAddBits, sps);
    prbDet = comm.PreambleDetector(refSamples, "Detections", "First");
    
    % 샘플링 레이트 및 주파수 보정 객체 설정
    phyFactor = 1 + strcmp(phyMode, "LE2M");
    sampleRate = 1e6 * phyFactor * sps;
    coarsesync = comm.CoarseFrequencyCompensator("Modulation", "OQPSK", ...
        "SampleRate", sampleRate, "SamplesPerSymbol", 2*sps, "FrequencyResolution", 100);
    
    % 패킷 구조 파라미터
    headerLen = 16 + 8 * strcmp(dfPacketType, "ConnectionCTE");
    preambleLen = 8 * phyFactor;
    
    % CRC 오류 체크 객체 설정
    crcLen = 24;
    crcDet = comm.CRCDetector("x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1", "DirectMethod", true, ...
        "InitialConditions", int2bit(hex2dec(crcInit), crcLen).');
    rng('default');             % 난수 초기화
    
    % SNR 산출 : 샘플링 속도가 빨라질수록, 관측하는 대역폭이 증가하여 잡음전력이 증가 (신호 대역폭은 일정)
    snr = EbNo - 10*log10(sps);
    
    % 화이트닝 (Whitening / De-whitening) 객체 설정
    dewhitenStateLen = 6;
    chanIdxBin = int2bit(channelIndex, dewhitenStateLen).';     % 화이트닝은 채널에 의존적
    initState = [1 chanIdxBin];
    dewhiten = bluetoothWhiten(InitialConditions=initState');
    
    % 전체 결과 저장용 버퍼 초기화
    fim = zeros(numDimensions, numDimensions, numNodePositions);                   % FIM 기반 평가 지표 
    posNodeEst = zeros(numDimensions, numNodePositions);    % 지점별 측위 위치
    posLocatorBuffer = cell(1, numNodePositions);           % 지점별 사용 로케이터
    S = RandStream('mt19937ar', 'Seed', 1234);
    
    % 통신 채널의 아날로그 열화(Impairment) 모델 객체
    pfo = comm.PhaseFrequencyOffset(SampleRate=sampleRate); % 주파수 오프셋에 적용
    phaseNoise = comm.PhaseNoise(Level=[-130 -136], FrequencyOffset=[1e4 1e5], SampleRate=sampleRate);  % 위상 잡음의 적용
    
    %% 5. 다중 경로 레이트레이싱 환경 구성
    if enableMultipath && ~isempty(stlFileName)
        % Viewer를 띄우고 지도를 불러옵니다.
        sv = siteviewer("SceneModel", stlFileName);
        pm = propagationModel("raytracing", "Method", "sbr", "CoordinateSystem", "cartesian", "SurfaceMaterial", material);
    end
            
    %% 6. 메인 루프 : 노드 이동 시뮬레이션
    for inumNode = 1:numNodePositions
        % [1] 현재 노드 위치에서 각 로케이터까지의 거리 계산 및 유효 로케이터(80m 이내) 선별
        distanceAll = vecnorm(locatorsPos - posNode(:, inumNode));   % 1 X N
        activeIdx = distanceAll <= visibility;                      % Ex: [1, 0, 1, 1]
        
        % 활성화 노드에 대한 데이터 추출
        posActiveLocators = locatorsPos(:, activeIdx);
        distanceActive = distanceAll(activeIdx)';
        activeLocIndices = find(activeIdx);
        numActiveLocators = sum(activeIdx);
        posLocatorBuffer{1, inumNode} = posActiveLocators;  % 전체 결과 버퍼에 저장
        
        % 거리 기반 경로 손실(Path Loss) 선형 값 계산
        plLin = helperBluetoothEstimatePathLoss(rangeConfig.Environment, distanceActive);
    
        % FIM 계산
        rotMatActive = rotMatArray(:, :, activeIdx);
        base_snr_lin = 10^(snr / 10);
        SNR_Lin_Array = base_snr_lin ./ plLin'; % 벡터 연산을 통해 각 앵커별 선형 SNR 획득

        fim(:, :, inumNode) = cumulativeFIMCalculation(posNode(:, inumNode), posActiveLocators', rotMatActive, pos, lambda, SNR_Lin_Array);
        
        % 임시 데이터 저장용 파라미터 생성
        angleEstGlobal = zeros(numActiveLocators, numDimensions-1);
        [pathLossdB, linkFailFlag] = deal(zeros(numActiveLocators, 1));
        angleGtLocal = zeros(numActiveLocators, numDimensions-1);

        % 앵커별 신뢰도(가중치) 저장용 배열 초기화
        confidenceWeights = ones(numActiveLocators, 1);
    
        % BLE 데이터 패킷(PDU) 및 파형 생성
        data = helperBLEGenerateDFPDU(dfPacketType, cteLength, cteType, payloadLength, crcInit);
        bleWaveform = bleWaveformGenerator(data, "Mode", phyMode, "SamplesPerSymbol", sps, ...
            "ChannelIndex", channelIndex, "DFPacketType", dfPacketType, "AccessAddress", accessAddBits);
    
        % Timing offset을 고려한 Safe 버퍼 추가
        maxTimingOffset = 20;       % 현재 설정된 최대값
        extraMargin = 10;           % 필터 여유분
        safeBleWaveform = [bleWaveform; zeros(maxTimingOffset + extraMargin, 1)];
          
        % [2] 각 로케이터별 신호 송수신 및 각도 측정 시뮬레이션
        for i = 1:numActiveLocators
            locIdx = activeLocIndices(i);
            
            % [좌표 변환] 글로벌 방향 벡터를 로케이터의 로컬 좌표계로 회전 (역회전 conj 사용)
            dirVecGlobal = (posNode(:, inumNode) - posActiveLocators(:, i))'; 
            dirVecLocal = rotatepoint(conj(locatorQuat(locIdx)), dirVecGlobal);
            
            % 로컬 좌표에서의 실제 방위각(Azimuth) 및 고도각(Elevation) 계산
            % Azimuth는 front 방향을 기준으로 Y축(반시계)방향으로 측정
            [az_true, el_true, ~] = cart2sph(dirVecLocal(1), dirVecLocal(2), dirVecLocal(3));
            angleGtLocal(i, :) = [rad2deg(az_true), rad2deg(el_true)];
            
            % 주파수 오프셋
            freqOffset = randsrc(1, 1, -10e3:100:10e3, S);
            pfo.FrequencyOffset = freqOffset;
            freqWaveform = pfo(safeBleWaveform);
            reset(pfo);
            
            % 타이밍 오프셋
            timingoff = randsrc(1, 1, 1:0.2:maxTimingOffset, S);
            timingOffWaveform = helperBLEDelaySignal(freqWaveform, timingoff);
            
            % DC 오프셋
            dcMaxValue = max(max(real(timingOffWaveform)), max(imag(timingOffWaveform)));
            dcValue = (5/100) * randsrc(1, 1, (-1:0.05:1)*dcMaxValue) * (sqrt(0.5)+sqrt(0.5)*1i);
            dcWaveform = timingOffWaveform + dcValue;
       
            % 위상잡음
            noisyWaveform = phaseNoise(dcWaveform);
            reset(phaseNoise);
            
            % 단위 변환 상수
            dBmConverter = 30; 
    
            if enableMultipath
                % 직교 좌표계(Cartesian) 기반 레이트레이싱 송수신점 생성
                tx = txsite("cartesian", "AntennaPosition", posNode(:, inumNode));
                rx = rxsite("cartesian", "AntennaPosition", posActiveLocators(:, i));
                
                % 레이트레이싱 수행, Ray마다 별도의 PathLoss와 PhaseShift를 가짐
                raysAll = raytrace(tx, rx, pm);
                
                % 시간축 샘플 갯수 X 전체 안테나 갯수
                steeredWaveform = zeros(length(noisyWaveform), prod(arraySize));
    
                % 처리 가능 송수신쌍 존재 여부 확인
                if ~isempty(raysAll) && ~isempty(raysAll{1})
                    rays = raysAll{1};
                    
                    for rIdx = 1:numel(rays)
                        % 글로벌 도래각 (수신기로 도착하는 방향의 각도)
                        ray_az = rays(rIdx).AngleOfArrival(1);
                        ray_el = rays(rIdx).AngleOfArrival(2);
                        
                        % 수신기에서 송신기(혹은 반사점)를 향하는 벡터
                        [rx_x, rx_y, rx_z] = sph2cart(deg2rad(ray_az), deg2rad(ray_el), 1);
                        rayDirGlobal = [rx_x, rx_y, rx_z];
                        
                        % [멀티패스 좌표변환] 앵커의 로컬 좌표계로 회전
                        rayDirLocal = rotatepoint(conj(locatorQuat(locIdx)), rayDirGlobal);
                        [raz_local, rel_local, ~] = cart2sph(rayDirLocal(1), rayDirLocal(2), rayDirLocal(3));
                        rayAngleLocal = [rad2deg(raz_local), rad2deg(rel_local)];
                        
                        % 멀티패스 각도별 Steering Vector 적용
                        steerVec_ray = helperBLESteeringVector(rayAngleLocal, pos);
                        
                        % 경로 손실 및 위상 편이 적용
                        rayAmp = 10^((rangeConfig.TransmitterPower - dBmConverter)/20) * 10^(-rays(rIdx).PathLoss/20);
                        rayPhase = exp(-1i * rays(rIdx).PhaseShift);
                        delayedWaveform = noisyWaveform * rayAmp * rayPhase;
                        
                        % 모든 멀티패스 신호 중첩
                        steeredWaveform = steeredWaveform + delayedWaveform .* steerVec_ray.';
                    end

                else
                    % 벽 등에 완전히 막혀 경로가 하나도 없는 경우 통신 실패 처리
                    steeredWaveform = zeros(length(noisyWaveform), prod(arraySize));
                end
            else
                % 단일 경로 시뮬레이션 : 경로가 하나이므로 위상차 무시
                txImpairedWaveform = 10^((rangeConfig.TransmitterPower-dBmConverter)/20)*noisyWaveform/plLin(i);
                steerVec = helperBLESteeringVector(angleGtLocal(i,:), pos);
                steeredWaveform = txImpairedWaveform .* steerVec.';
            end
            
            % [3] 수신기 신호 처리 (디모듈레이션 및 동기화)
            
            % 로케이터 별 수신 신호 처리 플래그
            packetDetect = 0;       % 패킷(접속 주소) 탐지 여부
            headerFlag = 1;         % 패킷을 감지했을 때, 헤더를 읽을 단계인지 여부
            pduCRCFlag = 0;         % 본문(PDU)과 에러 체크(CRC)를 처리할 단계인지 여부
            samplingFlag = 0;       % 패킷을 처리하고 있는지 '상태'를 가리키는 지표
            crcError = 1;           % 무결성 검사 통과 여부
            
            % 패킷 처리 단위 (프레임)
            samplesPerFrame = 8*sps;
            samplesPerModule = samplesPerFrame+sps;                             % 중첩을 고려하려 프레임은 1비트 여유를 가지고 읽음
            numTimes = ceil(length(timingOffWaveform)/samplesPerFrame)+1;       % 프레임 처리 반복 횟수 (+1의 마진)
            
            countpacketDetect = 0;      % 탐지 시도 횟수
            moduleStartInd = 0;         % 프레임 탐지 인덱스
            
            % 반복문에서 out of index 에러가 발생하지 않도록 최소한의 패딩을 더함
            rxWaveformBuffer = [steeredWaveform; zeros(samplesPerFrame*numTimes-length(steeredWaveform)+samplesPerFrame, size(steeredWaveform,2))];
            rcvSigBuffer = [];      % 수신 신호 누적 버퍼
            
            % 프레임 단위로 수신 신호 처리
            for j = 1:numTimes
                % 프레임 추출
                if (j-1)*samplesPerFrame + moduleStartInd + samplesPerModule <= length(rxWaveformBuffer)
                    rxChunkWaveform = rxWaveformBuffer((j-1)*samplesPerFrame+moduleStartInd+(1:samplesPerModule), :);
                else
                    % 패킷의 끝을 포함하는 프레임
                    rxChunkWaveform = rxWaveformBuffer((j-1)*samplesPerFrame+moduleStartInd+1:end, :);
                end
                
                % 데이터가 비면 즉시 루프 탈출
                if isempty(rxChunkWaveform)
                    break; 
                end
    
                % 패킷 감지 상태 (Preamable + PDU + CRC)
                if ~samplingFlag
                    % CTE 구간이 아닐 경우, 첫번째 안테나만 사용, 안테나 열잡음 추가
                    rxNoisyWaveform = awgn(rxChunkWaveform(:,1), snr, "measured"); 
                else
                    % CTE 구간일 경우, 안테나 스위칭 시뮬레이션 및 IQ 샘플 추출
                    rxSwitchWaveform = helperBLESwitchAntenna(rxChunkWaveform, phyMode, sps, slotDuration, switchingPattern);
                    rxNoisyWaveform = awgn(rxSwitchWaveform, snr, "measured"); 
                    
                    % 유효 CTE 구간 추출 및 IQ 샘플 변경
                    cteSamples = rxNoisyWaveform(1:cteTime*8*sps*phyFactor);
                    iqSamples = bleCTEIQSample(cteSamples, "Mode", phyMode, ...
                        "SamplesPerSymbol", sps, "SlotDuration", slotDuration, "SampleOffset", sampleOffset);
                    samplingFlag = 0;
                    break;
                end
                
                % 패킷을 아직 감지 못했을 경우
                if packetDetect == 0
                    % 패킷 카운트 증가, 수신 신호 버퍼에 누적
                    countpacketDetect = countpacketDetect + 1;
                    rcvSigBuffer((countpacketDetect-1)*(samplesPerModule-sps)+(1:samplesPerModule)) = rxNoisyWaveform;
                    
                    % (프리앰블 + 접속 주소 + 헤더)을 모두 수용할 만큼 버퍼가 쌓였을 때 패킷 검출 시도
                    if countpacketDetect >= (preambleLen+accessAddBitsLen+headerLen)*sps/samplesPerFrame
                        % 프리앰블 검출
                        [~, dtMt] = prbDet(rcvSigBuffer.'); % 상관관계 점수 계산
                        prbDet.Threshold = max(dtMt);       % 시작점을 추출하기 위해 peak를 threshold로 강제 설정
                        prbIdx = prbDet(rcvSigBuffer.');    % 패킷 시작점 추출 (프리앰즐이 끝난 지점)
                        release(prbDet);
                        
                        % 프리앰블 시작 부분으로 옮김
                        if prbIdx >= length(refSamples)
                            rcvTrim = rcvSigBuffer(1+prbIdx-length(refSamples):end).';
                        else
                            % 누적 버퍼보다 프리앰블보다 짧더라도 일단 시작지점 지정, 복조기 내부 저항성 활용
                            rcvTrim = rcvSigBuffer.';
                        end
                        
                        % DC 오프셋 보정
                        estimatedDCOffset = mean(rcvTrim(1:length(refSamples))) - mean(refSamples)*sqrt(var(rcvTrim));
                        rcvDCFree = rcvTrim - estimatedDCOffset;
                        
                        % 주파수 오프셋 보정
                        [~, freqoff] = coarsesync(rcvDCFree);
                        release(coarsesync);
                        [rcvFreqOffsetFree, iniFreqState] = helperBLEFrequencyOffset(rcvDCFree, sampleRate, -freqoff);
                        
                        % 1바이트 단위로 GMSK 복조 실행
                        x = rem(length(rcvFreqOffsetFree), sps);
                        if x, remsad = sps - x; else, remsad = x; end
                        
                        % 1바이트로 딱 떨어지지지 않는 데이터는 다음 단계 (PDU 분석)에서 재활용
                        remNumSamples = x;      
                        remSamples = rcvFreqOffsetFree(end-remNumSamples+1:end);
                        
                        % demodSoftBits: 소숫점 표현 / demodInitPhase: 마지막 위상값 
                        [demodSoftBits, demodInitPhase] = helperBLEGMSKDemod(rcvFreqOffsetFree(1:end-remNumSamples), phyMode, sps, 0);
                        demodBits = demodSoftBits > 0;
                        
                        if length(demodBits) >= preambleLen + accessAddBitsLen
                            packetAddress = int8(demodBits(preambleLen+(1:accessAddBitsLen)) > 0);
                            decodeData = int8(demodBits(accessAddBitsLen+preambleLen+1:end) > 0);
                            if isequal(accessAddBits, packetAddress)
                                packetDetect = 1; % 패킷 검출 성공
                                samplesPerModule = headerLen*sps - length(decodeData)*sps + remsad + sps;       % 파라미터 업데이트
                                rcvSigBuffer = [];
                            end
                        end
                    end
                end
                
                % PDU & CTE 분석 단계
                if packetDetect
                    if headerFlag && (length(rxNoisyWaveform) ~= samplesPerFrame+sps || samplesPerModule <= sps)
                        % DC 및 주파수 오프셋 보정
                        rxDCFreeWaveform = rxNoisyWaveform - estimatedDCOffset;
                        [rxNoisyWaveform, iniFreqState] = helperBLEFrequencyOffset(rxDCFreeWaveform, sampleRate, -freqoff, iniFreqState);
                        
                        % 자투리 샘플 보정 (비트 단위 정렬) 및 복조
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
                        
                        % 데이터 화이트닝  
                        dewhitenedBits = dewhiten(decodeDataHeader);
    
                        pduLenField = double(dewhitenedBits(9:16));
                        pduLenInBits = bi2de(pduLenField') * 8;
                        
                        % CTE 타입 확인
                        [cteTime, cteTypeDec] = helperBLECTEInfoExtract(dewhitenedBits, dfPacketType);
                        if cteTypeDec == 1 || cteTypeDec == 2
                            slotDuration = cteTypeDec;
                        else
                            slotDuration = cfg.SlotDuration;
                        end
                        
                        % 파라미터 업데이트
                        if samplesPerModule, moduleStartInd = samplesPerModule-samplesPerFrame-sps; else, moduleStartInd = -sps; end
                        samplesPerModule = (pduLenInBits+crcLen-length(dewhitenedBits)+headerLen+1)*sps;
                        pduCRCFlag = 1; headerFlag = 0;
                        rcvSigHeaderVar = rxNoisyWaveform;
                        rxNoisyWaveform = [];
                    end
                    
                    % CRC 검사 수행
                    if pduCRCFlag && ~isempty(rxNoisyWaveform)
                        % DC 오프셋 및 주파수 오프셋 보정
                        rxDCFreeWaveform = rxNoisyWaveform - estimatedDCOffset;
                        [rxNoisyWaveform, iniFreqState] = helperBLEFrequencyOffset(rxDCFreeWaveform, sampleRate, -freqoff, iniFreqState);
                        
                        % 자투리 샘플 보정 (비트 단위 정렬)
                        x = rem(length(rxNoisyWaveform), sps);
                        if x, remNumSamples = sps - x; else, remNumSamples = x; end
                        
                        % 데이터 화이트닝  
                        demodPDUCRC = helperBLEGMSKDemod([rxNoisyWaveform; zeros(remNumSamples,1)], phyMode, sps, demodInitPhase);
                        dewhitenedPDUCRC = dewhiten(demodPDUCRC);
                        reset(dewhiten);
    
                        % 헤더와 본문 합체
                        headerPDUCRC = [double(dewhitenedBits); dewhitenedPDUCRC];
                        
                        % CRC 에러 검사 수행
                        [dfPDU, crcError] = crcDet(headerPDUCRC);
                        if crcError, break; end % CRC 오류 시 루프 탈출
                        
                        % 파라미터 업데이트
                        moduleStartInd = samplesPerModule - samplesPerFrame + moduleStartInd - sps;
                        samplesPerModule = cteTime * 8 * sps * phyFactor;
                        samplingFlag = 1; pduCRCFlag = 0;
                        rcvSigPDUVar = rxNoisyWaveform;
                    end
                end
            end
            
            % [4] CTE IQ 샘플을 바탕으로 AoA 계산
            refSampleLength = 8;                                            
            minIQSamples = refSampleLength + getNumElements(cfg) - 1; % 안테나 별 최소 1회 이상의 샘플링 데이터
            
            if ~crcError && length(nonzeros(iqSamples)) >= minIQSamples     
                % 고윳값 분산 비율을 통한 신뢰도 가중치 계산 (Eigenvalue Spread)
                if enableWeighted
                    numAnts = prod(arraySize);                                          % 안테나 개수에 맞춰 공간 스냅샷 행렬 생성
                    X = reshape(iqSamples, numAnts, []);
                    Rxx = (X * X') / size(X, 2);                                        % 공간 공분산 행렬 계산
                    eigVals = sort(real(eig(Rxx)), 'descend');                          % 고윳값 추출 및 내림차순 정렬
                    confidenceWeights(i) = eigVals(1) / (mean(eigVals(2:end)) + eps);   % 주신호 전력 / 멀티패스(노이즈) 전력 평균 비율 계산
                end

                % MUSIC 알고리즘 : 로컬 좌표계 기준의 도래각(AoA) 추정
                angleEstLocal = musicAngleEstimation(iqSamples, cfg, 1);
    
                % [좌표 변환] 추정된 로컬 각도를 글로벌 좌표계로 다시 변환 (정방향 회전)
                [lx, ly, lz] = sph2cart(deg2rad(angleEstLocal(1)), deg2rad(angleEstLocal(2)), 1);
                dirVecGlobalEst = rotatepoint(locatorQuat(locIdx), [lx, ly, lz]);
                
                % 최종 글로벌 방위각 및 고도각 저장
                [gaz, gel, ~] = cart2sph(dirVecGlobalEst(1), dirVecGlobalEst(2), dirVecGlobalEst(3));
                angleEstGlobal(i, :) = [rad2deg(gaz), rad2deg(gel)];
                
                % RSSI를 기반으로 경로 손실 역산
                rangeConfig.ReceivedSignalPower = 10*log10(var([rcvFreqOffsetFree; rcvSigHeaderVar; rcvSigPDUVar])) + 30;
                pathLossdB(i) = pathLoss(rangeConfig);
            else
                linkFailFlag(i) = 1; % 통신 실패 표시
            end
        end
        
        % [5] 삼각측량 알고리즘 수행
        validIdx = find(~linkFailFlag);
        validLocatorsCount = length(validIdx);
        
        % 최소 2개 이상의 로케이터로부터 유효한 각도를 얻었을때 위치 계산
        if validLocatorsCount >= 2 && ~all(angleEstGlobal(:,1) == angleEstGlobal(1,1))
            % 선형 삼각측량법으로 기초 위치 계산
            posLin = linearTriangulation(posActiveLocators(:, validIdx), angleEstGlobal(validIdx, :), confidenceWeights(validIdx));
    
            if strcmp(estimationMethod, "Non-linear")
                % 비선형 정밀화(가우스-뉴턴 등)를 통해 좌표 보정
                posNodeEst(:, inumNode) = nonLinearTriangulation(posLin, posActiveLocators(:, validIdx), angleEstGlobal(validIdx, :), confidenceWeights(validIdx));
            else
                % 선형 측정 결과 그대로 사용
                posNodeEst(:, inumNode) = posLin;
            end
        else
            % 추정 불가능한 경우 NaN 처리 : RMSE 계산에서 배제
            posNodeEst(:, inumNode) = NaN(numDimensions, 1);
        end
    end
end