% =========================================================================
% 1. 파라미터 초기화 및 설정 (Parameters Setup)
% =========================================================================
% 시뮬레이션 공간 설정 (가로 x 세로 x 높이)
roomSize = [10, 8, 3];

% 로케이터(앵커) 위치 초기 파라미터 (예: 4개의 앵커)
% locatorPos = [5, 0, 1.5; 0, 4, 1.5; 5, 8, 1.5; 10, 4, 1.5]'; 
% locatorPos = [5, 0, 1.5; 2, 0, 1.5; 6, 0, 1.5; 10, 0, 1.5]'; 
% locatorPos = [0, 0, 1.5; 0, 8, 1.5; 10, 8, 1.5; 10, 0, 1.5]';
locatorPos = [5, 0, 1.5; 0, 4, 1.5; 5, 8, 1.5; 5, 4, 3]';

% 로케이터 방향 초기 파라미터 (단위 벡터)
% nor: Elevation 0도 평면의 수선 벡터 / azi: Azimuth 90도 벡터
% locator_ori_nor = [0, 1, 0; 1, 0, 0; 0, -1, 0; -1, 0, 0]; 
% locator_ori_azi = [1, 0, 0; 0, -1, 0; -1, 0, 0; 0, 1, 0];
% locator_ori_nor = [0, 1, 0; 0, 1, 0; 0, 1, 0; 0, 1, 0]; 
% locator_ori_azi = [1, 0, 0; 0, 1, 0; 0, 1, 0; 0, 1, 0];
% locator_ori_nor = [1, 1, 0; 1, -1, 0; -1, -1, 0; -1, 1, 0]; 
% locator_ori_azi = [1, -1, 0; -1, -1, 0; -1, 1, 0; 1, 1, 0];
locator_ori_nor = [0, 1, 0; 1, 0, 0; 0, -1, 0; 0, 0, -1]; 
locator_ori_azi = [1, 0, 0; 0, -1, 0; -1, 0, 0; 1, 0 , 0];

% 이동 노드 궤적 초기 파라미터 및 보간 설정
nodeWaypoints = [2, 6, 1.5; 2, 2, 1.5; 5, 2, 1.5; 5, 6, 1.5; 8, 6, 1.5; 8, 2, 1.5];
interpInterval = 0.5; % 경로 보간 간격 (m)

% 삼각측량 수행 방식
estimationMethod = "Linear";        % Linear(선형), Non-Linear(비선형)

% 3차원 / AoA 고정 파라미터
numDimensions = 3;                      % 무조건 3차원
dfPacketType = "ConnectionCTE";         % 방향 탐지용 패킷 유형
phyMode = "LE1M";                       % PHY 전송 모드
arraySize = [4 4];                      % 3차원 추정을 위한 16소자 평면 배열(URA)
elementSpacing = [0.5 0.5];             % 안테나 소자 간 간격 (파장 대비 비율)
switchingPattern = 1:prod(arraySize);   % 안테나 스위칭 패턴
slotDuration = 2;                       % 슬롯 지속 시간 (마이크로초)
sps = 4;                                % 심볼당 샘플 수
channelIndex = 17;                      % 데이터 채널 인덱스
crcInit = '555551';                     % CRC 초기화 값
accessAddress = '01234567';             % 액세스 주소
payloadLength = 1;                      % 페이로드 길이
cteLength = 160;                        % CTE 길이
sampleOffset = 2;                       % 샘플 오프셋
EbNo = 15;                              % 비트 에너지 대비 잡음 밀도 (dB)
environment = "Outdoor";                % 통신 환경
txPower = 0;                            % BLE 송신 전력 (dBm)

% Bluetooth 경로 손실 설정
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
rotMatArray = zeros(3, 3, numLocators); % 3x3xN 회전 행렬 배열 할당

% 로케이터별 로컬 방향(설치각도)을 글로벌 좌표계로 변환하기 위한 회전 행렬 계산
for i = 1:numLocators
    x_vec = locator_ori_nor(i, :);          % Normal 벡터를 x축으로 매핑
    y_vec = locator_ori_azi(i, :);          % Azimuth 90도 벡터를 y축으로 매핑
    z_vec = cross(x_vec, y_vec);            % 외적을 통한 z축 계산
    rotMatArray(:, :, i) = [x_vec', y_vec', z_vec']; % 회전 행렬 적재
end

% 3x3xN 회전 행렬을 한 번에 쿼터니언 배열 객체로 변환 (에러 해결 부분)
locatorQuat = quaternion(rotMatArray, 'rotmat', 'point');

% 노드 궤적 선형 보간 계산
dists = sqrt(sum(diff(nodeWaypoints).^2, 2));
cumDists = [0; cumsum(dists)];
queryPoints = 0:interpInterval:cumDists(end);
posNode = interp1(cumDists, nodeWaypoints, queryPoints, 'linear')'; % 3 x N 행렬
numNodePositions = size(posNode, 2);

% =========================================================================
% 3. 수신기 및 BLE 객체 구성
% =========================================================================
% BLE 각도 추정용 설정 객체(앵커) 생성
cfg = bleAngleEstimateConfig("ArraySize", arraySize, ...
    "SlotDuration", slotDuration, ...
    "SwitchingPattern", switchingPattern, ...
    "ElementSpacing", elementSpacing);
validateConfig(cfg);
pos = getElementPosition(cfg);

cteType = [0;0]; % AoA는 기본적으로 연속 톤 송신

accessAddBitsLen = 32;
accessAddBits = int2bit(hex2dec(accessAddress), accessAddBitsLen, false);
refSamples = helperBLEReferenceWaveform(phyMode, accessAddBits, sps);
prbDet = comm.PreambleDetector(refSamples, "Detections", "First");

phyFactor = 1 + strcmp(phyMode, "LE2M");
sampleRate = 1e6 * phyFactor * sps;
coarsesync = comm.CoarseFrequencyCompensator("Modulation", "OQPSK", ...
    "SampleRate", sampleRate, "SamplesPerSymbol", 2*sps, "FrequencyResolution", 100);

crcLen = 24;
crcDet = comm.CRCDetector("x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1", "DirectMethod", true, ...
    "InitialConditions", int2bit(hex2dec(crcInit), crcLen).');

rng('default');
snr = EbNo - 10*log10(sps);
headerLen = 16 + 8 * strcmp(dfPacketType, "ConnectionCTE");
preambleLen = 8 * phyFactor;

dewhitenStateLen = 6;
chanIdxBin = int2bit(channelIndex, dewhitenStateLen).';
initState = [1 chanIdxBin];
dewhiten = bluetoothWhiten(InitialConditions=initState');

posNodeEst = zeros(numDimensions, numNodePositions);
posLocatorBuffer = cell(1, numNodePositions);
S = RandStream('mt19937ar', 'Seed', 5489);
pfo = comm.PhaseFrequencyOffset(SampleRate=sampleRate);
phaseNoise = comm.PhaseNoise(Level=[-130 -136], FrequencyOffset=[1e4 1e5], SampleRate=sampleRate);

% =========================================================================
% 4. 메인 루프: 노드 이동 시뮬레이션
% =========================================================================
for inumNode = 1:numNodePositions
    % 거리 계산 및 활성 로케이터(80m 이내) 필터링
    distanceAll = vecnorm(locatorPos - posNode(:, inumNode));
    activeIdx = distanceAll <= 80;
    
    posActiveLocators = locatorPos(:, activeIdx);
    distanceActive = distanceAll(activeIdx)';
    activeLocIndices = find(activeIdx);
    numActiveLocators = sum(activeIdx);
    
    posLocatorBuffer{1, inumNode} = posActiveLocators;
    plLin = helperBluetoothEstimatePathLoss(rangeConfig.Environment, distanceActive);
    
    angleEst = zeros(numActiveLocators, numDimensions-1);
    [pathLossdB, linkFailFlag] = deal(zeros(numActiveLocators, 1));
    angleActiveLocal = zeros(numActiveLocators, numDimensions-1);
    
    % 각 활성 로케이터에 대해 반복
    for i = 1:numActiveLocators
        locIdx = activeLocIndices(i);
        
        % 방향 벡터 (Global -> Local 쿼터니언 역회전 변환)
        dirVecGlobal = (posNode(:, inumNode) - posActiveLocators(:, i))'; % 1x3 행벡터
        dirVecLocal = rotatepoint(conj(locatorQuat(locIdx)), dirVecGlobal);
        [az_true, el_true, ~] = cart2sph(dirVecLocal(1), dirVecLocal(2), dirVecLocal(3));
        angleActiveLocal(i, :) = [rad2deg(az_true), rad2deg(el_true)];
        
        % 송신기(노드) 시뮬레이션
        data = helperBLEGenerateDFPDU(dfPacketType, cteLength, cteType, payloadLength, crcInit);
        bleWaveform = bleWaveformGenerator(data, "Mode", phyMode, "SamplesPerSymbol", sps, ...
            "ChannelIndex", channelIndex, "DFPacketType", dfPacketType, "AccessAddress", accessAddBits);
        
        % 송신 채널 왜곡
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
        
        % 송신 전력 및 경로 손실 적용
        dBmConverter = 30;
        txImpairedWaveform = 10^((rangeConfig.TransmitterPower-dBmConverter)/20)*noisyWaveform/plLin(i);
        
        % AoA 방식 수신단 안테나 조향(Steering)
        steerVec = helperBLESteeringVector(angleActiveLocal(i,:), pos);
        steeredWaveform = txImpairedWaveform .* steerVec.';
        
        % --- 수신기 처리 (Receiver Processing) ---
        packetDetect = 0; headerFlag = 1; pduCRCFlag = 0; samplingFlag = 0; crcError = 1;
        samplesPerFrame = 8*sps; samplesPerModule = samplesPerFrame+sps;
        numTimes = ceil(length(timingOffWaveform)/samplesPerFrame)+1;
        countpacketDetect = 0; moduleStartInd = 0;
        
        rxWaveformBuffer = [steeredWaveform; zeros(samplesPerFrame*numTimes-length(steeredWaveform)+samplesPerFrame, size(steeredWaveform,2))];
        rcvSigBuffer = [];
        
        for j = 1:numTimes
            if (j-1)*samplesPerFrame + moduleStartInd + samplesPerModule <= length(rxWaveformBuffer)
                rxChunkWaveform = rxWaveformBuffer((j-1)*samplesPerFrame+moduleStartInd+(1:samplesPerModule), :);
            else
                rxChunkWaveform = rxWaveformBuffer((j-1)*samplesPerFrame+moduleStartInd+1:end, :);
            end
            
            if ~samplingFlag
                rxNoisyWaveform = awgn(rxChunkWaveform(:,1), snr, "measured"); 
            else
                % AoA 방식의 수신기 안테나 스위칭
                rxSwitchWaveform = helperBLESwitchAntenna(rxChunkWaveform, phyMode, sps, slotDuration, switchingPattern);
                rxNoisyWaveform = awgn(rxSwitchWaveform, snr, "measured"); 
        
                cteSamples = rxNoisyWaveform(1:cteTime*8*sps*phyFactor);
                iqSamples = bleCTEIQSample(cteSamples, "Mode", phyMode, ...
                    "SamplesPerSymbol", sps, "SlotDuration", slotDuration, "SampleOffset", sampleOffset);
                samplingFlag = 0;
                break;
            end
            
            if packetDetect == 0
                countpacketDetect = countpacketDetect + 1;
                rcvSigBuffer((countpacketDetect-1)*(samplesPerModule-sps)+(1:samplesPerModule)) = rxNoisyWaveform;
                
                if countpacketDetect >= (preambleLen+accessAddBitsLen+headerLen)*sps/samplesPerFrame
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
                    
                    estimatedDCOffset = mean(rcvTrim(1:length(refSamples))) - mean(refSamples)*sqrt(var(rcvTrim));
                    rcvDCFree = rcvTrim - estimatedDCOffset;
                    
                    [~, freqoff] = coarsesync(rcvDCFree);
                    release(coarsesync);
                    [rcvFreqOffsetFree, iniFreqState] = helperBLEFrequencyOffset(rcvDCFree, sampleRate, -freqoff);
                    
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
                            packetDetect = 1;
                            samplesPerModule = headerLen*sps - length(decodeData)*sps + remsad + sps;
                            rcvSigBuffer = [];
                        end
                    end
                end
            end
            
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
                    if crcError, break; end
                    
                    moduleStartInd = samplesPerModule - samplesPerFrame + moduleStartInd - sps;
                    samplesPerModule = cteTime * 8 * sps * phyFactor;
                    samplingFlag = 1; pduCRCFlag = 0;
                    rcvSigPDUVar = rxNoisyWaveform;
                end
            end
        end
        
        % --- 각도 추정 (Angle Estimation) ---
        refSampleLength = 8;                                            
        minIQSamples = refSampleLength + getNumElements(cfg) - 1;
        
        if ~crcError && length(nonzeros(iqSamples)) >= minIQSamples     
            % 수신기 안테나가 측정하는 방향은 '로컬 각도'
            angleEstLocal = bleAngleEstimate(iqSamples, cfg);
            
            % 위치 추정을 위해 로컬 각도를 다시 글로벌 각도로 변환
            [lx, ly, lz] = sph2cart(deg2rad(angleEstLocal(1)), deg2rad(angleEstLocal(2)), 1);
            
            % 글로벌 각도 복구 (정방향 회전)
            dirVecGlobalEst = rotatepoint(locatorQuat(locIdx), [lx, ly, lz]);
            
            [gaz, gel, ~] = cart2sph(dirVecGlobalEst(1), dirVecGlobalEst(2), dirVecGlobalEst(3));
            angleEst(i, :) = [rad2deg(gaz), rad2deg(gel)];

            rangeConfig.ReceivedSignalPower = 10*log10(var([rcvFreqOffsetFree; rcvSigHeaderVar; rcvSigPDUVar])) + 30;
            pathLossdB(i) = pathLoss(rangeConfig);
        else
            linkFailFlag(i) = 1;
        end
    end
    
    % 수신 실패 로케이터 처리
    if any(linkFailFlag == 1)
        idx = find(linkFailFlag == 1);
        posActiveLocators(:,idx) = [];
        angleEst(idx,:) = [];
        pathLossdB(idx) = [];
    end
    
    % --- 최종 위치 추정 ---
    validIdx = find(~linkFailFlag);
    numValid = length(validIdx);
    
    validLocatorsCount = numActiveLocators - nnz(linkFailFlag);
    if validLocatorsCount >= 2 && ~all(angleEst(:,1) == angleEst(1,1))
        % 선형 삼각측량 (Linear Estimation).
        posLin = blePositionEstimate(posActiveLocators(:, validIdx), "angulation", angleEst(validIdx, :).');

        if strcmp(estimationMethod, "Non-linear")
            % 비선형 정밀화 (Non-linear Refinement)
            posNodeEst(:, inumNode) = nonLinearTriangulation(...
                posActiveLocators(:, validIdx), ...
                angleEst(validIdx, :), ...
                posLin, ...
                roomSize);
        else
            % 선형 측정 방식만 사용 시
            posNodeEst(:, inumNode) = posLin;
        end
    else
        % 추정 불가 시 NaN 처리
        posNodeEst(:, inumNode) = NaN(numDimensions, 1);
    end
end

% =========================================================================
% 5. 결과 출력 및 시각화
% =========================================================================
posErr = sqrt(sum((posNodeEst - posNode).^2)); 
disp(["Positioning error (RMSE) in meters = ", num2str(mean(posErr, 'omitnan'))])

if ~all(isnan(posNodeEst(1,:)))
    helperBLEVisualizeNodeTracking(locatorPos, posNode, posLocatorBuffer, posNodeEst)
    
    % --- 추가할 부분: 시각화 범위 고정 ---
    % 우리가 선언한 roomSize = [10, 6, 3]를 활용합니다.
    axis([0 roomSize(1) 0 roomSize(2) 0 roomSize(3)]); 
    
    % 시점을 보기 좋게 조정 (필요시)
    view(45, 30); 
    title('BLE 3D Tracking Simulation Space');
end