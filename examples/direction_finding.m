% =========================================================================
% 1. 파라미터 초기화 및 설정 (Parameters Setup)
% =========================================================================
numDimensions = 3;       % BLE 기기 위치의 차원 (2D 또는 3D)
numLocators = 3;         % 로케이터(수신기)의 개수 (삼각측량을 위해 최소 2개 이상 필요)
EbNo = 6:2:16;           % 에너지 대 잡음 전력 밀도 비 (단위: dB). SNR을 정규화한 값. 6부터 16까지 2간격으로 신호세기를 늘리며 오차변화 측정.
numIterations = 5;      % 위치 오차 평균을 내기 위한 반복(Iteration) 횟수
dfMethod = "AoA";        % 방향 탐지 방식: "AoA" or "DoA"
dfPacketType = "ConnectionCTE"; % 방향 탐지 패킷 유형: "ConnectionCTE" or "ConnectionlessCTE". 각도 추정 정확도에는 무관.
phyMode = "LE1M";        % PHY 전송 모드 "LE1M" or "LE2M". LE2M의 경우 대역폭이 넓고 에너지밀도가 낮아 잡음에 취약
arraySize = [3, 3];      % 안테나 배열 크기. 3D 위치 추정이므로 URA(Uniform Rectangular Array) 형태인 1x2 벡터 사용
elementSpacing = 0.5;    % 안테나 소자 간의 간격 (파장 대비 정규화된 값, 보통 0.5 파장 사용). 가로 세로 행렬로 표현 가능.
switchingPattern = 1:prod(arraySize); % 안테나 스위칭 패턴 (1부터 9까지 순차적으로 스위칭)
slotDuration = 2;        % 안테나 스위칭 슬롯의 지속 시간 (마이크로초). "1" or "2"
cteLength = 160;         % CTE의 길이 (마이크로초). 16~160 범위.
sps = 8;                 % 심볼 당 샘플 수 (Samples per symbol) : 샘플링속도 = 샘플링주파수(phyMode) X sps 
channelIndex = 17;       % 사용할 블루투스 채널 인덱스
crcInit = '555551';      % CRC 초기화 값 (16진수 형태)
accessAddress = '01234567'; % 16진수 접속 주소 (Access address)
payloadLength = 1;       % 페이로드 길이 (바이트 단위). ConnnectionCTE는 1바이트, Connectionless의 경우 송신자 주소를 담아야해서 3바이트 이상 
minLocators = 2;         % 위치 추정(삼각측량)을 위한 최소 로케이터 개수

% =========================================================================
% 2. 설정 값 유효성 검사 (Validation)
% =========================================================================
% 차원(2D/3D)에 맞게 안테나 배열 크기가 올바르게 설정되었는지 확인
if numDimensions == 2 && size(arraySize,2) ~= 1
    error('2D 위치 추정을 위해서는 arraySize가 스칼라(1D 안테나 배열)여야 합니다.');
end
if numDimensions == 3 && size(arraySize,2) ~= 2
    error('3D 위치 추정을 위해서는 arraySize가 1x2 벡터여야 합니다.');
end
if numLocators < minLocators
    error(['로케이터의 수(numLocators)는 최소 ' num2str(minLocators) ' 이상이어야 합니다.']);
end

% 패킷 유형에 따른 페이로드 길이 검사
if strcmp(dfPacketType,'ConnectionCTE') && payloadLength ~= 1
    error('ConnectionCTE 패킷의 페이로드 길이는 1바이트여야 합니다.');
elseif strcmp(dfPacketType,'ConnectionlessCTE') && payloadLength < 3
    error('ConnectionlessCTE 패킷의 페이로드 길이는 3바이트 이상이어야 합니다.');
end

% 3D 위치 추정 시 안테나 간격 설정 보정 (x축, y축 간격을 동일하게 설정)
if numDimensions == 3 && isscalar(elementSpacing)
    elementSpacing = [elementSpacing elementSpacing]; 
end

% BLE 각도 추정용 설정 객체(앵커) 생성 및 규격 검증
cfg = bleAngleEstimateConfig('ArraySize',arraySize,'SlotDuration',slotDuration,'SwitchingPattern', ...
                                    switchingPattern,'ElementSpacing',elementSpacing);
validateConfig(cfg);

% CTE 타입 설정 (AoA/AoD 및 슬롯 지속 시간에 따라 결정)
% 패킷 생성에 쓰이는 값으로. AoA는 기본 CTE를 생성하지만, AoD는 안테나를 스위칭하며 생성. 
if strcmp(dfMethod,'AoA')
    cteType = [0;0];
else
    cteType = [0;1];
    if slotDuration == 1
        cteType = [1;0];
    end
end

% 접속 주소를 16진수에서 32비트 이진 배열로 변환
accessAddBits = int2bit(hex2dec(accessAddress),32,false);

% =========================================================================
% 3. 시뮬레이션 변수 초기화. 메모리 및 실행 최적화.
% =========================================================================
numEbNo = numel(EbNo); % 테스트할 Eb/No 포인트 개수
posNode = zeros(numDimensions,numEbNo);
posLocator = zeros(numDimensions,numLocators,numEbNo);
angleEst = zeros(numLocators,numDimensions-1,numEbNo);
posNodeEst = zeros(numDimensions,numEbNo);
validResult = zeros(1,numEbNo);
avgPositionError = zeros(1,numEbNo);

% =========================================================================
% 4. 메인 시뮬레이션 루프 (Eb/No 값에 따른 반복)
% =========================================================================
% parfor iEbNo = 1:numEbNo % 병렬 처리를 원하면 parfor 사용
for iEbNo = 1:numEbNo % 디버깅을 위해 일반 for문 사용
    
    % 재현성을 위해 랜덤 시드 고정 (각 Eb/No마다 동일한 난수 스트림 사용)
    stream = RandStream('combRecursive','Seed',12345);
    stream.Substream = iEbNo; % 2. for문 별로 substream을 고정하여 충돌 방지.
    RandStream.setGlobalStream(stream); % 3. 시트림 지정. MATLAB의 모든 랜덤 함수(randn, rand 등)에 반영.
    
    posErr = zeros(1,numIterations); % 각 반복(Iteration)에서의 오차를 저장할 배열
    iterationFailCount = 0;          % 실패한 이터레이션 횟수 카운트 (패킷 검출 실패, 가용 로케이터 부족 등)
    
    % 이터레이션 반복 루프
    for iterCount = 1:numIterations
        
        % 노드와 로케이터의 무작위 위치 및 실제 도달 각도 생성
        % tempPosNode: 타겟 노드의 [x; y; z] 좌표
        % tempPosLocator : 로케이터의 위치를 담은 행렬 (3차원일 경우 3 X numLocators 크기)
        % ang : 각 로케이터에서의 실제 정답 각도
        [tempPosNode,tempPosLocator,ang] = helperBLEGeneratePositions(numLocators,numDimensions);
        posNode(:,iEbNo) = tempPosNode;
        posLocator(:,:,iEbNo) = tempPosLocator;
        
        % 3차원의 경우 Azimuth, Elevation / 2차원의 경우 Azimuth
        tempAngleEst = zeros(numLocators,numDimensions-1);
        idx = []; % 실패한 로케이터 인덱스
        linkFailFlag = zeros(numLocators,1); % 1일 경우 통신 실패
        
        % 각 로케이터(수신기)에 대한 처리 루프
        for i=1:numLocators
            
            % 1) 방향 탐지 패킷(PDU) 생성
            % CTE를 포함한 블루투스 표준에 맞는 디지털 비트열 생성
            data = helperBLEGenerateDFPDU(dfPacketType,cteLength,cteType,payloadLength,crcInit);
            
            % 2) BLE 기본 파형 생성.
            % 블루투스 표준 변조 방식인 GFSK를 사용하여 아날로그 복소수(IQ) 파형 생성.
            bleWaveform = bleWaveformGenerator(data,'Mode',phyMode,'SamplesPerSymbol',sps,...
                'ChannelIndex',channelIndex,'DFPacketType',dfPacketType,'AccessAddress',accessAddBits);
            
            % 3) 안테나 조향(Steering) 및 스위칭 적용 (다중 안테나 효과 시뮬레이션)
            % 다중 안테나 스위칭을 바탕으로 하나의 긴 시퀀스 데이터 변형
            dfWaveform = helperBLESteerSwitchAntenna(bleWaveform,ang(i,:),...
                                        phyMode,sps,dfPacketType,payloadLength,cfg);
                                        
            % 4) AWGN (백색 가우시안 잡음) 반영
            snr = EbNo(iEbNo) - 10*log10(sps); % Eb/No를 SNR로 변환
            noiseWaveform = awgn(dfWaveform,snr,'measured'); % measured 옵션으로 측정 전력을 바탕으로 노이즈 추가
            
            % 5) 하드웨어 결함이 없는 이상적인 수신기를 통해 신호를 수신하고 IQ 샘플 추출
            % CTE 구간을 집중적으로 분석하여, 주파수 및 위상을 복구 후 IQ 샘플 추출
            [~, ~, iqSamples] = bleIdealReceiver(noiseWaveform,'Mode',phyMode,...
                'SamplesPerSymbol',sps,'ChannelIndex',channelIndex,'DFPacketType',...
                dfPacketType,'SlotDuration',slotDuration);
            
            % 6) IQ 샘플을 이용한 각도(AoA) 추정
            refSampleLength = 8; % 기준 샘플 길이. 블루투스 표준 규격. helperBLESwitchAntenna 내부 정의.
            minIQSamples = refSampleLength+getNumElements(cfg)-1; % 패킷 검출 판단 기준. 안테나 별 최소 한개 이상의 샘플.
            
            if length(nonzeros(iqSamples)) >= minIQSamples % 유효한 IQ 샘플이 충분한 경우
                % 안테나 별 위상차를 바탕으로 MLE 기반의 각도 추정(스캐닝)
                tempAngleEst(i,:) = bleAngleEstimate(iqSamples,cfg); % 각도 추정 수행
            else
                linkFailFlag(i) = 1; % 패킷 검출 실패 시 플래그 설정
                idx = [idx i]; % 실패한 로케이터 인덱스 저장
            end
        end
        
        % 삼각 측량을 통한 최종 노드 위치 추정
        % 조건: 성공한 링크 수가 최소 요구사항(minLocators) 이상이며, 추정된 각도가 모두 같지 않을 때
        if (numLocators-nnz(linkFailFlag)) >= minLocators && ~isequal(tempAngleEst(:,1),repmat(tempAngleEst(1,1),numLocators,1))
            
            posLocatorEbNo = posLocator(:,:,iEbNo); % 현재 E_b/N_o 루프에서 사용할 로케이터들의 [x; y; z] 좌표
            tempAngEstTri = tempAngleEst;
            
            % 실패한 링크가 있다면 계산에서 제외 처리 (NaN 할당)
            if any(linkFailFlag == 1)
                posLocatorEbNo(:,idx) = [];
                tempAngleEst(idx,:) = NaN(numel(idx),numDimensions-1);
                tempAngEstTri(idx,:) = [];
            end
            
            % 추정된 각도들을 바탕으로 삼각 측량(Angulation) 수행하여 노드 위치 도출
            % 선형 최소 제곱법(Linear Least Squares, LLS)
            posNodeEst(:,iEbNo) = blePositionEstimate(posLocatorEbNo,'angulation',tempAngEstTri.');
            angleEst(:,:,iEbNo) = tempAngleEst;
            
            % 실제 위치와 추정 위치 간의 유클리드 거리 오차 계산
            posErr(iterCount) = sqrt(sum((posNodeEst(:,iEbNo)-posNode(:,iEbNo)).^2));
        else
            iterationFailCount = iterationFailCount + 1; % 삼각 측량 실패 카운트 증가
        end
    end
    
    % 해당 Eb/No에서 결과 평가 및 출력
    if(iterationFailCount == numIterations) % 모든 이터레이션 실패 시
        disp(['At Eb/No = ',num2str(EbNo(iEbNo)),' dB, all direction finding packet transmissions failed'])
        validResult(iEbNo) = 0; % 플롯 대상에서 제외
    else
        % 성공한 이터레이션들의 오차 평균 계산
        avgPositionError(iEbNo) = sum(posErr)/(numIterations-iterationFailCount);
        disp(['At Eb/No = ',num2str(EbNo(iEbNo)),' dB, positioning error in meters = ', num2str(avgPositionError(iEbNo))]) 
        validResult(iEbNo) = 1; % 플롯 대상으로 포함
    end
end

% =========================================================================
% 5. 결과 시각화 (Visualization)
% =========================================================================
[~,validIdx] = find(validResult==1); % 성공한 결과의 인덱스 탐색
if ~isempty(validIdx)
    EbNoValid = EbNo(validIdx);
    [~,EbNoIdx] = max(EbNoValid);
    EbNoValidIdx = validIdx(EbNoIdx); % 가장 높은 Eb/No 결과를 시각화 대상으로 선택
    
    % 3D 환경에서의 로케이터와 노드의 위치 및 추정 방향을 플롯
    helperBLEVisualizePosition(posLocator(:,:,EbNoValidIdx),posNode(:,EbNoValidIdx),...
                        angleEst(:,:,EbNoValidIdx),posNodeEst(:,EbNoValidIdx));
                        
    % Eb/No에 따른 오차 그래프 플롯
    figure
    plot(EbNoValid,avgPositionError(validIdx),'-b*','LineWidth',2, ...
            'MarkerEdgeColor','b','MarkerSize',10)
    grid on
    xlabel('Eb/No (dB)')
    ylabel('Estimated Position Error (meters)')
    title('Position Accuracy in Bluetooth LE network')
end