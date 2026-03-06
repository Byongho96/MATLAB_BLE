% =========================================================================
% 1. 파라미터 초기화 및 설정 (Parameters Setup)
% =========================================================================
EbNo = 2:4:24;                                  % 비트 에너지 대 잡음 밀도 비 (dB 단위)
sps = 4;                                        % 심볼당 샘플 수 (1보다 커야 함)
dataLength = 128;                               % 데이터 길이 27~255 (바이트 단위, 헤더/페이로드/CRC 포함)
simMode = ["LE1M","LE2M","LE500K","LE125K"];    % 시뮬레이션할 BLE 물리 계층(PHY) 모드들
channelModel = "Raytracing Channel";            % 채널 모델 설정 AWGN(단순잡음) | Rayleigh, Rician(일반적페이딩) | Raytracing Channel(도면기반 모델)
enableEqualizer = false;                        % 수신기 이퀄라이저 활성화 여부
enableAGC = true;                               % 자동 이득 제어(AGC) 활성화 여부
maxNumErrors = 10;                              % 시뮬레이션을 중단할 최대 에러 비트 수
maxNumPackets = 100;                            % 시뮬레이션할 최대 패킷 수
numMode = numel(simMode);                       % PHY 모드 개수
snrLength = length(EbNo);                       % 테스트할 Eb/No 지점 개수
[ber,per] = deal(zeros(numMode,snrLength));     % BER(비트 에러율) 및 PER(패킷 에러율) 저장 배열 초기화

if channelModel=="Raytracing Channel"
    visualVar = cell(numMode,snrLength);        % 레이트레이싱 시각화 데이터를 저장할 셀 배열
end
bitsPerByte = 8;    % 상수값 정의 

% =========================================================================
% 2. PHY 모드별 반복 루프 (Main Loop)
% =========================================================================
for countMode = 1:numMode
    phyMode = simMode(countMode);               % 현재 시뮬레이션 중인 PHY 모드
    
    % PHY 모드에 따른 SNR(신호 대 잡음비) 계산
    if any(phyMode==["LE1M","LE2M"])
        snrVec = EbNo - 10*log10(sps);          % 코딩되지 않은 모드 (LE1M, LE2M)
    else
        % codeRate가 작아질수록 에러 정정형 부호가 길어져 데이터 전송률이 떨어짐
        if phyMode == "LE500K"
            codeRate = 1/2;                     % LE500K 코딩율
        else
            codeRate = 1/8;                     % LE125K 코딩율
        end
        snrVec = EbNo + 10*log10(codeRate) - 10*log10(sps); % 코딩된 모드의 SNR 계산
    end
    
    % PHY 모드에 따른 샘플링 레이트 설정 (LE2M은 2Msps, 나머지는 1Msps 기반)
    % samples per bit = sps / codeRate * symbolRate
    sampleRate = sps*(1+(phyMode=="LE2M"))*1e6;
    
    % =========================================================================
    % 3. Eb/No(SNR) 지점별 반복 루프
    % =========================================================================
    for countSnr = 1:snrLength
        % 난수 생성을 위한 스트림 설정 (결과 재현성 확보)
        stream = RandStream("combRecursive");
        stream.Substream = countSnr;
        RandStream.setGlobalStream(stream);
        
        % 에러율 계산 객체 생성 (사용자 정의 샘플 범위 지정)
        % 전송한 비트와 수신한 비트를 비교해 BER을 계산 (실제 데이터 부분의 비트만 비교)
        errorRate = comm.ErrorRate(Samples="Custom", ...
            CustomSamples=1:(dataLength*bitsPerByte-1));
            
        % BLE 신호 왜곡(Impairments) 및 채널 초기화
        % 실제 무선 칩셋 하드웨어에서 발생하는 주파수 오프셋, 위상 잡음, 타이밍 드리프트 등을 자도 생성
        initImp = helperBLEImpairmentsInit(phyMode,sps);
        channelInit = cell(1,1);
        if channelModel~="AWGN"
            % 레이트레이싱 같은 복잡한 채널 모델을 물리적인 샘플링 레이트
            channelInit = helperBluetoothChannelInit(sampleRate,channelModel);
        end
        
        % 수신기 설정 (모드, 주파수 보상기, 프리앰블 검출기 등)
        rxCfg = struct(Mode=phyMode,SamplesPerSymbol=sps, ...
            DFPacketType="Disabled");
        rxCfg.CoarseFreqCompensator = comm.CoarseFrequencyCompensator(Modulation="OQPSK", ...
            SampleRate=sampleRate, ...
            SamplesPerSymbol=2*sps, ...
            FrequencyResolution=30);
        rxCfg.PreambleDetector = comm.PreambleDetector(Detections="First");
        
        % 카운터 초기화 (누적 비트 에러, 누적 패킷 에러, 현재 패킷 번호)
        [numErrors,perCount,numPacket] = deal(0,0,1);
        
        % =========================================================================
        % 4. 패킷 전송 반복 루프 (While Loop)
        % =========================================================================
        while numErrors <= maxNumErrors && numPacket <= maxNumPackets
            % 전송할 랜덤 데이터 비트 생성
            txBits = randi([0 1],dataLength*bitsPerByte,1,"int8");
            
            % 0~39 사이의 랜덤 채널 인덱스 선택 (블루투스 주파수 호핑 모사)
            channelIndex = randi([0 39],1,1);
            if channelIndex <= 36
                % 데이터 채널용 액세스 주소 (표준 규격 준수)
                accessAddress = [1 0 0 0 1 1 1 0 1 1 0 0 1  ...
                    0 0 1 1 0 1 1 1 1 1 0 1 1 0 1 0 1 1 0]';
            else
                % 광고 채널용 액세스 주소
                accessAddress = [0 1 1 0 1 0 1 1 0 1 1 1 1 1 0 1 1 0 0  ...
                    1 0 0 0 1 0 1 1 1 0 0 0 1]';
            end
            
            % 1) BLE 송신 파형 생성
            txWaveform = bleWaveformGenerator(txBits,Mode=phyMode, ...
                SamplesPerSymbol=sps, ...
                ChannelIndex=channelIndex, ...
                AccessAddress=accessAddress);
            
            % 2) 신호 왜곡 추가 (주파수/위상 오프셋, 타이밍 드리프트, DC 오프셋)
            initImp.pfo.FrequencyOffset = randsrc(1,1,-50e3:10:50e3); 
            initImp.pfo.PhaseOffset = randsrc(1,1,-10:5:10);
            initoff = 0.15*sps; 
            stepsize = 20*1e-6; 
            initImp.vdelay = (initoff:stepsize:initoff+stepsize*(length(txWaveform)-1))'; 
            initImp.dc = 20;
            txImpairedWfm = helperBLEImpairmentsAddition(txWaveform,initImp);
            
            % 3) 채널 통과 (Multipath Fading 적용)
            if channelModel=="AWGN"
                txChanWfm = txImpairedWfm;
            else
                % 페이딩 채널 필터 지연 보정 및 신호 통과
                chanDelay = info(channelInit.fadingChan).ChannelFilterDelay;
                txChanWfm = channelInit.fadingChan([txImpairedWfm; zeros(chanDelay,1)]);
                txChanWfm = txChanWfm(chanDelay+1:end,1);
                
                % 레이트레이싱 결과 저장
                if channelModel=="Raytracing Channel"
                    visualVar{countMode,countSnr} = channelInit.VisualVar;
                end
            end
            
            % 4) 가우시안 열잡음(AWGN) 추가
            rxWaveform = awgn(txChanWfm,snrVec(countSnr),"measured");
            
            % 5) 수신 처리 (복조 및 비트 복원)
            rxCfg.ChannelIndex = channelIndex;
            rxCfg.AccessAddress = accessAddress;
            rxCfg.EqualizerFlag = enableEqualizer;
            rxCfg.AGCFlag = enableAGC;
            [rxBits,recAccessAddress] = helperBLEPracticalReceiver(rxWaveform,rxCfg);
            
            % 6) 에러 통계 계산
            if(length(txBits) == length(rxBits))
                errors = errorRate(txBits,rxBits);      % 비트 에러 누적
                ber(countMode,countSnr) = errors(1);    
                currentErrors = errors(2)-numErrors;    % 현재 패킷의 에러 비트 수
                if(currentErrors)                       % 에러가 있으면 PER 카운트 증가
                    perCount = perCount + 1;            
                end
                numErrors = errors(2);                  
            else
                perCount = perCount + 1;                % 길이 불일치 시 패킷 에러 처리
            end
            numPacket = numPacket + 1;
            
            if channelModel~="AWGN"
                reset(channelInit.fadingChan);          % 다음 패킷을 위해 채널 상태 리셋
            end
        end
        
        % 결과 출력
        per(countMode,countSnr) = perCount/(numPacket-1);
        disp("Mode "+phyMode+","+ ...
            " simulating for "+channelModel+" model,"+ ...
            " Eb/No = "+num2str(EbNo(countSnr))+"dB,"+ ...
            " data length = "+num2str(dataLength)+"bytes,"+ ...
            " BER: "+num2str(ber(countMode,countSnr))+","+ ...
            " PER: "+num2str(per(countMode,countSnr)));
    end
end

% =========================================================================
% 5. 성능 결과 시각화 (BER/PER 그래프)
% =========================================================================
figure('Name', 'Communication Performance') % 그래프 전용 창 생성
marker = "ox*s";
color = "bmgr";
legendVar = strings(numMode,1);
for countMode = 1:numMode
    % BER 그래프
    subplot(2,1,1),semilogy(EbNo,ber(countMode,:).',"-"+marker{1}(countMode)+color{1}(countMode));
    hold on;
    % PER 그래프
    subplot(2,1,2),semilogy(EbNo,per(countMode,:).',"-"+marker{1}(countMode)+color{1}(countMode));
    hold on;
    legendVar(countMode) = simMode(countMode);
end

% 그래프 레이아웃 설정
subplot(2,1,1), grid on; xlabel("Eb/No (dB)"); ylabel("BER"); legend(legendVar);
title(join(["BER of Bluetooth LE communication in ",channelModel],""));
subplot(2,1,2), grid on; xlabel("Eb/No (dB)"); ylabel("PER"); legend(legendVar);
title(join(["PER of Bluetooth LE communication in ",channelModel],""));

% =========================================================================
% 6. 레이트레이싱 3D 장면 시각화 (설정 시에만 실행)
% =========================================================================
if (channelModel=="Raytracing Channel")
    visualVar = visualVar{end,end};
    % 회의실(Conference Room) 맵 표시
    viewer = siteviewer(SceneModel=visualVar.MapFileName);
    % 송신 및 수신 위치에 아이콘 표시
    show(visualVar.TxSite,Icon="bleTxIcon.png");
    show(visualVar.RxSite,Icon="bleRxIcon.png");
    % 전파 경로(Rays) 시각화 (수신 전도 기반 색상 표시)
    plot(visualVar.Rays,Type="power",ColorLimits=[-30 0]);
end

% 그래프 창을 맨 앞으로 가져오기
shg;