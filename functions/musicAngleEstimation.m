function ang = musicAngleEstimation(iqSamples, cfg, resolution)
% musicAngleEstimation 
% 기존 MATLAB Bluetooth Toolbox의 bleAngleEstimate 및 ble.internal.musicEstimate 
% 로직을 그대로 가져오되, MUSIC 알고리즘의 각도 탐색 해상도(resolution)를 
% 외부 인자로 받아 연산 속도를 조절할 수 있도록 수정한 커스텀 함수입니다.

%#codegen
% Check the number of input arguments
narginchk(3,3)

% Validate the configuration object
validateattributes(cfg,{'bleAngleEstimateConfig'},{'scalar'},'bleAngleEstimate','cfg',2);
validateConfig(cfg);

% Validate the IQ samples
validateattributes(iqSamples,{'double','single'},{'nonnan','finite','column'}, mfilename,'IQSamples');
refSampleLength = 8;
coder.internal.errorIf(length(nonzeros(iqSamples))<(refSampleLength+getNumElements(cfg)-1),...
                        'bluetooth:bleAngleEstimate:InputLength',...
                        refSampleLength+getNumElements(cfg)-1,length(nonzeros(iqSamples)));

% Adjust switching pattern as per the Bluetooth Core Specification, version 5.1
expectedSPLen = length(iqSamples)-(refSampleLength-1); % Expected switching pattern length
spLength = length(cfg.SwitchingPattern); % Given switching pattern length
if expectedSPLen >= spLength
    repFactor = ceil(expectedSPLen/spLength);
    switchingPatternRep =  repmat(cfg.SwitchingPattern,1,repFactor);
    switchingPattern = switchingPatternRep(1:expectedSPLen);
else
    switchingPattern = cfg.SwitchingPattern(1:expectedSPLen);
end

% Derive phase difference for four microseconds duration (GFSK effect estimation)
phaseRef = unwrap(angle(iqSamples(1:refSampleLength)));
deltaFreq = diff(phaseRef)./(2*pi);
avgDeltaFreq = mean(abs(deltaFreq))*1e6;
if avgDeltaFreq < 375e3 % For LE1M
    phDiff = 1;
else % For LE2M
    phDiff = 2;
end

% Consider 1 sample from first antenna by taking an average over 8 reference samples
tRef = (0:refSampleLength-1)'; % Reference time in microseconds
iqRef = iqSamples(1:refSampleLength).*exp(-1i*tRef*phDiff*pi/2); % Remove phase shift caused due to GFSK

% Average over 8 reference samples and shift the sample phase
iqRefSample = mean(iqRef).*exp(1i*(phaseRef(8)-phaseRef(1)));
iqSamples = [iqRefSample;iqSamples(refSampleLength+1:end)].'; % Append reference sample with remaining samples

% Remove GFSK effect for all the IQ Samples
gfskAngles = exp(-1i*(0:cfg.SlotDuration*pi*phDiff:cfg.SlotDuration*pi*phDiff*(length(iqSamples)-1)));
IQGFSKRemoved = iqSamples.*gfskAngles;

% Sort the switching pattern and re-order the samples
[switchingPattern,sortedInd]=sort(switchingPattern);
IQGFSKRemovedSort = IQGFSKRemoved(sortedInd);

% Average the IQSamples which are collected from the same antenna
u = unique(switchingPattern); % Accept the unique values
n = histc(switchingPattern,u); %#ok<HISTC> % Returns number of values repeated at each index
d = u(n>1); % Extract repetitive indices
removeRepIdx = zeros(1,0);
if ~isempty(d) % If there is any repetitive index
    for j = 1:length(d) 
        ind = find(ismember(switchingPattern,d(j))); % Exact indices for repetitive index
        IQGFSKRemovedSort(ind(1)) = mean(IQGFSKRemovedSort(ind)); % Average
        removeRepIdx = [removeRepIdx ind(2:end)]; %#ok<AGROW> % Remove repetitive indices
    end
   IQGFSKRemovedSort(removeRepIdx) = 0;
end
IQSamplesAveraged = nonzeros(IQGFSKRemovedSort);

% Estimate AoA or AoD using custom MUSIC algorithm with adjustable resolution
ang = musicEstimation(IQSamplesAveraged.', cfg, resolution);
end


function ang = musicEstimation(dataIn, cfg, resolution)
% 내부 함수: 기존 ble.internal.musicEstimate 로직에 resolution 인자 적용

%#codegen
persistent sv pos
[u,~,~] = svd(dataIn.'); % Perform singular value decomposition and extract unitary matrix, U
noiseVectors = u(:,2:end); % Noise vectors

% Derive array design and scan angles based on custom resolution
if isa(dataIn,'single')
    azimScanAng = single(-90:resolution:90);
else
    azimScanAng = -90:resolution:90;
end

if ~cfg.EnableCustomArray
    if size(cfg.ArraySize,2)==2 % URA
        eleScanAng = azimScanAng;
    else % ULA
        eleScanAng = zeros('like',dataIn);
    end
else
    posCheck = getElementPosition(cfg);
    if rank(posCheck) == 1
        eleScanAng = zeros('like',dataIn); % If the element positions corresponds to linear array
    else
        eleScanAng = azimScanAng;
    end
end

% Derive power pattern for each azimuth and elevation angle.
% Make position and steering vectors as persistent to improve time performance
if isempty(pos)
    pos = getElementPosition(cfg); % Array element positions
    sv = ble.internal.steeringVector(azimScanAng,eleScanAng,pos); % Steering vector
else
    posTemp = getElementPosition(cfg);
    % Update if array size or element position varies
    if size(pos,2)~=size(posTemp,2) || any(any(pos ~= posTemp))
        pos = posTemp;
        sv = ble.internal.steeringVector(azimScanAng,eleScanAng,pos); % Steering vector
    end
end

% MUSIC Spectrum calculation
D = sum(abs(((sv*ones('like',dataIn))'*noiseVectors)).^2,2)+eps;
pattern = (1./D);
scanPat = reshape(pattern,[numel(eleScanAng) numel(azimScanAng)]);

% Determine the azimuth and elevation angles by locating the peaks
if cfg.EnableCustomArray || size(cfg.ArraySize,2)==2
    [~,col] = max(max(scanPat,[],1),[],2);
    [~,row] = max(max(scanPat,[],2),[],1);
    ang = [azimScanAng(col);eleScanAng(row)];
else % For ULA
    [~,b] = max(scanPat,[],2);
    ang = azimScanAng(b);
end
end