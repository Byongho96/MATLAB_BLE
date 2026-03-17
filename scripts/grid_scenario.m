clearvars;                              % 이전 메모리 초기화

%% 1. 파라미터 설정
% [0] 룸사이즈 설정 (설정시 멀티패스 무효화)
% roomSize = [10, 8, 3];               % MATLAB 오른손 좌표계 사용 : 엄지 +X, 검지 +Y, 중지 +Z
    
% [1] 로케이터 설정
% Diagonal : 45.5739 / 0.65397 | 0.62783
% locatorsPos = [0, 0, 1.5; 0, 8, 1.5; 10, 8, 1.5; 10, 0, 1.5]';
% locatorsFront = [1, 1, 0; 1, -1, 0; -1, -1, 0; -1, 1, 0]; 
% locatorsUp = [0, 0, 1; 0, 0, 1; 0, 0, 1; 0, 0, 1];

% Rectangular : 30.5686 / 0.50061 | 0.50012
% locatorsPos = [5, 0, 1.5; 0, 4, 1.5; 5, 8, 1.5; 10, 4, 1.5]';
% locatorsFront = [0, 1, 0; 1, 0, 0; 0, -1, 0; -1, 0, 0]; 
% locatorsUp = [0, 0, 1; 0, 0, 1; 0, 0, 1; 0, 0, 1];

% Ceiling : 28.0968 / 16523.92 | 0.56356
% locatorsPos = [3, 2, 3; 3, 6, 3; 7, 2, 3; 7, 6, 3]';
% locatorsFront = [0, 0, -1; 0, 0, -1; 0, 0, -1; 0, 0, -1]; 
% locatorsUp = [0, 1, 0; 0, 1, 0; 0, 1, 0; 0, 1, 0];

% Orthogonal : 20.9816 / 0.38736 | 0.37568
locatorsPos = [5, 0, 1.5; 0, 4, 1.5; 10, 4, 1.5; 5, 4, 3]';
locatorsFront = [0, 1, 0; 1, 0, 0; -1, 0, 0; 0, 0, -1]; 
locatorsUp = [0, 0, 1; 0, 0, 1; 0, 0, 1; 0, 1, 0];

% [2] FIM 분석 지표 설정
fimMetric = 'A-opt';                  % false | 'A-opt' | 'D-opt' | 'E-opt' | 'Cond'

% [3] 멀티패스 파라미터 설정
enableMultipath = false;                % 멀티패스 레이트레이싱 시뮬레이션 on/off
stlFileName = 'test_room_10x8x3.stl';   % 로드할 환경 맵 STL 파일 (실제 파일 경로로 변경 필요)
material = 'concrete';

% [4] 위치 추정 방식 설정
visibility = 80;                            % 앵커 가시 거리
enableWeighted = true;
estimationMethod = "Non-linear";            % Linear: 선형 삼각측량, Non-linear: 비선형 최적화)

% [5] 측위 포인트 설정
% 이동 노드(태그)의 그리드 간격 설정
fixedHeight = 1.5;                      % 측위를 진행할 2차원 평면의 고정 높이 (m)
gridSpacing = 1;                      % 2차원 공간 분할 간격 (m)
offset = 1;

% roomSize 변수가 선언되어 있고 비어있지 않은지 확인
if exist('roomSize', 'var') && ~isempty(roomSize)
    % roomSize가 선언된 경우: 직사각형 공간 기반 그리드 생성
    % 방의 원점을 (0,0)으로 가정하고, offset을 반영하여 안전 구역(최소/최대 좌표) 설정
    xMin = offset;
    xMax = roomSize(1) - offset;
    yMin = offset;
    yMax = roomSize(2) - offset;
    
    % 지정된 간격(gridSpacing)으로 2D 그리드 생성
    [X, Y] = meshgrid(xMin:gridSpacing:xMax, yMin:gridSpacing:yMax);
    
    validX = X(:);
    validY = Y(:);
    
    % 고정 Z높이 반영하여 최종 3D 측위 포인트 변환
    validZ = repmat(fixedHeight, length(validX), 1);
    posNode = [validX, validY, validZ]';
    numNodePositions = size(posNode, 2);

    % simuation 정합성을 위한 초기화
    stlFileName ='' ;
    
    disp('roomSize를 기반으로 그리드 측위 위치를 생성했습니다.');

else
    % [2] roomSize가 선언되지 않은 경우: STL 파일 기반 그리드 생성 (기존 로직)
    % STL 파일 로드 및 좌표 추출
    TR = stlread(stlFileName);  % MATLAB의 'triangulation' 객체 생성
    P = TR.Points;              % [N X 3] 크기 행렬로 모든 Vertex 로드
    
    % Z축 좌표계 분석 및 바닥면 투영
    % 2D .pol 파일에서 Z축을 늘려 만든 특수한 .stl 데이터
    Z = P(:,3);                 % Z축 좌표만 추출하여 행렬 구성
    zMin = min(Z);              % 최솟값 = 0
    
    % 정점 3개가 모두 바닥면(zMin)에 위치한 바닥면 삼각형(Face)의 인덱스만 필터링
    floorFacesIdx = all(abs(Z(TR.ConnectivityList) - zMin) < 1e-4, 2);  % (부동소수점 오차 방지를 위해 1e-4 마진 적용)
    floorTriangles = TR.ConnectivityList(floorFacesIdx, :);
    
    % 바닥면 삼각형들을 2D polyshape 배열로 만들어 한개 폴리곤으로 병합
    polyArray = repmat(polyshape, size(floorTriangles, 1), 1);
    for i = 1:size(floorTriangles, 1)
        idx = floorTriangles(i, :);
        polyArray(i) = polyshape(P(idx, 1), P(idx, 2));                 % Z좌표 제외 X, Y 좌표만 사용한 2D 삼각형 생성
    end
    roomPoly = union(polyArray);
    
    % 이격 거리(offset) 적용을 통한 100% 안전 구역 정의
    safeRoomPoly = polybuffer(roomPoly, -offset);
    
    % Grid 한 측위 지점 생성
    [xlimBox, ylimBox] = boundingbox(safeRoomPoly);
    [X, Y] = meshgrid(xlimBox(1):gridSpacing:xlimBox(2), ylimBox(1):gridSpacing:ylimBox(2));
    X = X(:); Y = Y(:);
    
    % 재검증 작업 : 유효 포인트 필터링
    validGridIdx = isinterior(safeRoomPoly, X, Y);  % 폴리곤 내부에 있는지 판별
    
    % 고정 Z높이 반영하여 최종 3D 측위 포인트 변환
    validX = X(validGridIdx);
    validY = Y(validGridIdx);
    validZ = repmat(fixedHeight, length(validX), 1);
    
    posNode = [validX, validY, validZ]';
    numNodePositions = size(posNode, 2);
    
    disp('STL 맵을 기반으로 그리드 측위 위치를 생성했습니다.');
end

%% 2. 시뮬레이션 함수 호출
[posNodeEst, fim] = simulation(posNode, locatorsPos, locatorsUp, locatorsFront, ...
    "enableMultipath", enableMultipath, ...
    "enableWeighted", enableWeighted, ...
    "estimationMethod", estimationMethod, ...
    "visibility", visibility, ...
    "stlFileName", stlFileName, ...
    "material", material);

%% 3. 결과 출력 및 시각화
% FIM 지수 검사
metric = zeros(1, numNodePositions);
for inumNode = 1:numNodePositions
    FIM = fim(:, :, inumNode);
    
    % 특이행렬(Singular Matrix) 검사
    if rcond(FIM) < 1e-12
        disp(fim);
        metric(inumNode) = inf;
    elseif strcmp(fimMetric, 'A-opt')
        metric(inumNode) = trace(inv(FIM));
    elseif strcmp(fimMetric, 'D-opt')
        metric(inumNode) = det(FIM);
    elseif strcmp(fimMetric, 'E-opt')
        metric(inumNode) = min(eig(FIM));
    else
        metric(inumNode) = cond(FIM);
    end
end

% [1] RMSE 및 위치 추정 실패율 터미널 출력
posErr = sqrt(sum((posNodeEst - posNode).^2)); 
disp("======================================================");
disp(["Positioning error (RMSE) in meters = ", num2str(mean(posErr, 'omitnan'))]);
failedPointsCount = sum(isnan(posNodeEst(1,:))); % X좌표가 NaN이면 실패로 간주
disp(["총 이동 경로(Waypoints) 개수 : ", num2str(numNodePositions)]);
disp(["위치 추정 실패 포인트 (Valid Locator < 2) : ", num2str(failedPointsCount), " / ", num2str(numNodePositions)]);

% [2] 측위 위치별 fim 지수의 값
figure('Name', ['FIM Distribution - ', fimMetric], 'Color', 'w');

% 튀는 값(Inf)을 시각화하기 위해 임시로 대체값 치환 및 시각화
displayMetric = metric;
infIdx = isinf(displayMetric);
if any(infIdx)
    fprintf('경고: %d개 지점에서 FIM이 특이행렬(Singular)입니다.\n', sum(infIdx));
    % 시각화를 위해 Inf를 유효한 최댓값의 1.2배로 설정 (색상 구분을 위함)
    maxValid = max(displayMetric(~infIdx));
    if isempty(maxValid), maxValid = 1; end % 모든 값이 Inf일 경우를 대비한 예외 처리
    displayMetric(infIdx) = maxValid * 1.2;
end

% scatter3를 이용하여 위치별 metric 값 표시
% 'filled'와 'v' (데이터 값에 따른 색상) 사용
scatter3(posNode(1,:), posNode(2,:), posNode(3,:), 60, displayMetric, 'filled');
hold on;

% 로케이터 위치 표시 (비교용)
plot3(locatorsPos(1,:), locatorsPos(2,:), locatorsPos(3,:), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(locatorsPos(1,:), locatorsPos(2,:), locatorsPos(3,:)+0.2, 'Anchor', 'FontSize', 10, 'FontWeight', 'bold');

colorbar;
colormap('jet'); % 낮은 값(좋음)은 파란색, 높은 값(나쁨)은 빨간색
title(['Spatial Distribution of ', fimMetric, ' Metric']);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
grid on; view(0, 90); % 2D 평면처럼 보기 위해 위에서 아래로 조망
hold off;

% [3] fim 지수와 측위 오차 상관관계 스캐터 플롯 출력
figure('Name', 'FIM A-Optimality vs RMSE', 'Color', 'w');

% 에러가 계산 불가능(NaN)하거나 FIM이 특이행렬(inf)인 포인트 제외
validPlotIdx = ~isnan(posErr) & ~isinf(metric);
validMetric = metric(validPlotIdx);
validRMSE = posErr(validPlotIdx);

if ~isempty(validMetric)
    avgMetric = mean(validMetric); 
else
    avgMetric = inf; % 유효한 포인트가 없을 경우
end

disp(["FIM 평균 지표 (", fimMetric ,") : ", num2str(avgMetric)]);

if length(validMetric) > 1
    % 피어슨 상관계수 (Pearson correlation r) 계산
    R = corrcoef(validMetric, validRMSE);
    pearsonR = R(1, 2);
    
    scatter(validMetric, validRMSE, 40, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
    hold on;
    
    % 1차 회귀 추세선
    p = polyfit(validMetric, validRMSE, 1);
    x_fit = linspace(min(validMetric), max(validMetric), 100);
    y_fit = polyval(p, x_fit);
    plot(x_fit, y_fit, 'r-', 'LineWidth', 2);
    
    title('Correlation between Metric and Localization RMSE');
    xlabel('A-optimality (Lower is Better)');
    ylabel('Localization Error [m] (RMSE)');
    grid on;
    legend('Simulated Points', ['Trend Line (r = ', num2str(pearsonR, '%.4f'), ')'], 'Location', 'best');
    hold off;
else
    disp('상관관계를 그리기 위한 유효한 데이터 포인트가 부족합니다.');
end

% [4] 3D 트래킹 시각화 맵 생성
if ~all(isnan(posNodeEst(1,:)))
    visualizeLocalization(locatorsPos, posNode, posNodeEst)
    
    % 시각화 축(Axis) 동적 설정
    if enableMultipath
        % STL 파일을 읽어와서 3D 모델의 실제 크기(바운딩 박스)로 범위 지정
        TR = stlread(stlFileName);
        minB = min(TR.Points); % [minX, minY, minZ]
        maxB = max(TR.Points); % [maxX, maxY, maxZ]
        axis([minB(1) maxB(1) minB(2) maxB(2) minB(3) maxB(3)]);

        % 바닥면에 방 구조(STL) 테두리 그리기
        hold on;
        
        trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), repmat(minB(3), size(TR.Points,1), 1), ...
            'FaceColor', 'none', 'EdgeColor', [0.7 0.7 0.7], 'EdgeAlpha', 0.5);
        
        hold off;
    else
        % 로케이터 위치와 노드 전체 경로의 좌표를 병합하여 범위 계산
        allPoints = [locatorsPos, posNode];
        minB = min(allPoints, [], 2)'; % 각 행(X, Y, Z)의 최솟값
        maxB = max(allPoints, [], 2)'; % 각 행(X, Y, Z)의 최댓값
        
        % 그래프 가장자리가 잘리지 않도록 1m의 여백(Margin) 추가
        margin = 1.0;
        axis([minB(1)-margin maxB(1)+margin minB(2)-margin maxB(2)+margin minB(3)-margin maxB(3)+margin]);
    end
    
    view(45, 30); 
    title('BLE 3D Tracking Simulation Space');
end