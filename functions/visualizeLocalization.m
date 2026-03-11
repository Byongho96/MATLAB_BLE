function visualizeLocalization(posLocators,posNode,posActiveLocators,posNodeTrackEst,varargin)
%helperBLEVisualizeNodeTracking Generates Bluetooth LE node grid positioning evaluation
%   (기존 트래킹 애니메이션을 제거하고, 정적 그리드 측위 오차 시각화로 변경됨)

% 차원 및 노드 위치 개수 계산
[numDimensions, numNodePositions] = size(posNode);
% 로케이터 개수 계산
numLocators = size(posLocators,2);

% 방 크기 지정
if nargin <= 4
    roomSize = [70,70,50];
else
    roomSize = varargin{1};
end

% 플롯 크기 지정
plotSizeX = max(roomSize(1), max(round(abs(posNode(1,:))'))) + 2;
plotSizeY = max(roomSize(2), max(round(abs(posNode(2,:))'))) + 2;

% Figure 생성 및 UI 패널(다크 테마) 설정
figure
if numDimensions == 2
    set(gcf, 'Position',  [200, 400, 600, 700])
else
    set(gcf, 'Position',  [200, 400, 1000, 700])
end

p2 = uipanel('BackgroundColor',[0.1570 0.1570 0.1570],'ForegroundColor',[1 1 1]);
ax = axes('Parent',p2, 'XLim',[0 plotSizeX], 'YLim',[0 plotSizeY], ...
    'NextPlot','add', 'Color',[0 0 0], 'GridColor',0.68*[1 1 1], 'GridAlpha',0.4);

grid(ax,'on');
ax.XLimMode = 'manual';
ax.YLimMode = 'manual';
ax.XColor = [1 1 1]*0.68;
ax.YColor = [1 1 1]*0.68;

% 2D/3D 환경에 따른 Z축 처리 및 View 설정
if numDimensions == 2
    ax.ZLimMode = 'manual';
    posLocators(3,:) = zeros(1,numLocators);
    posNode(3,:) = zeros(1,numNodePositions);
    posNodeTrackEst(3,:) = zeros(1,numNodePositions);
    view(ax, 0, 90); % 2D는 위에서 내려다보도록 고정
else
    plotSizeZ = max(roomSize(3), max(round(abs(posNode(3,:))'))) + 2;
    set(ax, 'ZLim', [0 plotSizeZ]);
    ax.ZLimMode = 'manual';
    ax.ZColor = [1 1 1]*0.68;
    view(ax, -20, 50);
end

% =========================================================================
% 시각화 요소 일괄 렌더링 (애니메이션 없음)
% =========================================================================

% 1. 로케이터 위치 (파란색 삼각형)
plot3(ax, posLocators(1,:), posLocators(2,:), posLocators(3,:), '^', 'MarkerSize',8, ...
    'Color',[19, 159, 255]/255, 'MarkerFaceColor',[19, 159, 255]/255, ...
    'DisplayName', 'Locator Positions');

% 2. 실제 위치 Ground Truth (기존 History 색상인 분홍색 원 활용)
scatter3(ax, posNode(1,:), posNode(2,:), posNode(3,:), 30, ...
    'MarkerEdgeColor',[255, 19, 166]/255, 'MarkerFaceColor',[255, 19, 166]/255, ...
    'MarkerFaceAlpha',0.8, 'Marker','o', 'DisplayName', 'True Grid Positions');

% 3. 추정 위치 Estimated (기존 흰색 십자가 마커 활용)
scatter3(ax, posNodeTrackEst(1,:), posNodeTrackEst(2,:), posNodeTrackEst(3,:), 40, ...
    'MarkerEdgeColor',[1 1 1], 'MarkerFaceColor',[1 1 1], ...
    'MarkerFaceAlpha',0.8, 'Marker','+', 'LineWidth',1.5, ...
    'DisplayName', 'Estimated Positions');

% 4. 실제 위치와 추정 위치를 연결하는 오차선 (청록색 점선)
% (범례에 하나만 표시하기 위한 더미 플롯)
plot3(ax, nan, nan, nan, ':c', 'LineWidth', 1.2, 'DisplayName', 'Estimation Error');

% 실제 선 긋기
for j = 1:numNodePositions
    if ~isnan(posNodeTrackEst(1,j))
        plot3(ax, [posNode(1,j) posNodeTrackEst(1,j)], ...
                  [posNode(2,j) posNodeTrackEst(2,j)], ...
                  [posNode(3,j) posNodeTrackEst(3,j)], ...
              ':c', 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

% =========================================================================
% 범례 및 라벨 설정
% =========================================================================

% 범례 표시 (애니메이션 상태값 제거 및 간소화)
legend('TextColor','white', 'Position',[0.7236 0.7429 0.2251 0.1664], ...
       'AutoUpdate','off', 'Color',[0.5 0.5 0.5]);

% 축 제목 및 라벨
title('Bluetooth LE Grid Positioning Evaluation','Color','white');
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');

end