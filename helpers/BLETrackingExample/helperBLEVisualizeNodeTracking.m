function helperBLEVisualizeNodeTracking(posLocators,posNode,posActiveLocators,posNodeTrackEst,varargin)
%helperBLEVisualizeNodeTracking Generates Bluetooth LE node tracking
%visualization
%   helperBLEVisualizeNodeTracking(POSLOCATORS,POSNODE,POSACTIVELOCATORS,
%   POSNODETRACKEST) plots the positions of locators, actual node, active
%   locators, estimated node and history of node positions.
%
%   POSLOCATORS is a matrix of size 2-by-N or 3-by-N, represents the
%   position of the N number of locators in a network. Each column of
%   POSLOCATORS denotes the 2D or 3D position of each locator.
%
%   POSNODE is a vector of size 2-by-M or 3-by-M, represents the M node
%   positions in a network. Each column of POSNODE denotes the 2D or 3D
%   position of node.
%
%   POSACTIVELOCATORS is a matrix of size 2-by-P or 3-by-P, represents the
%   position of the P number of active locators in a network. Each column
%   of POSACTIVELOCATORS denotes the 2D or 3D position of each active
%   locator.
%
%   POSNODETRACKEST is a vector of size 2-by-M or 3-by-M, represents the M
%   estimated node positions using linear Kalman filter. Each column of
%   POSNODEEST denotes the 2D or 3D position of estimated node.
%
%   helperBLEVisualizeNodeTracking(...,ROOMSIZE) plots the positions of
%   locators, actual node, active locators, estimated node and history of
%   node positions within the view point of room size, ROOMSIZE.
%
%   ROOMSIZE is a 2 element or 3 element vector in 2-D or 3-D space
%   respectively.

%   Copyright 2021-2025 The MathWorks, Inc.

% Compute the dimensions and number of node positions
[numDimensions,numNodePositions] = size(posNode);

% Compute the number of locators
numLocators = size(posLocators,2);

% Specify the room size
if nargin <= 4
    roomSize = [70,70,50];
else
    roomSize = varargin{1};
end

% Specify the plot size
plotSizeX = max(roomSize(1),max(round(abs(posNode(1,:))')))+2;
plotSizeY = max(roomSize(2),max(round(abs(posNode(2,:))')))+2;

% Define a figure for 2D plane or a 3D space
if numDimensions == 2
    % Create an empty figure
    figure
    set(gcf, 'Position',  [200, 400, 600, 700])

    % Specify a dark background for plot
    p2 = uipanel('BackgroundColor',[0.1570 0.1570 0.1570],'ForegroundColor',[1 1 1]);

    % Specify a black color for axes and create an axes handle
    ax  = axes('Parent',p2,'XLim',[0 plotSizeX],'YLim',[0 plotSizeY],...
        'NextPlot','add','Color',[0 0 0],'GridColor',0.68*[1 1 1],'GridAlpha',0.4);

    % Make grid on
    grid(ax,'on');

    % Make the axes tick limits as manual
    ax.XLimMode = 'manual';
    ax.YLimMode = 'manual';
    ax.ZLimMode = 'manual';

    % Specify color of axes in x, y and z axis
    ax.XColor = [1 1 1]*0.68;
    ax.YColor = [1 1 1]*0.68;

    % Make the 3rd dimension positions as zero for 2D plane plots
    posLocators(3,:) = zeros(1,numLocators);
    posNode(3,:) = zeros(1,numNodePositions);
    posNodeTrackEst(3,:) = zeros(1,numNodePositions);

elseif numDimensions == 3

    % Specify plot size in z directions
    plotSizeZ = max(roomSize(3),max(round(abs(posNode(3,:))')))+2;

    % Create an empty figure
    figure
    set(gcf,"Position",[200 400 1000 700])

    % Specify a dark background for plot
    p2 = uipanel('BackgroundColor',[0.1570 0.1570 0.1570],'ForegroundColor',[1 1 1]);

    % Specify a black color for axes and create an axes handle
    ax  = axes('Parent',p2,'XLim',[0 plotSizeX],'YLim',[0 plotSizeY],'ZLim',[0 plotSizeZ],...
        'NextPlot','add','Color',[0 0 0],'GridColor',0.68*[1 1 1],'GridAlpha',0.4);

    % Make grid on
    grid(ax,'on');

    % Make the axes tick limits as manual
    ax.XLimMode = 'manual';
    ax.YLimMode = 'manual';
    ax.ZLimMode = 'manual';

    % Specify color of axes in x, y and z axis
    ax.XColor = [1 1 1]*0.68;
    ax.YColor = [1 1 1]*0.68;
    ax.ZColor = [1 1 1]*0.68;

    % Make view of axes with azimuth as -30 and elevation as 20 degrees
    view(ax,-20,50);
end

% Plot the positions of locators in 3D space using a ^ marker
plot3(posLocators(1,:),posLocators(2,:),posLocators(3,:),'^','MarkerSize',8,...
    'Color',[19, 159, 255]/255,'MarkerFaceColor',[19, 159, 255]/255);

% Create an plot3 handle with empty positions of active locators
locatorPlotter = plot3(ax,nan,nan,nan,'^','MarkerSize',8,'Color',...
    [255, 255, 17]/255,'MarkerFaceColor',[255, 255, 17]/255);

% Create a scatter3 handle with empty positions of current asset positions
pLine = scatter3(ax,nan,nan,nan,'MarkerEdgeColor',[255, 19, 166]/255,'Marker','o',...
    'MarkerFaceColor',[255, 19, 166]/255,'MarkerFaceAlpha',1);

% Create a scatter3 handle with empty positions of asset position history
hLine = scatter3(ax,nan,nan,nan,'MarkerEdgeColor',[255, 19, 166]/255,...
    'MarkerFaceColor',[255, 19, 166]/255,'MarkerFaceAlpha',...
    0.4,'MarkerEdgeAlpha',0.4,'Marker','o');

% Create a scatter3 handle with empty positions of estimated current asset
% positions
pLineEst = scatter3(ax,nan,nan,nan,'MarkerEdgeColor',[255, 105, 41]/255,...
    'MarkerFaceColor',[255, 105, 41]/255,'MarkerFaceAlpha',...
    1,'Marker','+','LineWidth',2);

% Create a scatter3 handle with empty positions of estimated asset position
% history
hLineEst = scatter3(ax,nan,nan,nan,'MarkerEdgeColor',[1 1 1],...
    'MarkerFaceColor',[1 1 1],'MarkerFaceAlpha',...
    0.4,'MarkerEdgeAlpha',0.4,'Marker','+','LineWidth',2);

% Specify legend values
legend('Locator Positions','Active Locator Positions','Current Asset Position',...
    'Asset Positions History','Current Estimated Asset Position','Estimated Asset Positions History',...
    'TextColor','white','Position',[0.7236 0.7429 0.2251 0.1664],'AutoUpdate','off','Color',[0.5 0.5 0.5]);

% Specify title and labels for x, y and z directions
title('Bluetooth LE Asset Tracking','Color','white');
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');

% Initiate memory for history points of trajectory
HistoryPositions = {};
HistoryPositionsEst = {};

% Plot in a loop
for j = 1:numNodePositions

    % Update and plot the position of node for a node position
    HistoryPositions = plotTrack(posNode(:,j).',pLine,hLine,HistoryPositions);

    % Update and plot the estimated position of node for a node position
    HistoryPositionsEst = plotTrack(posNodeTrackEst(:,j).',pLineEst,hLineEst,HistoryPositionsEst);

    % Pause for 0.25 seconds
    pause(0.25)
    if numDimensions == 2
        posActiveLocators{j}(3,:) = zeros(1,length(posActiveLocators{j}(1,:)));
    end

    % Update the handle for active locator positions with positions of
    % active locators
    set(locatorPlotter,'XData',posActiveLocators{j}(1,:),'YData',...
        posActiveLocators{j}(2,:),'ZData',posActiveLocators{j}(3,:));

    if ~isempty(HistoryPositionsEst) && size(HistoryPositionsEst,2) > 1
        % Plot the current and one history element of trajectory
        plot3([HistoryPositionsEst{end-1}(1) HistoryPositionsEst{end}(1)], ...
            [HistoryPositionsEst{end-1}(2) HistoryPositionsEst{end}(2)], ...
            [HistoryPositionsEst{end-1}(3) HistoryPositionsEst{end}(3)],':c');
    end
end
end

function HistoryPositions = plotTrack(positions,pLine,hLine,HistoryPositions)
% PLOTTRACK Plots the asset position and trajectory of motion
%
%   HISTORYPOSITIONS = plotTrack(POSITIONS,PLINE,HLINE,HISTORYPOSITIONS)
%   plot and the current positions, POSITIONS, by using the scatter3 handle,
%   PLINE, and updates the history positions, HISTORYPOSITIONS, by using
%   the scatter3 handle of history positions.
%
%   POSITIONS is a 2 element or 3 element current node position vector in
%   2D or 3D space.
%
%   PLINE is scatter3 handle for present positions of node
%
%   HLINE is scatter3 handle for history positions of node
%
%   HISTORYPOSITIONS is a 2 element or 3 element history node position
%   vector in 2D or 3D space.

% Update the pline handle with current positions
set(pLine,'XData',positions(:,1),'YData',positions(:,2),'ZData',positions(:,3));

% Vertically concatenate the history positions and update the history
% handle of node positions
lineData = vertcat(zeros(0,3),HistoryPositions{:});
set(hLine,'XData',lineData(:,1),'YData',lineData(:,2),'ZData',lineData(:,3));

% Update the history with new locator positions
HistoryPositions{end+1} = positions;
end
