function helperBLEVisualizePosition(posLocators,posNode,angleEst,posNodeEst)
%helperBLEVisualizePosition Generates Bluetooth LE position visualization
%   helperBLEVisualizePosition(POSLOCATORS,POSNODE,ANGLEEST,POSNODEEST)
%   plots the positions of locators, actual node, estimated node and
%   triangulation lines.
%
%   POSLOCATORS is a matrix of size 2-by-N or 3-by-N, represents the
%   position of the N number of locators in a network. Each column of
%   POSLOCATORS denotes the 2D or 3D position of each locator.
%
%   POSNODE is a vector of size 2-by-1 or 3-by-1, represents the position
%   of the node in a network.
%
%   ANGLEEST is a vector of size N-by-1 or matrix of size N-by-2. The first
%   column represents the estimated azimuth angles and the second column
%   represents the estimated elevation angles, if present.
%
%   POSNODEEST represents the estimated 2D or 3D position of the node.

%   Copyright 2020-2021 The MathWorks, Inc.

numDimensions = size(posLocators,1);
numLocators = size(posLocators,2);
lineLength = 0:0.1:100;
figure
if numDimensions == 2
    h(1) = plot(posLocators(1,:),posLocators(2,:),'b*', ...
            'MarkerSize', 7, 'LineWidth', 2);
    axis([-100 100 -100 100])
    hold on
    h(2) = plot(posNode(1),posNode(2),'r*','MarkerSize', 7, 'LineWidth', 2);
    grid on
    baseLength = 0:0.1:15;
    for i = 1:numLocators
        pause(1)
        if ~isnan(angleEst(i))
            if angleEst(i)>=0
                anglePlot = 0:360/720:angleEst(i);
            else
                anglePlot = 0:-360/720:angleEst(i);
            end
            x = posLocators(1,i) + lineLength * cosd(angleEst(i));
            y = posLocators(2,i) + lineLength * sind(angleEst(i));
            x1 = posLocators(1,i) + baseLength;
            x2 = 4 * cosd(anglePlot) + posLocators(1,i);
            y2 = 4 * sind(anglePlot) + posLocators(2,i);
            c = plot(x, y, 'LineWidth', 2);
            c.Color = '#99c7e5';
            c.LineStyle  = '--';
            plot(x1, repmat(posLocators(2,i),1,length(baseLength)),'k', 'LineWidth', 1);
            plot(x2, y2,'k', 'LineWidth', 1);
            text(x2(1)-5,y2(1)-5,[num2str(angleEst(i)) char(176)])
        else
            plot(posLocators(1,i),posLocators(2,i),'rX', ...
                'MarkerSize', 14, 'LineWidth', 2);
        end
    end
    h(3) = plot(posNodeEst(1),posNodeEst(2),'ko','MarkerSize', 7, 'LineWidth', 2);
else
    h(1) = plot3(posLocators(1,:),posLocators(2,:),posLocators(3,:),'b*', ...
                    'MarkerSize', 7, 'LineWidth', 2);
    hold on
    grid on
    axis([-100 100 -100 100 -100 100])
    h(2) = plot3(posNode(1),posNode(2),posNode(3),'r*',...
                    'MarkerSize', 7, 'LineWidth', 2);
    for i = 1:numLocators
        pause(1)
        if ~isnan(angleEst(i,:))
            x = posLocators(1,i) + lineLength * cosd(angleEst(i,1))*cosd(angleEst(i,2));
            y = posLocators(2,i) + lineLength * sind(angleEst(i,1))*cosd(angleEst(i,2));
            z = posLocators(3,i) + lineLength * sind(angleEst(i,2));
            c = plot3(x,y,z, 'LineWidth', 2);
            c.Color = '#99c7e5';
            c.LineStyle  = '--';
        else
             plot3(posLocators(1,i),posLocators(2,i),posLocators(3,i),'rX', ...
                    'MarkerSize', 14, 'LineWidth', 2);
        end
    end
    h(3) = plot3(posNodeEst(1),posNodeEst(2),posNodeEst(3),'ko','MarkerSize', 7, 'LineWidth', 2);
    zlabel('Z (meters)');
end
legend(h([1 2 3]),{'Locator position','Node position','Estimated node position'},...
                                    'Location','northeast');
title('Bluetooth LE Positioning Using Direction Finding');
xlabel('X (meters)');
ylabel('Y (meters)');
end