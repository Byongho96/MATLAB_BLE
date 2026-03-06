function [posNode,posLocator] = helperBLEPositions(motionModel,numNodePositions,numLocators,varargin)
%helperBLEPositions Generates locators and node positions
%   [POSNODE,POSLOCATORS] = helperBLEPositions(MOTIONMODEL,
%   NUMNODEPOSITIONS,NUMLOCATORS) generates Bluetooth LE node and locator
%   positions in 2-D or 3-D space.
%
%   POSNODE is a vector of size 2-by-M or 3-by-M, represents the M node
%   positions in a network. Each column of POSNODE denotes the 2-D or 3-D
%   position of node.
%
%   POSLOCATORS is a matrix of size 2-by-N or 3-by-N, represents the
%   position of the N number of locators in a network. Each column of
%   POSLOCATORS denotes the 2-D or 3-D position of each locator.
%
%   MOTIONMODEL is a character vector, represents the linear motion model.
%   This value must be one of '2-D Constant Velocity', '3-D Constant
%   Velocity', '2-D Constant Acceleration', or '3-D Constant Acceleration'.
%
%   NUMNODEPOSITIONS is a scalar, represents the number of node positions
%   to track in a network.
%
%   NUMLOCATORS is a scalar, represents the number of locators in a
%   network.
%
%   [POSNODE,POSLOCATORS] = helperBLEPositions(MOTIONMODEL,
%   NUMNODEPOSITIONS,NUMLOCATORS,ROOMSIZE,TRAJECTORYMODEL) generates
%   Bluetooth LE node and locator positions in 2-D or 3-D space within the
%   room size constraint, ROOMSIZE, and trajectory of node,
%   TRAJECTORYMODEL.
%
%   ROOMSIZE is a 2 element or 3 element vector in 2-D or 3-D space
%   respectively.
%
%   TRAJECTORYMODEL is a character vector, represents the trajectory motion
%   model of the Bluetooth LE node. This value must be one of the 'Linear
%   Path', 'Random Path', 'Fixed Path', or 'Circular Path'.

%   Copyright 2021-2025 The MathWorks, Inc.

% Compute the number of dimensions of motion to generate asset positions.
numDimensions = 2 + any(motionModel==["3-D Constant Velocity","3-D Constant Acceleration"]);

% Specify the size of the room and model of trajectory
if nargin <= 3
    roomSize = [70;70;50];
    trajectoryModel = "Linear Path";
else
    roomSize = varargin{1}(:);
    trajectoryModel = varargin{2};
    % Validate the number of dimensions of trajectory and the room size.
    if numDimensions == 3 && numel(roomSize) ==2
        error("To estimate 3-D position of the asset, room size must be a three-element column.")
    end
    if norm(roomSize) > 149
        error("Room size must have the body diagonal less than 150 meters.")
    end
end

% Specify the size of room in x and y direction and coordinates of the
% center of the room.
xMax = roomSize(1);
yMax = roomSize(2);
centerOfRoom = roomSize(:)/2;

% Preallocate space for position of locators in the four quadrants in x,
% and y directions or in the eight octets in x, y and z directions for 2D
% and 3D respectively in meters
posLocator = zeros(numDimensions,numLocators);
if numDimensions == 3

    % Generate random locator points for each octet in 3 D space
    for countQuater = 1:numLocators
        counter = mod(countQuater,8);
        switch counter
            case 0
                posLocator(:,countQuater) = [ 1; 1; 1].*rand(numDimensions,1)/2;
            case 1
                posLocator(:,countQuater) = [ 1; 1;-1].*rand(numDimensions,1)/2;
            case 2
                posLocator(:,countQuater) = [ 1;-1; 1].*rand(numDimensions,1)/2;
            case 3
                posLocator(:,countQuater) = [ 1;-1;-1].*rand(numDimensions,1)/2;
            case 4
                posLocator(:,countQuater) = [-1; 1; 1].*rand(numDimensions,1)/2;
            case 5
                posLocator(:,countQuater) = [-1; 1;-1].*rand(numDimensions,1)/2;
            case 6
                posLocator(:,countQuater) = [-1;-1; 1].*rand(numDimensions,1)/2;
            case 7
                posLocator(:,countQuater) = [-1;-1;-1].*rand(numDimensions,1)/2;
        end
    end
else
    % Generate random locator points for each quadrant in 2 D plane
    for countQuater = 1:numLocators
        counter = mod(countQuater,4);
        switch counter
            case 0
                posLocator(:,countQuater) = [ 1; 1].*rand(numDimensions,1)/2;
            case 1
                posLocator(:,countQuater) = [ 1;-1].*rand(numDimensions,1)/2;
            case 2
                posLocator(:,countQuater) = [-1; 1].*rand(numDimensions,1)/2;
            case 3
                posLocator(:,countQuater) = [-1;-1].*rand(numDimensions,1)/2;
        end
    end
end

% Move the positions of locators to actual room-size
posLocator = round(roomSize(1:numDimensions,1).*posLocator*0.9+centerOfRoom(1:numDimensions,1),2);

% Linear trajectory
if trajectoryModel == "Linear Path"

    % Specify start point for x, and y directions
    startpointX = centerOfRoom(1) - xMax*(rand/4+0.5)/2;
    startpointY = centerOfRoom(2) - yMax*(rand/4+0.5)/2;

    % Initialize a time sample vector
    iX = linspace(0.2,1,numNodePositions);
    iY = linspace(0.2,1,numNodePositions);
    if any(motionModel==["2-D Constant Velocity","3-D Constant Velocity"])

        % Specify the velocities in x, and y directions in m/s
        vX = 3;
        vY = 4;

        % Specify the time step
        T = 0.25;

        % Position of node in x, and y directions
        posNode = [startpointX + vX*T*iX*xMax; startpointY + vY*T*iY*yMax];
        if motionModel=="3-D Constant Velocity"

            % Specify the size of room in z direction in meters.
            zMax = roomSize(3);
            iZ = linspace(0.2,1,numNodePositions);

            % Velocities in z direction in m/s
            vZ = 3;

            % Specify start point for z direction
            startpointZ = centerOfRoom(3) - zMax*(rand/4+0.5)/2;

            % Position of node in z direction in m/s
            posNode(3,:) = startpointZ + vZ*T*iZ*zMax;
        end
    else
        % Acceleration in x, and y directions in m/sec^2
        aX = 1.75;
        aY = 1.35;

        % Position of node in x, and y direction in m/s
        posNode = [startpointX + xMax*0.5*aX*((iX).^2); ...
            startpointY + yMax*0.5*aY*((iY).^2)];

        if motionModel=="3-D Constant Acceleration"
            % Specify the size of room in z direction in meters.
            zMax = roomSize(3);
            iZ = linspace(0,1,numNodePositions);

            % Acceleration in z direction in m/sec^2
            aZ = 1.25;
            startpointZ = centerOfRoom(3) - zMax*(rand/4+0.75)/2;

            % Position of node in z direction
            posNode(3,:) = startpointZ + zMax*(0.5*aZ*(iZ).^2)';
        end
    end
elseif trajectoryModel == "Circular Path"
    % Curved trajectory Define the circular motion radius and center.
    radius = 0.5*(min(xMax,yMax));
    center = [xMax yMax]/2;

    % Generate the index values from 0 to 1
    index = linspace(0, 1, numNodePositions);
    if any(motionModel==["2-D Constant Velocity","3-D Constant Velocity"])
        % Calculate the angle theta
        theta = 2 * pi * index;

        % Position of node in x, and y directions
        posNode(1,:) = center(1) + radius * cos(theta);
        posNode(2,:) = center(2) + radius * sin(theta);
        if motionModel=="3-D Constant Velocity"
            zMax = roomSize(3);

            % Position of node in z direction
            posNode(3,:) = zMax*(0.75*(index)+0.125);
        end
    else
        % Calculate the angle theta with quadratic progression
        theta = 2 * pi * index.^2;

        % Position of node in x, and y directions
        posNode(1,:) = center(1) + radius * cos(theta);
        posNode(2,:) = center(2) + radius * sin(theta);
        if motionModel=="3-D Constant Acceleration"

            % Specify the size of room in z direction in meters.
            zMax = roomSize(3);

            % Position of node in z direction in m/sec^2
            posNode(3,:) = zMax*(0.75*(index)+0.125);

            % Position of Locator in z direction posLocators(3,:) =
            % centerOfRoom(3) + zMax*(rand(1,numLocators) - 0.5);
        end
    end
elseif trajectoryModel == "Fixed Path"

    % Fixed trajectory Specify the lengths of x and y direction movement
    length1 = 0.5*xMax;
    length2 = 0.5*yMax;

    % Calculate the total length of the path
    totalLength = 2 * length1 + length2;

    % Calculate the number of points per segment based on their lengths
    points1 = round(numNodePositions * (length1 / totalLength));
    points2 = round(numNodePositions * (length2 / totalLength));
    points3 = numNodePositions - points1 - points2; % Remaining points for the last segment
    if any(motionModel==["2-D Constant Velocity","3-D Constant Velocity"])

        % Position of node in x, and y directions
        posNode(1,:) = [linspace(0, length1, points1), length1 * ones(1, points2), length1 + linspace(0, length1, points3)]*0.9 + length1*0.1;
        posNode(2,:) = [zeros(1,points1), linspace(0, length2, points2), length2 * ones(1,points3)] + length2/2;
        if motionModel=="3-D Constant Velocity"

            % Specify the size of room in z direction in meters.
            zMax = roomSize(3);

            % Position of node in z direction in m/s
            posNode(3,:) = linspace(0, 0.85*zMax, numNodePositions)*0.9 + zMax*0.1;
        end
    else
        % First segment (horizontal with acceleration)
        t1 = linspace(0, 1, points1);

        % Second segment (vertical with acceleration)
        t2 = linspace(0, 1, points2);

        % Third segment (horizontal with acceleration)
        t3 = linspace(0, 1, points3);

        % Position of node in x, and y directions
        posNode(1,:) = [length1 * (t1.^2), length1 * ones(size(t2)), length1 + length1 * (t3.^2)]*0.9 + length1*0.1; % Quadratic progression
        posNode(2,:) = [zeros(size(t1)), length2 * (t2.^2), length2 * ones(size(t3))] + length2/2;
        if motionModel=="3-D Constant Acceleration"
            zMax = roomSize(3);

            % Position of node in z direction
            posNode(3,:) = zMax*(linspace(0, 0.8, numNodePositions).^2)*0.9 + zMax*0.1;
        end
    end
elseif trajectoryModel == "Random Path"
    % Random trajectory
    % Specify start point for x, and y directions
    startpointX = centerOfRoom(1) - xMax*(rand/4+0.75)/2;
    startpointY = centerOfRoom(2) - yMax*(rand/4+0.75)/2;

    % Specify end point for x, and y directions
    endpointX = centerOfRoom(1) + xMax*(rand/4+0.75)/2;
    endpointY = centerOfRoom(2) + yMax*(rand/4+0.75)/2;

    % Specify the nominal standard deviation for each step for x, and y
    % directions
    stdDevX = 2;
    stepsX = randn(1,numNodePositions-1)*stdDevX;
    stdDevY = 3;
    stepsY = randn(1,numNodePositions-1)*stdDevY;

    % Subtract the mean of steps to keep steps unbiased for x, and y
    % directions
    stepsX = stepsX - mean(stepsX);
    stepsY = stepsY - mean(stepsY);

    % Add in a bias to each step to progress from start points to end
    % points for x, and y directions.
    stepsX = stepsX + (endpointX - startpointX)/(numNodePositions-1);
    stepsY = stepsY + (endpointY - startpointY)/(numNodePositions-1);

    % Specify the cumulative sum for a progressive walk in x, and y
    % directions.
    walkX = cumsum([startpointX,stepsX]);
    walkY = cumsum([startpointY,stepsY]);

    % Position of node in x, and y direction
    posNode = [walkX;walkY];
    if any(motionModel==["3-D Constant Acceleration","3-D Constant Velocity"])

        % Specify the size of room in z direction in meters.
        zMax = roomSize(3);

        % Specify start point and end point for z direction
        startpointZ = centerOfRoom(3) - zMax*(rand/4+0.75)/2;
        endpointZ = centerOfRoom(3) + zMax*(rand/4+0.75)/2;

        % Specify the nominal standard deviation for each step for z
        % directions
        stdDevZ = 2;
        stepsZ = randn(1,numNodePositions-1)*stdDevZ;

        % Subtract the mean of steps to keep steps unbiased for z
        % directions
        stepsZ = stepsZ - mean(stepsZ);

        % Add in a bias to each step to progress from start points to end
        % points for z directions.
        stepsZ = stepsZ + (endpointZ - startpointZ)/(numNodePositions-1);

        % Specify the cumulative sum for a progressive walk in z
        % directions.
        walkZ = cumsum([startpointZ,stepsZ]);

        % Position of node in z direction
        posNode(3,:) = walkZ;
    end
end
end