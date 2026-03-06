function [posActiveLocators,angle,distance] = helperBLEActiveLocators(posNode,posLocators,varargin)
%helperBLEActiveLocators Generates active locators parameters
%   [POSACTIVELOCATOR,ANGLE,DISTANCE] = helperBLEActiveLocators(POSNODE,
%   POSLOCATORS) generates parameters corresponding to active locators.
%
%   POSACTIVELOCATORS is a matrix of size 2-by-P or 3-by-P, represents the
%   position of the P number of active locators in a network of N locators.
%   Each column of POSACTIVELOCATORS denotes the 2-D or 3-D position of
%   each active locator.
%
%   ANGLE is a vector of size P-by-1 or a matrix of size P-by-2, represents
%   the AoA or AoD between the active locators and node in degrees. First
%   column of ANGLE denotes the azimuth angles and second column denotes
%   the elevation angles, if present.
%
%   DISTANCE is a vector of size P-by-1, represents the distance between
%   the active locators and node in meters.
%
%   POSNODE is a vector of size 2-by-1 or 3-by-1, represents the 2-D or 3-D
%   position of the node in a network.
%
%   POSLOCATORS is a matrix of size 2-by-N or 3-by-N, represents the
%   position of the N number of locators in a network. Each column of
%   POSLOCATORS denotes the 2-D or 3-D position of each locator.
%
%   [POSACTIVELOCATOR,ANGLE,DISTANCE] = helperBLEActiveLocators(POSNODE,
%   POSLOCATORS,METHOD) generates parameters cooresponding to the METHOD.
%   METHOD can be one of the "angulation", "distance-angle" or
%   "lateration".

%   Copyright 2021-2025 The MathWorks, Inc.
if nargin > 2
    method = varargin{1};
else
    method = "angulation";
end

% Compute distance between all the locators and node.
distance = sqrt(sum((posNode-posLocators).^2));

% Compute the active locators positions with respect to the method of
% simulations
if method == "angulation" || method == "distance-angle"
    % Consider the locators which are in 80 meters range to the node.
    distanceIdx = find(distance<80);

    % Compute the azimuth angles between all the locators and node. The
    % azimuth angles are restricted to -90 to 90 as the ULA and URA array
    % designs have ambiguity with geometry. Consider the angles from -80 to
    % 80 degrees to eliminate the discontinuities in the estimated angle.
    diffTerm = posNode-posLocators;
    azimAngle = (atan2d(diffTerm(2,:),diffTerm(1,:))).';
    azimAngleIdx = find(azimAngle>=-80 & azimAngle<=80);

    % Compute the elevation angles between all the locators and node.
    % Consider the angles from -80 to 80 degrees to eliminate the
    % discontinuities in the estimated angle.
    if size(posNode,1) == 2
        eleAngle = zeros(size(posLocators,2),1);
    else
        t = sqrt(sum((posNode(1:2)-posLocators(1:2,:)).^2));
        eleAngle = atan2d(posNode(3)-posLocators(3,:),t).';
    end
    eleAngleIdx = find(eleAngle>=-80 & eleAngle<=80);

    % Consider the positions, distances, and angles of corresponding active
    % locators
    intersectIdx1 = intersect(distanceIdx,azimAngleIdx);
    intersectIdx2 = intersect(intersectIdx1,eleAngleIdx);
    distance = distance(intersectIdx2).';
    angle(:,1) = azimAngle(intersectIdx2);
    angle(:,2) = eleAngle(intersectIdx2);
    posActiveLocators = posLocators(:,intersectIdx2);
elseif method == "lateration"
    % Compute the number of dimensions of motion to generate asset
    % positions.
    numDimensions = size(posLocators,1);

    % Specify minimum number of N nearest locators required, where N is 5
    % for 2D and 6 for 3D simulations
    minNumLocators = numDimensions + 3;

    % Specify locators within 30 meters range
    selectIdx = find(distance<=30);

    if numel(selectIdx) >= minNumLocators
        % Specify the distance of the active locators
        distance = distance(selectIdx);

        % Specify the positions of the active locators
        posActiveLocators = posLocators(:,selectIdx);
    else
        % Sort the distance to consider N nearest locators
        sortDistance = sort(distance);

        % Compute the active locators positions
        activeLocatorDist = sortDistance(minNumLocators);

        % Select the index of nearest N locators
        selectIdx = distance<=activeLocatorDist;

        % Specify the distance of the active locators
        distance = distance(selectIdx);

        % Specify the positions of the active locators
        posActiveLocators = posLocators(:,selectIdx);
    end
    % Specify NaN as angle output
    angle = NaN;
end