function [posNode,posLocator,angle] = helperBLEGeneratePositions(n,numDimension)
%helperBLEGeneratePositions Generates locators and node positions
%   [POSNODE,POSLOCATOR,ANGLE] = helperBLEGeneratePositions(N,...
%   NUMDIMENSION) places Bluetooth LE node at origin and N number of
%   locators randomly in 2D or 3D space, if NUMDIMESION is 2 or 3,
%   respectively. The X, Y and Z positions will be in meters.
%
%   POSNODE represents the 2D or 3D position of the node at origin.
%
%   POSLOCATOR is a matrix of size 2-by-N or 3-by-N, represents the
%   positions of the locators. Each column of POSLOCATOR denotes the 2D or
%   3D position of each locator.
%
%   ANGLE is a vector of size N-by-1 or a matrix of size N-by-2, represents
%   the AoA or AoD between the locators and node in degrees. First column
%   of ANGLE denotes the azimuth angles and second column denotes the
%   elevation angles, if present.
%
%   N is the number of locators in a network.
%
%   NUMDIMENSION is the number of dimensions, which can be 2 or 3.

%   Copyright 2020-2021 The MathWorks, Inc.

% Generate radius values in meters from the uniform distribution on the
% interval [0, 100] by considering Bluetooth LE range as 100 meters.
radius = 100.*rand(1,n);

% The azimuth angles are restricted to -90 to 90 as the ULA and URA array
% designs have ambiguity with geometry. In order to get this range for AoA
% or AoD at locators, the angles at the node must be within [90, 270].
% Generate the angles from -80 to 80 degrees at locators to eliminate the
% discontinuities in the estimated angle.
repFactor = repmat([1 -1],1,ceil(n/2));
az = 180+80.*rand(1,n).*repFactor(1:n);

% Generate elevation angles only when 3D positioning is considered
if numDimension == 3
    % Generate angles in degrees within [-80, 80] to eliminate the
    % discontinuities in the estimated angle.
    el = 80.*rand(1,n).*repFactor(1:n);

    % Transform spherical to cartesian coordinates
    [x,y,z] = sph2cart(az*pi/180,el*pi/180,radius);
    posLocator = [x; y; z]; % X, Y and Z positions in meters
    posNode = [0;0;0]; % Node is always assumed to be at origin
else
    % Transform polar to cartesian coordinates
    [x, y] = pol2cart(az*pi/180, radius);
    posLocator = [x; y]; % X and Y positions in meters
    posNode = [0;0]; % Node is always assumed to be at origin
end

% Generated azimuth (az) and elevation (el) angles are at the node and the
% angles at the locators gives the AoA or AoD. To get AoA or AoD at the
% locators subtract 180 degrees from azimuth angles and negate the
% elevation angles.
angle(:,1) = (az-180).';
if numDimension == 3
    angle(:,2) = -el.';
end
end