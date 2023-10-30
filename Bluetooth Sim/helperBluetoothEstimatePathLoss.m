function [plLin,pldB] = helperBluetoothEstimatePathLoss(environment,distance,varargin)
%helperBluetoothEstimatePathLoss Estimates the path loss between the
%transmitter and receiver based on the environment and the distance between
%them.
%
%   [PLLIN,PLDB] = helperBluetoothEstimatePathLoss(ENVIRONMENT,DISTANCE)
%   estimates the path loss both in linear scale and dB scale, PLLIN and
%   PLDB, based on the distance between the transmitter and receiver,
%   DISTANCE and environment, ENVIRONMENT.
%
%   DISTANCE is a scalar representing the distance in meters.
%
%   ENVIRONMENT is a string scalar or character vector which can be any one
%   of these: {"Outdoor','Industrial,'Home','Office'}.
%
%   These are the optional input arguments for each environment.
%   -----------------------------------------------------------------------
%   |   Environment                | varargin
%   -----------------------------------------------------------------------
%   |  'Industrial'                | varargin(1) - Frequency of transmitted
%   |                              |               signal in Hz.
%   |                              | varargin(2) - Path loss exponent.
%   |                              | varargin(3) - Random Gaussian variable  
%   |                              |               with mean zero.
%   -----------------------------------------------------------------------
%   |  'Outdoor'                   | varargin(1) - Height of transmitter in
%   |                              |               meters.
%   |                              | varargin(2) - Height of receiver in
%   |                              |               meters.
%   |                              | varargin(3) - Transmitter antenna
%   |                              |               gain.
%   |                              | varargin(4) - Receiver antenna gain.
%   -----------------------------------------------------------------------

%  Copyright 2020-2021 The MathWorks, Inc.

if strcmp(environment,'Industrial')
    % Log normal shadowing model
    fc = 2.402*1e9; % Frequency of the transmitted signal
    plExp = 2; % Path loss exponent
    stdDev = 4; % standard deviation
    rng default;
    Xg = stdDev.*randn(1,1);  % Random gaussian variable with mean 0, standard deviation, stdDev
    if nargin > 4
        fc = varargin{1};
        plExp = varargin{2};
        Xg = varargin{3};
    elseif nargin > 3
        fc = varargin{1};
        plExp = varargin{2};
    elseif nargin > 2
        fc = varargin{1};
    end
    c = 3e8;
    lambda = c/fc;  % wavelength = velocity/frequency
    plLin = ((4*pi*1)/(lambda));% Pathloss at reference distance(d0 = 1 meter here)
    pLd0 = 20*log10(plLin); 
    pldB = pLd0 + (10*plExp*log10(distance)) + Xg;
    plLin = 10.^(pldB/20);
elseif strcmp(environment,'Outdoor')
    % Two ray ground reflection model
    ht = 1; % Height of the transmitting antenna in meters
    hr = 1; % Height of the receiving antenna in meters
    Gt  = 1; % Gain of the transmitting antenna
    Gr  = 1; % Gain of the receiving antenna
    if nargin > 5
        ht = varargin{1};
        hr = varargin{2};
        Gt  = varargin{3};
        Gr  = varargin{4};
    elseif nargin > 4
        ht = varargin{1};
        hr = varargin{2};
        Gt  = varargin{3};
    elseif nargin > 3
        ht = varargin{1};
        hr = varargin{2};
    elseif nargin > 2
        ht = varargin{1};
    end
    G = Gt*Gr;
    plLin = sqrt((distance.^4)/((G*(ht^2)*(hr^2))));
    % Convert path loss from linear scale to dB scale 
    pldB = 20*log10(plLin);
elseif strcmp(environment,'Home')
    d0 = 1; % Reference distance
    pLd0 = 12.5; % Path loss at reference distance d0
    n0 = 4.2; % Path loss exponent for first environment
    n1 = 7.6; % Path loss exponent for second environment
    d1 = 11.0; % Break point at which path loss exponent changes from n0 to n1
    if distance > d1
        pldB = pLd0 + 10*n0*log10(distance);
    else
        pldB = pLd0 + 10*n0*log10(d1/d0) + 10*n1*log10(distance/d1);
    end
    plLin = 10.^(pldB/20);
else
    d0 = 1; % Reference distance
    pLd0 = 26.8; % Path loss at reference distance d0
    n0 = 4.2; % Path loss exponent for first environment
    n1 = 8.7; % Path loss exponent for second environment
    d1 = 10; % Break point at which path loss exponent changes from n0 to n1
    if distance > d1
        pldB = pLd0 + 10*n0*log10(distance);
    else
        pldB = pLd0 + 10*n0*log10(d1/d0) + 10*n1*log10(distance/d1);
    end
    plLin = 10.^(pldB/20);
end
end