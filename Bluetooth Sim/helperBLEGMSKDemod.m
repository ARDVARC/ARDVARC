function [z,varargout] = helperBLEGMSKDemod(y,nSamp,varargin)
%helperBLEGMSKDemod Gaussian minimum shift keying demodulation
%   Z = helperBLEGMSKDemod(Y,NSAMP) demodulates the complex envelope Y of
%   a signal using the Gaussian minimum shift keying method and produces
%   soft output Z. NSAMP denotes the number of samples per symbol and must
%   be a positive integer.
%
%   Z = helperBLEGMSKDemod(Y,NSAMP,INIPHASE) specifies the initial phase
%   of the demodulator. INIPHASE is a scalar and the default value is 0.
%
%   [Z,PHASEOUT] = helperBLEGMSKDemod(...) returns the final phase of Y
%   for use in demodulating a future signal.

%   Copyright 2021 The MathWorks, Inc.

% Check the number of input arguments
narginchk(2,3)
if nargin == 2
    initPhaseOffset = 0;
else
    initPhaseOffset = varargin{1};
end
[z,phaseOut] = ble.internal.gmskdemod(y,nSamp,initPhaseOffset);
varargout{1} = phaseOut;
end