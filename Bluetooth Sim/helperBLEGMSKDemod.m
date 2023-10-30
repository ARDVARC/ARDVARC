function [z,varargout] = helperBLEGMSKDemod(y,mode,nSamp,varargin)
%helperBLEGMSKDemod Gaussian minimum shift keying demodulation
%   Z = helperBLEGMSKDemod(Y,MODE,NSAMP) demodulates the complex envelope Y
%   of a signal using the Gaussian minimum shift keying method and produces
%   soft output Z. MODE denotes the PHY transmission mode. NSAMP denotes
%   the number of samples per symbol and must be a positive integer.
%
%   Z = helperBLEGMSKDemod(Y,NSAMP,INIPHASE) specifies the initial phase
%   of the demodulator. INIPHASE is a scalar and the default value is 0.
%
%   [Z,PHASEOUT] = helperBLEGMSKDemod(...) returns the final phase of Y
%   for use in demodulating a future signal.

%   Copyright 2021-2023 The MathWorks, Inc.

% Check the number of input arguments
narginchk(3,4)
if nargin == 3
    initPhaseOffset = 0;
    remSym = 0;
else
    initPhaseOffset = varargin{1};
    remSym = 1;
end
[z,phaseOut] = ble.internal.gfskdemod(y,mode,nSamp,0.5,1,1,initPhaseOffset,remSym);
varargout{1} = phaseOut;
end