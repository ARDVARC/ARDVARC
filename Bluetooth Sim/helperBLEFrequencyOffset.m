function [outData,varargout] = helperBLEFrequencyOffset(inData,...
                                    sampleRate,frequencyOffset,varargin)
%helperBLEFrequencyOffset Apply frequency offset to the input signal
%
%   OUTDATA = helperBLEFrequencyOffset(INDATA,SAMPLERATE,FREQUENCYOFFSET)
%   applies the specified frequency offset to the input signal, INDATA.
%
%   OUTDATA = helperBLEFrequencyOffset(INDATA,SAMPLERATE,FREQUENCYOFFSET,
%   INISTATE) specifies the initial state for applying the frequency offset
%   to the input data.
%
%   [OUTDATA,STATEOUT] = helperBLEFrequencyOffset(...) returns the final
%   frequency offset value for use in applying frequency offset to a future
%   signal.
%
%   OUTDATA is the frequency offset output of the same size as INDATA.
%
%   STATEOUT is the output state in Hz.
%
%   INDATA is the complex column vector.
%
%   SAMPLERATE is the sampling rate in Hz.
%
%   FREQUENCYOFFSET is the frequency offset to apply to the input in Hz.
%
%   INISTATE is the input state in Hz. The default value is 0.
%
%   See also comm.PhaseFrequencyOffset.

%   Copyright 2021 The MathWorks, Inc.

% Check the number of input arguments
narginchk(3,4);
if nargin == 4
    iniState = varargin{1};
else
    iniState = 0;
end

% Create vector of time samples
inputLength = length(inData);
timeSteps = (0:inputLength).';
freqVec = frequencyOffset * timeSteps /sampleRate;
outData = inData.*exp(1i*2*pi*(iniState + freqVec(1:inputLength)));
varargout{1} = iniState + freqVec(inputLength + 1);
end