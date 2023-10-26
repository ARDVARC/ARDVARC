function dataOut = helperBLEDelaySignal(dataIn,delay)
%helperBLEDelaySignal Introduces time delay in the signal
%   DATAOUT = helperBLEDelaySignal(DATAIN, DELAY) returns the delayed
%   signal DATAOUT by applying DELAY to the input signal, DATAIN.
%
%   DATAOUT is a column vector which represents the delayed version of
%   DATAIN.
%
%   DATAIN is a column vector which represents the input signal that needs
%   to be delayed.
%
%   DELAY is a real scalar value (in samples).

%   Copyright 2021 The MathWorks, Inc.

% Initialization
inputLength = size(dataIn,1);        % Input signal length
delayInt = round(delay);             % Integer delay
delayFrac = delay - delayInt;        % Fractional delay
outputLength = inputLength+delayInt; % Output signal length
dataOut = complex(zeros(outputLength,1));

% Perform delay operation
startInd = delayInt + 1;
if delayFrac
    nfft = 2^nextpow2(outputLength);
    binStart = floor(nfft/2);
    % Notice the FFT bins must belong to [-pi, pi]
    fftBin = 2*pi*ifftshift(((0:nfft-1)-binStart).')/nfft;
    tmpxd = fft(dataIn,nfft);
    tmpxd = ifft(tmpxd.*exp(-1i*delay*fftBin));
    dataOut(startInd:outputLength) = tmpxd(startInd:outputLength);
else
    % Integer sample shift
    dataOut(startInd:outputLength) = dataIn;
end
end