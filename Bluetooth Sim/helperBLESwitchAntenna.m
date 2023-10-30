function dataOut = helperBLESwitchAntenna(dataIn,phyMode,sps,slotDuration,switchingPattern)
%helperBLESwitchAntenna Performs antenna switching
%   DATAOUT = helperBLESwitchAntenna(DATAIN,PHYMODE,SPS,SLOTDURATION,
%   SWITCHINGPATTERN) performs antenna switching on the data input, DATAIN.
%
%   DATAOUT is a complex column vector which represents the switched
%   Bluetooth LE waveform corresponding to CTE.
%
%   DATAIN is a complex matrix of size N-by-M, where N represents the
%   number of CTE samples and M represents the number of incoming signals.
%
%   PHYMODE represents the uncoded physical layer transmission mode. This
%   value must be either 'LE1M' or 'LE2M'.
%
%   SPS represents the samples per symbol.
%
%   SLOTDURATION specifies the sample slot duration in microseconds. This
%   value must be either 1 or 2.
%
%   SWITCHINGPATTERN represents the switching pattern as a 1xM row vector,
%   where M must be in the range [2,74/SlotDuration+1].

%   Copyright 2020-2021 The MathWorks, Inc.

% Compute samples length corresponding to guard and reference symbols
phyFactor = (1+strcmp(phyMode, 'LE2M'));
guardLen = 4; % Guard length
referencePeriodLen = 8; % Reference period length
antenna1SamplesLength = (guardLen+referencePeriodLen)*phyFactor*sps;

% Calculate number of switch and sample slots based on CTE length and slot
% duration
numSlots = (length(dataIn)/(sps*phyFactor)-guardLen-referencePeriodLen)/(2*slotDuration);

% Adjust switching pattern as per the Bluetooth Core Specification, version
% 5.1, volume 6, part A, section 5. During antenna switching, if the
% specified switching pattern is exhausted before the last sample slot,
% then the pattern is restarted from the beginning. If the pattern has not
% been completely used by the end of CTE, any remaining values shall be
% ignored.
if length(switchingPattern) <= numSlots
    repFactor = ceil(numSlots/length(switchingPattern))+1;
    switchingPattern = repmat(switchingPattern,1,repFactor);
end
switchingPattern = switchingPattern(1:numSlots+1);

% Use the first antenna in the switching pattern to collect the samples up
% to reference period
antenna1IQSamples = dataIn(1:antenna1SamplesLength,switchingPattern(1));

% After the reference period, switch the antennas based on switching
% pattern and collect IQ samples during the sample slots. Append zeros for
% the switch slot samples to construct the Bluetooth LE waveform with CTE.
slotPosition = 0;
switchSampleSlot = [];
sampleDurationFactor = (slotDuration*sps*phyFactor);
switchSlot = zeros(sampleDurationFactor, 1);
for i = 1:numSlots
    antennaInUse = switchingPattern(i+1);
    antennaInUseSamples = dataIn(:,antennaInUse);
    sampleSlot = antennaInUseSamples(antenna1SamplesLength+...
               sampleDurationFactor+slotPosition+(1:sampleDurationFactor));
    switchSampleSlot = [switchSampleSlot; switchSlot; sampleSlot]; %#ok<AGROW>
    slotPosition = slotPosition+2*sampleDurationFactor;
end

% Append switch and sample slot samples to the first antenna samples
% corresponding to CTE
dataOut = [antenna1IQSamples;switchSampleSlot];
end