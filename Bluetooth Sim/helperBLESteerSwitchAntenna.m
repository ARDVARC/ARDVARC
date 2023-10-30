function dataOut = helperBLESteerSwitchAntenna(dataIn,angle,phyMode,sps,...
                                                dfPacketType,pduLen,cfg)
%helperBLESteerSwitchAntenna Performs antenna steering and switching
%   DATAOUT = helperBLESteerSwitchAntenna(DATAIN,ANGLE,PHYMODE,SPS,...
%   DFPACKETTYPE,PDULEN,CTELEN,CFG) performs antenna steering and switching
%   on the data input, DATAIN.
%
%   DATAOUT is a complex column vector which represents the steered and
%   switched Bluetooth LE waveform.
%
%   DATAIN is a complex column vector which represents the Bluetooth LE
%   waveform.
%
%   ANGLE is a scalar or a vector of size 1-by-2. A scalar ANGLE represents
%   the azimuthal angle. The first and second value of the 1-by-2 vector
%   ANGLE represents the azimuth and elevation angles, respectively.
%
%   PHYMODE represents the uncoded physical layer transmission mode. This
%   value must be either 'LE1M' or 'LE2M'.
%
%   SPS represents the samples per symbol.
%
%   DFPACKETTYPE represents direction finding packet type, which can be
%   either ConnectionCTE or ConnectionlessCTE.
%
%   PDULENGTH represents the length of pdu in bytes.
%
%   CFG is a configuration object of type <a
%   href="matlab:help('bleAngleEstimateConfig')">bleAngleEstimateConfig</a>
%   that configures the properties required for the Bluetooth LE angle
%   estimation function.

%   Copyright 2021 The MathWorks, Inc.

% Generate steering vector with known angles
pos = getElementPosition(cfg);
if size(angle,2)==1
    angle(2) = 0;
end
steerVec = ble.internal.steeringVector(angle(1),angle(2), pos);

% Steer the waveform
rotatedWaveform = dataIn .* steerVec.';

% Variables initialization
phyFactor = (1+strcmp(phyMode, 'LE2M'));
preambleLen = 8*phyFactor; % Preamble length
accAddLen = 32; % Access address length
crcLen = 24; % CRC length
headerLen = 16+8*strcmp(dfPacketType, 'ConnectionCTE'); % Header length

% Consider the length of waveform till CTE
antenna1SamplesTillCTE = (preambleLen+accAddLen+headerLen+(pduLen*8)+crcLen)*sps;
waveformTillCTE = rotatedWaveform(1:antenna1SamplesTillCTE,1);

% Perform antenna switch on CTE
cteRotatedSamples = rotatedWaveform(antenna1SamplesTillCTE+1:end,:);
cteSamples = helperBLESwitchAntenna(cteRotatedSamples,phyMode,sps,cfg.SlotDuration,cfg.SwitchingPattern);

% Append reconstructed CTE samples to the first antenna samples till CTE to
% construct the Bluetooth LE waveform with CTE
dataOut = [waveformTillCTE;cteSamples];

end