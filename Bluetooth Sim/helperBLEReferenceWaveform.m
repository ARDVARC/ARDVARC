function refSamples = helperBLEReferenceWaveform(phyMode,accessAddress,sps)
%helperBLEReferenceWaveform Generates Bluetooth LE reference samples
%   REFSAMPLES = helperBLEReferenceWaveform(PHYMODE,ACCESSADDRESS,SPS)
%   returns IQ samples corresponding to preamble and access address.
%
%   REFSAMPLES is a complex column vector which represents the Bluetooth LE
%   reference samples.
%
%   PHYMODE represents the uncoded physical layer transmission mode. This
%   value must be one of 'LE1M', 'LE2M', 'LE500K' and 'LE125K'.
%
%   ACCESSADDRES is a 32-bit column vector which represents the access
%   address.
%
%   SPS is a scalar which represents the samples per symbol.

%   Copyright 2021 The MathWorks, Inc.

preamble = ble.internal.preambleGenerator(phyMode,accessAddress);
refBits  = [preamble;accessAddress];
refSamples = ble.internal.gmskmod(refBits,sps);
end