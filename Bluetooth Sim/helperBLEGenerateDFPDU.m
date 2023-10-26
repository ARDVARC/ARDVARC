function dfPDU = helperBLEGenerateDFPDU(dfPacketType,cteLength,...
                                            cteType,payloadLength,crcInit)
%helperBLEGenerateDFPDU Generates direction finding packet pdu
%   DFPDU = helperBLEGenerateDFPDU(DFPACKETTYPE,CTELENGTH,CTETYPE,...
%   PAYLOADLENGTH,CRCINIT) generates direction finding packet pdu, DFPDU,
%   corresponding to connection or connectionless communication.
%
%   DFPDU represents direction finding packet pdu.
%
%   DFPACKETTYPE represents direction finding packet type, which can be
%   either ConnectionCTE or ConnectionlessCTE.
%
%   CTELENGTH represents the length of CTE in microseconds, must be in the
%   range [20, 160].
%
%   CTETYPE represents the type of CTE and duration of switch and sample
%   slots, must be [0;0], [0;1] or [1;0].
%
%   PAYLOADLENGTH represents the length of payload in bytes.
%
%   CRCINIT is the initialization value for CRC calculation, specified as a
%   6-element character vector.

%   Copyright 2020-2021 The MathWorks, Inc.

% Derive CTE field information using CTE length and CTE type information
cteLengthField = int2bit(cteLength/8, 5, false);
cteInfo = [cteLengthField;0;cteType];

% Generate PDU
if strcmp(dfPacketType, 'ConnectionCTE') % PDU type is LL_CTE_RSP
    payloadLengthBits = int2bit(payloadLength, 8, false);
    pduHeader = [randi([0 1], 8, 1);payloadLengthBits;cteInfo];
    pdu = [pduHeader;randi([0 1], payloadLength*8, 1)];
else % For ConnectionlessCTE, CTE is present in the AUX_SYNC_IND PDU
    payloadLengthBits = int2bit(payloadLength, 8, false);
    pduHeader = [randi([0 1], 8, 1);payloadLengthBits];
    extHeaderLength = randi([0 1], 6, 1);
    advMode = [0;0];
    extHeaderFlag = [0;0;1;0;randi([0 1], 1, 1);0;randi([0 1], 2, 1)];
    remPayloadLength = payloadLength*8-length(extHeaderLength)-length(advMode)...
                        -length(extHeaderFlag)-length(cteInfo);
    pdu = [pduHeader;extHeaderLength;advMode;extHeaderFlag;cteInfo;randi([0 1], remPayloadLength, 1)];
end

% Append CRC to the PDU
crcInitBinary = int2bit(hex2dec(crcInit), 24)';
persistent crcGen
if isempty(crcGen)
    crcPolynomial = 'x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1';
    crcGen = comm.CRCGenerator(crcPolynomial, 'InitialConditions', ...
    crcInitBinary, 'DirectMethod', true);
else
    if any(crcGen.InitialConditions ~= crcInitBinary)
        release(crcGen)
        crcGen.InitialConditions = int2bit(hex2dec(crcInit), 24)';
    end
end
dfPDU = crcGen(pdu);
end