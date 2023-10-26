function [cteTime,cteType] = helperBLECTEInfoExtract(bits,dfPacketType)
%helperBLECTEInfoExtract Extracts the constant tone extension (CTE)
%information
%   [CTETIME, CTETYPE] = helperBLECTEInfoExtract(BITS, DFPACKETTYPE)
%   extracts CTETIME and CTETYPE values from the given bits, BITS.
%
%   CTETIME is an integer of type int8, which defines the length of CTE in
%   8 microseconds units. The value of CTETIME must be between 2 and 20,
%   inclusive.
%
%   CTETYPE is an integer of type int8, which defines the type of CTE and
%   the duration of the switch slots. This value must be either 0,1 or 2.
%
%   BITS is a column vector of numeric or logical values, which contains
%   CTEInfo field if CTE is enabled.
%
%   DFPACKETTYPE is a character vector or a string scalar specifying the
%   direction finding packet type. This value must be either
%   'ConnectionCTE' or 'ConnectionlessCTE'.

%   Copyright 2021 The MathWorks, Inc.

[cteTime,cteType] = ble.internal.cteInfoExtract(bits,dfPacketType);
end