function [y,varargout] = helperBLEWhiten(x,whitenInit)
%helperBLEwhiten Whiten/Dewhitens binary input
%   Y = helperBLEwhiten(X,WHITENINIT) whitens or dewhitens the binary
%   input X, by using the generator polynomial x^7+x^4+1. WHITENINIT is a
%   7-by-1 binary column vector of numeric type. The same function is used
%   to whiten at the transmitter and dewhiten at the receiver.
%
%   [Y,STATEOUT] = helperBLEwhiten(...) returns the final state of shift
%   register for use in whitening/dewhitening the future bits.
%
%   Reference - Bluetooth specifications version 5.0, vol-6, part-B,
%   section-3.2.

%   Copyright 2021 The MathWorks, Inc.

[y,stateOut] = ble.internal.whiten(x,whitenInit);
varargout{1} = stateOut;
end