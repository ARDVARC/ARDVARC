function [data, info] = eulers
%Eulers gives an empty data for ardvarc/Eulers
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ardvarc/Eulers';
[data.Eulers_, info.Eulers_] = ros.internal.ros.messages.ros.default_type('double',3);
info.MessageType = 'ardvarc/Eulers';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'eulers';
