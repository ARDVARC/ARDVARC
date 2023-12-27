function [data, info] = vector
%Vector gives an empty data for ardvarc/Vector
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ardvarc/Vector';
[data.Vector_, info.Vector_] = ros.internal.ros.messages.ros.default_type('double',3);
info.MessageType = 'ardvarc/Vector';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'vector';
