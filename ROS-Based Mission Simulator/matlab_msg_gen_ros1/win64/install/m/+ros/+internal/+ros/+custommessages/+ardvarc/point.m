function [data, info] = point
%Point gives an empty data for ardvarc/Point
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ardvarc/Point';
[data.Point_, info.Point_] = ros.internal.ros.messages.ros.default_type('double',3);
info.MessageType = 'ardvarc/Point';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'point';
