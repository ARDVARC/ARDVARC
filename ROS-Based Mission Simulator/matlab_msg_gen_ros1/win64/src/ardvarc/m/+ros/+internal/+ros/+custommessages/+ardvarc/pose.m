function [data, info] = pose
%Pose gives an empty data for ardvarc/Pose
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ardvarc/Pose';
[data.Position, info.Position] = ros.internal.ros.custommessages.ardvarc.point;
info.Position.MLdataType = 'struct';
[data.Eulers, info.Eulers] = ros.internal.ros.custommessages.ardvarc.eulers;
info.Eulers.MLdataType = 'struct';
info.MessageType = 'ardvarc/Pose';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'position';
info.MatPath{2} = 'position.point';
info.MatPath{3} = 'eulers';
info.MatPath{4} = 'eulers.eulers';
