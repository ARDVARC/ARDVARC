function [data, info] = flightPlan
%FlightPlan gives an empty data for ardvarc/FlightPlan
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ardvarc/FlightPlan';
[data.GoToCenter, info.GoToCenter] = ros.internal.ros.custommessages.ardvarc.point;
info.GoToCenter.MLdataType = 'struct';
[data.OrbitRadius, info.OrbitRadius] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LookAt, info.LookAt] = ros.internal.ros.custommessages.ardvarc.point;
info.LookAt.MLdataType = 'struct';
info.MessageType = 'ardvarc/FlightPlan';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'go_to_center';
info.MatPath{2} = 'go_to_center.point';
info.MatPath{3} = 'orbit_radius';
info.MatPath{4} = 'look_at';
info.MatPath{5} = 'look_at.point';
