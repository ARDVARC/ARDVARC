classdef RGV < matlab.System
    properties (Access = private)
        currentPathSegement (1,1) PathSegment = PathSegment();
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.currentPathSegement = PathSegment();
        end

        function [pos, rotm] = stepImpl(obj,time)
            while (obj.currentPathSegement.endTime < time)
                [pos, rotm] = obj.currentPathSegement.getStateAlongPathAt(obj.currentPathSegement.endTime);
                obj.currentPathSegement = obj.currentPathSegement.change(pos, rotm);
            end
            [pos, rotm] = obj.currentPathSegement.getStateAlongPathAt(time);
        end
    end
end
