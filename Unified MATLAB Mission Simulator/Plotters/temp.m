clc
clear
close all

[inputFile,inputPath] = uigetfile(".mat","Select Simulation Data File");
if inputFile == 0
    return
end
load([inputPath,inputFile]);

vec_times = predictorData.vec_times;
trix3_splineVars = permute(predictorData.trix4_splineVars(end,:,:,:), [2 3 4 1]);
trix_vec_rgv1Positions_en = interp1(truthData.vec_times, truthData.trix_vec_rgv1Positions_enu(1:2,:)', vec_times');
trix_vec_rgv2Positions_en = interp1(truthData.vec_times, truthData.trix_vec_rgv2Positions_enu(1:2,:)', vec_times');
trix_vec_uasPositions_en = interp1(truthData.vec_times, truthData.trix_vec_uasStates(:,1:2), vec_times');

% figure
% hold on
% grid minor
% % axis equal
% view(3)
% scatter3(permute(trix3_splineVars(1,1,:),[3,1,2]),permute(trix3_splineVars(2,1,:),[3,1,2]),vec_times,'ro')
% scatter3(trix_vec_rgv1Positions_en(:,1),trix_vec_rgv1Positions_en(:,2),vec_times,'r.')
% scatter3(permute(trix3_splineVars(1,2,:),[3,1,2]),permute(trix3_splineVars(2,2,:),[3,1,2]),vec_times,'bo')
% scatter3(trix_vec_rgv2Positions_en(:,1),trix_vec_rgv2Positions_en(:,2),vec_times,'b.')
% set(gca,'DataAspectRatio',[50,50,600])


figure
hold on
grid minor
rgv1Error = vecnorm(permute(trix3_splineVars(:,1,:),[3,1,2])-trix_vec_rgv1Positions_en,2,2);
rgv2Error = vecnorm(permute(trix3_splineVars(:,2,:),[3,1,2])-trix_vec_rgv2Positions_en,2,2);
plot(vec_times,rgv1Error)
plot(vec_times,rgv2Error)
maxErrorMoving = movmax(max(rgv1Error,rgv2Error),30,Endpoints=max(max(rgv1Error,rgv2Error)));
plot(vec_times,maxErrorMoving)


% figure
% hold on
% grid minor
% rgv1Error = vecnorm(permute(trix3_splineVars(:,1,:),[3,1,2])-trix_vec_rgv1Positions_en,2,2);
% rgv2Error = vecnorm(permute(trix3_splineVars(:,2,:),[3,1,2])-trix_vec_rgv2Positions_en,2,2);
% distUas2Rgv1 = vecnorm(trix_vec_uasPositions_en-trix_vec_rgv1Positions_en,2,2);
% distUas2Rgv2 = vecnorm(trix_vec_uasPositions_en-trix_vec_rgv2Positions_en,2,2);
% scatter(distUas2Rgv1,rgv1Error,'.')
% scatter(distUas2Rgv2,rgv2Error,'.')