function plotMeasuredDistanceVsTrueDistance()
    % close all
    
    [inputFile,inputPath] = uigetfile(".mat","Select Simulation Data File");
    if inputFile == 0
        return
    end
    load([inputPath,inputFile],"truthData","sensorData");

    times = sensorData.vec_times;
    trueUasPositions = interp1(truthData.vec_times,truthData.trix_vec_uasStates(:,1:2),times);

    measuredRgv1Positions = permute(sensorData.trix3_vec_rgvPositionMeasurement_enu(1:2,1,:),[1,3,2]);
    measuredRgv1Distances = vecnorm(trueUasPositions.'-measuredRgv1Positions);
    trueRgv1Positions = interp1(truthData.vec_times,truthData.trix_vec_rgv1Positions_enu(1:2,:).',times);
    trueRgv1Distances = vecnorm(trueUasPositions.'-trueRgv1Positions.');

    measuredRgv2Positions = permute(sensorData.trix3_vec_rgvPositionMeasurement_enu(1:2,2,:),[1,3,2]);
    measuredRgv2Distances = vecnorm(trueUasPositions.'-measuredRgv2Positions);
    trueRgv2Positions = interp1(truthData.vec_times,truthData.trix_vec_rgv2Positions_enu(1:2,:).',times);
    trueRgv2Distances = vecnorm(trueUasPositions.'-trueRgv2Positions.');

    trueRgvDistances = [trueRgv1Distances,trueRgv2Distances];
    measuredRgvDistances = [measuredRgv1Distances,measuredRgv2Distances];

    figure
    hold on
    grid minor
    axis equal
    scatter(measuredRgvDistances,trueRgvDistances,'.')

    % A = [ones(length(measuredRgvDistances),1),measuredRgvDistances.',(measuredRgvDistances.^2).',(measuredRgvDistances.^4).'];
    % B = trueRgvDistances.';
    % x = A\B
    % fplot(@(n) x(1)+x(2).*n+x(3).*n.^2+x(4).*n.^4,'k:')

    A = [measuredRgvDistances.'];
    B = trueRgvDistances.';
    x = A\B
    fplot(@(n) x(1).*n,'k--')
    fplot(@(n) n,'k--')
end