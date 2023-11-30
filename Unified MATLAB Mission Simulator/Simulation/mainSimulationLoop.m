function [rawTruthData, sensorData, predictorData] = mainSimulationLoop(simParams, rgv1, rgv2)
    % Runs a UAS physics simulation loop while periodically updating the
    % estimated state with new sensor data
    arguments(Input)
        simParams (1,1) SimParams
        rgv1 (1,1) RGV
        rgv2 (1,1) RGV
    end
    
    splineCount = 3;
    nodeGap = 10;

    %% Prepare interupts
    % Flight plan inturupts
    vec_flightPlanInteruptTimes = (nodeGap*splineCount+1/simParams.bluetoothRate):(1/simParams.flightPlanRate):simParams.duration;
    flightPlanInteruptCount = size(vec_flightPlanInteruptTimes, 2);
    vec_flightPlanInteruptKinds(1:flightPlanInteruptCount) = SimulationInteruptKind.FlightPlan;

    % Bluetooth sensor interupts
    vec_bluetoothInteruptTimes = 0:(1/simParams.bluetoothRate):simParams.duration;
    bluetoothInteruptCount = size(vec_bluetoothInteruptTimes, 2);
    vec_bluetoothInteruptKinds(1:bluetoothInteruptCount) = SimulationInteruptKind.Bluetooth;

    % Sort all interupts
    vec_allInteruptTimes = [vec_bluetoothInteruptTimes vec_flightPlanInteruptTimes];
    vec_allInteruptKinds = [vec_bluetoothInteruptKinds vec_flightPlanInteruptKinds];
    [vec_sortedInteruptTimes, vec_sortIndicies] = sort(vec_allInteruptTimes);
    vec_sortedInteruptKinds = vec_allInteruptKinds(vec_sortIndicies);
    interuptCount = size(vec_sortedInteruptTimes, 2);

    %% Run simulation loop
    vec_times = 0;
    trix_vec_uasStates = simParams.vec_uasStartState';
    vec_goToCenter_enu = [0;0;-simParams.targetUasHeight];
    vec_lookAt_en = [0;0];
    orbitRadius = 10;

    hasLocalizedRgv1 = false;
    rgv1LocalizeAmount = 0;
    hasLocalizedRgv2 = false;
    rgv2LocalizeAmount = 0;
    amLocalizingRgv2 = false;

    bluetoothInteruptCounter = 0;
    flightPlanInteruptCounter = 0;

    trix3_vec_rgvPositionEstimate_enu = zeros(3,2,bluetoothInteruptCount);
    trix4_splineVars = zeros(splineCount+1,2,2,flightPlanInteruptCount);
    vec_flightPlanTypes = zeros(flightPlanInteruptCount,1,"uint8");
    
    for i = 1:interuptCount
        nextTime = vec_sortedInteruptTimes(i);
        
        % Handle interupt
        switch (vec_sortedInteruptKinds(i))
            case (SimulationInteruptKind.FlightPlan)
                flightPlanInteruptCounter = flightPlanInteruptCounter + 1;
                % Fit cubic spline to recent measurements for both RGVs
                endIndex = bluetoothInteruptCounter;
                startIndex = endIndex - nodeGap*splineCount*simParams.bluetoothRate;
                trix_vec_projectedGroundPositionToFit_en = trix3_vec_rgvPositionEstimate_enu(1:2,:,startIndex:endIndex);
                vec_trueTimeToFit = vec_bluetoothInteruptTimes(startIndex:endIndex);
                startTime = vec_trueTimeToFit(1);
                vec_timeToFit = vec_trueTimeToFit - startTime;
    
                vec_splineIndex = floor(vec_timeToFit/nodeGap);
                vec_percentInSpline = (vec_timeToFit - vec_splineIndex*nodeGap)/nodeGap;
                vec_splineIndex(end) = 1;
                vec_percentInSpline(end) = 1;
                trix_timeMatrix = zeros(length(vec_timeToFit),2*splineCount);
                for j = 1:splineCount
                    vec_timesInThisSpline = vec_percentInSpline(vec_splineIndex==(j-1))';
                    trix_timeMatrix(vec_splineIndex==(j-1),(2*j-1):(2*j)) = [ones(length(vec_timesInThisSpline),1),vec_timesInThisSpline];
                end
    
                trix_characteristicPiece = [ 1, 0
                                            -1, 1];
                trix_characteristicMatrix = zeros(2*(splineCount-1),splineCount+1);
                for j = 1:splineCount
                    trix_characteristicMatrix(j*2-1:j*2,j:j+1) = trix_characteristicPiece;
                end
    
                trix_vec_rgv1SplineKnots_en = (trix_timeMatrix*trix_characteristicMatrix)\permute(trix_vec_projectedGroundPositionToFit_en(:,1,:),[3,1,2]);
                trix_vec_rgv2SplineKnots_en = (trix_timeMatrix*trix_characteristicMatrix)\permute(trix_vec_projectedGroundPositionToFit_en(:,2,:),[3,1,2]);
                trix4_splineVars(:,:,1,flightPlanInteruptCounter) = trix_vec_rgv1SplineKnots_en;
                trix4_splineVars(:,:,2,flightPlanInteruptCounter) = trix_vec_rgv2SplineKnots_en;

                % Determine if each RGV is standing still
                rgv1IsStill = norm(trix_vec_rgv1SplineKnots_en(end-1,:)-trix_vec_rgv1SplineKnots_en(end,:))/nodeGap < 0.2;
                rgv2IsStill = norm(trix_vec_rgv2SplineKnots_en(end-1,:)-trix_vec_rgv2SplineKnots_en(end,:))/nodeGap < 0.2;
                if ~rgv2IsStill
                    amLocalizingRgv2 = false;
                end

                % Figure out what to do
                if(vec_times(end) + max(20, 2/simParams.flightPlanRate) > simParams.duration)
                    % Go home
                    vec_flightPlanTypes(flightPlanInteruptCounter) = 1;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= 1
                        disp("Going Home!")
                    end
                    vec_lookAt_en = simParams.vec_uasStartState(1:2)*2;
                    vec_goToCenter_enu = simParams.vec_uasStartState(1:3);
                elseif(~amLocalizingRgv2 && ~hasLocalizedRgv1 && rgv1IsStill)
                    % Orbit RGV 1
                    vec_flightPlanTypes(flightPlanInteruptCounter) = 2;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= 2
                        disp("Orbiting RGV 1...")
                    end
                    vec_lookAt_en = trix_vec_rgv1SplineKnots_en(end,:)';
                    vec_goToCenter_enu = [trix_vec_rgv1SplineKnots_en(end,:),-simParams.targetUasHeight]';
                    orbitRadius = simParams.orbitRadius;
                    rgv1LocalizeAmount = rgv1LocalizeAmount + 1/simParams.flightPlanRate;
                    if (rgv1LocalizeAmount > 60)
                        hasLocalizedRgv1 = true;
                        disp("RGV 1 Localized!")
                    end
                elseif(~hasLocalizedRgv2 && rgv2IsStill)
                    % Orbit RGV 2
                    vec_flightPlanTypes(flightPlanInteruptCounter) = 3;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= 3
                        disp("Orbiting RGV 2...")
                    end
                    amLocalizingRgv2 = true;
                    vec_lookAt_en = trix_vec_rgv2SplineKnots_en(end,:)';
                    vec_goToCenter_enu = [trix_vec_rgv2SplineKnots_en(end,:),-simParams.targetUasHeight]';
                    orbitRadius = simParams.orbitRadius;
                    rgv2LocalizeAmount = rgv2LocalizeAmount + 1/simParams.flightPlanRate;
                    if (rgv2LocalizeAmount > 60)
                        hasLocalizedRgv2 = true;
                        amLocalizingRgv2 = false;
                        disp("RGV 2 Localized!")
                    end
                elseif(~hasLocalizedRgv1)
                    % Trail RGV 1
                    vec_flightPlanTypes(flightPlanInteruptCounter) = 4;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= 4
                        disp("Trailing RGV 1...")
                    end
                    vec_rgv2uas_en = trix_vec_uasStates(end,1:2)-trix_vec_rgv1SplineKnots_en(end,:);
                    vec_rgv2uasPointing_en = vec_rgv2uas_en/norm(vec_rgv2uas_en);
                    vec_lookAt_en = trix_vec_rgv1SplineKnots_en(end,:)';
                    vec_goToCenter_enu = [trix_vec_rgv1SplineKnots_en(end,:) + vec_rgv2uasPointing_en*simParams.targetRgvTrailDistance,-simParams.targetUasHeight]';
                    orbitRadius = 0;
                elseif(~hasLocalizedRgv2)
                    % Trail RGV 2
                    vec_flightPlanTypes(flightPlanInteruptCounter) = 5;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= 5
                        disp("Trailing RGV 2...")
                    end
                    vec_rgv2uas_en = trix_vec_uasStates(end,1:2)-trix_vec_rgv2SplineKnots_en(end,:);
                    vec_rgv2uasPointing_en = vec_rgv2uas_en/norm(vec_rgv2uas_en);
                    vec_lookAt_en = trix_vec_rgv2SplineKnots_en(end,:)';
                    vec_goToCenter_enu = [trix_vec_rgv2SplineKnots_en(end,:) + vec_rgv2uasPointing_en*simParams.targetRgvTrailDistance,-simParams.targetUasHeight]';
                    orbitRadius = 0;
                else
                    % Joint
                    vec_flightPlanTypes(flightPlanInteruptCounter) = 6;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= 6
                        disp("Joint Localizing...")
                    end
                    vec_rgv12rvg2_en = trix_vec_rgv2SplineKnots_en(end,:) - trix_vec_rgv1SplineKnots_en(end,:);
                    vec_rgv12rvg2dir_en = vec_rgv12rvg2_en/norm(vec_rgv12rvg2_en);
                    vec_lookAt_en = trix_vec_rgv1SplineKnots_en(end,:)';
                    vec_goToCenter_enu = [trix_vec_rgv1SplineKnots_en(end,:)+vec_rgv12rvg2dir_en*simParams.orbitRadius,-simParams.targetUasHeight]';
                    orbitRadius = 0;
                end
            case (SimulationInteruptKind.Bluetooth)
                bluetoothInteruptCounter = bluetoothInteruptCounter + 1;
                % Measure pointing vectors in bluetooth frame
                vec_uasState = trix_vec_uasStates(end,:);
                vec_pointingVecToRgv1_bluetooth = recordBluetooth(simParams, vec_uasState, getRgvStateAtTime(simParams.rgvParams, rgv1, vec_times(end)));
                vec_pointingVecToRgv2_bluetooth = recordBluetooth(simParams, vec_uasState, getRgvStateAtTime(simParams.rgvParams, rgv2, vec_times(end)));
                trix_vec_pointingVec_bluetooth = [vec_pointingVecToRgv1_bluetooth,vec_pointingVecToRgv2_bluetooth];
                % Convert vectors to ENU
                trix_vec_pointingVec_enu = getDcmUas2Enu(vec_uasState) * simParams.dcm_bluetooth2uas * trix_vec_pointingVec_bluetooth;
                % Get RGV position estimates
                altitude = vec_uasState(3);
                coeffs = altitude./-trix_vec_pointingVec_enu(3,:);
                trix3_vec_rgvPositionEstimate_enu(:,:,bluetoothInteruptCounter) = vec_uasState(1:3)'+coeffs.*trix_vec_pointingVec_enu;
                trix3_vec_rgvPositionEstimate_enu(1:2,:,bluetoothInteruptCounter) = trix3_vec_rgvPositionEstimate_enu(1:2,:,bluetoothInteruptCounter);
        end

        % Check if there is more physics to do before next interupt
        if (abs(nextTime-vec_times(end)) < 0.01)
            continue;
        end

        % Continue physics simulation
        [vec_newTime, trix_vec_newTrueUasState] = simulateUasPhysics(simParams, vec_goToCenter_enu, vec_lookAt_en, orbitRadius, vec_times(end), nextTime, trix_vec_uasStates(end,:)');
        newValueCount = size(vec_newTime,1)-1;
        vec_times(end+1:end+newValueCount,:) = vec_newTime(2:end);
        trix_vec_uasStates(end+1:end+newValueCount,:) = trix_vec_newTrueUasState(2:end,:);
    end

    rawTruthData.vec_times = vec_times;
    rawTruthData.trix_vec_uasStates = trix_vec_uasStates;
    sensorData.vec_times = vec_bluetoothInteruptTimes;
    sensorData.trix3_vec_rgvPositionEstimate_enu = trix3_vec_rgvPositionEstimate_enu;
    predictorData.vec_times = vec_flightPlanInteruptTimes;
    predictorData.trix4_splineVars = trix4_splineVars;
end