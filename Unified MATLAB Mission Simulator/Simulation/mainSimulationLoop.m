function [rawTruthData, sensorData, predictorData] = mainSimulationLoop(simParams, rgv1, rgv2)
    % Runs a UAS physics simulation loop while periodically updating the
    % estimated state with new sensor data
    % arguments(Input)
    %     simParams (1,1) SimParams
    %     rgv1 (1,1) RGV
    %     rgv2 (1,1) RGV
    % end

    %% Prepare interupts
    % Flight plan inturupts
    vec_flightPlanInteruptTimes = (simParams.nodeGap*simParams.splineCount+1/simParams.bluetoothRate):(1/simParams.flightPlanRate):simParams.duration;
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
    vec_times = zeros(10000*simParams.duration,1);
    trix_vec_uasStates = zeros(10000*simParams.duration,12);
    vec_times(1) = 0;
    trix_vec_uasStates(1,:) = simParams.vec_uasStartState';
    truthDataIndex = 1;
    trix_vec_goToCenter_enu = zeros(3,flightPlanInteruptCount+1);
    trix_vec_goToCenter_enu(:,1) = [0;0;-simParams.targetUasHeight];
    trix_vec_lookAt_en = zeros(2,flightPlanInteruptCount+1);
    vec_orbitRadius = zeros(1,flightPlanInteruptCount+1);
    vec_orbitRadius(1) = 10;

    hasLocalizedRgv1 = false;
    rgv1LocalizeAmount = 0;
    hasLocalizedRgv2 = false;
    rgv2LocalizeAmount = 0;
    amLocalizingRgv2 = false;

    bluetoothInteruptCounter = 0;
    flightPlanInteruptCounter = 1;

    trix3_vec_rgvPositionMeasurement_enu = zeros(3,2,bluetoothInteruptCount);
    trix4_splineVars = zeros(simParams.splineCount+1,2,2,flightPlanInteruptCount+1);
    trix4_splineVars(:,:,:,1) = NaN;
    vec_flightPlanTypes = FlightPlanState.empty();
    vec_flightPlanTypes(flightPlanInteruptCount) = FlightPlanState.GoHome;
    
    for i = 1:interuptCount
        % Handle interupt
        switch (vec_sortedInteruptKinds(i))
            case (SimulationInteruptKind.FlightPlan)
                flightPlanInteruptCounter = flightPlanInteruptCounter + 1;
                % Fit cubic spline to recent measurements for both RGVs
                endIndex = bluetoothInteruptCounter;
                startIndex = endIndex - simParams.nodeGap*simParams.splineCount*simParams.bluetoothRate;
                trix_vec_projectedGroundPositionToFit_en = trix3_vec_rgvPositionMeasurement_enu(1:2,:,startIndex:endIndex);
                vec_trueTimeToFit = vec_bluetoothInteruptTimes(startIndex:endIndex);
                startTime = vec_sortedInteruptTimes(i) - simParams.nodeGap*simParams.splineCount;
                vec_timeToFit = vec_trueTimeToFit - startTime;
    
                vec_splineIndex = floor(vec_timeToFit/simParams.nodeGap);
                vec_percentInSpline = (vec_timeToFit - vec_splineIndex*simParams.nodeGap)/simParams.nodeGap;
                vec_splineIndex(end) = simParams.splineCount-1;
                vec_percentInSpline(end) = 1;
                trix_timeMatrix = zeros(length(vec_timeToFit),2*simParams.splineCount);
                for j = 1:simParams.splineCount
                    vec_timesInThisSpline = vec_percentInSpline(vec_splineIndex==(j-1))';
                    trix_timeMatrix(vec_splineIndex==(j-1),(2*j-1):(2*j)) = [ones(length(vec_timesInThisSpline),1),vec_timesInThisSpline];
                end
    
                trix_characteristicPiece = [ 1, 0
                                            -1, 1];
                trix_characteristicMatrix = zeros(2*(simParams.splineCount-1),simParams.splineCount+1);
                for j = 1:simParams.splineCount
                    trix_characteristicMatrix(j*2-1:j*2,j:j+1) = trix_characteristicPiece;
                end
    
                trix_vec_rgv1SplineKnots_en = (trix_timeMatrix*trix_characteristicMatrix)\permute(trix_vec_projectedGroundPositionToFit_en(:,1,:),[3,1,2]);
                trix_vec_rgv2SplineKnots_en = (trix_timeMatrix*trix_characteristicMatrix)\permute(trix_vec_projectedGroundPositionToFit_en(:,2,:),[3,1,2]);
                trix4_splineVars(:,:,1,flightPlanInteruptCounter) = trix_vec_rgv1SplineKnots_en;
                trix4_splineVars(:,:,2,flightPlanInteruptCounter) = trix_vec_rgv2SplineKnots_en;

                % Determine if each RGV is standing still
                rgv1IsStill = norm(trix_vec_rgv1SplineKnots_en(end-1,:)-trix_vec_rgv1SplineKnots_en(end,:))/simParams.nodeGap < simParams.stillSpeedTolerance;
                rgv2IsStill = norm(trix_vec_rgv2SplineKnots_en(end-1,:)-trix_vec_rgv2SplineKnots_en(end,:))/simParams.nodeGap < simParams.stillSpeedTolerance;
                if ~rgv2IsStill
                    amLocalizingRgv2 = false;
                end

                if ~hasLocalizedRgv1 && ~rgv1IsStill && rgv1LocalizeAmount > 0
                    if rgv1LocalizeAmount > simParams.localizeMinDuration
                        hasLocalizedRgv1 = true;
                        disp("RGV 1 Localized! (Sufficient)")
                    else
                        rgv1LocalizeAmount = 0;
                        disp("RGV 1 Localize Attempt Abandoned")
                    end
                end
                if ~hasLocalizedRgv2 && ~rgv2IsStill && rgv2LocalizeAmount > 0
                    if rgv2LocalizeAmount > simParams.localizeMinDuration
                        hasLocalizedRgv2 = true;
                        disp("RGV 2 Localized! (Sufficient)")
                    else
                        rgv2LocalizeAmount = 0;
                        disp("RGV 2 Localize Attempt Abandoned")
                    end
                end

                % Figure out what to do
                if(vec_times(truthDataIndex) + max(20, 2/simParams.flightPlanRate) > simParams.duration)
                    % Go home
                    vec_flightPlanTypes(flightPlanInteruptCounter) = FlightPlanState.GoHome;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= FlightPlanState.GoHome
                        disp("Going Home!")
                    end
                    trix_vec_lookAt_en(:,flightPlanInteruptCounter) = simParams.vec_uasStartState(1:2)*2;
                    trix_vec_goToCenter_enu(:,flightPlanInteruptCounter) = simParams.vec_uasStartState(1:3);
                    vec_orbitRadius(flightPlanInteruptCounter) = 0;
                elseif(~amLocalizingRgv2 && ~hasLocalizedRgv1 && rgv1IsStill)
                    % Orbit RGV 1
                    vec_flightPlanTypes(flightPlanInteruptCounter) = FlightPlanState.OrbitRGV1;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= FlightPlanState.OrbitRGV1
                        disp("Orbiting RGV 1...")
                    end
                    trix_vec_lookAt_en(:,flightPlanInteruptCounter) = trix_vec_rgv1SplineKnots_en(end,:)';
                    trix_vec_goToCenter_enu(:,flightPlanInteruptCounter) = [trix_vec_rgv1SplineKnots_en(end,:),-simParams.targetUasHeight]';
                    vec_orbitRadius(flightPlanInteruptCounter) = simParams.orbitRadius;
                    rgv1LocalizeAmount = rgv1LocalizeAmount + 1/simParams.flightPlanRate;
                    if (rgv1LocalizeAmount >= simParams.localizeTargetDuration)
                        hasLocalizedRgv1 = true;
                        disp("RGV 1 Localized!")
                    end
                elseif(~hasLocalizedRgv2 && rgv2IsStill)
                    % Orbit RGV 2
                    vec_flightPlanTypes(flightPlanInteruptCounter) = FlightPlanState.OrbitRGV2;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= FlightPlanState.OrbitRGV2
                        disp("Orbiting RGV 2...")
                    end
                    amLocalizingRgv2 = true;
                    trix_vec_lookAt_en(:,flightPlanInteruptCounter) = trix_vec_rgv2SplineKnots_en(end,:)';
                    trix_vec_goToCenter_enu(:,flightPlanInteruptCounter) = [trix_vec_rgv2SplineKnots_en(end,:),-simParams.targetUasHeight]';
                    vec_orbitRadius(flightPlanInteruptCounter) = simParams.orbitRadius;
                    rgv2LocalizeAmount = rgv2LocalizeAmount + 1/simParams.flightPlanRate;
                    if (rgv2LocalizeAmount >= simParams.localizeTargetDuration)
                        hasLocalizedRgv2 = true;
                        amLocalizingRgv2 = false;
                        disp("RGV 2 Localized!")
                    end
                elseif(~hasLocalizedRgv1)
                    % Trail RGV 1
                    vec_flightPlanTypes(flightPlanInteruptCounter) = FlightPlanState.TrailRGV1;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= FlightPlanState.TrailRGV1
                        disp("Trailing RGV 1...")
                    end
                    vec_rgv2uas_en = trix_vec_uasStates(truthDataIndex,1:2)-trix_vec_rgv1SplineKnots_en(end,:);
                    vec_rgv2uasPointing_en = vec_rgv2uas_en/norm(vec_rgv2uas_en);
                    trix_vec_lookAt_en(:,flightPlanInteruptCounter) = trix_vec_rgv1SplineKnots_en(end,:)';
                    trix_vec_goToCenter_enu(:,flightPlanInteruptCounter) = [trix_vec_rgv1SplineKnots_en(end,:) + vec_rgv2uasPointing_en*simParams.targetRgvTrailDistance,-simParams.targetUasHeight]';
                    vec_orbitRadius(flightPlanInteruptCounter) = 0;
                elseif(~hasLocalizedRgv2)
                    % Trail RGV 2
                    vec_flightPlanTypes(flightPlanInteruptCounter) = FlightPlanState.TrailRGV2;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= FlightPlanState.TrailRGV2
                        disp("Trailing RGV 2...")
                    end
                    vec_rgv2uas_en = trix_vec_uasStates(truthDataIndex,1:2)-trix_vec_rgv2SplineKnots_en(end,:);
                    vec_rgv2uasPointing_en = vec_rgv2uas_en/norm(vec_rgv2uas_en);
                    trix_vec_lookAt_en(:,flightPlanInteruptCounter) = trix_vec_rgv2SplineKnots_en(end,:)';
                    trix_vec_goToCenter_enu(:,flightPlanInteruptCounter) = [trix_vec_rgv2SplineKnots_en(end,:) + vec_rgv2uasPointing_en*simParams.targetRgvTrailDistance,-simParams.targetUasHeight]';
                    vec_orbitRadius(flightPlanInteruptCounter) = 0;
                elseif flightPlanInteruptCounter < simParams.jointFocusTime*simParams.flightPlanRate || vec_flightPlanTypes(ceil(flightPlanInteruptCounter+1-simParams.jointFocusTime*simParams.flightPlanRate)) ~= FlightPlanState.JointFocusRGV1
                    % Joint, Focus RGV 1
                    vec_flightPlanTypes(flightPlanInteruptCounter) = FlightPlanState.JointFocusRGV1;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= FlightPlanState.JointFocusRGV1
                        disp("Joint Localizing (Focus RGV 1) ...")
                    end
                    vec_rgv12rgv2_en = trix_vec_rgv2SplineKnots_en(end,:)-trix_vec_rgv1SplineKnots_en(end,:);
                    vec_rgv12rgv2Pointing_en = vec_rgv12rgv2_en/norm(vec_rgv12rgv2_en);
                    trix_vec_lookAt_en(:,flightPlanInteruptCounter) = trix_vec_rgv1SplineKnots_en(end,:)';
                    trix_vec_goToCenter_enu(:,flightPlanInteruptCounter) = [trix_vec_rgv1SplineKnots_en(end,:) + vec_rgv12rgv2Pointing_en*(rgv1IsStill*simParams.orbitRadius+~rgv1IsStill*simParams.targetRgvTrailDistance),-simParams.targetUasHeight]';
                    vec_orbitRadius(flightPlanInteruptCounter) = 0;
                else
                    % Joint, Focus RGV 2
                    vec_flightPlanTypes(flightPlanInteruptCounter) = FlightPlanState.JointFocusRGV2;
                    if flightPlanInteruptCounter == 1 || vec_flightPlanTypes(flightPlanInteruptCounter-1) ~= FlightPlanState.JointFocusRGV2
                        disp("Joint Localizing (Focus RGV 2) ...")
                    end
                    vec_rgv22rgv1_en = trix_vec_rgv1SplineKnots_en(end,:)-trix_vec_rgv2SplineKnots_en(end,:);
                    vec_rgv22rgv1Pointing_en = vec_rgv22rgv1_en/norm(vec_rgv22rgv1_en);
                    trix_vec_lookAt_en(:,flightPlanInteruptCounter) = trix_vec_rgv2SplineKnots_en(end,:)';
                    trix_vec_goToCenter_enu(:,flightPlanInteruptCounter) = [trix_vec_rgv2SplineKnots_en(end,:) + vec_rgv22rgv1Pointing_en*(rgv2IsStill*simParams.orbitRadius+~rgv2IsStill*simParams.targetRgvTrailDistance),-simParams.targetUasHeight]';
                    vec_orbitRadius(flightPlanInteruptCounter) = 0;
                end
            case (SimulationInteruptKind.Bluetooth)
                bluetoothInteruptCounter = bluetoothInteruptCounter + 1;
                % Measure pointing vectors in bluetooth frame
                vec_uasState = trix_vec_uasStates(truthDataIndex,:);
                vec_pointingVecToRgv1_bluetooth = recordBluetooth(simParams, vec_uasState, getRgvStateAtTime(simParams.rgvParams, rgv1, vec_times(truthDataIndex)));
                vec_pointingVecToRgv2_bluetooth = recordBluetooth(simParams, vec_uasState, getRgvStateAtTime(simParams.rgvParams, rgv2, vec_times(truthDataIndex)));
                trix_vec_pointingVec_bluetooth = [vec_pointingVecToRgv1_bluetooth,vec_pointingVecToRgv2_bluetooth];
                % Convert vectors to ENU
                phi = vec_uasState(4);
                theta = vec_uasState(5);
                psi = vec_uasState(6);
                dcm_uas2enu = [cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                               cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                               -sin(theta)        ,sin(phi)*cos(theta)                           ,cos(phi)*cos(theta)                           ];
                trix_vec_pointingVec_enu = dcm_uas2enu * simParams.dcm_bluetooth2uas * trix_vec_pointingVec_bluetooth;
                % Get RGV position estimates
                altitude = vec_uasState(3);
                coeffs = altitude./-trix_vec_pointingVec_enu(3,:);
                trix3_vec_rgvPositionMeasurement_enu(:,:,bluetoothInteruptCounter) = vec_uasState(1:3)'+coeffs.*trix_vec_pointingVec_enu;
                % Adjust by linear distortion factor
                trix3_vec_rgvPositionMeasurement_enu(:,:,bluetoothInteruptCounter) = 0.025*[vec_uasState(1:2),0].'+0.975*trix3_vec_rgvPositionMeasurement_enu(:,:,bluetoothInteruptCounter);
                % Clamp to mission area
                trix3_vec_rgvPositionMeasurement_enu(:,:,bluetoothInteruptCounter) = max(min(trix3_vec_rgvPositionMeasurement_enu(:,:,bluetoothInteruptCounter),simParams.missionAreaHalfWidth),-simParams.missionAreaHalfWidth);
        end

        % Check if there is more physics to do before next interupt
        if i+1 <= interuptCount
            nextTime = vec_sortedInteruptTimes(i+1);
        else
            nextTime = simParams.duration;
        end

        if abs(nextTime-vec_times(truthDataIndex)) < 0.01
            continue;
        end

        % Continue physics simulation
        [vec_newTime, trix_vec_newTrueUasState] = simulateUasPhysics(simParams, trix_vec_goToCenter_enu(:,flightPlanInteruptCounter), trix_vec_lookAt_en(:,flightPlanInteruptCounter), vec_orbitRadius(flightPlanInteruptCounter), vec_times(truthDataIndex), nextTime, trix_vec_uasStates(truthDataIndex,:)');
        newValueCount = size(vec_newTime,1)-1;
        vec_times(truthDataIndex+1:truthDataIndex+newValueCount,:) = vec_newTime(2:end);
        trix_vec_uasStates(truthDataIndex+1:truthDataIndex+newValueCount,:) = trix_vec_newTrueUasState(2:end,:);
        truthDataIndex = truthDataIndex + newValueCount;
    end

    rawTruthData.vec_times = vec_times(1:truthDataIndex);
    rawTruthData.trix_vec_uasStates = trix_vec_uasStates(1:truthDataIndex,:);
    sensorData.vec_times = vec_bluetoothInteruptTimes;
    sensorData.trix3_vec_rgvPositionMeasurement_enu = trix3_vec_rgvPositionMeasurement_enu;
    predictorData.vec_times = [0,vec_flightPlanInteruptTimes];
    predictorData.trix4_splineVars = trix4_splineVars;
    predictorData.trix_vec_goToCenter_enu = trix_vec_goToCenter_enu;
    predictorData.trix_vec_lookAt_en = trix_vec_lookAt_en;
    predictorData.vec_orbitRadius = vec_orbitRadius;
    predictorData.vec_flightPlanTypes = vec_flightPlanTypes;
end