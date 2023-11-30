function animateSimulation2D()
    close all force;
    
    frameRate = 5;
    playbackRate = 5;
    mainPixelRadius = 2;
    trailPixelRadius = 0;
    sensorPixelRadius = 1;

    [inputFile,inputPath] = uigetfile(".mat","Select Simulation Data File");
    if inputFile == 0
        return
    end
    load([inputPath,inputFile]);
    
    truthDataIndex = 1;
    sensorIndex = 1;

    vec_frameTimes = 0:(1/frameRate):simParams.duration;
    frameCount = length(vec_frameTimes);

    res = [1024,1024];

    trix_vec_rgv1PositionsScaled_en = (truthData.trix_vec_rgv1Positions_enu(1:2,:)/simParams.missionAreaHalfWidth+1)/2;
    trix_vec_rgv1PositionsScaled_en(1,:) = round(trix_vec_rgv1PositionsScaled_en(1,:)*(res(1)-1))+1;
    trix_vec_rgv1PositionsScaled_en(2,:) = round(trix_vec_rgv1PositionsScaled_en(2,:)*(res(2)-1))+1;
    trix_vec_rgv2PositionsScaled_en = (truthData.trix_vec_rgv2Positions_enu(1:2,:)/simParams.missionAreaHalfWidth+1)/2;
    trix_vec_rgv2PositionsScaled_en(1,:) = round(trix_vec_rgv2PositionsScaled_en(1,:)*(res(1)-1))+1;
    trix_vec_rgv2PositionsScaled_en(2,:) = round(trix_vec_rgv2PositionsScaled_en(2,:)*(res(2)-1))+1;
    trix_vec_uasPositionsScaled_en = (truthData.trix_vec_uasStates(:,1:2)'/simParams.missionAreaHalfWidth+1)/2;
    trix_vec_uasPositionsScaled_en(1,:) = round(trix_vec_uasPositionsScaled_en(1,:)*(res(1)-1))+1;
    trix_vec_uasPositionsScaled_en(2,:) = round(trix_vec_uasPositionsScaled_en(2,:)*(res(2)-1))+1;
    trix3_vec_rgvPositionEstimatesScaled_en = (sensorData.trix3_vec_rgvPositionEstimate_enu(1:2,:,:)/simParams.missionAreaHalfWidth+1)/2;
    trix3_vec_rgvPositionEstimatesScaled_en(1,:,:) = max(min(round(trix3_vec_rgvPositionEstimatesScaled_en(1,:,:)*(res(1)-1))+1,res(1)),1);
    trix3_vec_rgvPositionEstimatesScaled_en(2,:,:) = max(min(round(trix3_vec_rgvPositionEstimatesScaled_en(2,:,:)*(res(2)-1))+1,res(2)),1);

    defaultOutputFile = strrep(strrep(inputFile,"SimData","SimVid"),".mat",".mj2");
    [outputFile,outputPath] = uiputfile(".mj2","Save Simulation Video", defaultOutputFile);
    if outputFile == 0
        return
    end
    
    hfov = 10;
    vfov = 10;
    vec_cameraCenter_cam = [1;0;0];
    [a1,a2,a3] = sph2cart(deg2rad(vfov/2),deg2rad(hfov/2),1);
    vec_cameraDR_cam = [a1;a2;a3];
    [a1,a2,a3] = sph2cart(-deg2rad(vfov/2),deg2rad(hfov/2),1);
    vec_cameraUR_cam = [a1;a2;a3];
    [a1,a2,a3] = sph2cart(deg2rad(vfov/2),-deg2rad(hfov/2),1);
    vec_cameraDL_cam = [a1;a2;a3];
    [a1,a2,a3] = sph2cart(-deg2rad(vfov/2),-deg2rad(hfov/2),1);
    vec_cameraUL_cam = [a1;a2;a3];
    trix_vec_cameraBitsPointing_cam = [vec_cameraCenter_cam,vec_cameraDR_cam,vec_cameraUR_cam,vec_cameraDL_cam,vec_cameraUL_cam];

    vec_cameraForwardLong_uas = [simParams.orbitRadius;0;simParams.targetUasHeight];
    vec_cameraForward_uas = vec_cameraForwardLong_uas/norm(vec_cameraForwardLong_uas);
    vec_cameraRight_uas = [0;1;0];
    vec_cameraDown_uas = cross(vec_cameraForward_uas,vec_cameraRight_uas);
    dcm_cam2uas = [vec_cameraForward_uas,vec_cameraRight_uas,vec_cameraDown_uas];

    trix_vec_cameraBitsPointing_uas = dcm_cam2uas*trix_vec_cameraBitsPointing_cam;

    v = VideoWriter([outputPath,outputFile],"Motion JPEG 2000");
    v.LosslessCompression = true;
    v.FrameRate = frameRate*playbackRate;
    open(v);

    wb = waitbar(0,"Animating...");
    for i = 1:frameCount
        time = vec_frameTimes(i);

        while truthDataIndex < length(truthData.vec_times) && truthData.vec_times(truthDataIndex) <= time
            truthDataIndex = truthDataIndex + 1;
        end
        while sensorIndex < length(sensorData.vec_times) && sensorData.vec_times(sensorIndex) <= time
            sensorIndex = sensorIndex + 1;
        end

        frame = zeros([res,3],"uint8");

        for j = max(1, sensorIndex-simParams.bluetoothRate*2):sensorIndex
            frame = paintFrame(frame,trix3_vec_rgvPositionEstimatesScaled_en(:,1,j),sensorPixelRadius,1,127);
            frame = paintFrame(frame,trix3_vec_rgvPositionEstimatesScaled_en(:,2,j),sensorPixelRadius,3,127);
        end
        for j = max(1, truthDataIndex-simParams.sampleRate):max(1, truthDataIndex-1)
            frame = paintFrame(frame,trix_vec_rgv1PositionsScaled_en(:,j),trailPixelRadius,1,255);
            frame = paintFrame(frame,trix_vec_uasPositionsScaled_en(:,j),trailPixelRadius,2,255);
            frame = paintFrame(frame,trix_vec_rgv2PositionsScaled_en(:,j),trailPixelRadius,3,255);
        end
        for j = max(1, truthDataIndex):truthDataIndex
            frame = paintFrame(frame,trix_vec_rgv1PositionsScaled_en(:,j),mainPixelRadius,1,255);
            frame = paintFrame(frame,trix_vec_uasPositionsScaled_en(:,j),mainPixelRadius,2,255);
            frame = paintFrame(frame,trix_vec_rgv2PositionsScaled_en(:,j),mainPixelRadius,3,255);
        end

        dcm_uas2enu = getDcmUas2Enu(truthData.trix_vec_uasStates(truthDataIndex,:));
        trix_vec_cameraBitsPointing_enu = dcm_uas2enu * trix_vec_cameraBitsPointing_uas;

        vec_coeff = -truthData.trix_vec_uasStates(truthDataIndex,3)./trix_vec_cameraBitsPointing_enu(3,:);
        trix_vec_cameraGroundPoint_enu = truthData.trix_vec_uasStates(truthDataIndex,1:3)'+vec_coeff.*trix_vec_cameraBitsPointing_enu;
        trix_vec_cameraGroundPointScaled_en = (trix_vec_cameraGroundPoint_enu(1:2,:)/simParams.missionAreaHalfWidth+1)/2;
        trix_vec_cameraGroundPointScaled_en(1,:) = max(min(round(trix_vec_cameraGroundPointScaled_en(1,:)*(res(1)-1))+1,res(1)),1);
        trix_vec_cameraGroundPointScaled_en(2,:) = max(min(round(trix_vec_cameraGroundPointScaled_en(2,:)*(res(2)-1))+1,res(2)),1);
        frame = paintFrame(frame,trix_vec_cameraGroundPointScaled_en(:,1),sensorPixelRadius,1:3,255);
        for j = 2:5
            frame = paintFrame(frame,trix_vec_cameraGroundPointScaled_en(:,j),sensorPixelRadius,1:3,127);
        end

        writeVideo(v,frame);
        waitbar(i/frameCount,wb);
    end
    close(v);
    close(wb);
end