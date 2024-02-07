%% Read in April Tags
% % Setup: create Video Reader and Writer
% videoFileReader = VideoReader("DJI_0014_2_30_V - Copy - Trim.MP4");
% myVideo = VideoWriter('DJI_0014R_2_30_V');
% tagFamily = "tag16H5";
% depVideoPlayer = vision.DeployableVideoPlayer;
% open(myVideo);
% %% Detect faces in each frame
% while hasFrame(videoFileReader)
% 
% 	% read video frame
% 	videoFrame = readFrame(videoFileReader);
% 	% process frame
%     [id,loc,detectedFamily] = readAprilTag(videoFrame,tagFamily);
% 
% 
%     for idx = 1:length(id)
%             % Display the ID and tag family
%             % disp("Detected Tag ID, Family: " + id(idx) + ", " ...
%             %     + detectedFamily(idx));
% 
%             % Insert markers to indicate the locations
%             markerRadius = 8;
%             numCorners = size(loc,1);
%             markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
%             if id(idx) == 25
%                 videoFrame = insertShape(videoFrame,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
%             end
%     end
% 
%     % % Display video frame to screen
% 	% depVideoPlayer(videoFrame);
%     % Write frame to final video file
% 	writeVideo(myVideo, videoFrame);
% 	pause(1/videoFileReader.FrameRate);
% 
% end
% close(myVideo)

%% Archived Code
I = imread("aprilTag16H5_30.png");
[id,loc,detectedFamily] = readAprilTag(I);
for idx = 1:length(id)
            % Display the ID and tag family
            disp("Detected Tag ID, Family: " + id(idx) + ", " ...
                + detectedFamily(idx));

            % Insert markers to indicate the locations
            markerRadius = 4;
            numCorners = size(loc,1);
            markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
            if id(idx) == 25
                I = insertShape(I,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
            end
end
imshow(I);