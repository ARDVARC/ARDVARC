%% ARDVARC
% Takes in an image frame and outputs the frame with bounding box and
% center of mass
function [centroidPixel,detectedImg] = frame2objectdetection(frame)
close all;
%%Inputs: frame - frame of reference for object detection

Im = imread(frame); %read in image
% Im = rgb2gray(Im);
% figure()
% imshow(Im)

%% LoG Filter for blob/edge detection
%Define the kernels for the laplacian blob/edge detection
kernel_1 = [0 -1 0; -1 4 -1; 0 -1 0];

%Apply a guassian smoothing method
sigma = 5;
threshold = 70;

figure
imshow(Im);


Im_blur = imgaussfilt3(Im,sigma);
Im_blur = rgb2gray(Im_blur);
Im_blur = (Im_blur <= threshold);
% Im_filt = conv2(Im_blur, threshold * kernel_1);
figure()
imshow(Im_blur)

% LoG_filt = fspecial('log',[5 5],sigma);

% Im_LoG = imfilter(Im_blur,LoG_filt,'symmetric', 'conv');



% figure
% imshow(Im_LoG); hold on
% outputArg1 = inputArg1;
% outputArg2 = inputArg2;
end


%% Color Filter
% I = rgb2hsv(Im);
% % Define thresholds for channel 1 based on histogram settings
% channel1Min = 0.404;
% channel1Max = 0.851;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min = 0.000;
% channel2Max = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min = 0.000;
% channel3Max = 1.000;
% 
% % Create mask based on chosen histogram thresholds
% sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
%     (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
%     (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
% BW = sliderBW;
% % Initialize output masked image based on input image.
% maskedRGBImage = Im;
% 
% % Set background pixels where BW is false to zero.
% maskedRGBImage(repmat(~BW,[1 1 3])) = 0;






% close all
% 

%% Deep learning Options

% tic
% model = solov2("resnet50-coco");
% I = imread("Screenshot_2023-11-08_at_6.18.19_PM.png");
% [masks,labels,scores] = segmentObjects(model,I,Threshold=0.1);
% overlayedImage = insertObjectMask(I,masks,"MaskColor","red");
% figure
% imshow(overlayedImage)
% toc


%Implement YOLO 
% detector = yolov4ObjectDetector("csp-darknet53-coco"); %This takes a long time, likely will preload this
% Run detector.
% [bboxes, scores, labels] = detect(detector, Im); %Yolo Deep learning program
% toc
% Display results.
% detectedImg = insertObjectAnnotation(Im, 'Rectangle', bboxes, labels);
% toc
