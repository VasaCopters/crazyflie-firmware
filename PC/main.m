%webcamlist
numberOfCameras = 2;

for i=1:numberOfCameras
    
    cam(i) = webcam(i);
    
    cam(i).ExposureMode = 'manual';
    cam(i).Zoom = 100;
    cam(i).FocusMode = 'manual';
    cam(i).Contrast = 128;
    cam(i).Gain = 0;
    cam(i).Brightness = 128;
    cam(i).BacklightCompensation = 0;
    cam(i).Sharpness = 128;
    cam(i).Saturation = 128;
    cam(i).Exposure = -5;
    cam(i).WhiteBalance = 5000;
    cam(i).Focus = 0;

    figure
    imshow(snapshot(cam(i)));
end

clear cam(1)
clear cam(2)

