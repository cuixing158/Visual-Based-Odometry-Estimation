function outputImage = helperBlendImages(I1, I2) 
%% The helperBlendImages function performs alpha blending to the given two input images, I1 and I2, 
% with alpha values that are proportional to the center seam of each image. 
% The output Iout is a linear combination of the input images:
% Iout = alpha * I1 + (1 - alpha) * I2

% Copyright 2021 MathWorks, Inc.

%#codegen

    % Identify the image regions in the two images by masking out the black
    % regions.
    maskA = sum(I1, 3) ~= 0;
    maskB = sum(I2, 3) ~= 0;
    maskAB = maskA & maskB;
    
    % Compute alpha values that are proportional to the center seam of the two
    % images.
    alpha1 = ones(size(maskA,1:2));
    alpha2 = ones(size(maskB,1:2));
    dist1  = bwdist(edge(maskA));
    dist2  = bwdist(edge(maskB));

    % alpha1,alpha2
    alpha1(maskAB) = dist1(maskAB)./(dist1(maskAB)+dist2(maskAB));
%     alpha1(maskB) = 0;

    alpha2(maskAB) = 1-alpha1(maskAB);
%     alpha2(maskA) = 0;

    I1 = double(I1);
    I2 = double(I2);        
    outputImage = alpha1.*I1 + alpha2.*I2;    
    outputImage = uint8(outputImage);
end