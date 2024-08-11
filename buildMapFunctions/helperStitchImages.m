function [outputImage, outputView] = helperStitchImages(images, tforms) 
    % The helperStitchImages function applies the transforms tforms to the input images and blends 
    % them to produce the outputImage. It additionally returns the outputView, which you can use to 
    % transform any point from the first image in the given image sequence to the output image. 
    % Copyright 2021 MathWorks, Inc.
    
    %#codegen
    numImages = numel(images);
    imageSize = zeros(numImages,2);
    xlim = zeros(numImages,2);
    ylim = zeros(numImages,2);
    
    % Compute the output limits for each transform.
    for i = 1:numImages
        imageSize(i,:) = size(images{i}, 1:2);
        [xlim(i,:), ylim(i,:)] = outputLimits(tforms{i}, ...
                                              [1 imageSize(i,2)], ...
                                              [1 imageSize(i,1)]);
    end    

    % Find the minimum and maximum output limits.
%     maxImageSize = max(imageSize);
    
    xMin = min(xlim(:));
    xMax = max(xlim(:));

    yMin = min(ylim(:));
    yMax = max(ylim(:));

    % Width and height of panorama.
    width  = round(xMax - xMin);
    height = round(yMax - yMin);

    % Initialize the "empty" panorama.
    outputImage = zeros([height width 3], "uint8");

    % Create a 2-D spatial reference object defining the size of the panorama.
    xLimits = [xMin xMax];
    yLimits = [yMin yMax];
    
    outputView = imref2d([height width], xLimits, yLimits);

    % Step 7 - Stitch the images.
    for i = 1:numel(tforms)
        % Apply transformation.
        warpedImage = imwarp(images{i}, tforms{i},'linear','OutputView',outputView,'FillValues',0,'SmoothEdges',false);
        
        % Blend the images.
        outputImage = helperBlendImages(warpedImage, outputImage);
    end
end