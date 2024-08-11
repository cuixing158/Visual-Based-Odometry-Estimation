function [features, featureMetrics,locations] = helperDetectAndExtractFeatures(Irgb, ...
    scaleFactor, numLevels)
arguments
    Irgb
    scaleFactor (1,1) double = 1.2
    numLevels (1,1) double =8
end
rng(0);
numPoints   = 2000;
maxNumFeatures = 5000;

% Detect ORB features
Igray  = im2gray(Irgb);

% points = detectORBFeaturesCustom(Igray, 'ScaleFactor', scaleFactor, 'NumLevels', numLevels,'NumFeatures',maxNumFeatures);
points = detectORBFeatures(Igray, 'ScaleFactor', scaleFactor, 'NumLevels', numLevels);

% Select a subset of features, uniformly distributed throughout the image
points = selectUniform(points, min(numPoints,maxNumFeatures), size(Igray, 1:2));


% Extract features
[features, validPoints] = extractFeatures(Igray, points);

% Compute the Feature Metric. Use the variance of features as the metric
featureMetrics = var(single(features.Features),[],2);

locations = double(validPoints.Location);% single to double,避免后续estgeotform2d函数导致奇异值
end