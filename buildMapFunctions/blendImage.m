function bigImgSt = blendImage(bigImgSt,currImg,currRef,maskImg)%#codegen
% 由当前的视角图像构建大图
persistent alphablend
if isempty(alphablend)
    alphablend = vision.AlphaBlender('Operation','Binary Mask',...
        'MaskSource','Input port',...
        'LocationSource','Input port');
end
if isempty(bigImgSt.bigImg)
    bigImgSt.bigImg = currImg;
    bigImgSt.ref = struct("XWorldLimits",currRef.XWorldLimits,...
    "YWorldLimits",currRef.YWorldLimits,...
    "ImageSize",currRef.ImageSize);% h*w;
    return
end

padLeftX = max(bigImgSt.ref.XWorldLimits(1)-currRef.XWorldLimits(1),0);
padRightX = max(currRef.XWorldLimits(2)-bigImgSt.ref.XWorldLimits(2),0);
padUpperY = max(bigImgSt.ref.YWorldLimits(1)-currRef.YWorldLimits(1),0);
padDownY = max(currRef.YWorldLimits(2)-bigImgSt.ref.YWorldLimits(2),0);

bigImgSt.bigImg = padarray(bigImgSt.bigImg,round([padUpperY,padLeftX]),0,'pre');
bigImgSt.bigImg = padarray(bigImgSt.bigImg,round([padDownY,padRightX]),0,'post');

% construct new spatial refrence information
newXLimits = [min(currRef.XWorldLimits(1),bigImgSt.ref.XWorldLimits(1)),...
    max(currRef.XWorldLimits(2),bigImgSt.ref.XWorldLimits(2))];
newYLimits = [min(currRef.YWorldLimits(1),bigImgSt.ref.YWorldLimits(1)),...
    max(currRef.YWorldLimits(2),bigImgSt.ref.YWorldLimits(2))];
currRef = imref2d(size(bigImgSt.bigImg),newXLimits,newYLimits);
bigImgSt.ref = struct("XWorldLimits",currRef.XWorldLimits,...
    "YWorldLimits",currRef.YWorldLimits,...
    "ImageSize",currRef.ImageSize);% h*w

% 贴图
xWorld = currRef.XWorldLimits(1);
yWorld = currRef.YWorldLimits(1);
[xIntrinsic, yIntrinsic] = worldToIntrinsic(currRef,xWorld,yWorld);
xIntrinsic = int32(round(xIntrinsic));
yIntrinsic = int32(round(yIntrinsic));

mask = sum(currImg, 3) ~= 0;
maskImg = maskImg&mask;
if coder.target("MATLAB")
    bigImgSt.bigImg = alphablend(bigImgSt.bigImg,currImg,maskImg,...
        int32(round([xIntrinsic,yIntrinsic])));
else
    bigImgSt.bigImg = alphablend_opencv(bigImgSt.bigImg,currImg,maskImg,xIntrinsic,yIntrinsic);
end
end

