function localImagePts = vehicleToLocalImage(refBirdsEye,refImg,vehiclePts)
% Brief: 在2D鸟瞰图中把局部vehiclePts转换到当前四副鱼眼环视图拼接的局部图像坐标
% Details:
%    vehicle坐标系符合matlab自动驾驶工具箱定义，即x轴朝向车辆正前方，y轴朝向车辆左侧。
% 
% Syntax:  
%     localImagePts = vehicleToLocalImage(refBirdsEye,refImg,vehiclePts)
% 
% Inputs:
%    refBirdsEye - [1,1] size,[birdsEyeView] type,matlab build-in type
%    refImg - [1,1] size,[imref2d] type,matlab build-in type,四副环视拼接参考对象
%    vehiclePts - [m,2] size,[double] type,待转换到局部车辆坐标点
% 
% Outputs:
%    localImagePts - [m,2] size,[double] type,四副环视拼接的鸟瞰图像中的局部像素坐标
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         13-Oct-2022 09:00:37
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022b
% Copyright © 2022 TheMatrix.All Rights Reserved.
%

refBaseImagePts = vehicleToImage(refBirdsEye,vehiclePts);

localImagePts = zeros(size(vehiclePts));
[localImagePts(:,1),localImagePts(:,2)] = worldToIntrinsic(refImg,...
    refBaseImagePts(:,1),refBaseImagePts(:,2));
end