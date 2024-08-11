function imagePose = worldToGlobalImagePose(anchorPose,worldPose,localOrigin,resolutionXY)%#cogen
% Brief: 2D鸟瞰图场景下世界坐标系姿态worldPose转换为以anchorPose的图像坐标系的坐 
% 标姿态imagePose
% Details:
%       姿态theta为在世界坐标系中以x轴正方向逆时针旋转为正，在图像坐标系中以x轴
% 正方向顺时针旋转为正。都满足右手定则。localOrigin指定图像与anchorPose世界点对应。
% Syntax:  
%     imagePose = worldToGlobalImagePose(anchorPose,worldPose)
% 
% Inputs:
%    anchorPose - [1,3] size,[double] type,形如[x,y,theta]二维姿态，theta为弧度
%    worldPose - [m,3] size,[double] type,形如[x,y,theta]二维姿态，theta为弧度
%    localOrigin - [1,2] size,[double] type,形如[xOri,yOri],单位像素,
%                    前摄像头中心在四副拼接图中的像素坐标
%    resolutionXY - [1,2] size,[double] type,每个像素在世界坐标系下的长度
% 
% Outputs:
%    imagePose - [m,3] size,[double] type,图像坐标姿态,每行与worldPose一一对应，
%              形如[x,y,theta] ,theta单位为度
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         11-Oct-2022 17:15:34
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022b
% Copyright © 2022 TheMatrix.All Rights Reserved.
%
arguments
    anchorPose (1,3) double % 世界坐标系下
    worldPose (:,3) double % 世界坐标系下
    localOrigin (1,2) double % 图像坐标系下
    resolutionXY (1,2) double % 世界坐标系下
end

x0 = anchorPose(1);
y0 = anchorPose(2);
theta0 = rad2deg(anchorPose(3));

xOriRealImg = localOrigin(1)*resolutionXY(1);
yOriRealImg = localOrigin(2)*resolutionXY(2);
rotatedRect = [x0,y0,2*xOriRealImg,2*yOriRealImg,theta0];%初始位置
vertices = getVertices(rotatedRect);
oriRealWorld = vertices(4,:);% top left corner

R = rotz(theta0-90)*rotx(-180);% 坐标轴旋转等价的点旋转矩阵
t = [-oriRealWorld(1),-oriRealWorld(2),0]';
% tform = rigidtform3d(R,t);% 世界坐标系到图像物理坐标系的位置转换
tformTheta = -rad2deg(worldPose(:,3))+theta0-90;% 世界坐标系到图像物理坐标系到姿态（角度）转换

worldPts = worldPose';
worldPts(end,:)=0;
imgWorldPts = R*(worldPts+t);
imgWorldPts = imgWorldPts(1:2,:);
imgPts = imgWorldPts'./resolutionXY;

imagePose = [imgPts,tformTheta];
end