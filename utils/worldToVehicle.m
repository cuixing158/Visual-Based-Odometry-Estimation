function vehiclePts = worldToVehicle(vehiclePose,worldPts)
% brief:2D鸟瞰图中，把世界坐标系坐标点worldPts转换为以当前车辆坐标系姿态vehiclePose为
% 原点的坐标变换。
% inputs:
%  vehiclePose为1*3大小，[xc,yc,theta],角度为弧度，worldPts为m*2大小坐标,输入参数
%   均在世界坐标系下的物理坐标。
% outputs:
%  返回vehiclePts也为m*2大小坐标，为以vehiclePose为原点的车辆坐标系坐标。
%
xc = vehiclePose(1);
yc = vehiclePose(2);
theta = -vehiclePose(3);
rot = [cos(theta),-sin(theta);
    sin(theta),cos(theta)];
vehiclePts = rot*(worldPts'-[xc;yc]);
vehiclePts = vehiclePts';
end