function worldPts = vehicleToWorld(vehiclePose,vehiclePts)
% vehiclePose为1*3大小，[xc,yc,theta],角度为弧度，vehiclePts为m*2大小坐标
% 返回worldPts也为m*2大小坐标
xc = vehiclePose(1);
yc = vehiclePose(2);
theta = vehiclePose(3);
rot = [cos(theta),-sin(theta);
    sin(theta),cos(theta)];
worldPts = rot*vehiclePts'+[xc;yc];
worldPts = worldPts';
end