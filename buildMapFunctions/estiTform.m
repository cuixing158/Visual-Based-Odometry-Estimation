function [tform,inlierIdx,validInd1,validInd2,isOneSide,status,noRigidInd1,noRigidInd2] = ...
   estiTform(preFeatures,prePointsLoc,currFeatures,currPointsLoc,vehicleROI)%#codegen
% Brief: 从previousImg图像到currImg的转换估算
arguments
    preFeatures 
    prePointsLoc (:,2) {mustBeNumeric}
    currFeatures
    currPointsLoc (:,2) {mustBeNumeric}
    vehicleROI (4,2) {mustBeNumeric}=[307.6150,463.4945;% 见mainBuildMap,左下
  307.6150,303.8412; %左上
  384.2479,303.8412;% 右上
  384.2479,463.4945]% 右下
end

% 排除ego本身的匹配，即车身周围矩形范围的点不应该考虑
% persistent in1 % 此申明目的是为了减少计算量inpolygon，但要保证下一次currFeatures,currPointsLoc赋值给preFeatures,prePointsLoc
% if isempty(in1)
%     in1 = inpolygon(prePointsLoc(:,1),prePointsLoc(:,2),vehicleROI(:,1),vehicleROI(:,2));
% end
in1 = inpolygon(prePointsLoc(:,1),prePointsLoc(:,2),vehicleROI(:,1),vehicleROI(:,2));
in2 = inpolygon(currPointsLoc(:,1),currPointsLoc(:,2),vehicleROI(:,1),vehicleROI(:,2));

index1 = 1:size(prePointsLoc,1);
index2 = 1:size(currPointsLoc,1);
index1 = index1(~in1);
index2 = index2(~in2);

% Extract features
preValidPoints = prePointsLoc(index1,:);%getSubInds(prePoints,index1);
currValidPoints = currPointsLoc(index2,:);%getSubInds(currPoints,index2);

preFeatures = binaryFeatures(preFeatures.Features(index1,:));
currFeatures = binaryFeatures(currFeatures.Features(index2,:));

% Match features
indexPairs = matchFeatures(preFeatures,currFeatures,...
    'MatchThreshold',50.000000,'MaxRatio',0.500000,...
    'Unique', true);


ind1 = indexPairs(:,1);
ind2 = indexPairs(:,2);

noRigidInd1 = index1(ind1);
noRigidInd2 = index2(ind2);

preMatchedPoints = preValidPoints(ind1,:);%getSubInds(preValidPoints,ind1);
currMatchedPoints = currValidPoints(ind2,:);%getSubInds(currValidPoints,ind2);


% Apply transformation - Results may not be identical between runs because of the randomized nature of the algorithm
[tform,inlierIdx,status] = estgeotform2d(preMatchedPoints,...
    currMatchedPoints,'rigid');% preMatchedPoints和currMatchedPoints必须为double类型，因为如果是single类型会导致求逆导致奇异矩阵发生

validInd1 = index1(ind1(inlierIdx));
validInd2 = index2(ind2(inlierIdx));

preTformPoints = preMatchedPoints(inlierIdx,:);
currTformPoints = currMatchedPoints(inlierIdx,:);%getSubInds(currMatchedPoints,inlierIdx);

isOneSide = true;
if ~isempty(currTformPoints)
    A = (vehicleROI(1,:)+vehicleROI(end,:))/2;
    B = (vehicleROI(2,:)+vehicleROI(3,:))/2;
    a = A(2)-B(2);
    b = B(1)-A(1);
    c = A(1)*B(2)-A(2)*B(1);
    % ax+by+c==0
    isOneSide = all(a*currTformPoints(:,1)+b*currTformPoints(:,2)+c>0)||...
        all(a*currTformPoints(:,1)+b*currTformPoints(:,2)+c<0);

    % too close should exclude
    thresholdRadius = 20;% 像素半径
    xRange1 = max(preTformPoints(:,1))-min(preTformPoints(:,1));
    yRange1 = max(preTformPoints(:,2))-min(preTformPoints(:,2));
    xRange2 = max(currTformPoints(:,1))-min(currTformPoints(:,1));
    yRange2 = max(currTformPoints(:,2))-min(currTformPoints(:,2));
    radius1 = sqrt(xRange1.^2+yRange1.^2);
    radius2 = sqrt(xRange2.^2+yRange2.^2);
    if radius1<thresholdRadius||radius2<thresholdRadius
        status = int32(3);
    end
end
% in1 = in2;

% debug
% figure;imshow(previousImg);hold on;plot(prePoints,'ShowScale',false)
% figure;imshow(currImg);hold on;plot(currPoints,'ShowScale',false)
% figure;showMatchedFeatures(previousImg,currImg,preMatchedPoints,currMatchedPoints,'montage')

% numsTemp = size(preMatchedPoints.Location,1);
% % t = datetime('now','InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
% % t.Format = 'yyyy-MM-dd_HH-mm-ss_SSS';
% % name = [char(t),'.txt'];
% persistent ii
% if isempty(ii)
%     ii = 1;
% else
%     ii = ii+1;
% end
% name = string(ii)+".txt";
% fid = fopen(name,"w");
% fprintf(fid,"------------------------------------------------\n");
% fprintf(fid,"preMatchedPoints, currMatchedPoints,nums:%d\n",int64(numsTemp));
% for ith = 1:numsTemp
%     fprintf(fid,"(%-010.3f, %-010.3f),(%-010.3f, %-010.3f)\n",preMatchedPoints.Location(ith,1),preMatchedPoints.Location(ith,2),...
%         currMatchedPoints.Location(ith,1),currMatchedPoints.Location(ith,2));
% end
% fprintf(fid,"tform.A:\n");
% temA = tform.A';
% for inum = 1:3
%     for jnum = 1:3
%         fprintf(fid,"%-08.5f,",temA(inum,jnum));
%     end
%     fprintf(fid,"\n");
% end
% fprintf(fid,"\n");
% fclose(fid);
% figure; showMatchedFeatures(previousImg,currImg,preMatchedPoints,currMatchedPoints,'montage')
end

function dstORBPoints = getSubInds(srcORBPoints,validIdxs)
%#codegen
Location = srcORBPoints.Location(validIdxs,:);
Metric = srcORBPoints.Metric(validIdxs);
Scale = srcORBPoints.Scale(validIdxs);
Orientation = srcORBPoints.Orientation(validIdxs);
dstORBPoints = ORBPoints(Location,"Metric",Metric,"Orientation",Orientation,"Scale",Scale);
end