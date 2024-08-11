function R = rotx(degree)
R = [1,0,0;
    0,cosd(degree),-sind(degree);
    0,sind(degree),cosd(degree)];
end