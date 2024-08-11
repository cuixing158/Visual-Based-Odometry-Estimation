function R = rotz(degree)
R = [cosd(degree),-sind(degree),0;
    sind(degree),cosd(degree),0;
    0,0,1];
end