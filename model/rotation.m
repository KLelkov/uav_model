function R= rotation(theta)
R = [(cosd(theta(3))*cosd(theta(2))) (cosd(theta(3))*sind(theta(2))*sind(theta(1))-sind(theta(3))*cosd(theta(1)))  (cosd(theta(3))*sind(theta(2))*cosd(theta(1))+sind(theta(3))*sind(theta(1)));
    (sind(theta(3))*cosd(theta(2)))  (sind(theta(3))*sind(theta(2))* sind(theta(1))+cosd(theta(3))*cosd(theta(1)))   (sind(theta(3))* sind(theta(2))*cosd(theta(1))-cosd(theta(3))* sind(theta(1)));
    - sind(theta(2))  (sind(theta(1))*cosd(theta(2))) (cosd(theta(1))*cosd(theta(2)))];
% % R = [(cosd(theta(1))*cosd(theta(3))-cosd(theta(2))*sind(theta(1))*sind(theta(3))) (-cosd(theta(3))*sind(theta(1))-cosd(theta(1))*cosd(theta(2))*sind(theta(3))) (sind(theta(2))*sind(theta(3)));
% %     (cosd(theta(2))*cosd(theta(3))*sind(theta(1))+cosd(theta(1))*sind(theta(3))) (cosd(theta(1))*cosd(theta(2))*cosd(theta(3))-sind(theta(1))*sind(theta(3))) -cosd(theta(3))*sind(theta(2));
% %     sind(theta(1))*sind(theta(2)) cosd(theta(1))*sind(theta(2)) cosd(theta(2))];
end