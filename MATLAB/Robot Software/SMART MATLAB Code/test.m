close all
figure
hold on
for i = 1:size(SD.Lidar_Angle)
   polarplot(SD.Lidar_Angle(i), SD.Lidar_Range(i))
end