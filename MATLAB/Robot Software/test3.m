player = pcplayer([-200 200],[-200,200],[0,1]);

while isOpen(player)
      [x y z] = scan_lidar;
      temp = [x y z];
      ptCloud = pointCloud(temp);
      view(player, ptCloud)
end

