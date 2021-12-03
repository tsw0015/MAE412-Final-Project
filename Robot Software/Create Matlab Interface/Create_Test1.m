% this code is for testing the Create Matlab Interface Functions.

clear all
close all

S_Create=RoombaInit(0);     % 5 on another computer

flushinput(S_Create);       % Flush the iRobot Create serial port
fwrite(S_Create, [142 0]);  % Request all sensor data from Create

BeepRoomba(S_Create);       % Make a Beeping Sound

travelDist(S_Create, 0.5, 1.0); % travel full speed for a meter
BeepRoomba(S_Create);       % Make a Beeping Sound
pause(1);
turnAngle(S_Create, 0.2, 90); % turn full speed for 90 degrees
BeepRoomba(S_Create);       % Make a Beeping Sound
pause(1);


Total_Dist=0;
while Total_Dist<=2
    fwrite(S_Create, [142 0]);  % Request all sensor data from Create    
    pause(0.01);  % Wait sometime for Create to respond
    [BumpRight,BumpLeft,BumpFront,Wall,VirtWall,CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,LeftCurrOver,RightCurrOver,DirtL,DirtR,ButtonPlay,ButtonAv,Dist,Angle,CreateVolts,CreateCurrent,Temp,Charge,Capacity,pCharge]=Read_Create_2(S_Create); 
    Total_Dist=Total_Dist+Dist
    SetDriveWheelsCreate(S_Create, 0.5, 0.5);
end
SetDriveWheelsCreate(S_Create, 0, 0);
BeepRoomba(S_Create);       % Make a Beeping Sound
pause(1);

delete(S_Create)
clear S_Create  
