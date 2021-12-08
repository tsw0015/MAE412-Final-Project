%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the code for plotting the SMART data                      %
% Author: Yu Gu                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

load SMART_DATA.mat;

[temp Total_Steps]=size(SD.Time);
k=0
for i=1:Total_Steps
    Time_Vector(i)=SD.Time(i)-SD.Time(1);
    if i>1
        if SD.Time(i)-SD.Time(i-1)<0
            k=k+1;
        end
        Time_Vector(i)=Time_Vector(i)+k*60;     % Correct the clock by adding the minutes
    end
end
plot(Time_Vector);
title('Time Vector');
ylabel('Time(s)');
xlabel('Index');

Choice=0;
while(Choice~=10)
    Choice=menu('Select an Item to Plot', 'Time-Delay-Counter','Acceleration','Angular Rates','Magnetic Field','Attitude Angles','Position','Range Finders','Dist-Angle','Volts-Current','Exit');
    switch Choice
        case 1                  % Time-Delay-Counter
            subplot(4,1,1);
            plot(SD.Index,SD.Time);
            title('Time at Each Step');
            subplot(4,1,2);
            plot(SD.Index,SD.Time_Diff);
            title('Time Difference Between Steps');           
            subplot(4,1,3);
            plot(SD.Index,SD.Delay);  
            title('Delay Needed Between Each Step');
            subplot(4,1,4);
            plot(SD.Index,SD.Logger_Counter);  
            title('Data Logger Counter');
         case 2                 % Accelerations
            subplot(3,1,1);
            plot(Time_Vector,SD.Ax);
            title('3-Axis Accelerations');
            ylabel('Ax(g)');
            subplot(3,1,2);
            plot(Time_Vector,SD.Ay);        
            ylabel('Ay(g)');
            subplot(3,1,3);
            plot(Time_Vector,SD.Az);  
            ylabel('Az(g)');
            xlabel('Time(s)');
         case 3                 % Angular Rates
            subplot(3,1,1);
            plot(Time_Vector,rad2deg(SD.P));
            title('3-Axis Angular Rates');
            ylabel('P(deg/sec)');
            subplot(3,1,2);
            plot(Time_Vector,rad2deg(SD.Q));        
            ylabel('Q(deg/sec)');
            subplot(3,1,3);
            plot(Time_Vector,rad2deg(SD.R));  
            ylabel('R(deg/sec)');
            xlabel('Time(s)');
        case 4                 % Magnetic Field
            subplot(3,1,1);
            plot(Time_Vector,SD.Mx);
            title('3-Axis Calibrated and Normalized Magnetic Field');
            ylabel('Mx');
            subplot(3,1,2);
            plot(Time_Vector,SD.My);        
            ylabel('My');
            subplot(3,1,3);
            plot(Time_Vector,SD.Mz);  
            ylabel('Mz');
            xlabel('Time(s)');
        case 5                 % Attitude Angles
            subplot(3,1,1);
            plot(Time_Vector, rad2deg(SD.Roll));
            title('Attitude Angles');
            ylabel('Roll(deg)');
            subplot(3,1,2);
            plot(Time_Vector,rad2deg(SD.Pitch));        
            ylabel('Pitch(deg)');
            subplot(3,1,3);
            plot(Time_Vector,rad2deg(SD.Yaw),'b',Time_Vector,rad2deg(SD.Yaw(1)-SD.TotalAngle),'r',Time_Vector,rad2deg(SD.Mag_Heading-SD.Mag_Heading(1)),'g' );  
            legend('Yaw','Encoder','Magnetic');
            ylabel('Yaw(deg)');
            xlabel('Time(s)');
        case 6                 % Position
            subplot(1,1,1);
            plot(SD.X,SD.Y);
            title('Robot Position');
            xlabel('X(m)');
            ylabel('Y(m)');
        case 7                 % Range Finders
            subplot(5,1,1);
            plot(Time_Vector,SD.RF_F);
            title('Range Finders');
            ylabel('F RF(mm)');
%             subplot(6,1,2);
%             plot(Time_Vector,SD.RF_FL);        
%             ylabel('FL RF(mm)');
            subplot(5,1,2);
            plot(Time_Vector,SD.RF_L);  
            ylabel('L RF(mm)');
            xlabel('Time(s)');
            subplot(5,1,3);
            plot(Time_Vector,SD.RF_B);
            ylabel('B RF(mm)');
            subplot(5,1,4);
            plot(Time_Vector,SD.RF_R);        
            ylabel('R RF(mm)');
            subplot(5,1,5);
            plot(Time_Vector,SD.Laser_RF);  
            ylabel('Laser RF(mm)');
            xlabel('Time(s)');
        case 8                 % Distance-Angle
            subplot(2,1,1);
            plot(Time_Vector,SD.TotalDist);
            title('Total Distance and Angle Traveled');
            ylabel('Distance(m)');
            subplot(2,1,2);
            plot(Time_Vector,rad2deg(SD.TotalAngle));        
            ylabel('Angle(deg)');
            xlabel('Time(s)');
        case 9                 % Voltage-Current
            subplot(2,1,1);
            plot(Time_Vector,SD.CreateVolts);
            title('iRobot Create Voltage and Current');
            ylabel('Voltage(V)');
            subplot(2,1,2);
            plot(Time_Vector,SD.CreateCurrent);        
            ylabel('Current(I)');
            xlabel('Time(s)');

    end
end

for i=1:Total_Steps
    Total_Acceleration(i)=(SD.Ax(i)^2+SD.Ay(i)^2+SD.Az(i)^2)^.5;
    Total_Mag(i)=(SD.Mx(i)^2+SD.My(i)^2+SD.Mz(i)^2)^.5;    
end
subplot(1,1,1);
plot(Time_Vector,Total_Acceleration);
title('Total Acceleration');
ylabel('(g)');
xlabel('Time(s)');

figure
subplot(2,1,1);
plot(Time_Vector,Total_Mag);
title('Total Magnetic Field');
ylabel('Normalized');
xlabel('Time(s)');
grid on;
subplot(2,1,2);
plot(Time_Vector,rad2deg(wrapToPi(SD.Yaw)),'b',Time_Vector,rad2deg(wrapToPi(SD.Yaw(1)-SD.TotalAngle)),'r',Time_Vector,rad2deg(SD.Mag_Heading-SD.Mag_Heading(1)),'g' );  
legend('Yaw','Encoder','Magnetic');
ylabel('Yaw(deg)');
xlabel('Time(s)');


