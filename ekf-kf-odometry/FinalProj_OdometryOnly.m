function Function_Output= MAE_CPE_412_Robot_Control(serPort)
% Simple program for controlling the iRobot Create
% Gu, Nov 2020
%     clc
    % Set constants for this program
    Tend=60*10;                        % Simultation time in seconds;

    Ts_Desired=0.5;                 % Desired sampling time
    Ts=0.5;                         % sampling time is 0.5 second. It can be reduced 
                                    % slightly to offset other overhead in the loop
    Total_Steps=Tend/Ts_Desired;    % The total number of time steps;

    Create_Full_Speed=1;          % The highest speed the robot can travel. (Max is 0.5m/s)
    gravity = 9.81;                 % Earth's gravity

    % Rate Gyro Biases (unit, rad/s)
    P_Bias=0.002;
    Q_Bias=0.003;
    R_Bias=0.001;

    % Accelerometer Biases (unit, m/s2)
    Ax_Bias=0.005;
    Ay_Bias=0.007;
    Az_Bias=0.009;
    
    % Gyro, Acceleraometer, range, and Euler angle measurement noise sigma (standard deviation)
    % Do not change the noise values here!
    Gyro_Sigma=0.01;        % unit, rad/s
    Accel_Sigma=0.1;        % unit, m/s2
    Range_Sigma=0.1;        % unit, m
    Euler_Sigma=0.05;       % unit, rad
    Angle_Sigma=0.01;       % odometry angle, unit, rad
    Dist_Sigma=0.01;        % odometry distance, unit, m
    
    % Define the main Data Structure
    SD= struct( 'Index',zeros(1,Total_Steps),...            % SD stands for SMART Data, which stores all the robot data
            'Time', zeros(1,Total_Steps),...            % The actual time at each time step (s), (Measurement)  
            'Time_Diff', zeros(1,Total_Steps),...       % The time difference between two step (s) 
            'Delay',zeros(1,Total_Steps),...            % Delay needed between each time step. It indicate how much time avaliable for other compuations. 
            'Lidar_Angle', zeros(682,Total_Steps),...   % Hukuyo Lidar Scan Angles             
            'Lidar_Range', zeros(682,Total_Steps),...   % Hukuyo Lidar Scan Range             
            'Logger_Counter', zeros(1,Total_Steps),...  % The Counter number for the SMART data logger 
            'Truth_x',zeros(1,Total_Steps),...          % position along the local x-Axis (m), (truth)
            'Truth_y',zeros(1,Total_Steps),...          % position along the local y-Axis (m), (truth)
            'Truth_theta',zeros(1,Total_Steps),...      % Heading angle of the robot (rad), (truth)
            'Vx',zeros(1,Total_Steps),...               % Velocity along the x-Axis (m/s), (truth)
            'Vy',zeros(1,Total_Steps),...               % Velocity along the y-Axis (m/s), (truth)
            'Vz',zeros(1,Total_Steps),...               % Velocity along the z-Axis (m/s), (truth)
            'Ax',zeros(1,Total_Steps),...               % Acceleration along the x-Axis (m/s2), (Measurement)  
            'Ay',zeros(1,Total_Steps),...               % Acceleration along the y-Axis (m/s2), (Measurement) 
            'Az',zeros(1,Total_Steps),...               % Acceleration along the z-Axis (m/s2), (Measurement) 
            'P',zeros(1,Total_Steps),...                % Roll Rate (rad/s), (Measurement) 
            'Q',zeros(1,Total_Steps),...                % Pitch Rate (rad/s), (Measurement) 
            'R',zeros(1,Total_Steps),...                % Yaw Rate (rad/s), (Measurement) 
            'Roll',zeros(1,Total_Steps),...             % Roll Angle (rad), (Measurement) 
            'Pitch',zeros(1,Total_Steps),...            % Pitch Angle (rad), (Measurement) 
            'Yaw',zeros(1,Total_Steps),...              % Yaw Angle (rad), (Measurement)  
            'X',zeros(1,Total_Steps),...                % Robot X Position (m), (Measurement)  
            'Y',zeros(1,Total_Steps),...                % Robot Y Position (m), (Measurement)  
            'RF_F',zeros(1,Total_Steps),...             % Front Range Finder (m), (Measurement) 
            'RF_L',zeros(1,Total_Steps),...             % Left Range Finder (m), (Measurement) 
            'RF_B',zeros(1,Total_Steps),...             % Back Range Finder (m), (Measurement) 
            'RF_R',zeros(1,Total_Steps),...             % Right Range Finder (m), (Measurement) 
            'Lidar', zeros(360,Total_Steps),...         % Store the Lidar data (m),(measurement) 
            'Dist', zeros(1,Total_Steps),...            % Distance Traveled Since Last Call (m), (Measurement) 
            'TotalDist', zeros(1,Total_Steps),...       % Total Distance Traveled(m), (Measurement) 
            'Angle', zeros(1,Total_Steps),...           % Angle Traveled Since Last Call (rad), (Measurement) 
            'TotalAngle', zeros(1,Total_Steps));        % Total Angle Traveled (rad), (Measurement) 


        % Initialize robot as stopped.
        SetDriveWheelsCreate(serPort,0,0)
    
    % Flags and variables to be used in the custom drive code
        Drive_Straight=1;      % 1 - start, 0 finished
        Turn_In_Place=0;        % 1 - start, 0 finished
        Last_Dist=0;            % The TotalDistance at the last stop
        Last_Angle=0;           % The TotalAngle at the last stop
        Yaw_Reference = 0;   % Referece yaw angle to be achieved by feedback control 
        Proportional_Gain=0.1;
        Int_Gain=0;
        Deriv_Gain=0;
        Sum_error=0;
        WO_pos=[0;0;0];
        loopCnt=0;
        ygBias=0;
        
    % Enter the main loop
    i=0;
    while loopCnt<3
        tic
        SD.Index(i)=i;
        % Read the time      
        Time=clock;                     % Mark the current time;
        SD.Time(i)=Time(6);             % Store the seconds;
        if i==1
            SD.Time_Diff(i)=Ts_Desired;
        else
            SD.Time_Diff(i)=SD.Time(i)-SD.Time(i-1);
            if SD.Time_Diff(i)<0
                SD.Time_Diff(i)=SD.Time_Diff(i)+60;                               % Compensate for the minute change
            end
        end

        % Read the wheel encoder data 
        SD.Dist(i)=DistanceSensorRoomba(serPort)+ normrnd(0, Dist_Sigma);         % with noise added
        SD.Angle(i)=AngleSensorRoomba(serPort)+ normrnd(0, Angle_Sigma);          % with noise added

        if i>1
            SD.TotalDist(i)=SD.TotalDist(i-1)+SD.Dist(i);                         % Calculate the total traveled distance based on the encoder reading
            SD.TotalAngle(i)=SD.TotalAngle(i-1)+SD.Angle(i);                      % Calculate the total traveled angle based on the encoder reading
        end
        
        % Read the Sonar data       
        distSonar=genSonar(serPort);
        SD.RF_R(i)=distSonar(1)+ normrnd(0, Range_Sigma);              % with noise added
        SD.RF_F(i)=distSonar(2)+ normrnd(0, Range_Sigma);              % with noise added
        SD.RF_L(i)=distSonar(3)+ normrnd(0, Range_Sigma);              % with noise added
        SD.RF_B(i)=distSonar(4)+ normrnd(0, Range_Sigma);              % with noise added
        
        % Read the Lidar data       
        SD.Lidar(:,i)= LidarSensorCreate(serPort);
        %plot(distScan);
        
        % Simulate the IMU here
        [Create_x Create_y Create_theta]= OverheadLocalizationCreate(serPort);
        SD.Truth_x(i)=Create_x;
        SD.Truth_y(i)=Create_y;
        SD.Truth_theta(i)=Create_theta;       % getting the truth values
        
        SD.Vz(i)=0;                             
        SD.Az(i)=-gravity + normrnd(Az_Bias, Accel_Sigma);                   % with noise added
        SD.P(i)=0+normrnd(P_Bias, Gyro_Sigma);                               % with noise added
        SD.Q(i)=0+normrnd(Q_Bias, Gyro_Sigma);                               % with noise added
        if i>1
            Local_Vx=(SD.Truth_x(i)-SD.Truth_x(i-1))/SD.Time_Diff(i);
            Local_Vy=(SD.Truth_y(i)-SD.Truth_y(i-1))/SD.Time_Diff(i);
            temp_delta_theta=SD.Truth_theta(i)-SD.Truth_theta(i-1); 
            if temp_delta_theta<-pi            %fixing the wrap around issue.
                temp_delta_theta=temp_delta_theta+2*pi;
            end
            if temp_delta_theta>pi             %fixing the wrap around issue.
                temp_delta_theta=temp_delta_theta-2*pi;
            end
            SD.R(i)=(temp_delta_theta)/SD.Time_Diff(i) + normrnd(R_Bias, Gyro_Sigma);   % with noise added
            SD.Vx(i)=cos(SD.Truth_theta(i))*Local_Vx+sin(SD.Truth_theta(i))*Local_Vy;                        % rotate to the body frame
            SD.Vy(i)=-sin(SD.Truth_theta(i))*Local_Vx+cos(SD.Truth_theta(i))*Local_Vy;
            SD.Ax(i)=(SD.Vx(i)-SD.Vx(i-1))/SD.Time_Diff(i)/gravity + normrnd(Ax_Bias, Accel_Sigma);          % with noise added
            SD.Ay(i)=(SD.Vy(i)-SD.Vy(i-1))/SD.Time_Diff(i)/gravity + normrnd(Ay_Bias, Accel_Sigma);          % with noise added
        end
        SD.Roll(i)=0 + normrnd(0, Euler_Sigma);                              % with noise added
        SD.Pitch(i)=0 + normrnd(0, Euler_Sigma);                             % with noise added
        SD.Yaw(i)=Create_theta + normrnd(0, Euler_Sigma);                    % with noise added
        
        if i==1
            SD.X(i)=SD.Truth_x(i);
            SD.Y(i)=SD.Truth_y(i);
        else
            SD.X(i)=SD.X(i-1)+SD.Dist(i)*cos(SD.Yaw(i));                     % Dead Reckoning for X position
            SD.Y(i)=SD.Y(i-1)+SD.Dist(i)*sin(SD.Yaw(i));                     % Dead Reckoning for Y position
        end
        
        
        %% Put your custom robot control code here or run one of the examples below
        %----------------------------------------------------------
%         %% Beginnings of Decision Making Algorithm
        B=[cos(WO_pos(3))/2 cos(WO_pos(3))/2 0;
           sin(WO_pos(3))/2 sin(WO_pos(3))/2 0;
           0 0 1]*Ts;
        Yaw_error = Yaw_Reference - WO_pos(3);      
        Sum_error= Sum_error + Yaw_error;
        Prop_term= Proportional_Gain*Yaw_error;
        Int_term= Int_Gain*Sum_error;
        Deriv_term= Ts*Deriv_Gain*SD.R(i);
        if(i<=20)
            leftWSpd= 0;
            rightWSpd= 0;
            if(i==20)
                ygBias= mean(SD.R)
            end
        elseif(WO_pos(1)>=3 && WO_pos(3)<(pi/2)) % && (-0.1<WO_pos(2) && WO_pos(2)<0.1)
            leftWSpd= -0.025*Create_Full_Speed;
            rightWSpd= 0.025*Create_Full_Speed;
            Yaw_Reference= pi/2;
        elseif(WO_pos(2)>=3 && WO_pos(3)<pi) % (2.9<WO_pos(1) && WO_pos(1)<3.1) &&
            leftWSpd= -0.025*Create_Full_Speed;
            rightWSpd= 0.025*Create_Full_Speed;
            Yaw_Reference= pi;
        elseif(WO_pos(1)<=0 && WO_pos(3)>=pi && WO_pos(3)<(3*pi/2)) % && (2.9<WO_pos(2) && WO_pos(2)<3.1)
            leftWSpd= -0.025*Create_Full_Speed;
            rightWSpd= 0.025*Create_Full_Speed;
            Yaw_Reference= 3*pi/2;
        elseif(WO_pos(2)<=0 && WO_pos(3)>=(3*pi/2) && WO_pos(3)<(2*pi)) % (-0.1<WO_pos(1) && WO_pos(1)<0.1) && 
            leftWSpd= -0.025*Create_Full_Speed;
            rightWSpd= 0.025*Create_Full_Speed;
            Yaw_Reference= 2*pi;
        else
            leftWSpd= (0.25-Deriv_term-Prop_term-Int_term)*Create_Full_Speed;
            rightWSpd= (0.25+Deriv_term+Prop_term+Int_term)*Create_Full_Speed;
            if(WO_pos(3)<=2*pi+0.1 && WO_pos(3)>=2*pi-0.1)
                WO_pos(3)=WO_pos(3)-2*pi;
                loopCnt= loopCnt+1
                Yaw_Reference= 0;
            end
        end
        SetDriveWheelsCreate(serPort, rightWSpd, leftWSpd);
        Vw=[leftWSpd;rightWSpd;SD.R(i)];
        WO_pos=WO_pos+B*Vw
        
        %% End of the Custom Control Code
        
        % Calculate the time left in the constant interval
        SD.Delay(i)=Ts-toc;
        if SD.Delay(i)>0
            pause(SD.Delay(i));     % Kill the remaining time
        end
        i=i+1;
    end
    
    Total_Elapse_Time = SD.Time(Total_Steps)-SD.Time(1)  % Calcualte the total elapse time, not counting the minutes
    SetDriveWheelsCreate(serPort, 0, 0);                 % stop the robot
    BeepRoomba(serPort);       % Make a Beeping Sound
    % Specify output parameter
    % Function_Output = 0;
    
    % Stop robot motion
    SetFwdVelAngVelCreate(serPort,0,0)
    save('SMART_DATA.mat', 'SD');               % Save all the collected data to a .mat file
    Create_Sim_PLOT;                            % Plot all the robot data
end

