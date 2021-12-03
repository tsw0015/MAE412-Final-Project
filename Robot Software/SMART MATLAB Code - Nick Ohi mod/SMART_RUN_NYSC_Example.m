%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the main code for runing the SMART Robot                  %
% Author: Yu Gu                                                     %
% This is the version with the Lightware Laser interface            %
% The Kinect interface is removed                                   % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

% Definitions and initializations
Ts_Desired=0.1;                              % Desired sampling time
Ts=0.1;                                      % sampling time is 0.1 second. It can be reduced 
                                             % slightly to offset other overhead in the loop
Tend=300;                                    % Was 60 seconds;
Total_Steps=Tend/Ts_Desired;                 % The total number of time steps;
Create_Full_Speed=0.2;                       % The highest speed the robot can travel. (Max is 0.5m/s)
Create_Min_Speed=0.07;                       % The minimum speed in order for the robot to move (m/s)
avoiding = false;                            % Set to true when avoiding an obstacle
avoidingState = 1;                           % 1 = turn, 2 = drive forward
avoidAngleToTurnLeft = -deg2rad(35);          % Angle to turn left when avoiding (radians)
avoidAngleToTurnRight = deg2rad(35);         % Angle to turn left when avoiding (radians)
avoidAngleToTurn = avoidAngleToTurnLeft;     % Initial value for angle to turn during avoid (radians)
avoidDriveDistance = 0.25;                    % Distance to drive forward during avoid (meters)
avoidBackupDistance = -0.2;                   % Distance to drive backward during avoid (meters)
avoidTriggerDistance = 5;                  % Distance from front range finder (mm)
goalX = 3.66;                                % Goal x location (meters)
goalY = 0.61;                                % Goal y location (meters)

% Keyboard input definitions
global keyboardStruct
global fig
keyboardStruct = struct('quit',0,'leftSpeed',0,'rightSpeed',0,'maxSpeed',Create_Full_Speed);
fig = figure;

% Magnetometer Calibration Data
Mag_A=[  2.3750     0.2485      -0.2296
         0          2.6714      -0.1862
         0          0           2.5061];        % estimated shape of the soft iron effect
Mag_c=[ 0.0067;     0.2667;     0.0473];        % estimated center of the hard iron effect

% Rate Gyro Biases
P_Bias=0;
Q_Bias=0;
R_Bias=0;
% P_Bias=-0.0019;
% Q_Bias= 0.0053;
% R_Bias=-0.0149;

% Define the main Data Structure
SD= struct( 'Index',zeros(1,Total_Steps),...            % SD stands for SMART Data, which stores all the robot data
            'Time', zeros(1,Total_Steps),...            % The actual time at each time step (s) 
            'Time_Diff', zeros(1,Total_Steps),...       % The time difference between two step (s) 
            'Delay',zeros(1,Total_Steps),...            % Delay needed between each time step. It indicate how much time avaliable for other compuations. 
            'Lidar_Angle', zeros(682,Total_Steps),...   % Hukuyo Lidar Scan Angles             
            'Lidar_Range', zeros(682,Total_Steps),...   % Hukuyo Lidar Scan Range             
            'Logger_Counter', zeros(1,Total_Steps),...  % The Counter number for the SMART data logger 
            'Ax',zeros(1,Total_Steps),...               % Acceleration along the x-Axis (g)
            'Ay',zeros(1,Total_Steps),...               % Acceleration along the y-Axis (g)
            'Az',zeros(1,Total_Steps),...               % Acceleration along the z-Axis (g)
            'P',zeros(1,Total_Steps),...                % Roll Rate (deg/s)
            'Q',zeros(1,Total_Steps),...                % Pitch Rate (deg/s)
            'R',zeros(1,Total_Steps),...                % Yaw Rate (deg/s)
            'Mx',zeros(1,Total_Steps),...               % Magnetic strength along the x-axis (G)
            'My',zeros(1,Total_Steps),...               % Magnetic strength along the y-axis (G)
            'Mz',zeros(1,Total_Steps),...               % Magnetic strength along the z-axis (G)
            'IMU_T',zeros(1,Total_Steps),...            % IMU internal temperacture (C)
            'Roll',zeros(1,Total_Steps),...             % Roll Angle (rad)
            'Pitch',zeros(1,Total_Steps),...            % Pitch Angle (rad)
            'Yaw',zeros(1,Total_Steps),...              % Yaw Angle (rad) 
            'Mag_Heading',zeros(1,Total_Steps),...      % Magnetic Heading (rad) 
            'X',zeros(1,Total_Steps),...                % Robot X Position (m) 
            'Y',zeros(1,Total_Steps),...                % Robot Y Position (m) 
            'RF_F',zeros(1,Total_Steps),...             % Front Range Finder (mm)
            'RF_FL',zeros(1,Total_Steps),...            % Front-Left Range Finder (mm)
            'RF_L',zeros(1,Total_Steps),...             % Left Range Finder (mm)
            'RF_B',zeros(1,Total_Steps),...             % Back Range Finder (mm)
            'RF_R',zeros(1,Total_Steps),...             % Right Range Finder (mm)
            'RF_FR',zeros(1,Total_Steps),...            % Front Right Range Finder (mm)
            'Laser_RF', zeros(1,Total_Steps),...        % Laser Range Finder (mm)
            'Wall', zeros(1,Total_Steps),...            % Wall sensor of Create (0/1)
            'VirtWall', zeros(1,Total_Steps),...        % Detect the Virtual Wall (0/1)
            'Dist', zeros(1,Total_Steps),...            % Distance Traveled Since Last Call (m)
            'TotalDist', zeros(1,Total_Steps),...       % Total Distance Traveled(m)
            'Angle', zeros(1,Total_Steps),...           % Angle Traveled Since Last Call (rad)
            'TotalAngle', zeros(1,Total_Steps),...      % Total Angle Traveled (rad)
            'CreateVolts', zeros(1,Total_Steps),...     % Voltage of the Create Robot (rad)
            'CreateCurrent', zeros(1,Total_Steps),...   % Current of the Create Robot (rad)
            'BumpRight', zeros(1,Total_Steps),...       % Right bump sensor state (0/1)
            'BumpLeft', zeros(1,Total_Steps),...        % Left bump sensor state (0/1)
            'BumpFront', zeros(1,Total_Steps),...       % Front bump sensor state (0/1)
            'CliffLeft', zeros(1,Total_Steps),...       % Left cliff sensor state (0/1)
            'CliffRight', zeros(1,Total_Steps),...      % Right cliff sensor state (0/1)
            'CliffFrontLeft', zeros(1,Total_Steps),...  % Front left cliff sensor state (0/1)
            'CliffFrontRight', zeros(1,Total_Steps));   % Front right cliff sensor state (0/1)
            
        
   
% Initialize the Serial Port for the Data Logger
S_Logger=Init_Logger('2');  % May be different on different computer
% Initialize the Serial Port for iRobot Create
S_Create=RoombaInit(1);    
% Initialize the Serial Port for LightWare Laser Rangefinder
S_LightWare=Init_LightWare('3');     

% Initialize drive data struct
driveData = struct('state',1,...               % execution state for drive functions 1 = init, 2 = exec, 3 = finish
                   'initX',0,...                    % initial x position
                   'initY',0,...                    % initial y position
                   'initYaw',0,...                  % initial yaw
                   'integralAccum',0,...            % accumulator for integral controllers
                   'maxSpeed',Create_Full_Speed,... % max speed robot wheels can move
                   'minSpeed',Create_Min_Speed);    % min speed to move robot


% Initialize the Serial Port for Hukuyo Lidar
% S_Hokuyo=Init_Hokuyo(7);    % initalize the Lidar
% pause(0.1);
% fscanf(S_Hokuyo)
% rangescan=zeros(1, 682);  

% for i=1:Total_Steps
%     SD.Lidar_Angle(:,i)=(-120:240/682:120-240/682)*pi/180;   % an Array of all the scan angles
% end
% fprintf(S_Hokuyo,'GD0044072500');           % request a scan

flushinput(S_Logger);       % Flush the data logger serial port
flushinput(S_Create);       % Flush the iRobot Create serial port
fwrite(S_Create, [142 0]);  % Request all sensor data from Create
BeepRoomba(S_Create);       % Make a Beeping Sound
pause(0.1);

% Starting the main loop
for i=1:Total_Steps
    tic
    SD.Index(i)=i;
    
    % Acquire data from the sensor interface board
    if i==1
        [SD.Logger_Counter(i), SD.Ax(i), SD.Ay(i), SD.Az(i), Raw_P, Raw_Q, Raw_R, Raw_Mx, Raw_My, Raw_Mz, SD.IMU_T(i), A2D_Ch1, A2D_Ch2, SD.RF_F(i), SD.RF_FL(i), SD.RF_L(i), SD.RF_B(i), SD.RF_R(i), SD.RF_FR(i)] = Read_Logger_2(S_Logger,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0); 
    else
        [SD.Logger_Counter(i), SD.Ax(i), SD.Ay(i), SD.Az(i), Raw_P, Raw_Q, Raw_R, Raw_Mx, Raw_My, Raw_Mz, SD.IMU_T(i), A2D_Ch1, A2D_Ch2, SD.RF_F(i), SD.RF_FL(i), SD.RF_L(i), SD.RF_B(i), SD.RF_R(i), SD.RF_FR(i)] = Read_Logger_2(S_Logger,SD.Logger_Counter(i-1),SD.Ax(i-1),SD.Ay(i-1),SD.Az(i-1),Raw_P, Raw_Q, Raw_R, Raw_Mx, Raw_My, Raw_Mz, SD.IMU_T(i-1), A2D_Ch1, A2D_Ch2, SD.RF_F(i-1), SD.RF_FL(i-1), SD.RF_L(i-1), SD.RF_B(i-1), SD.RF_R(i-1), SD.RF_FR(i-1)); 
    end
    SD.P(i)=Raw_P-P_Bias;    SD.Q(i)=Raw_Q-Q_Bias;    SD.R(i)=Raw_R-R_Bias;   % Calibrate the gyro data
    temp=Mag_A*[Raw_Mx-Mag_c(1); Raw_My-Mag_c(2); Raw_Mz-Mag_c(3)];           % Magnetometer Raw Data Correction
    SD.Mx(i)=temp(1); SD.My(i)=temp(2); SD.Mz(i)=temp(3);
    
    flushinput(S_Logger);       % Flush the data logger serial port

    % Acquire data from the LightWare Laser Rangefinder
    if i==1
        SD.Laser_RF(i) = Read_LightWare(S_LightWare,0);
    else
        SD.Laser_RF(i) = Read_LightWare(S_LightWare,SD.Laser_RF(i-1));
    end
    flushinput(S_LightWare);    % Flush the LightWare Laser Rangefinder serial port

    % Acquire the data from the iRobot Create
    [SD.BumpRight(i),SD.BumpLeft(i),SD.BumpFront(i),SD.Wall(i),SD.VirtWall(i),SD.CliffLeft(i),SD.CliffRight(i),SD.CliffFrontLeft(i),SD.CliffFrontRight(i),LeftCurrOver,RightCurrOver,DirtL,DirtR,ButtonPlay,ButtonAv,SD.Dist(i),SD.Angle(i),SD.CreateVolts(i),SD.CreateCurrent(i),Temp,Charge,Capacity,pCharge]=Read_Create_2(S_Create);
    flushinput(S_Create);       % Flush the iRobot Create serial port
    fwrite(S_Create, [142 0]);  % Request all sensor data from Create

    % Acquire data from Hukuyo Lidar
%     rangescan = Read_Hokuyo(S_Hokuyo, rangescan);
%     SD.Lidar_Range(:,i)=rangescan;
%     SD.Laser_RF(i) = rangescan(341);    % simulated laser rangefinder using the LIDAR center spot
%     plot(rangescan);
%     drawnow

%     flushinput(S_Hokuyo);       % Flush the Lidar serial port
%     fprintf(S_Hokuyo,'GD0044072500');           % request a new Lidar scan

    Time=clock;                     % Mark the current time;
    SD.Time(i)=Time(6);             % Store the seconds;
    if i==1
        SD.Roll(i)=atan2(SD.Ay(i), SD.Az(i));                                 % Calculate the Roll angle based on the gravity vector
        SD.Pitch(i)=atan(-SD.Ax(i)/(SD.Ay(i)*sin(SD.Roll(i))+SD.Az(i)*cos(SD.Roll(i))));    % Calculate the Roll angle based on the pitch vector
        SD.Mag_Heading(i)=atan2(-SD.My(i), SD.Mx(i));                         % The initial 2D magnetic heading of the robot
        SD.Yaw(i)=0;                                                          % Set the current yaw angle as zero
        Attitude_P=zeros(3,3);                                                % Initialization the error covariance matrix for attitude estimation 
        SD.X(i)=0.457;                                                        % Initial Robot X Position
        SD.Y(i)=1.981;                                                        % Inital Robot Y Position
    else                                                                      % If i>1
        SD.Time_Diff(i)=SD.Time(i)-SD.Time(i-1);                              % Calculate the time difference between steps
        if SD.Time_Diff(i)<0
            SD.Time_Diff(i)=SD.Time_Diff(i)+60;                               % Compensate for the minute change
        end
        SD.TotalDist(i)=SD.TotalDist(i-1)+SD.Dist(i);                         % Calculate the total traveled distance based on the encoder reading
        SD.TotalAngle(i)=SD.TotalAngle(i-1)+SD.Angle(i);                      % Calculate the total traveled angle based on the encoder reading
        
        SD.Mag_Heading(i)=atan2(-SD.My(i), SD.Mx(i));                         % The 2D magnetic heading of the robot

        [SD.Roll(i), SD.Pitch(i), SD.Yaw(i), Attitude_P] = Attitude_Estimation(SD.Mag_Heading(1), SD.Roll(i-1), SD.Pitch(i-1), SD.Yaw(i-1), SD.Time_Diff(i), Attitude_P, SD.Ax(i), SD.Ay(i), SD.Az(i), SD.P(i), SD.Q(i), SD.R(i), SD.Mx(i), SD.My(i), SD.Mz(i));     % Perform the attitude estimation 
        SD.X(i)=SD.X(i-1)+SD.Dist(i)*cos(SD.Yaw(i));                          % Dead Reckoning for X position
        SD.Y(i)=SD.Y(i-1)+SD.Dist(i)*sin(SD.Yaw(i));                          % Dead Reckoning for Y position
    end
    
    
    %% Put your custom control functions here
    %----------------------------------------------------------
    % Use the following function for the robot wheel control:
    % SetDriveWheelsSMART(S_Create, rightWheelVel, leftWheelVel, SD.CliffLeft(i),SD.CliffRight(i),SD.CliffFrontLeft(i),SD.CliffFrontRight(i));

    catchKeyPress;
    
    if avoiding
        switch avoidingState
            case 1 % back up
                [avoidDone, driveData] = DriveDistance(avoidBackupDistance, S_Create, SD, driveData, i);
                if avoidDone
                    avoidingState = 2;
                else
                    avoidingState = 1;
                end
                avoiding = true;
            case 2 % turn
                [avoidDone, driveData] = TurnAngle(avoidAngleToTurn, S_Create, SD, driveData, i);
                if avoidDone
                    avoidingState = 3;
                else
                    avoidingState = 2;
                end
                avoiding = true;
            case 3 % drive forward
                [avoidDone, driveData] = DriveDistance(avoidDriveDistance, S_Create, SD, driveData, i);
                if avoidDone
                    avoidingState = 1;
                    avoiding = false;
                    driveData = ResetDriveState(driveData);
                else
                    avoidingState = 3;
                    avoiding = true;
                end
                if SD.RF_F(i)<avoidTriggerDistance  || SD.BumpFront(i) || SD.BumpLeft(i) || SD.BumpRight(i)
                    if SD.BumpLeft(i)
                        avoidAngleToTurn = avoidAngleToTurnRight;
                    elseif SD.BumpRight(i)
                        avoidAngleToTurn = avoidAngleToTurnLeft;
                    else
                        if SD.RF_L < SD.RF_R
                            avoidAngleToTurn = avoidAngleToTurnRight;
                        else
                            avoidAngleToTurn = avoidAngleToTurnLeft;
                        end
                    end
                    avoiding = true;
                    avoidingState = 1;
                    SetDriveWheelsSMART(S_Create, 0, 0, SD.CliffLeft(i),SD.CliffRight(i),SD.CliffFrontLeft(i),SD.CliffFrontRight(i),0,0,0);
                    driveData = ResetDriveState(driveData);
                end
        end
    else
        if SD.RF_F(i)<avoidTriggerDistance || SD.BumpFront(i) || SD.BumpLeft(i) || SD.BumpRight(i)
            if SD.BumpLeft(i)
                avoidAngleToTurn = avoidAngleToTurnRight;
            elseif SD.BumpRight(i)
                avoidAngleToTurn = avoidAngleToTurnLeft;
            else
                if SD.RF_L < SD.RF_R
                    avoidAngleToTurn = avoidAngleToTurnRight;
                else
                    avoidAngleToTurn = avoidAngleToTurnLeft;
                end
            end
            avoiding = true;
            avoidingState = 1;
            forwardSpeed = 0;
            turningSpeed = 0;
            driveData = ResetDriveState(driveData);
        else
            yawError = atan2(goalY - SD.Y(i), goalX - SD.X(i)) - mod(SD.Yaw(i), sign(SD.Yaw(i))*2*pi);
            fprintf('yawError = %f\n',rad2deg(yawError));
            if yawError > -pi/2 && yawError < pi/2
                forwardSpeed = Create_Full_Speed*0.5*cos(yawError);
                turningSpeed = 1.5*yawError/(pi/2)*Create_Full_Speed*0.5;
            else
                forwardSpeed = 0;
                turningSpeed = sign(yawError)*Create_Full_Speed*0.5;
            end
        end
        fprintf('forwardSpeed = %f\n',forwardSpeed);
        fprintf('turningSpeed = %f\n',turningSpeed);
        fprintf('goalXY = [%f,%f]\n',goalX,goalY);
        fprintf('currentXY = [%f,%f]\n',SD.X(i),SD.Y(i));
        fprintf('currentYaw = %f\n',rad2deg(SD.Yaw(i)));
        fprintf('distanceToGoal = %f\n\n',sqrt((goalX - SD.X(i))^2 + (goalY - SD.Y(i))^2))
        rightWheelVel = forwardSpeed - turningSpeed;
        leftWheelVel = forwardSpeed + turningSpeed;
        SetDriveWheelsSMART(S_Create, rightWheelVel, leftWheelVel, SD.CliffLeft(i),SD.CliffRight(i),SD.CliffFrontLeft(i),SD.CliffFrontRight(i),0,0,0);
    end
    if sqrt((goalX - SD.X(i))^2 + (goalY - SD.Y(i))^2) < 0.2 || keyboardStruct.quit
        break
    end
    
    % Collision Avoidance Example(Set Tend=60s)
%     if SD.RF_F(i)<500
%         SetDriveWheelsSMART(S_Create, -Create_Full_Speed*.4, -Create_Full_Speed*.6, CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,BumpRight,BumpLeft,BumpFront);
%     elseif SD.RF_B(i)<300
%         SetDriveWheelsSMART(S_Create, Create_Full_Speed*.6, Create_Full_Speed*.4, CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,BumpRight,BumpLeft,BumpFront);
%     elseif SD.RF_L(i)<500
%         SetDriveWheelsSMART(S_Create, -Create_Full_Speed*.5, Create_Full_Speed*0.5, CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,BumpRight,BumpLeft,BumpFront);
%     elseif SD.RF_R(i)<500
%         SetDriveWheelsSMART(S_Create, Create_Full_Speed*0.5, -Create_Full_Speed*.5, CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,BumpRight,BumpLeft,BumpFront);
%     else
%         SetDriveWheelsSMART(S_Create, Create_Full_Speed, Create_Full_Speed, CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,BumpRight,BumpLeft,BumpFront);
%     end

    %----------------------------------------------------------
    %% End of the Custom Control Code

    % Calculate the time left in the constant interval
    SD.Delay(i)=Ts-toc;         
    if SD.Delay(i)>0
        pause(SD.Delay(i));     % Kill the remaining time
    end
end
Total_Elapse_Time=SD.Time(Total_Steps)-SD.Time(1)  % Calcualte the total elapse time, not counting the minutes
SetDriveWheelsSMART(S_Create, 0, 0, SD.CliffLeft(end),SD.CliffRight(end),SD.CliffFrontLeft(end),SD.CliffFrontRight(end),0,0,0);       % Stop the wheels
BeepRoomba(S_Create);       % Make a Beeping Sound

% Properly close the serial ports
delete(S_Logger)
clear S_Logger  
delete(S_Create)
clear S_Create  
delete(S_LightWare)
clear S_LightWare  
% fprintf(S_Hokuyo,'QT');
% fclose(S_Hokuyo);

save('SMART_DATA.mat', 'SD');       % Save all the collected data to a .mat file
%SMART_PLOT;                         % Plot all the robot data

% Define custom drive functions
function [driveDone, driveData] = DriveDistance(desiredDistance, S_Create, SD, driveDataIn, i)
    Kp = 1.2;
    driveData = driveDataIn;
    switch driveData.state
        case 1 % init
            driveData = ResetDriveState(driveData);
            speed = 0;
            driveData.initX = SD.X(i);
            driveData.initY = SD.Y(i);
            driveData.initYaw = SD.Yaw(i);
            driveDone = false;
            driveData.state = 2;
        case 2 % exec
            drivenDistance = sqrt((SD.X(i) - driveData.initX)^2 + (SD.Y(i) - driveData.initY)^2);
            if desiredDistance > 0
                remainingDistance = desiredDistance - drivenDistance;
            else 
                remainingDistance = desiredDistance + drivenDistance;
            end
            speed = Kp*remainingDistance;
            if speed > driveData.maxSpeed
                speed = driveData.maxSpeed;
            elseif speed < -driveData.maxSpeed
                speed = -driveData.maxSpeed;
            end
            if speed > 0 && speed < driveData.minSpeed
                speed = driveData.minSpeed;
            elseif speed < 0 && speed > -driveData.minSpeed
                speed = -driveData.minSpeed;
            end
            driveDone = false;
            if drivenDistance >= abs(desiredDistance)
                driveData.state = 3;
            else
                driveData.state = 2;
            end
        case 3 % finish
            speed = 0;
            driveDone = true;
            driveData = ResetDriveState(driveData);
    end
    SetDriveWheelsSMART(S_Create, speed, speed, SD.CliffLeft(i),SD.CliffRight(i),SD.CliffFrontLeft(i),SD.CliffFrontRight(i),0,0,0);
end

function [driveDone, driveData] = TurnAngle(desiredDeltaAngle, S_Create, SD, driveDataIn, i)
    Kp = 0.6;
    driveData = driveDataIn;
    switch driveData.state
        case 1 % init
            driveData = ResetDriveState(driveData);
            speed = 0;
            driveData.initX = SD.X(i);
            driveData.initY = SD.Y(i);
            driveData.initYaw = SD.Yaw(i);
            driveDone = false;
            driveData.state = 2;
        case 2 % exec
            turnedAngle = SD.Yaw(i) - driveData.initYaw;
            remainingAngle = desiredDeltaAngle - turnedAngle;
            speed = Kp*remainingAngle;
            if speed > driveData.maxSpeed
                speed = driveData.maxSpeed;
            elseif speed < -driveData.maxSpeed
                speed = -driveData.maxSpeed;
            end
            if speed > 0 && speed < driveData.minSpeed
                speed = driveData.minSpeed;
            elseif speed < 0 && speed > -driveData.minSpeed
                speed = -driveData.minSpeed;
            end
            driveDone = false;
            if abs(remainingAngle) <= deg2rad(1)
                driveData.state = 3;
            else
                driveData.state = 2;
            end
        case 3 % finish
            speed = 0;
            driveDone = true;
            driveData = ResetDriveState(driveData);
    end
    SetDriveWheelsSMART(S_Create, -speed, speed, SD.CliffLeft(i),SD.CliffRight(i),SD.CliffFrontLeft(i),SD.CliffFrontRight(i),0,0,0);
end

function driveData = ResetDriveState(driveDataIn)
    driveData = driveDataIn;
    driveData.state = 1; % Reset to init state
    driveData.integralAccum = 0; % Reset integral accumulator to zero
end