%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the main code for runing the SMART Robot                  %
% Author: Yu Gu                                                     %
% This is the version with Hukuyo lidar interface                   %
% The Kinect interface is removed                                   % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

% Definitions
Ts_Desired=0.1;           % Desired sampling time
Ts=0.1;                   % sampling time is 0.1 second. It can be reduced 
                          % slightly to offset other overhead in the loop
Tend=5;                  % Was 60 seconds;
Total_Steps=Tend/Ts_Desired;    % The total number of time steps;
Create_Full_Speed=1;      % The highest speed the robot can travel. (Max is 0.5m/s)

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
            'CreateCurrent', zeros(1,Total_Steps));     % Current of the Create Robot (rad)
   
% Initialize the Serial Port for the Data Logger
S_Logger=Init_Logger('1');  % May be different on different computer
% Initialize the Serial Port for iRobot Create
S_Create=RoombaInit('2');    
% Initialize the Serial Port for LightWare Laser Rangefinder
%_LightWare=Init_LightWare('3');     

% Initialize LIDAR using ROS
% rosshutdown % shutdown any previous node
% rosinit % initialize a ros node
% 
% lidar = rossubscriber('/scan');

rosshutdown
rosinit
markerTransform = rossubscriber('/marker/transform');
markerID = rossubscriber('/marker/ID');

%OUR VARIABLES
Drive_Straight=1;      % 1 - start, 0 finished
Turn_In_Place=0;        % 1 - start, 0 finished
Last_Dist=0;            % The TotalDistance at the last stop
Last_Angle=0;           % The TotalAngle at the last stop
Yaw_Reference = 0;   % Referece yaw angle to be achieved by feedback control
Proportional_Gain=0.25;
Int_Gain=0;
Deriv_Gain=0;
Sum_error=0;
WO_pos=[0;0;0];
pos_update=[0;0;0];
loopCnt=1;
li=0;
ygBias=0;
yawAcc=0;
turnAdj=0;
edge_Length = 3;
turn_speed = .025;
transform=[0,0,0];
x_A =[0;0;0];
C= [1 0 0;
    0 1 0;
    0 0 1];
Q= [0.01 0 0;
    0 0.01 0;
    0 0 0.01]*Ts^2;         % Process noise covariance
R= 0.01^2;        % Measurement noise variance
P_P=eye(3);
I=eye(3);

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

% Poll the data a few times to clean the buffer
for i=1:10
%     lidar_data = receive(lidar,2);
%     plot(lidar_data)
%     drawnow
    [SD.Logger_Counter(i), SD.Ax(i), SD.Ay(i), SD.Az(i), Raw_P, Raw_Q, Raw_R, Raw_Mx, Raw_My, Raw_Mz, SD.IMU_T(i), A2D_Ch1, A2D_Ch2, SD.RF_F(i), SD.RF_FL(i), SD.RF_L(i), SD.RF_B(i), SD.RF_R(i), SD.RF_FR(i)] = Read_Logger_2(S_Logger,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0); 
    [BumpRight,BumpLeft,BumpFront,SD.Wall(i),SD.VirtWall(i),CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,LeftCurrOver,RightCurrOver,DirtL,DirtR,ButtonPlay,ButtonAv,SD.Dist(i),SD.Angle(i),SD.CreateVolts(i),SD.CreateCurrent(i),Temp,Charge,Capacity,pCharge]=Read_Create_2(S_Create);
end

% Starting the main loop
i=1;
while loopCnt<=1
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
    
    %FLIP THE SMART_BOARD For teh New Robot
    
    SD.Ax(i)=SD.Ax(i);
    SD.Ay(i)=-SD.Ay(i);
    SD.Az(i)=-SD.Az(i);
    SD.P(i)=SD.P(i);
    SD.Q(i)=-SD.Q(i);
    SD.R(i)=-SD.R(i);
    SD.Mx(i)=SD.Mx(i);
    SD.My(i)=-SD.My(i);
    SD.Mz(i)=-SD.Mz(i);
    
    %     % Acquire data from the LightWare Laser Rangefinder
    %     if i==1
    %         SD.Laser_RF(i) = Read_LightWare(S_LightWare,0);
    %     else
    %         SD.Laser_RF(i) = Read_LightWare(S_LightWare,SD.Laser_RF(i-1));
    %     end
    %     flushinput(S_LightWare);    % Flush the LightWare Laser Rangefinder serial port
    
    % Acquire the data from the iRobot Create
    [BumpRight,BumpLeft,BumpFront,SD.Wall(i),SD.VirtWall(i),CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,LeftCurrOver,RightCurrOver,DirtL,DirtR,ButtonPlay,ButtonAv,SD.Dist(i),SD.Angle(i),SD.CreateVolts(i),SD.CreateCurrent(i),Temp,Charge,Capacity,pCharge]=Read_Create_2(S_Create);
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
        SD.Roll(i)=0; %atan2(SD.Ay(i), SD.Az(i));                                 % Calculate the Roll angle based on the gravity vector
        SD.Pitch(i)=0; %atan(-SD.Ax(i)/(SD.Ay(i)*sin(SD.Roll(i))+SD.Az(i)*cos(SD.Roll(i))));    % Calculate the Roll angle based on the pitch vector
        SD.Mag_Heading(i)=atan2(-SD.My(i), SD.Mx(i));                         % The initial 2D magnetic heading of the robot
        SD.Yaw(i)=0;                                                          % Set the current yaw angle as zero
        Attitude_P=zeros(3,3);                                                % Initialization the error covariance matrix for attitude estimation
        SD.X(i)=0;                                                            % Initial Robot X Position
        SD.Y(i)=0;                                                            % Inital Robot Y Position
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
    
    
    % Acquire data from Lidar
    % Remove this section if you don't need Lidar
    
    %     if mod(i,2)==0
    %         lidar_data = receive(lidar,2);
    %         display(lidar_data.Ranges(1))
    %     % Comment out the plot functions to have better performance.
    %         plot(lidar_data)
    %         drawnow
    %     end
    
    
    %% Put your custom control functions here
    %----------------------------------------------------------
    % Use the following function for the robot wheel control:
    % SetDriveWheelsSMART(S_Create, rightWheelVel, leftWheelVel, SD.CliffLeft(i),SD.CliffRight(i),SD.CliffFrontLeft(i),SD.CliffFrontRight(i));
    
    %     SetDriveWheelsSMART(S_Create, Create_Full_Speed*.4, Create_Full_Speed*.4, CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,BumpRight,BumpLeft,BumpFront)
    
    B=[cos(WO_pos(3))/2 cos(WO_pos(3))/2 0;
        sin(WO_pos(3))/2 sin(WO_pos(3))/2 0;
        0 0 1]*Ts;
    Yaw_error = Yaw_Reference - WO_pos(3);
    Sum_error= Sum_error + Yaw_error;
    Prop_term= Proportional_Gain*Yaw_error;
    Int_term= Int_Gain*Sum_error;
    Deriv_term= Ts*Deriv_Gain*SD.R(i);
%     % CUSTOM OBSTACLE AVOIDANCE CODE
%     %instantiate flags
%     qFlag=-1; %indicates quadrant of obstacle 1-front 2-right 3-left 4-back -1- error
%     oFlag=-1; % indicates whether obstacle is present 1-yes  -1-no obstacle
%     
%     %check back quadrant
%     for z=137:227
%         if(SD.Lidar_Range(z,i)<0.5)
%             qFlag=4;
%             oFlag=1;
%         end
%     end
%     %check left quadrant
%     for z=228:315
%         if(SD.Lidar_Range(z,i)<0.5)
%             qFlag=3;
%             oFlag=1;
%         end
%     end
%     %check right quadrant
%     for z=137:227
%         if(SD.Lidar_Range(z,i)<0.5)
%             qFlag=2;
%             oFlag=1;
%         end
%     end
%     %check front quadrant
%     for z = 1:45
%         if(SD.Lidar_Range(z,i)<0.5)
%             qFlag=1;
%             oFlag=1;
%         end
%     end
%     for z= 316:360
%         if(SD.Lidar_Range(z,i)<0.5)
%             qFlag=1;
%             oFlag=1;
%         end
%     end
    if(li<=30 && rem(loopCnt,2)==0)
        leftWSpd= 0;
        rightWSpd= 0;
        yawAcc=SD.R(i);
        if(li==30)
            ygBias = mean(SD.R(i-li:i));
        end
        li=li+1;
    elseif(WO_pos(1)>=edge_Length && WO_pos(3)<(pi/2))
        leftWSpd= -turn_speed*Create_Full_Speed;
        rightWSpd= turn_speed*Create_Full_Speed;
        turnAdj= pi/900;
        Yaw_Reference=pi/2;
    elseif(WO_pos(2)>=edge_Length && WO_pos(3)<pi)
        leftWSpd= -turn_speed*Create_Full_Speed;
        rightWSpd= turn_speed*Create_Full_Speed;
        turnAdj= pi/900;
        Yaw_Reference=pi;
    elseif(WO_pos(1)<=0 && WO_pos(3)>=pi && WO_pos(3)<3*pi/2)
        leftWSpd= -turn_speed*Create_Full_Speed;
        rightWSpd= turn_speed*Create_Full_Speed;
        turnAdj= pi/900;
        Yaw_Reference=3*pi/2;
    elseif(WO_pos(2)<=0 && WO_pos(3)>=3*pi/2 && WO_pos(3)<2*pi)
        leftWSpd= -turn_speed*Create_Full_Speed;
        rightWSpd= turn_speed*Create_Full_Speed;
        turnAdj= pi/900;
        Yaw_Reference=2*pi;
    else
        leftWSpd= (0.25-Deriv_term-Prop_term-Int_term)*Create_Full_Speed;
        rightWSpd= (0.25+Deriv_term+Prop_term+Int_term)*Create_Full_Speed;
        yawAcc=0;
        turnAdj=0;
        if(WO_pos(3)<=2*pi+0.1 && WO_pos(3)>=2*pi-0.1)
            WO_pos(3)=WO_pos(3)-2*pi;
            loopCnt = loopCnt+1;
            li=0;
            Yaw_Reference=0;
        end
    end
%     if(oFlag==1)
%         switch qFlag
%             
%             %object in front
%             case 1
%                 leftWSpd= 0;
%                 rightWSpd= 0;
%                 %object on right
%             case 2
%                 disp('flag 2');
%                 %object on left
%             case 3
%                 disp('flag 3');
%                 %object behind
%             case 4
%                 disp('flag 4');
%         end
%     end
    SetDriveWheelsSMART(S_Create, rightWSpd, leftWSpd, CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,BumpRight,BumpLeft,BumpFront);
    
    if(~isempty(markerTransform.LatestMessage))
        curr_transform=[markerTransform.LatestMessage.Translation.X,markerTransform.LatestMessage.Translation.Y,markerTransform.LatestMessage.Translation.Z];
    end
    if(~isempty(markerTransform.LatestMessage) && transform(1)~=curr_transform(1) && transform(2)~=curr_transform(2) && transform(3)~=curr_transform(3))
        xrot=markerTransform.LatestMessage.Rotation.X;
        yrot=markerTransform.LatestMessage.Rotation.Y;
        zrot=markerTransform.LatestMessage.Rotation.Z;
        w=markerTransform.LatestMessage.Rotation.W;
        
        xt=markerTransform.LatestMessage.Translation.X;
        yt=markerTransform.LatestMessage.Translation.Y;
        zt=markerTransform.LatestMessage.Translation.Z;
        
        %Translation/Rotation Camera to AruCo Frames
        tA_C=[xt;yt;zt];
        
        rA_C=[1-2*yrot^2-2*zrot^2,2*xrot*yrot-2*zrot*w,2*xrot*zrot+2*yrot*w;
              2*xrot*yrot+2*zrot*w,1-2*xrot^2-2*zrot^2,2*yrot*zrot-2*xrot*w;
              2*xrot*zrot-2*yrot*w,2*yrot*zrot+2*xrot*w,1-2*xrot^2-2*yrot^2];
        %Invert to get transform AruCo to Camera Frames
        rA_C=rA_C';
        tA_C=-rA_C*tA_C;
        hTA_C= [rA_C(1,:),tA_C(1);
                rA_C(2,:),tA_C(2);
                rA_C(3,:),tA_C(3);
                0,0,0,1];

        %Translation/Rotation Camera to Body Frames
        tC_B=[0.18;-0.015;0.1];
        r=pi/2;
        p=-pi/2;
        yr=0;
        rC_B=[cos(p)*cos(yr),-cos(r)*sin(yr)+sin(r)*sin(p)*cos(yr),sin(r)*sin(yr)+cos(r)*sin(p)*cos(yr)
            cos(p)*sin(yr),cos(r)*cos(yr)+sin(r)*sin(p)*sin(yr),-sin(r)*cos(yr)+cos(r)*sin(p)*sin(yr)
            -sin(p),sin(r)*cos(p),cos(r)*cos(p)];
        hTC_B= [rC_B(1,:),tC_B(1);
            rC_B(2,:),tC_B(2);
            rC_B(3,:),tC_B(3);
            0,0,0,1];
        Vw= [leftWSpd;rightWSpd;-(SD.R(i)-ygBias-yawAcc)];
        transform= curr_transform;
        if(markerID.LatestMessage.Data==1)
            %Translation/Rotation Inertia to AruCo Frames
            r1=-pi/2;
            p1=pi/2; 
            yr1=0;
            rI_A=[cos(p1)*cos(yr1),-cos(r1)*sin(yr1)+sin(r1)*sin(p1)*cos(yr1),sin(r1)*sin(yr1)+cos(r1)*sin(p1)*cos(yr1)
                cos(p1)*sin(yr1),cos(r1)*cos(yr1)+sin(r1)*sin(p1)*sin(yr1),-sin(r1)*cos(yr1)+cos(r1)*sin(p1)*sin(yr1)
                -sin(p1),sin(r1)*cos(p1),cos(r1)*cos(p1)];
            tI_A=[3.75;0;0.1];
            hTI_A= [rI_A(1,:),tI_A(1);
                    rI_A(2,:),tI_A(2);
                    rI_A(3,:),tI_A(3);
                    0,0,0,1];
            %Multiply tranforms to get Inertial to Body Frames
            hTI_B=hTI_A*hTA_C*hTC_B;
            pos_update=[hTI_B(1:2,4);atan2(hTI_B(3,2),hTI_B(3,3))]
            
            %EKF
            F=[1 0 -Ts*0.5*(leftWSpd+rightWSpd)*sin(WO_pos(3));
               0 1 Ts*0.5*(leftWSpd+rightWSpd)*cos(WO_pos(3));
               0 0 1];
            
            x_A=F*WO_pos+B*Vw;
            P_A=F*P_P*F'+Q;     % calcuate the a priori error covariance
            K_k=(C*P_A*C'+R)\(P_A*C');
            WO_pos=x_A+K_k*(pos_update-C*x_A);     % calcuate the posterior states
            P_P=(I-K_k*C)*P_A;      % calcuate the posterior error covariance
        elseif(markerID.LatestMessage.Data==2)
            %Translation/Rotation Inertia to AruCo Frames
            r2=-pi/2;
            p2=0; 
            yr2=0;
            rI_A=[cos(p2)*cos(yr2),-cos(r2)*sin(yr2)+sin(r2)*sin(p2)*cos(yr2),sin(r2)*sin(yr2)+cos(r2)*sin(p2)*cos(yr2)
                cos(p2)*sin(yr2),cos(r2)*cos(yr2)+sin(r2)*sin(p2)*sin(yr2),-sin(r2)*cos(yr2)+cos(r2)*sin(p2)*sin(yr2)
                -sin(p2),sin(r2)*cos(p2),cos(r2)*cos(p2)];
            tI_A=[3;3.75;0.1];
            hTI_A= [rI_A(1,:),tI_A(1);
                    rI_A(2,:),tI_A(2);
                    rI_A(3,:),tI_A(3);
                    0,0,0,1];
            %Multiply tranforms to get Inertial to Body Frames
            hTI_B=hTI_A*hTA_C*hTC_B;
            pos_update=[hTI_B(1:2,4);atan2(hTI_B(3,2),hTI_B(3,3))]
            
            %EKF
            F=[1 0 -Ts*0.5*(leftWSpd+rightWSpd)*sin(WO_pos(3));
               0 1 Ts*0.5*(leftWSpd+rightWSpd)*cos(WO_pos(3));
               0 0 1];
            
            x_A=F*WO_pos+B*Vw;
            P_A=F*P_P*F'+Q;     % calcuate the a priori error covariance
            K_k=(C*P_A*C'+R)\(P_A*C');
            WO_pos=x_A+K_k*(pos_update-C*x_A);     % calcuate the posterior states
            P_P=(I-K_k*C)*P_A;      % calcuate the posterior error covariance
        elseif(markerID.LatestMessage.Data==3)
            %Translation/Rotation Inertia to AruCo Frames
            r3=0;
            p3=0; 
            yr3=0;
            rI_A=[cos(p3)*cos(yr3),-cos(r3)*sin(yr3)+sin(r3)*sin(p3)*cos(yr3),sin(r3)*sin(yr3)+cos(r3)*sin(p3)*cos(yr3)
                cos(p3)*sin(yr3),cos(r3)*cos(yr3)+sin(r3)*sin(p3)*sin(yr3),-sin(r3)*cos(yr3)+cos(r3)*sin(p3)*sin(yr3)
                -sin(p3),sin(r3)*cos(p3),cos(r3)*cos(p3)];
            tI_A=[-0.75;3;0.1];
            hTI_A= [rI_A(1,:),tI_A(1);
                    rI_A(2,:),tI_A(2);
                    rI_A(3,:),tI_A(3);
                    0,0,0,1];
            %Multiply tranforms to get Inertial to Body Frames
            hTI_B=hTI_A*hTA_C*hTC_B;
            pos_update=[hTI_B(1:2,4);atan2(hTI_B(3,2),hTI_B(3,3))]
            
            %EKF
            
            F=[1 0 -Ts*0.5*(leftWSpd+rightWSpd)*sin(WO_pos(3));
                0 1 Ts*0.5*(leftWSpd+rightWSpd)*cos(WO_pos(3));
                0 0 1];
            
            x_A=F*WO_pos+B*Vw;
            P_A=F*P_P*F'+Q;     % calcuate the a priori error covariance
            K_k=(C*P_A*C'+R)\(P_A*C');
            WO_pos=x_A+K_k*(pos_update-C*x_A);     % calcuate the posterior states
            P_P=(I-K_k*C)*P_A;      % calcuate the posterior error covariance
        elseif(markerID.LatestMessage.Data==4)
            %Translation/Rotation Inertia to AruCo Frames
            r4=-pi/2;
            p4=pi;
            yr4=0;
            rI_A=[cos(p4)*cos(yr4),-cos(r4)*sin(yr4)+sin(r4)*sin(p4)*cos(yr4),sin(r4)*sin(yr4)+cos(r4)*sin(p4)*cos(yr4)
                cos(p4)*sin(yr4),cos(r4)*cos(yr4)+sin(r4)*sin(p4)*sin(yr4),-sin(r4)*cos(yr4)+cos(r4)*sin(p4)*sin(yr4)
                -sin(p4),sin(r4)*cos(p4),cos(r4)*cos(p4)];
            tI_A=[0;-0.75;0.1];
            hTI_A= [rI_A(1,:),tI_A(1);
                rI_A(2,:),tI_A(2);
                rI_A(3,:),tI_A(3);
                0,0,0,1];
            %Multiply tranforms to get Inertial to Body Frames
            hTI_B=hTI_A*hTA_C*hTC_B;
            pos_update=[hTI_B(1:2,4);atan2(hTI_B(3,2),hTI_B(3,3))]
            
            %EKF
            F=[1 0 -Ts*0.5*(leftWSpd+rightWSpd)*sin(WO_pos(3));
                0 1 Ts*0.5*(leftWSpd+rightWSpd)*cos(WO_pos(3));
               0 0 1];
            
            x_A=F*WO_pos+B*Vw;
            P_A=F*P_P*F'+Q;     % calcuate the a priori error covariance
            K_k=(C*P_A*C'+R)\(P_A*C');
            WO_pos=x_A+K_k*(pos_update-C*x_A);     % calcuate the posterior states
            P_P=(I-K_k*C)*P_A;      % calcuate the posterior error covariance
        end
    elseif(~isempty(markerTransform.LatestMessage)&& transform(1)==curr_transform(1) && transform(2)==curr_transform(2) && transform(3)==curr_transform(3))
        %Fallback on Dead Reckoning if message doesn't update
        Vw=[leftWSpd;rightWSpd;-(SD.R(i)-ygBias-yawAcc)];
        WO_pos=WO_pos+B*Vw
        WO_pos(3)=WO_pos(3)+turnAdj;
    elseif(isempty(markerTransform.LatestMessage))
        %Fallback on Dead Reckoning if no markerTransform recieved
        Vw=[leftWSpd;rightWSpd;-(SD.R(i)-ygBias-yawAcc)];
        WO_pos=WO_pos+B*Vw
        WO_pos(3)=WO_pos(3)+turnAdj;
    end
    %----------------------------------------------------------
    
    
    %% End of the Custom Control Code

% Calculate the time left in the constant interval
SD.Delay(i)=Ts-toc;
if SD.Delay(i)>0
    pause(SD.Delay(i));     % Kill the remaining time
end
i=i+1;
end
Total_Elapse_Time=SD.Time(Total_Steps)-SD.Time(1)  % Calcualte the total elapse time, not counting the minutes
SetDriveWheelsSMART(S_Create, 0, 0, CliffLeft,CliffRight,CliffFrontLeft,CliffFrontRight,BumpRight,BumpLeft,BumpFront);       % Stop the wheels
BeepRoomba(S_Create);       % Make a Beeping Sound

% Properly close the serial ports
delete(S_Logger)
clear S_Logger  
delete(S_Create)
clear S_Create  
%delete(S_LightWare)
%clear S_LightWare  
% fprintf(S_Hokuyo,'QT');
% fclose(S_Hokuyo);

save('SMART_DATA.mat', 'SD');       % Save all the collected data to a .mat file
% SMART_PLOT;                         % Plot all the robot data

