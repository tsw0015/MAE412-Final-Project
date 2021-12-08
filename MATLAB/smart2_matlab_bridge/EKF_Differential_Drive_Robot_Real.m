%Conner Castle 

%Based off of the EKF example in Robotics, Vision and Control
%(2nd ed) - Peter Corke 2017 

%The main body of the program works by simulating a differential driving
%robot traveling through a map with a specified number of random goal
%locations. Once it gets close enough to a location it moves to the next.
%While doing so the program attempts to run an EKF to localize the robot
%and compares against the truth data of the simulation. 

clear 
clc 

rosshutdown; % shutdown any previous node
rosinit; % initialize a ros node
robot_name = "smart2_01"; %Name of the robot group to subscribe to
global ros_odom_data; 
global ros_odom_old_data; 
ros_odom_old_data = [0,0]; 
global x; 
global P; 
global sigma; 

% Subscribing to IMU using callback functions:
ros_odom = rossubscriber(robot_name+'/odom');
ros_aruco = rossubscriber('/Cam_Pose','geometry_msgs/Pose'); 
cmd_vel = rospublisher(robot_name+'/cmd_vel', 'geometry_msgs/Twist');


%Initializing the map 
x_size = 5; %Meters 
y_size = 5; %Meters  BN
LandMarkMap = GenerateLandMarks(x_size,y_size,NumLandmarks); 
NumGoals = 4; 

%Initializing the starting location 
x_start = [0 0 pi/2]; 
x = x_start; 
P = diag([.005, .005, .001].^2); 
x_rand = x_start + [sqrt(P(1,1))*randn,sqrt(P(2,2))*randn,sqrt(P(3,3))*randn]; 

%Internal vehicle and sensor parameters and initial values 
V = 0; %m/s 
w = 0; %rad/s 
sigma = diag([1, 1*pi()/180].^2); 
wm = diag([.001, 10*pi()/180].^2); 
%Choose to simulate with or without the EKF update 
EKF = 1; 

%Computing states after one step, initializes the system 
%[odom_x, odom_y, theta, odom_x_rand, odom_y_rand, theta_rand, D, D_rand] = odom(x,x_rand,V,w,Ts,sigma); 
%x = x + [odom_x odom_y theta]; 
%x_rand = x_rand + [odom_x_rand*cos(theta_rand) odom_y_rand*sin(theta_rand) theta_rand]; 
r = [x(1),x(2)];  
count = 2; 
x_goals = [1,0;1,1;0,1;0,0]; 
tic 

%for currentGoal = 1:NumGoals 
%    x_goal = x_goals(currentGoal,:); 
%    goals(currentGoal,:) = x_goal; 
%    while (sqrt((x_goal(1)-x(1))^2+(x_goal(2)-x(2))^2)>.09)
      while(1)  
        message = receive(ros_odom); 
        marker = receive(ros_aruco); 
        %disp(marker.Position.X)
        %marker.
        t = toc; 
        receive_data(message,t); 
         tic 
         if(marker.Orientation.W > -1 && marker.Orientation.W < 5) 
             ComputeUpdate(wm,LandMarkMap,marker); 
             %disp([marker.Position.X, marker.Position.Z marker.Orientation.W]);
         end

%         %Using random path controller 
%         [V,w] = controller(x,x_goal);
%         if(V > .3) 
%             V = .3; 
%         end 
%         if(w > 1)
%             w = 1; 
%         end 
%         if(w < -1)
%             w =-1; 
%         end 
%         cmd_vel_message = rosmessage(cmd_vel); 
%         cmd_vel_message.Linear.X = V; 
%         cmd_vel_message.Angular.Z = w; 
%         send(cmd_vel, cmd_vel_message);
        %Computing odometry data after a step 
        
%         if EKF == 1 
%             [x,P] = ComputeUpdate(wm,LandMarkMap); 
%         end
        
        %Plotting commands 
        %disp([x(1) x(2) x(3)]) 
        %disp([marker.Position.X, marker.Position.Z marker.Orientation.W]);
        robot = [x(1),x(2);x(1)+.3*cos(x(3)),x(2)+.3*sin(x(3))]; 
        plot(robot(1,1),robot(1,2),'*b',robot(2,1),robot(2,2),'*r')
        grid on
        %robot_rand = [x_rand(1),x_rand(2);x_rand(1)+.3*cos(x_rand(3)),x_rand(2)+.3*sin(x_rand(3))]; 
        %r(count,:) = [x(1),x(2)]; 
        %plot(r(:,1),r(:,2),'b')
        %plot(x_goal(1),x_goal(2),'+r',robot(1,1),robot(1,2),'*b',robot(:,1),robot(:,2),'*b',LandMarkMap(:,1),LandMarkMap(:,2),'dc')%,'*y')
        xlim([-1 x_size]) 
        ylim([-1 y_size]) 
        pause(.01) 
        
        %Iterating while loop 
        count = count + 1; 
    end
%end 

%Plotting true and random paths 
hold on  
plot(x_start(1),x_start(2),'ok') 
%plot(goals(:,1),goals(:,2),'+k') 
plot(r(:,1),r(:,2),'b') 
xlim([0 x_size + 10]) 
ylim([0 y_size + 10]) 

%Function to control the robot to move to a point 
function [V,w] = controller(x,x_goal)
Kv = .2;
Kh = .6;
%Kv = .5; 
%Kh = 1; 
V = Kv*sqrt((x_goal(1)-x(1))^2+(x_goal(2)-x(2))^2);
theta_prime = wrapToPi(atan2((x_goal(2)-x(2)),(x_goal(1)-x(1))) - x(3));
w = Kh*(theta_prime); 
end

%Function to update states of the robot using the computed odometry data 
function [x] = Xupdate(x,odo)
x = x + [odo(1) odo(2) odo(3)];
x(3) = wrapToPi(x(3));
end

%Function to update probability distribution of the states using priors and
% a best guess of the covariance of our input odometry 
function [P] = Pupdate(P,sigma,x,odo_rand) 
global x; 
%Jacobians for equation (6.2) should be the same equation as it does not
%rely on vehicle model information 
Fx = [1 0 -odo_rand(4)*sin(x_rand(3));0 1 odo_rand(4)*cos(x_rand(3)); 0 0 1]; 
Fv = [cos(x_rand(3)) 0;sin(x_rand(3)) 0; 0 1]; 

P = Fx*P*Fx' + Fv*sigma*Fv';  
end

%Function to update and correct the prediction state of the EKF 
function ComputeUpdate(wm,LandMarkMap,marker) 
global x; 
global P; 

%DistanceVector represents the real valued (with represented sensor error)
%observation of a landmark in the environment (this would be replaced with
%the sensor measurement in actual implementation because it represents the
%truth or correction for the internal model to use 
%DistanceVector = DistanceToLandmarks(x,LandMarkMap,wm); 
side = marker.Position.X; 
z = marker.Position.Z; 
id = marker.Orientation.W+1 
%DistanceVector(:) = [sqrt(side^2+z^2), (atan2(side,z)]; 
Distance = LandMarkMap(id,:) - [side, z]; 
DistanceVector(:) = [sqrt(Distance(1)^2+Distance(2)^2), (atan2(side,z))] 

%PredictedDistanceVector represents the internal model of where the robot
%thinks it is based on model prediction (this would stay in the actual
%implementation 
%PredictedDistanceVector = DistanceToLandmarks(LandMarkMap,id) 
PredictedDistanceVector = [sqrt(x(1)^2 + x(2)^2), x(3)]; 
% Landmark = randi(length(DistanceVector)); 
% laziness = 0; 
% bruh = 0; 
% while (abs(PredictedDistanceVector(Landmark,2))>90*pi/180) && bruh == 0 
%     Landmark = randi(length(DistanceVector)); 
%     laziness = laziness + 1; 
%     if laziness > 20
%         bruh = 1; 
%     end
% end
v = DistanceVector - PredictedDistanceVector; 
Hx = [-(LandMarkMap(id,1)-x(1))/DistanceVector(1),-(LandMarkMap(id,2)-x(2))/DistanceVector(1),0;(LandMarkMap(id,2)-x(2))/DistanceVector(1)^2, -(LandMarkMap(id,1)-x(1))/DistanceVector(1)^2, -1]; 
Hw = diag([1,1]); 
S = Hx*P*Hx'+Hw*wm*Hw';  
K = P*Hx'/S; 
x = x+(K*v')'; 
P = P-K*Hx*P; 
end 

%Model of differential drive robot used to determine the predicted next
%odometry of the robot 
function [odom_x, odom_y, theta, odom_x_rand, odom_y_rand, theta_rand, D, D_rand] = odom(x,x_rand,V,w,Ts,sigma)
%Computing odometry from one step (with and without randomness) 
l = .263; 
r = .065; 
Vr = (2*V + w*l)/2; 
Vl = 2*V - Vr; 
wr = Vr/(r); 
wl = Vl/(r); 
Dl = (wl*r)*Ts; 
Dr = (wr*r)*Ts; 
D = (Dl+Dr)/2; 

theta = (Dr-Dl)/l; 
%theta = w*Ts; 
%disp(theta*180/pi()) 
%odom_x = V*cos(theta)*Ts; 
%odom_y = V*sin(theta)*Ts;
D_rand = D + sqrt((sigma(1,1)))*randn; 
theta_rand = theta + sqrt((sigma(2,2)))*randn; 
odom_x = D*cos(x(3)+theta); 
odom_y = D*sin(x(3)+theta); 
odom_x_rand = D_rand*cos(x_rand(3)+theta_rand); 
odom_y_rand = D_rand*sin(x_rand(3)+theta_rand);  
end 

%Function to generate a set of distances and headings to landmarks in the
%map 
function DistanceVector = DistanceToLandmarks(LandMarkMap,id) 
 global x; 
    %DistanceVector(:) = [sqrt((LandMarkMap(id,1)-x(1))^2+(LandMarkMap(id,2)-x(2))^2), (atan((LandMarkMap(id,2)-x(2))/(LandMarkMap(id,1)-x(1))) - x(3))]; 
end 

%Function to generate a set of landmarks in the simulated map 
function LandMarkMap = GenerateLandMarks(x_size,y_size,NumLandmarks)  
    LandMarkMap = [-32, 111;0,0;0,0;30,106;-4,110]/100; 
end

function receive_data(message,t) 
    global ros_odom_data; 
    ros_odom_data = message; 
    %t = ros_odom_data.Header.Stamp.Nsec/(10^9); 
    V = ros_odom_data.Twist.Twist.Linear.X; 
    w = ros_odom_data.Twist.Twist.Angular.Z; 
    odometry_update(V,w,t);
% global x; 
% x(1) = ros_odom_data.Pose.Pose.Position.X; 
% x(2) = ros_odom_data.Pose.Pose.Position.Y; 
% Q = [ros_odom_data.Pose.Pose.Orientation.W,ros_odom_data.Pose.Pose.Orientation.X,ros_odom_data.Pose.Pose.Orientation.Y,ros_odom_data.Pose.Pose.Orientation.Z]; 
% theta = quat2eul(Q); 
% x(3) = wrapToPi(theta(1)); 
end 
function odometry_update(V,w,Ts) 
global x; 
global P; 
global sigma; 
%Computing odometry from one step (with and without randomness) 
l = .263; 
r = .065; 
Vr = (2*V + w*l)/2; 
Vl = 2*V - Vr; 
wr = Vr/(r); 
wl = Vl/(r); 
Dl = (wl*r)*Ts; 
Dr = (wr*r)*Ts; 
D = (Dl+Dr)/2; 

theta = (Dr-Dl)/l; 
%theta = w*Ts; 
%disp(theta*180/pi()) 
%odom_x = V*cos(theta)*Ts; 
%odom_y = V*sin(theta)*Ts;
odom_x = D*cos(x(3)+theta); 
odom_y = D*sin(x(3)+theta); 
x = x + [odom_x odom_y theta]; 
x(3) = wrapToPi(x(3)); 

%Jacobians for equation (6.2) should be the same equation as it does not
%rely on vehicle model information 
Fx = [1 0 -D*sin(x(3));0 1 D*cos(x(3)); 0 0 1]; 
Fv = [cos(x(3)) 0;sin(x(3)) 0; 0 1]; 

P = Fx*P*Fx' + Fv*sigma*Fv';  
end 
