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

% rosshutdown % shutdown any previous node
% rosinit % initialize a ros node
% robot_name = "smart2_01" %Name of the robot group to subscribe to
% global ros_odom_data; 
% global ros_odom_old_data; 
% ros_odom_old_data = [0,0]; 
% global ros_odom_D_T; 
% % Subscribing to IMU using callback functions:
% ros_odom = rossubscriber(robot_name+'/odom',@ros_odom_Callback)
% cmd_vel = rospublisher(robot_name+'/cmd_vel', 'geometry_msgs/Twist')
% cmd_vel_message = rosmessage(cmd_vel); 

%Initializing the map 
x_size = 5; %Meters 
y_size = 5; %Meters 
NumLandmarks = 20; 
LandMarkMap = GenerateLandMarks(x_size,y_size,NumLandmarks); 
NumGoals = 5; 

%Initializing the starting location 
x_start = [0 0 0]; 
x = x_start; 
P = diag([.005, .005, .001].^2); 
x_rand = x_start + [sqrt(P(1,1))*randn,sqrt(P(2,2))*randn,sqrt(P(3,3))*randn]; 

%Internal vehicle and sensor parameters and initial values 
V = 0; %m/s 
w = 0; %rad/s 
sigma = diag([.05, 5*pi()/180].^2); 
wm = diag([.001, .1*pi()/180].^2); 
Ts = .01; 
%Choose to simulate with or without the EKF update 
EKF = 1; 

%Computing states after one step, initializes the system 
[odom_x, odom_y, theta, odom_x_rand, odom_y_rand, theta_rand, D, D_rand] = odom(x,x_rand,V,w,Ts,sigma); 
x = x + [odom_x odom_y theta]; 
x_rand = x_rand + [odom_x_rand*cos(theta_rand) odom_y_rand*sin(theta_rand) theta_rand]; 
r = [x(1),x(2),x_rand(1),x_rand(2)]; 
count = 2; 

for currentGoal = 1:NumGoals 
    x_goal = [randi(x_size) randi(y_size)]; 
    goals(currentGoal,:) = x_goal; 
    while (sqrt((x_goal(1)-x(1))^2+(x_goal(2)-x(2))^2)>.1)
        
        %Using random path controller 
        [V,w] = controller(x,x_goal);
        %cmd_vel_message.Linear.X = V; 
        %cmd_vel_message.Angular.Z = w; 
        %send(cmd_vel, cmd_vel_message);
        %Computing odometry data after a step 
        [odom_x, odom_y, theta, odom_x_rand, odom_y_rand, theta_rand, D, D_rand] = odom(x,x_rand,V,w,Ts,sigma);
        odo = [odom_x, odom_y, theta, D]; 
        odo_rand = [odom_x_rand, odom_y_rand, theta_rand, D_rand]; 
        
        %Performing model prediction after a step 
        x = Xupdate(x,odo); 
        x_rand = Xupdate(x_rand,odo_rand); 
        if EKF == 1 
            P = Pupdate(P,sigma,x_rand,odo_rand);
            [x_rand,P] = ComputeUpdate(x,x_rand,P,wm,LandMarkMap); 
        end
        
        %Plotting commands 
        robot = [x(1),x(2);x(1)+.3*cos(x(3)),x(2)+.3*sin(x(3))]; 
        robot_rand = [x_rand(1),x_rand(2);x_rand(1)+.3*cos(x_rand(3)),x_rand(2)+.3*sin(x_rand(3))]; 
        r(count,:) = [x(1),x(2),x_rand(1),x_rand(2)]; 
        plot(x_goal(1),x_goal(2),'+r',robot(1,1),robot(1,2),'*b',robot(:,1),robot(:,2),'*b',LandMarkMap(:,1),LandMarkMap(:,2),'dc')%,'*y')
        xlim([0 x_size + 10]) 
        ylim([0 y_size + 10]) 
        pause(.01) 
        
        %Iterating while loop 
        count = count + 1; 
    end
end 

%Plotting true and random paths 
hold on  
plot(x_start(1),x_start(2),'ok') 
plot(goals(:,1),goals(:,2),'+k') 
plot(r(:,1),r(:,2),'b',r(:,3),r(:,4),'r') 
xlim([0 x_size + 10]) 
ylim([0 y_size + 10]) 

%Function to control the robot to move to a point 
function [V,w] = controller(x,x_goal)
Kv = 2;
Kh = 5;
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
function [P] = Pupdate(P,sigma,x_rand,odo_rand) 
%Jacobians for equation (6.2) should be the same equation as it does not
%rely on vehicle model information 
Fx = [1 0 -odo_rand(4)*sin(x_rand(3));0 1 odo_rand(4)*cos(x_rand(3)); 0 0 1]; 
Fv = [cos(x_rand(3)) 0;sin(x_rand(3)) 0; 0 1]; 

P = Fx*P*Fx' + Fv*sigma*Fv';  
end

%Function to update and correct the prediction state of the EKF 
function [x_rand,P] = ComputeUpdate(x,x_rand,P,wm,LandMarkMap) 
%DistanceVector represents the real valued (with represented sensor error)
%observation of a landmark in the environment (this would be replaced with
%the sensor measurement in actual implementation because it represents the
%truth or correction for the internal model to use 
DistanceVector = DistanceToLandmarks(x,LandMarkMap,wm); 

%PredictedDistanceVector represents the internal model of where the robot
%thinks it is based on model prediction (this would stay in the actual
%implementation 
PredictedDistanceVector = DistanceToLandmarks(x_rand,LandMarkMap,wm); 
Landmark = randi(length(DistanceVector)); 
laziness = 0; 
bruh = 0; 
while (abs(PredictedDistanceVector(Landmark,2))>90*pi/180) && bruh == 0 
    Landmark = randi(length(DistanceVector)); 
    laziness = laziness + 1; 
    if laziness > 20
        bruh = 1; 
    end
end
v = DistanceVector(Landmark,:) - PredictedDistanceVector(Landmark,:); 
Hx = [-(LandMarkMap(Landmark,1)-x_rand(1))/DistanceVector(Landmark,1),-(LandMarkMap(Landmark,2)-x_rand(2))/DistanceVector(Landmark,1),0;(LandMarkMap(Landmark,2)-x_rand(2))/DistanceVector(Landmark,1)^2, -(LandMarkMap(Landmark,1)-x_rand(1))/DistanceVector(Landmark,1)^2, -1]; 
Hw = diag([1,1]); 
S = Hx*P*Hx'+Hw*wm*Hw';  
K = P*Hx'/S; 
x_rand = x_rand+(K*v')'; 
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
function DistanceVector = DistanceToLandmarks(x,LandMarkMap,wm) 
for counter=1:length(LandMarkMap) 
    DistanceVector(counter,:) = [sqrt((LandMarkMap(counter,1)-x(1))^2+(LandMarkMap(counter,2)-x(2))^2)+sqrt(wm(1,1))*randn, (atan((LandMarkMap(counter,2)-x(2))/(LandMarkMap(counter,1)-x(1))) - x(3))+sqrt(wm(2,2))*randn]; 
end 
end 

%Function to generate a set of landmarks in the simulated map 
function LandMarkMap = GenerateLandMarks(x_size,y_size,NumLandmarks)  
for counter = 1:NumLandmarks 
    LandMarkMap(counter,:) = [randi(x_size), randi(y_size)]; 
end 
end
%IMU:

function ros_odom_Callback(src, message)
    global ros_odom_data; 
    ros_odom_data = message; 
    t = ros_odom_data.Header.Stamp.Nsec/(10^9); 
    V = ros_odom_data.Twist.Linear.X; 
    w = ros_odom_data.Twist.Angular.Z; 
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
