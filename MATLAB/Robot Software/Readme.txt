To setup a new robot:

First turn off anti-virus scheduled scans. Disable everything in start up. Disable WIFI (airplane mode). 
Install the USB to serial driver (go to device manager and update the driver).
Put MATLAB and MTTTY icon in the bottom bar
Then follow these steps:

Step #1: Add to MATLAB Path

addpath('C:\MAE493_Mobile_Robotics\Robot Software\Create Matlab Interface')
addpath('C:\MAE493_Mobile_Robotics\Robot Software\SMART MATLAB Code')
savepath

Step #2: Run Create_Test1.m to evaluate the Create interface

Step #3: Run SMART_Run.m to evaluate the rest of the robot