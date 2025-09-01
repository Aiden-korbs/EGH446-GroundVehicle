%% close previously open model
close_system('sl_groundvehicleDynamics',0);
clear all;
 

waypoints.x = randi([-500 500],1,10);
waypoints.y = randi([-500 500],1,10);

%% add toolboxes to path
homedir = pwd; 
addpath( genpath(strcat(homedir,[filesep,'toolboxes'])));

cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;

cd(homedir);

%% open current model
open_system('sl_groundvehicleDynamics'); %differential robot

cd(homedir);





