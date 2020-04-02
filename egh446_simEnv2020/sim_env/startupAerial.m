close_system('sl_quadrotorDynamics',0);
 
homedir = pwd; 
addpath( genpath(strcat(homedir,[filesep,'rvctools'])));
addpath( genpath(strcat(homedir,[filesep, 'slprj'])));
addpath( genpath(strcat(homedir,[filesep,'matfiles'])));
 
cd rvctools;
startup_rvc;

cd MRTB;
startMobileRoboticsSimulationToolbox;

cd ..; 
open_system('sl_quadrotorDynamics'); % quadrotor model


cd(homedir);




