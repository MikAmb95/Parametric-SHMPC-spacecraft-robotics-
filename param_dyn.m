
clc,close all,clear all

% Characteristics of the robot---------------------------------------------
par = [];
par.a1=2; %length of the link #1 [m]
par.a2=2; %length of the link #2 [m]
par.a3=2; %length of the link #3 [m]
par.ac1=par.a1/2; %CoM link #1 [m]
par.ac2=par.a2/2; %CoM link #2 [m]
par.ac3=par.a3/2; %CoM link #3 [m]
par.m1=2; %mass link #1 [kg]
par.m2=2; %mass link #2 [kg]
par.m3=2; %mass link #3 [kg]
par.mb=5; %mass link #1 [kg]
par.mp=1; %mass EE [kg]
par.Izz1=1/12*par.m1*par.a1^2; %moment of inertia link #1 [kg*m^2]
par.Izz2=1/12*par.m2*par.a2^2; %moment of inertia link #2 [kg*m^2]
par.Izz3=1/12*par.m3*par.a3^2; %moment of inertia link #3 [kg*m^2]
par.W1=1; % (the base is modeled as a rectangular) side #1 [m]
par.W2=0.5; % (the base is modeled as a rectangular) side #2 [m]
par.Izzb=1/12*par.mb*(4*par.W1^2+4*par.W2^2); % moment of inertia base [kg*m^2]
par.ex=par.W1-0.1; %where the robot is connected to the base (x-direction)
par.ey=par.W2-0.1; %where the robot is connected to the base (y-direction)

save('param','par')
