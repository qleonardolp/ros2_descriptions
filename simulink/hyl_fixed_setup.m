%% HyL Simulink/Simscape Setup %%

% Run this script to generate the HyL RigidBodyTree struct for Simulink

clear
addpath(genpath('description'))  % add folder and subfolders to path
HyLRigidBodyTree = importrobot('description\hyl_fixed_simulink.urdf', CollisionDecomposition=true);
HyLRigidBodyTree.Gravity = [0 0 -9.80665];
% show(HyLRigidBodyTree)
clc

% Controller period
Ts = 0.001;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 HPU and Oil Parameters                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Pump
supply_pressure = 207*10^5; %Pa (3000 psi)
tank_pressure   = 0;        %Pa

%Fluid: ISO 68
fluid_density =    875.0;   %kg/m3
bulk_modulus  =  1.340e9;   %Pa | Bulk modulus (1340 MPa)
kine_viscosity = 68.0e-6;   %m^2/s (e-6 cSt) | Kinematic viscosity
relative_air   =   0.005;   %'percentage' | Relative amount of trapped air

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     Valve Parameters                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Moog E024

valve_rated_flow    = 7.5/60000;    %m3/s | Nominal flow
valve_pressure_drop =   70*10^5;    %Pa   | Nominal pressure drop
valve_input         =     0.010;    %A (m)| Nominal valve input
% Valve spool dynamics
spool_nfreq = 250;                  %Hz    | Natural frequency
spool_omg   = 2*pi*spool_nfreq;     %rad/s | Natural frequency
spood_damp  = 0.5;                  %1     | Damping coefficient

% Linearized model
pa_o = supply_pressure/2;           %Pa
pb_o = supply_pressure/2;           %Pa

valve_gain   = valve_rated_flow/(valve_input*sqrt(valve_pressure_drop/2));  %Valve gain

% Orifice area (Simscape valve block)
Cd = 0.64;
area_gradient = valve_gain/Cd * sqrt(fluid_density/2);
orifice_area_max = area_gradient * valve_input;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  Cylinder Parameters                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

De = 0.0190;                  %m | Piston diameter
Dh = 0.0095;                  %m | Rod diameter
cyl_length = 0.080;           %m | Length

area_a = pi*(De^2)/4;               %m2 | Chamber A area
area_b = pi*(De^2)/4 - pi*(Dh^2)/4; %m2 | Chamber B area
area_r = pi*(Dh^2)/4;               %m2 | Rod area
area_p = area_a;                    %m2 | Piston area

alfa = area_b/area_a;               %1  | Area ratio
 
Vpl = 1.21e-3;                      %m^3 | Pipeline vol.
dead_vol = Vpl;
Va = area_a*(cyl_length/2);         %m^3 
Vb = area_b*(cyl_length/2);         %m^3
Va0 = Vpl + Va;                     %m^3  - Initial volume in chamber A
Vb0 = Vpl + Vb;                     %m^3  - Initial volume in chamber B

Po = supply_pressure/(1+alfa);      %Pa - Initial pressures for the simulation 
pa_o_p = alfa*Po;                   %Pa
pa_o_n = alfa*Po;                   %Pa
pb_o_p = Po;                        %Pa
pb_o_n = Po;                        %Pa

rod_vol  = area_r*cyl_length;          %m3 | Rod volume
pist_vol = area_a*0.15;                %m3 | Piston volume
piston_mass = 7860*(rod_vol+pist_vol); %kg | Piston mass

cyl_leakage = 1.70e-13;       %m3/(s.Pa) | Cylinder leakage

viscous_coeff = 700;          %N/(m/s) | Viscous friction coeff. (B)
coul_force    = 100;          %N   | Coulomb friction force
brk_force     = 100;          %N   | Breakaway friction force
brk_velocity  = 0.02/sqrt(2); %m/s | Breakaway friction velocity
