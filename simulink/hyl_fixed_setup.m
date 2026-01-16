%% HyL Simulink/Simscape Setup %%

% Run this script to generate the HyL RigidBodyTree struct for Simulink

clear
addpath('description','-end')
HyLRigidBodyTree = importrobot('description\hyl_fixed_simulink.urdf', CollisionDecomposition=true);
HyLRigidBodyTree.Gravity = [0 0 -9.80665];
% show(HyLRigidBodyTree)
clc