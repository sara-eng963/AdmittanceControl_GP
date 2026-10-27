%% Admittance Control - Graduation Project
% 6-DoF Cartesian Admittance Controller
% MATLAB R2025b
%
% Purpose:
% Mathematical modeling, control design, discretization,
% simulation, and verification of the embedded controller.

clearvars;
clc;
close all;

%% Project initialization

projectRoot = fileparts(mfilename('fullpath'));

addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'models'));
addpath(fullfile(projectRoot, 'analysis'));

disp('Admittance Control - Design Environment');
disp('Project initialized successfully.');