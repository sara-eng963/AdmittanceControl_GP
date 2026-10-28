%% Admittance Control - Graduation Project
% MATLAB workspace initializer (run this first from the repository).
% Adds both 1D and 6D scripts and reusable model functions to the path.
% Does not execute experiments or change any controller parameters.

clearvars;
clc;
close all;

projectRoot = fileparts(mfilename('fullpath'));

folders = {
    fullfile(projectRoot, '1D', 'models')
    fullfile(projectRoot, '1D', 'analysis')
    fullfile(projectRoot, '1D', 'simulink')
    fullfile(projectRoot, '6D')
    fullfile(projectRoot, '6D', 'models')
    fullfile(projectRoot, '6D', 'analysis')
    fullfile(projectRoot, '6D', 'tests')
};

for k = 1:numel(folders)
    assert(isfolder(folders{k}), ...
        'Missing expected project folder: %s', folders{k});
    addpath(folders{k});
end

fprintf('Admittance Control - MATLAB paths initialized.\n');
fprintf('Repository: %s\n', projectRoot);
fprintf('Main 6D design: design_6axis_admittance\n');
