%  Copyright (C) 2017 EPFL-LSRO (Laboratoire de Systemes Robotiques).
% 
%  Licensed under the Apache License, Version 2.0 (the "License");
%  you may not use this file except in compliance with the License.
%  You may obtain a copy of the License at
% 
%       http://www.apache.org/licenses/LICENSE-2.0
% 
%  Unless required by applicable law or agreed to in writing, software
%  distributed under the License is distributed on an "AS IS" BASIS,
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%  See the License for the specific language governing permissions and
%  limitations under the License.

%% HRI_GUI CONSTANTS

%% GUI-related constants.
N_VARS_MAX = 25; % Maximum number of variables.
PLOT_SAMPLING_FREQS = [2 10]; % Plot frequencies proposed by the combobox [Hz].

%% Plot-related constants.
PLOT_N_SAMPLES = 1000; % Width (in samples) of the plot window.
PLOT_DOWNSAMPLING = 10; % Downsampling factor before plotting.

%% Logfiles options.
LOGFILE_NAMING = 'date'; % 'date' or 'counter'.