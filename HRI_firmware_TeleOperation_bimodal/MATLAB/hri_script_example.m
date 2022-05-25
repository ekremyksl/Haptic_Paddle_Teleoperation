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

%% Open the serial port.
% If the "HRI board communication" object already exists, delete it.
if exist('hbh', 'var')
    close(hbh);
    clear hbh;
end

if ~isempty(instrfind)
    fclose(instrfind);
end

% Find the serial port name.
boardComPort = hri_comm.getHriComPort();

% Create a "HRI board communication" object.
hbh = hri_comm(boardComPort, 0);
open(hbh);

%% Check if the board is responding.
ok = ping(hbh);

if ok
    disp('Board responded to the ping request.');
else
    warning('Board did not respond to the ping request.');
end

%% Get the variables list.
varsList = getVarsList(hbh);
dispVarsList(hbh);

%% Get and set a variable.

% Read the Hall sensor voltage.
hallSensorVarId = getVarIdFromName(hbh, 'hall_voltage [V]');
hallVoltage = getVar(hbh, hallSensorVarId); % [V].
fprintf('Hall voltage: %f V.\n', hallVoltage);

% Set a LED on and off.
led0VarId = getVarIdFromName(hbh, 'led_0 [0.0-1.0]');
setVar(hbh, led0VarId, 1.0); % On.
pause(0.5);
setVar(hbh, led0VarId, 0.0); % Off.

%% Continuously stream variables values.
% Start the streaming of two variables.
encoderVarId = getVarIdFromName(hbh, 'encoder_paddle_pos [deg]');
setStreamedVars(hbh, [encoderVarId hallSensorVarId]);

% Query periodically the values of the streamed variables.
for i=1:10
    % Get all the received values of the streamed variables, since the last
    % time.
    streamedVarsValues = getStreaming(hbh);
    encoderValues = streamedVarsValues(:, 1);
    hallValues = streamedVarsValues(:, 2);
    
    % Plot.
    sampleNumber = 1:size(streamedVarsValues, 1);
    
    yyaxis left;
    plot(sampleNumber, encoderValues);
    ylabel('Encoder position [deg]');
    
    yyaxis right;
    plot(sampleNumber, hallValues);
    ylabel('Hall voltage [V]');
    
    % Delay.
    pause(0.5);
end

% Stop the streaming.
setStreamedVars(hbh, []);

%% Close.
close(hbh);
clear hbh;