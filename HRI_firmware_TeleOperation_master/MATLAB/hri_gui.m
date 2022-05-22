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

function varargout = hri_gui(varargin)
% HRI_GUI MATLAB code for hri_gui.fig
%      HRI_GUI, by itself, creates a new HRI_GUI or raises the existing
%      singleton*.
%
%      H = HRI_GUI returns the handle to a new HRI_GUI or the handle to
%      the existing singleton*.
%
%      HRI_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HRI_GUI.M with the given input arguments.
%
%      HRI_GUI('Property','Value',...) creates a new HRI_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before hri_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to hri_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help hri_gui

% Last Modified by GUIDE v2.5 26-Dec-2015 20:20:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @hri_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @hri_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before hri_gui is made visible.
function hri_gui_OpeningFcn(hObject, eventdata, handles, varargin) %#ok<INUSL>
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to hri_gui (see VARARGIN)

% Choose default command line output for hri_gui
handles.output = hObject;

% UIWAIT makes hri_gui wait for user response (see UIRESUME)
% uiwait(handles.main_window);

hri_constants;

handles.streamedVars = [];
handles.varsWidgets = [];
handles.plotDataCurrentIndex = 1;
handles.plotData = [];

handles.plotFig = findobj('Tag', 'plot_figure');
handles.dataStreamingCheckbox = findobj('Tag', 'data_streaming_checkbox');

% Add the current directory, in order to ensure the callbacks work, even if
% the current directory is changed by user.
[baseDirectory, ~, ~] = fileparts(mfilename('fullpath'));
addpath(baseDirectory);

% Create the variables widgets.
lineHeight = 1 / N_VARS_MAX;
y = 1 - lineHeight;
    
for i=1:N_VARS_MAX
    % Create the UI controls at the right location.
    handles.varsWidgets(i).checkbox = uicontrol('Parent', handles.vars_panel, ...
                                                'Style', 'checkbox', ...
                                                'Units', 'normalized', ...
                                                'HandleVisibility', 'callback', ...
                                                'Position', [0 y 3/6 lineHeight], ...
                                                'Visible', 'off');

    handles.varsWidgets(i).get = uicontrol('Parent', handles.vars_panel, ...
                                           'Units', 'normalized', ...
                                           'HandleVisibility', 'callback', ...
                                           'Position', [3/6 y 1/6 lineHeight], ...
                                           'String', 'Get', ...
                                           'Visible', 'off');
              
    handles.varsWidgets(i).edit = uicontrol('Parent', handles.vars_panel, ...
                                            'Style', 'edit', ...
                                            'Units', 'normalized', ...
                                            'HandleVisibility', 'callback', ...
                                            'Position', [4/6 y 1/6 lineHeight], ...
                                            'String', '', ...
                                            'Visible', 'off');

    handles.varsWidgets(i).set = uicontrol('Parent', handles.vars_panel, ...
                                           'Units', 'normalized', ...
                                           'HandleVisibility', 'callback', ...
                                           'Position', [5/6 y 1/6 lineHeight], ...
                                           'String', 'Set', ...
                                           'Visible', 'off');
                                        
    y = y - lineHeight;
end

if isempty(varargin)
    % Find which serial port corresponds to the board.
    comPort = hri_comm.getHriComPort();
    
    if isempty(comPort)
        warning('The board is not detected.');
        handles.hbh = [];
        guidata(hObject, handles);
        return;
    else
        fprintf('The board was detected on %s.\n', comPort);
    end
else
    comPort = varargin{1};
end

% Open the serial link with the board.
handles.hbh = hri_comm(comPort, ...
                       @(varsList) hri_gui_UpdateVarsList(hObject, varsList));
open(handles.hbh);

% Set the plot figure update timer.
handles.plotFreqPopup = findobj('Tag', 'plot_freq_popup');
plotFreq = PLOT_SAMPLING_FREQS(handles.plotFreqPopup.Value);

handles.plotTimer = timer('Period', 1.0 / plotFreq, ...
                          'TimerFcn', @(~,~)updatePlot(hObject), ...
                          'ExecutionMode', 'fixedRate', ...
                          'BusyMode', 'drop');
start(handles.plotTimer);

%
guidata(hObject, handles);

function hri_gui_UpdateVarsList(hObject, varsList)

hri_constants;
handles = guidata(hObject);
handles.varsList = varsList;

% Stop data streaming and logging, if running.
handles.streamedVars = [];
set(handles.data_streaming_checkbox, 'Value', 0);
set(handles.log_to_file_checkbox, 'Value', 0);

% Error if there are more variables than GUI slots.
if length(handles.varsList) > N_VARS_MAX
    error('Cannot display so many variables. Max is %i.', N_VARS_MAX);
end

% 
for i=1:length(handles.varsList)
    % Modify the widgets to they match the associated variable.
    set(handles.varsWidgets(i).checkbox, 'String', handles.varsList(i).name);
    set(handles.varsWidgets(i).checkbox, 'Value', 0);
    set(handles.varsWidgets(i).checkbox, 'Visible', 'on');
    set(handles.varsWidgets(i).checkbox, 'ForegroundColor', 'black');
    
    if ~strcmp(varsList(i).access, 'writeonly')
        set(handles.varsWidgets(i).get, 'Visible', 'on');
    end
    
    set(handles.varsWidgets(i).edit, 'Visible', 'on');
    set(handles.varsWidgets(i).edit, 'String', '');
    
    if ~strcmp(varsList(i).access, 'readonly')
        set(handles.varsWidgets(i).set, 'Visible', 'on');
    end
    
    % Set the callbacks.
    handles.varsWidgets(i).checkbox.Callback = @(h,~)onToggleVarCheckbox(h,i);
    
    if handles.varsWidgets(i).get ~= 0
        handles.varsWidgets(i).get.Callback = @(h,~)onGetVar(h,i);
    end
    
    if handles.varsWidgets(i).set ~= 0
        handles.varsWidgets(i).set.Callback = @(h,~)onSetVar(h,i);
    end
end

% Hide the other widgets.
for i=length(handles.varsList)+1:N_VARS_MAX
    set(handles.varsWidgets(i).checkbox, 'Visible', 'off');
    set(handles.varsWidgets(i).get, 'Visible', 'off');
    set(handles.varsWidgets(i).edit, 'Visible', 'off');
    set(handles.varsWidgets(i).set, 'Visible', 'off');
end

% Update handles structure.
guidata(hObject, handles);


function onToggleVarCheckbox(hObject, varID, ~)
% Get handles structure.
handles = guidata(hObject);

% Add or remove the variable ID associated to the checkbox.
if handles.varsWidgets(varID).checkbox.Value == 0
    handles.streamedVars(handles.streamedVars == varID) = [];
else
    handles.streamedVars = sort([handles.streamedVars varID]);
end

% Change the streaming of the board, if it currently running.
guidata(hObject, handles);

if handles.dataStreamingCheckbox.Value ~= 0
    setupStreaming(hObject, handles.streamedVars);
end


function setupStreaming(hObject, streamedVars)
hri_constants;
handles = guidata(hObject);

% Stop the streaming, if running.
handles.log_to_file_checkbox.Value = 0;

% Request the board to change the streamed vars.
setStreamedVars(handles.hbh, streamedVars);

% Pre-allocate the plot points buffer.
% TODO: keep data for the variable that remain the same.
if ~isempty(streamedVars)
    handles.plotData = nan(PLOT_N_SAMPLES, length(handles.streamedVars));
    if ~isempty(handles.plotData)
        handles.plotFig = plot(1:PLOT_N_SAMPLES, handles.plotData);
    end
end

% Color the variables labels to match the lines colors.
% The standard MATLAB legend is too slow for live plots.
colors = get(handles.plot_figure, 'ColorOrder');

for i = 1:N_VARS_MAX
    streamVarIndex = find(i == streamedVars, 1);

    if ~isempty(streamVarIndex)
        set(handles.varsWidgets(i).checkbox, ...
            'ForegroundColor', colors(rem(streamVarIndex-1,7)+1, :));
    else
        set(handles.varsWidgets(i).checkbox, ...
            'ForegroundColor', 'black');
    end
end

guidata(hObject, handles);


function onGetVar(hObject, varID)
% Get handles structure.
handles = guidata(hObject);

% Get the variable value from the board.
varValue = getVar(handles.hbh, varID);

% Display the value on the associated edit box widget.
set(handles.varsWidgets(varID).edit, 'String', varValue);


function onSetVar(hObject, varID)
% Get handles structure.
handles = guidata(hObject);

% Get the desired variable value from the associated edit box widget.
varValue = str2double(handles.varsWidgets(varID).edit.String);

% Set the variable value on the board.
setVar(handles.hbh, varID, varValue);


function updatePlot(hObject)
hri_constants;

try
    handles = guidata(hObject);
catch
    return;
end

if ~isfield(handles, 'plotData') || isempty(handles.plotData)
    return;
end

vars = getStreaming(handles.hbh);

if isempty(vars) || (size(vars, 2) ~= size(handles.plotData, 2))
    return;
end

% If the plot buffer is full, restart.
if handles.plotDataCurrentIndex + size(vars,1) > PLOT_N_SAMPLES - 1
    handles.plotDataCurrentIndex = 1;
end

range = handles.plotDataCurrentIndex:handles.plotDataCurrentIndex+size(vars,1)-1;
handles.plotData(range, :) = vars;
handles.plotData(handles.plotDataCurrentIndex+size(vars,1), :) = nan;
handles.plotDataCurrentIndex = handles.plotDataCurrentIndex + size(vars,1);

for i=1:size(handles.plotData, 2)
    set(handles.plotFig(i), 'ydata', handles.plotData(:,i));
end

guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = hri_gui_OutputFcn(hObject, eventdata, handles) %#ok<INUSL>
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in data_streaming_checkbox.
function data_streaming_checkbox_Callback(hObject, ~, handles) %#ok<DEFNU>
% hObject    handle to data_streaming_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(hObject, 'Value') == 0
    % Stop the streaming.
    setupStreaming(hObject, []);
    
    % Stop the logging.
    handles.log_to_file_checkbox.Value = 0;
    guidata(hObject, handles);
    stopLogging(handles.hbh);
else
    setupStreaming(hObject, handles.streamedVars);
end


% --- Executes on button press in clear_plot_button.
function clear_plot_button_Callback(hObject, ~, handles) %#ok<DEFNU>
% hObject    handle to clear_plot_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.plotData = nan(size(handles.plotData));
handles.plotDataCurrentIndex = 1;

for i=1:size(handles.plotData, 2)
    set(handles.plotFig(i), 'ydata', handles.plotData(:,i));
end

guidata(hObject, handles);


% --- Executes on button press in log_to_file_checkbox.
function log_to_file_checkbox_Callback(hObject, ~, handles) %#ok<DEFNU>
% hObject    handle to log_to_file_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value') == 0
    % Stop the logging.
    stopLogging(handles.hbh);
    
    % Unlock the streaming checkboxes of the variables list.
    for j=1:length(handles.varsList)+1
        handles.varsWidgets(j).checkbox.Enable = 'on';
    end
else
    % Start the logging.
    startLogging(handles.hbh);
    
    % Lock the streaming checkboxes of the variables list.
    for j=1:length(handles.varsList)+1
        handles.varsWidgets(j).checkbox.Enable = 'off';
    end
end

% Update handles structure.
guidata(hObject, handles);


function main_window_CloseRequestFcn(hObject, ~, ~) %#ok<DEFNU>

handles = guidata(hObject);

if isfield(handles, 'plotTimer')
    stop(handles.plotTimer);
    delete(handles.plotTimer);
    handles = rmfield(handles, 'plotTimer');
end

if isfield(handles, 'hbh')
    delete(handles.hbh);
    handles = rmfield(handles, 'hbh');
end

guidata(hObject, handles);

delete(hObject);
close(gcf);


% --- Executes on selection change in plot_freq_popup.
function plot_freq_popup_Callback(hObject, ~, handles) %#ok<DEFNU>
% hObject    handle to plot_freq_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hri_constants; % Get the constants.

currentIndex = hObject.Value;
plotFreq = PLOT_SAMPLING_FREQS(currentIndex);

stop(handles.plotTimer);
set(handles.plotTimer, 'Period', 1.0 / plotFreq);
start(handles.plotTimer);
