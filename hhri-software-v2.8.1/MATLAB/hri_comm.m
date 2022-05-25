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

classdef hri_comm < handle
    % HRI_COMM Class to remotely control a HRI board.
    %   Manages the communication between MATLAB on PC and the HHRI board,
    %   and allows remote operation.
    
    properties(Access = private)
        hSerial
        packetBuffer
        rxCurrentMessageType
        rxBytesCount
        firstHalfByte
        rxTimer
        pendingPingback
        pendingGetVarsList
        pendingGetVar
        logfile
        varsList
        streamId
        varsIdsToStream
        streamPacketSize
        streamBuffer
        currentStreamBufferIndex
        gotVarsListCallbackFunc
        boardOfflineCallbackFunc
        plotDecimCounter
    end

    properties(Hidden = true, Access = private, Constant = true)
        %% Messages sent by MATLAB to the board.
        PC_MESSAGE_DO_NOTHING = 0; % Do nothing.
        PC_MESSAGE_PING = 1;
        PC_MESSAGE_GET_VARS_LIST = 2; % Request the device to send the variables list.
        PC_MESSAGE_SET_STREAMED_VAR = 3; % Set the variables to be streamed.
        PC_MESSAGE_GET_VAR = 4; % Request the device to send the selected value.
        PC_MESSAGE_SET_VAR = 5; % Set the selected variable.

        %% Messages sent by the board to MATLAB.
        STM_MESSAGE_PINGBACK = 0; % Response to a ping request.
        STM_MESSAGE_VAR = 1; % Variable state.
        STM_MESSAGE_STREAMING_PACKET = 2; % Streaming packet.
        STM_MESSAGE_DEBUG_TEXT = 3; % Debug text message.
        STM_MESSAGE_VARS_LIST = 4; % Monitored variables list.
        STM_MESSAGE_START_INFO = 5; % Notification that the board has (re)started.
        
        %% Other constants.
        SYNCVAR_NAME_SIZE = 50; % Max size of a SyncVar name, including the '\0' trailing character.
        STREAMING_BUFF_LEN = 1000; % [bytes].
        READ_BYTES_PERIOD = 0.100; % [s].
        VAR_ACCESSES = {'readonly', 'writeonly', 'read+write'};
        VAR_TYPES = {'bool', 'uint8', 'int8', 'uint16', 'int16', ...
                     'uint32', 'int32', 'uint64', 'int64', 'single', ...
                     'double'};
    end
    
    methods
        function hbh = hri_comm(comPortName, gotVarsListCallbackFunc)
            % HRI_COMM Class constructor.
            hbh.hSerial = serial(comPortName, ...
                                 'BaudRate', 1843200, ...
                                 'Parity', 'none', ...
                                 'StopBits', 1, ...
                                 'FlowControl', 'none', ...
                                 'Timeout', 0.5, ...
                                 'InputBufferSize', 100000, ...
                                 'ReadAsyncMode', 'continuous');
            hbh.packetBuffer = zeros(1, 1000, 'uint8');
            hbh.rxCurrentMessageType = 0;
            hbh.rxBytesCount = 0;
            hbh.pendingPingback = 0;
            hbh.logfile = 0;
            hbh.rxTimer = 0;
            hbh.streamId = 0;
            hbh.gotVarsListCallbackFunc = gotVarsListCallbackFunc;
            hbh.plotDecimCounter = 1;
        end
        
        function open(hbh)
            % OPEN Opens the serial link, and starts receiving data.
            % hbh hri_comm object.
            
            % Open the serial link.
            fopen(hbh.hSerial);
            flushinput(hbh.hSerial);
            
            % Setup the callback function, called periodically to read the
            % bytes sent by the board.
            hbh.rxTimer = timer('Period', hbh.READ_BYTES_PERIOD, ...
                                'TimerFcn', @(~,~)readBytes(hbh), ...
                                'ExecutionMode', 'fixedRate', ...
                                'BusyMode', 'drop');
            start(hbh.rxTimer);
            
            % Stop the streaming, if it was running.
            setStreamedVars(hbh, []);
            
            % Get the remote variables list, if the board is OK.
            if ping(hbh)
                requestVarsList(hbh);
            end
        end
    
        function close(hbh)
            % CLOSE Closes the serial link.
            % hbh hri_comm object.
            if hbh.rxTimer ~= 0
                stop(hbh.rxTimer);
            end
            
            if strcmp(hbh.hSerial.Status, 'open')
                fclose(hbh.hSerial);
                delete(hbh);
            end
        end
        
        function delete(hbh)
            % DELETE Class destructor.
            % hbh hri_comm object.
            close(hbh);
        end
        
        function ok = ping(hbh)
            % PING Checks if the board is responding to a ping request.
            % hbh hri_comm object.
            hbh.pendingPingback = 1;
            sendPacket(hbh, hbh.PC_MESSAGE_PING, []);
            
            startTime = tic;
            
            while hbh.pendingPingback && toc(startTime) < 1.0
                pause(0.1);
            end
            
            ok = (hbh.pendingPingback == 0);
        end
        
        function startLogging(hbh)
            % STARTLOGGING Starts the logging to a text file.
            % hbh hri_comm object.
            hri_constants; % Load the parameters file.
            
            % Get the directory of this file, to avoid problems when
            % changing the current MATLAB directory.
            [baseDirectory, ~, ~] = fileparts(mfilename('fullpath'));
            addpath(baseDirectory);
            
            % If the "logs" directory does not exist, create it.
            logsDirectory = [baseDirectory '/' 'logs/'];
            if ~exist(logsDirectory, 'dir')
                mkdir(logsDirectory);
            end
            
            % Open the logfile to write.
            if strcmp(LOGFILE_NAMING, 'date')
                % Use the current date/time to get a unique filename.
                filenameSuffix = datestr(now, 'yyyy-mm-dd_HH_MM_SS');
            elseif strcmp(LOGFILE_NAMING, 'counter')
                % List the existing logfiles, and look for the highest
                % number, in order to determine the next number.
                logfilesNames = dir([logsDirectory 'log_*.csv']);
                logfilesNames = {logfilesNames.name};
                a = regexp(logfilesNames, 'log_(?<n>\d{6}).csv', 'tokens');
                a(cellfun(@isempty, a)) = [];
                maxNumber = max(cellfun(@(x) str2double(x{1}), a));
                filenameSuffix = sprintf('%06i', maxNumber+1);
            end
            
            hbh.logfile = fopen([logsDirectory 'log_' filenameSuffix ...
                                 '.csv'], 'w');
            
            % Build the header.
            header = 'timestamp [us];';
            for i=hbh.varsIdsToStream
                header = [header hbh.varsList(i).name ';']; %#ok<AGROW>
            end
            header(end) = [];
                             
            fprintf(hbh.logfile, [header '\n']);
        end
        
        function stopLogging(hbh)
            % STOPLOGGING Stops the logging to a text file.
            % hbh hri_comm object.
            
            if hbh.logfile ~= 0
                fclose(hbh.logfile);
                hbh.logfile = 0;
            end
        end
        
        function varsList = getVarsList(hbh)
            % GETTVARSLIST Gets the variables list to the board.
            % Requests the variables list from the board, and waits until it
            % has been received.
            % hbh hri_comm object.
            hbh.pendingGetVarsList = 1;
            requestVarsList(hbh);
            
            while hbh.pendingGetVarsList
                pause(0.1);
            end
            
            varsList = hbh.varsList;
        end
        
        function setVar(hbh, varId, newValue)
            % SETVAR Sets a board variable to the desired value.
            % hbh hri_comm object.
            % varId the selected variable to set.
            % newValue the new desired value of the selected variable. The
            % given value will be cast to the correct type.
            
            % Send a packet to set the value of the variable.
            if strcmp(hbh.varsList(varId).type, 'bool')
                newValue = cast(newValue, 'uint8');
            else
                newValue = cast(newValue, hbh.varsList(varId).type);
            end
            data = [varId-1 typecast(newValue, 'uint8')];
            sendPacket(hbh, hbh.PC_MESSAGE_SET_VAR, data);
        end
        
        function varValue = getVar(hbh, varId)
            % REQUESTVAR Get the variable value from the board.
            % This function blocks until the variable has been received.
            % hbh hri_comm object.
            % varId the ID of the selected variable to get.
            hbh.pendingGetVar = 1;
            
            sendPacket(hbh, hbh.PC_MESSAGE_GET_VAR, varId-1);

            while(hbh.pendingGetVar)
                pause(0.01);
            end

            varValue = hbh.varsList(varId).value{:};
        end
        
        function dispVarsList(hbh)
            % DISPVARLIST Prints the variables list.
            % hbh hri_comm object.
            for i=1:length(hbh.varsList)
                fprintf('#%i %s %s (size %i) %s\n', i, ...
                        hbh.varsList(i).name, ...
                        hbh.varsList(i).type, ...
                        hbh.varsList(i).size, ...
                        hbh.varsList(i).access);
            end
        end
        
        function setStreamedVars(hbh, varsIdsToStream)
            % SETSTREAMEDVARS Setup the streaming with the given variables.
            % hbh hri_comm object.
            % varsIdsToStream array of indices of variables to stream.
            
            % Allocate the buffer.
            hbh.streamBuffer = zeros(1000, length(varsIdsToStream));
            hbh.currentStreamBufferIndex = 1;
            
            % Build the communication packet.
            hbh.streamId = hbh.streamId + 1;
            hbh.streamPacketSize = 1 + 4; % ID + timestamp.
            hbh.varsIdsToStream = varsIdsToStream;
            
            data = zeros(1, 2 + length(varsIdsToStream));
            
            data(1) = uint8(length(varsIdsToStream));
            data(2) = uint8(hbh.streamId);
            
            for i=1:length(varsIdsToStream)
                data(2+i) = uint8(varsIdsToStream(i)-1);
                hbh.streamPacketSize = hbh.streamPacketSize + ...
                                     hbh.varsList(varsIdsToStream(i)).size;
            end
            
            sendPacket(hbh, hbh.PC_MESSAGE_SET_STREAMED_VAR, data);
            
            % TODO: remove.
            % This avoids a bug when removing a variable while streaming.
            pause(0.1);
        end
        
        function vars = getStreaming(hbh)
            % GETSTREAMING Gets the variables values during streaming.
            % Returns an array containing the values received for each
            % variable, one variable per column.
            % hbh hri_comm object.
            vars = hbh.streamBuffer(1:hbh.currentStreamBufferIndex-1, :);
            hbh.currentStreamBufferIndex = 1;
        end
        
        function varId = getVarIdFromName(hbh, varName)
            % GETVARIDFROMNAME Gets the variable index from its name.
            % hbh hri_comm object.
            % varName name of the variable, as it appears in the list. If
            % it cannot be found, [] will be returned.
            varId = find(strcmp({hbh.varsList.name}, varName), 1);
            
            if isempty(varId)
                warning(['Could not find the variable "' varName ...
                         '" in the list.']);
            end
        end
    end
    
    methods(Static)
        function boardComPort = getHriComPort()
            % GETHRICOMPORT Finds the port corresponding to the board.
            % Tries to ping the board on all the serial ports that are
            % available. If an appropriate answer is received on one port,
            % its name will be returned, otherwise an empty string will be
            % returned.
            hwinfo = instrhwinfo('Serial');
            ports = hwinfo.AvailableSerialPorts;
            
            for i=1:length(ports)
                try
                    port = ports{i};
                    h = hri_comm(port, 0);
                    open(h);

                    if ping(h)
                        boardComPort = port;
                        delete(h);
                        return;
                    end
                catch
                    % Ignore the error, a keep trying the other serial
                    % ports.
                end
                
                if exist('h', 'var')
                    delete(h);
                    clear h;
                end
            end
            
            boardComPort = '';
        end
    end
    
    methods(Access = private)        
        function sendPacket(wmh, type, data)
            % SENDPACKET Sends a packet to the board.
            % hbh hri_comm object.
            % type type of the packet. See the PC_MESSAGE_* properties.
            % data data content of the packet. Its meaning depends on the
            % packet type.
            
            % Build the packet.
            txBuffer = zeros(1, 1 + length(data) * 2, 'uint8');
            txBuffer(1) = bitset(type, 8); % Set bit 8 to 1 ("start byte").
            
            for i = 1:length(data)
                b = data(i);
                txBuffer(i*2+0) = bitshift(b, -4); % MSB.
                txBuffer(i*2+1) = bitshift(bitshift(b, 4), -4); % LSB.
            end
            
            % Send the packet through the serial link.
            fwrite(wmh.hSerial, txBuffer); % TODO: set to 'async'?
        end
        
        function readBytes(hbh)
            % READBYTES Reads and interprets the data received.
            % hbh hri_comm object.
            try
                hri_constants;
                
                % Read the available bytes from the serial link.
                nAvailableBytes = hbh.hSerial.BytesAvailable;

                if nAvailableBytes == 0
                    return;
                end
 
                rxBytes = fread(hbh.hSerial, nAvailableBytes, 'uint8');
                
                % Cache the most accessed members.
                rxBytesCountC = hbh.rxBytesCount;
                packetBufferC = hbh.packetBuffer;
                
                % Process each byte, one by one.
                for rxByte = rxBytes'

                    if bitget(rxByte, 8) == 1 % Start byte.
                        hbh.rxCurrentMessageType = bitset(rxByte, 8, 0);
                        rxBytesCountC = 0;
                    else % "Normal" byte.
                        rxBytesCountC = rxBytesCountC + 1;
                    end

                    if rem(rxBytesCountC, 2) == 1 % Got first half byte.
                        hbh.firstHalfByte = rxByte;
                    else % Got nothing, or the second half byte.
                        dataBytesReady = floor(rxBytesCountC / 2);

                        % Accumulate the incoming byte in the RX packet buffer.
                        if dataBytesReady > 0
                            packetBufferC(dataBytesReady) = ...
                                bitshift(hbh.firstHalfByte, 4) + ... % MSB.
                                bitshift(bitshift(rxByte, 4), -4); % LSB.
                        end

                        % If enough data bytes have been received, interpret
                        % them.
                        switch hbh.rxCurrentMessageType
                            case hbh.STM_MESSAGE_PINGBACK
                                hbh.pendingPingback = 0;
                            case hbh.STM_MESSAGE_VAR
                                if dataBytesReady >= 1
                                    varId = packetBufferC(1) + 1;
                                    varSize = hbh.varsList(varId).size;

                                    if dataBytesReady == 1 + varSize
                                        if strcmp(hbh.varsList(varId).type, 'bool')
                                            hbh.varsList(varId).value = {(packetBufferC(2) ~= 0) * 1};
                                        else
                                            hbh.varsList(varId).value = {typecast(packetBufferC(2:1+varSize), hbh.varsList(varId).type)};
                                        end
                                    end

                                    hbh.pendingGetVar = 0;
                                end
                            case hbh.STM_MESSAGE_STREAMING_PACKET
                                if dataBytesReady == hbh.streamPacketSize
                                    if packetBufferC(1) == hbh.streamId
                                        timestamp = cast(typecast(packetBufferC(2:5), 'uint32'), 'double');
                                        p = 6;
                                        for varId=hbh.varsIdsToStream
                                            if strcmp(hbh.varsList(varId).type, 'bool')
                                                hbh.varsList(varId).value = {(packetBufferC(p) ~= 0) * 1};
                                                p = p + 1;
                                            else
                                                varSize = hbh.varsList(varId).size;
                                                hbh.varsList(varId).value = {typecast(packetBufferC(p:p+varSize-1), hbh.varsList(varId).type)};
                                                p = p + varSize;
                                            end   
                                        end

                                        % Add the values into the streaming buffer.
                                        streamBufferLine = cellfun(@(x)cast(x{:}, 'double'), {hbh.varsList(hbh.varsIdsToStream).value});
                                        
                                        if hbh.plotDecimCounter >= PLOT_DOWNSAMPLING
                                            hbh.plotDecimCounter = 1;
                                            hbh.streamBuffer(hbh.currentStreamBufferIndex,:) = streamBufferLine;
                                            hbh.currentStreamBufferIndex = hbh.currentStreamBufferIndex + 1;
                                            
                                            % If the streaming buffer is full, restart.
                                            if hbh.currentStreamBufferIndex >= hbh.STREAMING_BUFF_LEN
                                                hbh.currentStreamBufferIndex = 1;
                                                warning('The streaming buffer was full, and has been reseted.');
                                            end
                                        else
                                            hbh.plotDecimCounter = hbh.plotDecimCounter + 1;
                                        end

                                        % Log to file, if enabled.
                                        if hbh.logfile ~= 0
                                            fprintf(hbh.logfile, '%f;', [timestamp streamBufferLine]);
                                            fprintf(hbh.logfile, '\n');
                                        end
                                    end
                                end
                            case hbh.STM_MESSAGE_DEBUG_TEXT
                                if dataBytesReady > 0 && ...
                                    packetBufferC(dataBytesReady) == 0
                                    message = char(...
                                        packetBufferC(...
                                            1:find(packetBufferC == 0)-1));
                                    message(message == 13) = [];
                                    fprintf([message '\n']);
                                end
                            case hbh.STM_MESSAGE_VARS_LIST
                                if dataBytesReady > 0
                                    nVars = double(packetBufferC(1));

                                    hbh.varsList = [];

                                    if dataBytesReady == 1 + nVars * (hbh.SYNCVAR_NAME_SIZE + 3)
                                        j = 2;
                                        for i=1:nVars
                                            name = packetBufferC(j:j+hbh.SYNCVAR_NAME_SIZE);
                                            hbh.varsList(i).name = char(name(1:find(name == 0, 1, 'first')-1));
                                            j = j + hbh.SYNCVAR_NAME_SIZE;
                                            hbh.varsList(i).type = hbh.VAR_TYPES{packetBufferC(j)+1};
                                            j = j + 1;                                          
                                            hbh.varsList(i).access = hbh.VAR_ACCESSES{packetBufferC(j)+1};
                                            j = j + 1;
                                            hbh.varsList(i).size = double(packetBufferC(j));
                                            j = j + 1;

                                            if strcmp(hbh.varsList(i).type, 'bool')
                                                hbh.varsList(i).value = 0;
                                            else
                                                hbh.varsList(i).value = {cast(0, hbh.varsList(i).type)};
                                            end
                                        end

                                        hbh.pendingGetVarsList = 0;

                                        if isa(hbh.gotVarsListCallbackFunc, 'function_handle')
                                            hbh.gotVarsListCallbackFunc(hbh.varsList);
                                        end
                                    end
                                end
                            case hbh.STM_MESSAGE_START_INFO
                                if dataBytesReady == 0
                                    disp('The board has (re)started.');
                                    requestVarsList(hbh);
                                end
                        end
                    end
                end
                
                % De-cache the cached variables.
                hbh.rxBytesCount = rxBytesCountC;
                hbh.packetBuffer = packetBufferC;
            catch e
                % Having this variable in global allows accessing it from
                % the main workspace.
                global err; %#ok<TLEV>
                err = e;
                rethrow(e);
            end
        end
        
        function requestVarsList(hbh)
            % REQUESTVARSLIST Requests the variables list to the board.
            % hbh hri_comm object.
            sendPacket(hbh, hbh.PC_MESSAGE_GET_VARS_LIST, []);
        end
    end
end