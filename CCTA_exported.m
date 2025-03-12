classdef CCTA_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        TabGroup2                       matlab.ui.container.TabGroup
        MainTab                         matlab.ui.container.Tab
        Panel_4                         matlab.ui.container.Panel
        FlowAxes                        matlab.ui.control.UIAxes
        PumpControlPanel                matlab.ui.container.Panel
        ControlModeSwitch               matlab.ui.control.Switch
        ControlModeSwitchLabel          matlab.ui.control.Label
        PumpPowerSlider                 matlab.ui.control.Slider
        PumpPowerSliderLabel            matlab.ui.control.Label
        ConnectedLamp                   matlab.ui.control.Lamp
        COMPortDropDown                 matlab.ui.control.DropDown
        COMPortLabel                    matlab.ui.control.Label
        GraphOptionsLabel               matlab.ui.control.Label
        RefreshListButton               matlab.ui.control.Button
        ExportDataButton                matlab.ui.control.Button
        ClearDataButton                 matlab.ui.control.Button
        ConnectButton                   matlab.ui.control.Button
        HoldGraphsButton                matlab.ui.control.StateButton
        Panel                           matlab.ui.container.Panel
        PressureAxes                    matlab.ui.control.UIAxes
        FlowMonitoringControlPanel      matlab.ui.container.Panel
        Label_9                         matlab.ui.control.Label
        Flow2_LabelField                matlab.ui.control.EditField
        Flow1_LabelField                matlab.ui.control.EditField
        SensorLabel_2                   matlab.ui.control.Label
        GraphLabel_2                    matlab.ui.control.Label
        Flow1_GraphEnable               matlab.ui.control.CheckBox
        Flow2_GraphEnable               matlab.ui.control.CheckBox
        allvaluesinLminLabel            matlab.ui.control.Label
        Flow2_EditField_Current         matlab.ui.control.NumericEditField
        Label_4                         matlab.ui.control.Label
        Flow1_EditField_Current         matlab.ui.control.NumericEditField
        Label_3                         matlab.ui.control.Label
        SetpointLabel_2                 matlab.ui.control.Label
        CurrentLabel_2                  matlab.ui.control.Label
        Flow2_SetPoint                  matlab.ui.control.NumericEditField
        Flow1_SetPoint                  matlab.ui.control.NumericEditField
        Label_2                         matlab.ui.control.Label
        PressureMonitoringControlPanel  matlab.ui.container.Panel
        Pressure3_LabelField            matlab.ui.control.EditField
        Pressure2_LabelField            matlab.ui.control.EditField
        Pressure1_LabelField            matlab.ui.control.EditField
        Label_8                         matlab.ui.control.Label
        Pressure1_GraphEnable           matlab.ui.control.CheckBox
        Pressure2_GraphEnable           matlab.ui.control.CheckBox
        Pressure3_GraphEnable           matlab.ui.control.CheckBox
        GraphLabel                      matlab.ui.control.Label
        SensorLabel                     matlab.ui.control.Label
        allvaluesinmmHgLabel            matlab.ui.control.Label
        Pressure3_EditField_Current     matlab.ui.control.NumericEditField
        Label_7                         matlab.ui.control.Label
        Pressure2_EditField_Current     matlab.ui.control.NumericEditField
        Label_6                         matlab.ui.control.Label
        Pressure1_EditField_Current     matlab.ui.control.NumericEditField
        Label_5                         matlab.ui.control.Label
        SetpointLabel                   matlab.ui.control.Label
        CurrentLabel                    matlab.ui.control.Label
        Pressure3_SetPoint              matlab.ui.control.NumericEditField
        Pressure2_SetPoint              matlab.ui.control.NumericEditField
        Pressure1_SetPoint              matlab.ui.control.NumericEditField
        Label                           matlab.ui.control.Label
        SettingsOtherInfoTab            matlab.ui.container.Tab
        Panel_3                         matlab.ui.container.Panel
        KdEditFieldLabel                matlab.ui.control.Label
        KdFlowEditField                 matlab.ui.control.NumericEditField
        KiFlowEditField                 matlab.ui.control.NumericEditField
        KiEditFieldLabel                matlab.ui.control.Label
        KpFlowEditField                 matlab.ui.control.NumericEditField
        KpEditFieldLabel                matlab.ui.control.Label
        FlowControlLabel                matlab.ui.control.Label
        Panel_2                         matlab.ui.container.Panel
        KdEditField_2Label              matlab.ui.control.Label
        KdPressureEditField             matlab.ui.control.NumericEditField
        KiPressureEditField             matlab.ui.control.NumericEditField
        KiEditField_2Label              matlab.ui.control.Label
        KpPressureEditField             matlab.ui.control.NumericEditField
        KpEditField_2Label              matlab.ui.control.Label
        PressureControlLabel            matlab.ui.control.Label
        PIDCoefficientsPanel            matlab.ui.container.Panel
        PumpAnalogWriteValue0255EditField  matlab.ui.control.NumericEditField
        PumpAnalogWriteValue0255Label   matlab.ui.control.Label
        PressureValveControlNotAvailablePanel  matlab.ui.container.Panel
        ClosedSlider                    matlab.ui.control.Slider
        ClosedSliderLabel               matlab.ui.control.Label
        MotorStepNumberEditField        matlab.ui.control.NumericEditField
        MotorStepNumberEditFieldLabel   matlab.ui.control.Label
        SimulateDataCheckBox            matlab.ui.control.CheckBox
    end

    % TODO: Add "refresh ports" button

    properties (Access = private)
        plot_raw_data = true;
        F_REFRESH = 20;  % App refresh rate (Hz)
        ARDUINO_ANALOGWRITE_MAX = 255;
        ARDUINO_SERIAL_BITRATE = 115200;

        arduinoObj;
        tempLogFile = fullfile(pwd, 'console_output_temp.txt');  % Define a persistent temporary log file that always stores session history

        t = [];
        flow_vals1 = [];  % Containers for sensor data
        flow_vals2 = [];

        pressure_vals1 = [];
        pressure_vals2 = [];
        pressure_vals3 = [];

        % PID Variables
        DT_PID = 0.1;  % Time step (adjust based on system response)

        Kp_flow = 1;  % PID coefficients when controlling flow
        Ki_flow = 0;
        Kd_flow = 0;

        Kp_pressure = 2;  % PID coefficients when controlling pressure
        Ki_pressure = 0;
        Kd_pressure = 0;
        
        pressureMotorPosition = 0;
        MAX_PRESSURE_MOTOR_POSITION = -200*7;  % 7 rotations, 200 steps per rotation; TODO: read in max stepper value from Arduino instead of hard-coding to 7 rotations

        lastFlowDutyCycle = 128;  % for comparisons
        flowDutyCycle = 128;  % Initialize halfway
        PumpControlMode = 'Manual';

        PID_setpoint_id = "None";  % Identifier as to which sensor's setpoint is influencing PID control, since only one sensor at a time can do so
        PID_setpoint = 0;
        PID_output = 0;
        PID_prev_error = 0;
        PID_integral = 0;
        PID_derivative = 0;

        % Legend entry labels
        pressure1_label = "1";
        pressure2_label = "2";
        pressure3_label = "3";

        flow1_label = "1";
        flow2_label = "2";
    end

    properties (Dependent)
        dt_refresh
    end

    methods
        function value = get.dt_refresh(app)
            value = 1 / app.F_REFRESH;  % Compute refresh interval dynamically
        end
    end

    methods (Access = private)

        function [t, newFlowIndicator, flow_value1, ...
                flow_value2, pressure_value_raw1, ...
                pressure_value1, pressure_value_raw2, pressure_value2, ...
                pressure_value_raw3, pressure_value3] = parseArduinoData(app, line)

            % Reads and parses serial data from the Arduino serial port.
            % Inputs:
            %   serialObj - MATLAB serial object for Arduino communication.
            % Outputs:
            %   flow_value_raw - Raw ADC value for flow sensor.
            %   flow_value - Processed flow rate in L/min.
            %   pressure_value_raw - Raw ADC value for pressure sensor.
            %   pressure_value - Processed pressure in mmHg.

            t = toc;

            if isempty(line)
                disp("No serial data received...")
                newFlowIndicator = 'X';   % 'Y' or 'N'
                flow_value_raw1 = -999;
                flow_value1 = -999;
                flow_value_raw2 = -999;
                flow_value2 = -999;
                pressure_value_raw1 = -999;
                pressure_value1 = -999;
                pressure_value_raw2 = -999;
                pressure_value2 = -999;
                pressure_value_raw3 = -999;
                pressure_value3 = -999;
            end

            % Attempt to parse the serial data
            try
                % Ensure we received all expected values
                data = sscanf(line, ...
                    ['NF: %c, F1: %f L/min, F2: %f L/min, ' ...
                    'P1R: %d, P1: %f mmHg, ' ...
                    'P2R: %d, P2: %f mmHg, ' ...
                    'P3R: %d, P3: %f mmHg, ' ...
                    'Pump: %d/255\n']);

                % Ensure we received all expected values
                if numel(data) == 10
                    newFlowIndicator = char(data(1));  % 'Y' or 'N'
                    flow_value1 = data(2);
                    flow_value2 = data(3);
                    pressure_value_raw1 = data(4);
                    pressure_value1 = data(5);
                    pressure_value_raw2 = data(6);
                    pressure_value2 = data(7);
                    pressure_value_raw3 = data(8);
                    pressure_value3 = data(9);
                    pump_analog_write = data(10);

                    % Get timestamp
                    timestamp = datetime("now");

                    % Format the new sensor data as a string
                    newLogEntry = sprintf(['===== Sensor Data (%s) =====\n', ...
                        'NF: %c\n', ...
                        'F1: %.2f L/min, F2: %.2f L/min\n', ...
                        'P1R: %d, P1: %.2f mmHg\n', ...
                        'P2R: %d, P2: %.2f mmHg\n', ...
                        'P3R: %d, P3: %.2f mmHg\n', ...
                        'Pump: %d/255\n'], ...
                        timestamp, newFlowIndicator, ...
                        flow_value1, flow_value2, ...
                        pressure_value_raw1, pressure_value1, ...
                        pressure_value_raw2, pressure_value2, ...
                        pressure_value_raw3, pressure_value3, ...
                        pump_analog_write);

                    % Print to console
                    fprintf('%s', newLogEntry);
                else
                    warning('Failed to parse expected number of values from serial data.');
                    disp(line);
                end
            catch
                warning('Error reading and parsing serial data.');
                disp(line);
            end

        end

        function startLogging(app)
            % Start recording console output to the temporary file
            diary(app.tempLogFile);
            diary on;

            % Notify user
            fprintf('Logging started. All session logs will be recorded in: %s\n', app.tempLogFile);
        end

        function saveLog(app, filename)
            % Stop logging to the temporary file
            diary off;

            % Copy the temp log file to the new provided log file
            copyfile(app.tempLogFile, filename);

            % Notify user
            fprintf('Log file saved as: %s\n', filename);

            % Restart logging to the temporary file to maintain session history
            diary(app.tempLogFile);
            diary on;
        end

        
        function clearDataArrays(app)
            tic;
            app.t = [];

            app.flow_vals1 = [];  
            app.flow_vals2 = [];

            app.pressure_vals1 = [];
            app.pressure_vals2 = [];
            app.pressure_vals3 = [];

            pause(app.dt_refresh);

            app.runMainLoop();  % Re-run it here from the beginning to reset time array properly
        end
       
        
        function runMainLoop(app)
            while (1)
                line = app.getArduinoData();

                % Read sensor data
                [tNew, ~, flow_value1, flow_value2, ...
                    ~, pressure_value1, ~, pressure_value2, ...
                    ~, pressure_value3] = app.parseArduinoData(line);

                % Generate artificial values
                % tNew = 1;
                % flow_value_raw1 = -999;
                % flow_value1 = -999;
                % flow_value_raw2 = -999;
                % flow_value2 = -999;
                % pressure_value_raw1 = -999;
                % pressure_value1 = -999;
                % pressure_value_raw2 = -999;
                % pressure_value2 = -999;
                % pressure_value_raw3 = -999;
                % pressure_value3 = -999;

                % Append new sensor data
                app.t = [app.t tNew];
                app.flow_vals1 = [app.flow_vals1 flow_value1];
                app.flow_vals2 = [app.flow_vals2 flow_value2];

                app.pressure_vals1 = [app.pressure_vals1 pressure_value1];
                app.pressure_vals2 = [app.pressure_vals2 pressure_value2];
                app.pressure_vals3 = [app.pressure_vals3 pressure_value3];

                % Update Edit Fields with the latest sensor data
                app.Pressure1_EditField_Current.Value = pressure_value1;
                app.Pressure2_EditField_Current.Value = pressure_value2;
                app.Pressure3_EditField_Current.Value = pressure_value3;

                app.Flow1_EditField_Current.Value = flow_value1;
                app.Flow2_EditField_Current.Value = flow_value2;

                % Plot pressure data based on checkbox selections
                % as long as "hold graph" is not pressed
                if app.HoldGraphsButton.Value == false
                    hold(app.PressureAxes, 'on');
                    cla(app.PressureAxes); % Clear previous plots

                    hexColors_Pressure = {'#8c819b', '#d6d3db', '#000000'};
                    rgbColors_Pressure = cellfun(@(c) sscanf(c(2:end), '%2x%2x%2x', [1 3])' / 255, hexColors_Pressure, 'UniformOutput', false);
                    lineWidth = 1.5;

                    if app.Pressure1_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals1, '--', 'Color', rgbColors_Pressure{1}, 'DisplayName', app.pressure1_label, 'LineWidth', lineWidth);
                    end
                    if app.Pressure2_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals2, '-', 'Color', rgbColors_Pressure{2}, 'DisplayName', app.pressure2_label, 'LineWidth', lineWidth);
                    end
                    if app.Pressure3_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals3, '-.', 'Color', rgbColors_Pressure{3}, 'DisplayName', app.pressure3_label, 'LineWidth', lineWidth);
                    end

                    hold(app.PressureAxes, 'off');
                    title(app.PressureAxes, 'Pressure Data');
                    xlabel(app.PressureAxes, 'Time (s)');
                    ylabel(app.PressureAxes, 'Pressure (mmHg)');
                    ylim(app.PressureAxes, [0 100]);
                    legend(app.PressureAxes, 'show','Location','northwest');

                    % Plot flow data based on checkbox selections
                    hold(app.FlowAxes, 'on');
                    cla(app.FlowAxes); % Clear previous plots

                    hexColors_Flow = {'#4a5c3f', '#2e4030'};
                    rgbColors_Flow = cellfun(@(c) sscanf(c(2:end), '%2x%2x%2x', [1 3])' / 255, hexColors_Flow, 'UniformOutput', false);

                    if app.Flow1_GraphEnable.Value
                        plot(app.FlowAxes, app.t, app.flow_vals1, '--', 'Color', rgbColors_Flow{1}, 'DisplayName', app.flow1_label, 'LineWidth', lineWidth);
                    end
                    if app.Flow2_GraphEnable.Value
                        plot(app.FlowAxes, app.t, app.flow_vals2, '-', 'Color', rgbColors_Flow{2}, 'DisplayName', app.flow2_label, 'LineWidth', lineWidth);
                    end

                    hold(app.FlowAxes, 'off');
                    title(app.FlowAxes, 'Flow Data');
                    xlabel(app.FlowAxes, 'Time (s)');
                    ylabel(app.FlowAxes, 'Flow (L/min)');
                    ylim(app.FlowAxes, [0 5]);
                    legend(app.FlowAxes, 'show','Location','northwest');
                end

                % Update PID Outputs
                if app.PumpControlMode == "Auto"
                    app.updatePIDOutputs();
                end

                % Check whether disconnect button has been clicked
                if app.ConnectButton.Text == "CONNECT"
                    break
                end

                % Pause to prevent crashes
                % pause(app.dt_refresh);
            end
        end

        function setControlMode(app,mode)
            if lower(mode) == "manual"
                app.PumpControlMode = "Manual";
                app.ControlModeSwitch.Value = "Manual";
                app.ControlModeSwitch.Enable = "off";  % to prevent user from switching toggle when they should just input a setpoint
                highlightSetPointUI(app, "None");
            elseif lower(mode) == "auto"
                app.PumpControlMode = "Auto";
                app.ControlModeSwitch.Value = "Auto";
                app.ControlModeSwitch.Enable = "on";
            else
                warning("Invalid control mode selected.");
            end
        end
        
        function sendControlValuesToArduino(app)
            app.MotorStepNumberEditField.Value = app.pressureMotorPosition;

            if lower(app.PumpControlMode) == "auto"
                % Update control values based on PID algorithm
                app.flowDutyCycle = app.flowDutyCycle + round(app.PID_output);

                % Place PID output within limits
                if app.flowDutyCycle > app.ARDUINO_ANALOGWRITE_MAX
                    app.flowDutyCycle = app.ARDUINO_ANALOGWRITE_MAX;
                elseif app.flowDutyCycle < 0
                    app.flowDutyCycle = 0;
                end
            elseif lower(app.PumpControlMode) == "manual"
                % do nothing
            else
                error("Invalid pump control mode selected.")
            end

            % Update flow pump control UI
            app.PumpAnalogWriteValue0255EditField.Value = app.flowDutyCycle;
            app.PumpPowerSlider.Value = app.flowDutyCycle / app.ARDUINO_ANALOGWRITE_MAX * app.PumpPowerSlider.Limits(2);

            % Send command to Arduino
            command = sprintf("MOT: %d, PMP: %d",app.pressureMotorPosition,app.flowDutyCycle);
            writeline(app.arduinoObj,command);
            app.WaitForArduinoMessage();
        end
        
        function updatePIDOutputs(app)
            % Choose appropriate PID value based on setpoint ID
            newValues = struct(...
            'Pressure1', app.pressure_vals1(end), ...
            'Pressure2', app.pressure_vals2(end), ...
            'Pressure3', app.pressure_vals3(end), ...
            'Flow1', app.flow_vals1(end), ...
            'Flow2', app.flow_vals2(end) ...
            );

            % Ensure PID_setpoint_id is a valid field name
            if isfield(newValues, app.PID_setpoint_id)
                newValue = newValues.(app.PID_setpoint_id); % Use dynamic field referencing
            else
                error('Invalid field name: %s', app.PID_setpoint_id);
            end

            % Use the correct PID coefficients based on setpoint ID
            if contains(app.PID_setpoint_id, 'Pressure', 'IgnoreCase', true)
                Kp = app.Kp_pressure;
                Ki = app.Ki_pressure;
                Kd = app.Kd_pressure;
            elseif contains(app.PID_setpoint_id, 'Flow', 'IgnoreCase', true)
                Kp = app.Kp_flow;
                Ki = app.Ki_flow;
                Kd = app.Kd_flow;
            else
                error('Unknown PID setpoint ID: %s', app.PID_setpoint_id);
            end

            % Calculate PID output
            PID_error = app.PID_setpoint - newValue;
            app.PID_integral = app.PID_integral + PID_error * app.DT_PID;
            app.PID_derivative = (PID_error - app.PID_prev_error) / app.DT_PID;
            app.PID_output = Kp * PID_error + Ki * app.PID_integral + Kd * app.PID_derivative;
            app.PID_prev_error = PID_error;

            % Send PID output to Arduino
            app.sendControlValuesToArduino();
        end
        
        function updateSerialPortList(app)
            % Get list of available serial ports
            ports = serialportlist;

            % Read the "friendly names" of the COM Port devices using Powershell

            portInfo = {};

            % Use system command to get device descriptions (Windows)
            if ispc
                [~, cmdout] = system("powershell -Command ""Get-WMIObject Win32_PnPEntity | Where-Object { $_.Name -match '\(COM[0-9]+\)' } | ForEach-Object { $_.Name }""");
                deviceList = splitlines(strtrim(cmdout));
            else
                deviceList = repmat("Unknown Device", size(ports));
            end

            for i = 1:length(ports)
                matchIdx = find(contains(deviceList, ports(i)), 1);
                if ~isempty(matchIdx)
                    idn = deviceList{matchIdx};
                    idn = strtrim(erase(idn, ['(', ports(i), ')'])); % Strip duplicate COM port info from the name
                else
                    idn = "Unknown Device";
                end

                portInfo{i} = sprintf('%s: %s', ports(i), idn);
            end

            % Update dropdown with formatted port names
            app.COMPortDropDown.Items = portInfo;
            app.COMPortDropDown.ItemsData = ports;  % To allow proper connections when using serialport() later off of the selected dropdown value
        end
        
        function setValveControlPanelEnabled(app, value)
            % Enable/disable all pressure valve control panel child objects
            children = app.PressureValveControlNotAvailablePanel.Children;
            for i = 1:numel(children)
                if value == true
                    children(i).Enable = 'on';
                else
                    children(i).Enable = 'off';
                end
            end
        end
        
        function highlightSetPointUI(app, setpointID)
            % Define all setpoints in a struct for easy iteration
            setpoints = struct(...
            'Pressure1', app.Pressure1_SetPoint, ...
            'Pressure2', app.Pressure2_SetPoint, ...
            'Pressure3', app.Pressure3_SetPoint, ...
            'Flow1', app.Flow1_SetPoint, ...
            'Flow2', app.Flow2_SetPoint ...
            );

            % Iterate through all setpoints and update colors
            fieldNames = fieldnames(setpoints);
            for i = 1:length(fieldNames)
                if strcmp(fieldNames{i}, setpointID)
                    setpoints.(fieldNames{i}).BackgroundColor = 'green';
                else
                    setpoints.(fieldNames{i}).BackgroundColor = 'white';
                end
            end
        end

        
        function setUpArduinoConnection(app)
            app.arduinoObj = serialport(app.COMPortDropDown.Value, app.ARDUINO_SERIAL_BITRATE);
            configureTerminator(app.arduinoObj, "LF"); % Assuming newline ('\n') terminates messages
            app.arduinoObj.Timeout = 5; % Timeout for read operations 

            while app.arduinoObj.NumBytesAvailable == 0
                pause(0.1);  % wait for initial to come in
            end
        end
        
        function line = getArduinoData(app)
            if app.SimulateDataCheckBox.Value
                % Simulate data in GUI (for testing)
                line = "New Flow?: N, Flow Raw 1: 0, Flow 1: 1.50 L/min, Flow Raw 2: 0, Flow 2: 2.50 L/min, Pressure Raw 1: 1023, Pressure 1: 20 mmHg, Pressure Raw 2: 1023, Pressure 2: 50 mmHg, Pressure Raw 3: 1023, Pressure 3: 80 mmHg\n";
            else
                if app.arduinoObj.NumBytesAvailable > 0
                    flushinput(app.arduinoObj);
                    readline(app.arduinoObj);  % Read until next newline in case flush clears part of a line
                    line = readline(app.arduinoObj); % Read a line of text
                else
                    line = [];
                end
                pause(0.1); % Small delay to prevent CPU overuse
            end
        end
        
        function WaitForArduinoMessage(app)
            while (app.arduinoObj.NumBytesAvailable == 0)
                pause(0.0001);
            end
        end
        
        function updateUIPIDCoeffs(app)
            % Update UI PID coefficient values
            app.KpFlowEditField.Value = app.Kp_flow;  % PID coefficients when controlling flow
            app.KiFlowEditField.Value = app.Ki_flow;
            app.KdFlowEditField.Value = app.Kd_flow;

            app.KpPressureEditField.Value = app.Kp_pressure;  % PID coefficients when controlling pressure
            app.KiPressureEditField.Value = app.Ki_pressure;
            app.KdPressureEditField.Value = app.Kd_pressure; 
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.startLogging();
            app.updateSerialPortList();
            app.UIFigure.Name = 'Cardiac Catheterization Testing Apparatus (CCTA)';
            app.updateUIPIDCoeffs();

            % Other setup
            app.MotorStepNumberEditField.Limits = [app.MAX_PRESSURE_MOTOR_POSITION 0];
        end

        % Button pushed function: ConnectButton
        function ConnectButtonPushed(app, event)
            
            if app.ConnectButton.Text == "Connect"
                % Someone just connected to the serial device
                app.ConnectButton.Text = "Disconnect";
                app.ConnectedLamp.Color = "green";

                if app.SimulateDataCheckBox.Value == false
                    app.setUpArduinoConnection();
                    app.setValveControlPanelEnabled(true);
                else
                    app.setValveControlPanelEnabled(false);
                end

                app.setValveControlPanelEnabled(true);
                app.clearDataArrays();
    
                tic;
                app.startLogging();
                app.runMainLoop();

            elseif app.ConnectButton.Text == "Disconnect"
                % Someone just disconnected from the serial device
                app.arduinoObj = [];
                app.setValveControlPanelEnabled(false);
                app.ConnectButton.Text = "Connect";
                app.ConnectedLamp.Color = [0.80,0.80,0.80];
            else
                error("Connect button container contains invalid text.")
            end
        end

        % Button pushed function: ClearDataButton
        function ClearDataButtonPushed(app, event)
            % Empty time and sensor data arrays
            app.clearDataArrays();
        end

        % Button pushed function: ExportDataButton
        function ExportDataButtonPushed(app, event)
            app.ExportDataButton.Text = "Exporting...";
            app.ExportDataButton.Enable = 'off';

            % Get current timestamp
            timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd-HH-mm-ss'));
            folderName = fullfile(pwd, ['CCTA-', timestamp]);
            
            % Create subfolder
            if ~exist(folderName, 'dir')
                mkdir(folderName);
            end
            
            % Save Pressure and Flow Axes as .fig and .png
            exportgraphics(app.PressureAxes, fullfile(folderName, 'PressurePlot.png'));
            exportgraphics(app.FlowAxes, fullfile(folderName, 'FlowPlot.png'));
            
            % Save data arrays to .mat/.csv files
            data.t = app.t;
            data.flow_vals1 = app.flow_vals1;
            data.flow_vals2 = app.flow_vals2;
            data.pressure_vals1 = app.pressure_vals1;
            data.pressure_vals2 = app.pressure_vals2;
            data.pressure_vals3 = app.pressure_vals3;
            
            save(fullfile(folderName, 'ExportedData.mat'), '-struct', 'data');

            csvData = table( ...
                app.t(:), ...
                app.flow_vals1(:), ...
                app.flow_vals2(:), ...
                app.pressure_vals1(:), ...
                app.pressure_vals2(:), ...
                app.pressure_vals3(:), ...
                'VariableNames', { ...
                'Time (s)', ...
                sprintf('Flow %s (L/min)', app.flow1_label), ...  % use labels from GUI to label the columns
                sprintf('Flow %s (L/min)', app.flow2_label), ...
                sprintf('Pressure %s (mmHg)', app.pressure1_label), ...
                sprintf('Pressure %s (mmHg)', app.pressure2_label), ...
                sprintf('Pressure %s (mmHg)', app.pressure3_label) ...
                } ...
                );
            
            writetable(csvData, fullfile(folderName, 'ExportedData.csv'));

            % Save console output to text file
            app.saveLog(fullfile(folderName, 'console_output.txt'));
            
            % Display dialog box
            uialert(app.UIFigure, ['Data has been saved to ', folderName], 'Export Successful', 'Icon', 'info');

            app.ExportDataButton.Text = "Export Data";
            app.ExportDataButton.Enable = 'on';
        end

        % Value changed function: MotorStepNumberEditField
        function MotorStepNumberEditFieldValueChanged(app, event)
            value = app.MotorStepNumberEditField.Value;
            app.ClosedSlider.Value = value/app.MAX_PRESSURE_MOTOR_POSITION * app.ClosedSlider.Limits(2);  % update slider

            app.pressureMotorPosition = value;
            app.sendControlValuesToArduino();
        end

        % Value changed function: ClosedSlider
        function ClosedSliderValueChanged(app, event)
            percent_closed = app.ClosedSlider.Value/app.ClosedSlider.Limits(2);  % negative because gear rotates valve the other way
            app.pressureMotorPosition = round(app.MAX_PRESSURE_MOTOR_POSITION * percent_closed);

            highlightSetPointUI(app, "None");
            flushoutput(app.arduinoObj);  % in case this was done in the middle of sending another command
            app.sendControlValuesToArduino();
        end

        % Button pushed function: RefreshListButton
        function RefreshListButtonPushed(app, event)
            app.updateSerialPortList();
        end

        % Value changed function: PumpPowerSlider
        function PumpPowerSliderValueChanged(app, event)
            app.flowDutyCycle = round(app.PumpPowerSlider.Value/app.PumpPowerSlider.Limits(2)*app.ARDUINO_ANALOGWRITE_MAX);
            app.PumpAnalogWriteValue0255EditField.Value = app.flowDutyCycle;
            app.setControlMode("Manual");

            flushoutput(app.arduinoObj);  % in case this was done in the middle of sending another command
            app.sendControlValuesToArduino();
        end

        % Value changed function: PumpAnalogWriteValue0255EditField
        function PumpAnalogWriteValue0255EditFieldValueChanged(app, event)
            value = app.PumpAnalogWriteValue0255EditField.Value;
            app.flowDutyCycle = value;
            app.PumpPowerSlider.Value = value/app.ARDUINO_ANALOGWRITE_MAX * app.PumpPowerSlider.Limits(2);  % update slider
            app.setControlMode("Manual");
            app.sendControlValuesToArduino();
        end

        % Value changed function: Pressure1_LabelField
        function Pressure1_LabelFieldValueChanged(app, event)
            app.pressure1_label = app.Pressure1_LabelField.Value;
        end

        % Value changed function: Pressure2_LabelField
        function Pressure2_LabelFieldValueChanged(app, event)
            app.pressure2_label = app.Pressure2_LabelField.Value;
        end

        % Value changed function: Pressure3_LabelField
        function Pressure3_LabelFieldValueChanged(app, event)
            app.pressure3_label = app.Pressure3_LabelField.Value;
        end

        % Value changed function: Flow1_LabelField
        function Flow1_LabelFieldValueChanged(app, event)
            app.flow1_label = app.Flow1_LabelField.Value;
        end

        % Value changed function: Flow2_LabelField
        function Flow2_LabelFieldValueChanged(app, event)
            app.flow2_label = app.Flow2_LabelField.Value;
        end

        % Value changed function: ControlModeSwitch
        function ControlModeSwitchValueChanged(app, event)
            app.setControlMode(app.ControlModeSwitch.Value);
            app.highlightSetPointUI("None");
        end

        % Value changed function: Flow1_SetPoint
        function Flow1_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Flow1_SetPoint.Value;
            app.setControlMode("Auto");
            app.highlightSetPointUI("Flow1");
            app.PID_setpoint_id = "Flow1";
        end

        % Value changed function: Flow2_SetPoint
        function Flow2_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Flow2_SetPoint.Value;
            app.setControlMode("Auto");
            app.highlightSetPointUI("Flow2");
            app.PID_setpoint_id = "Flow2";
        end

        % Value changed function: Pressure1_SetPoint
        function Pressure1_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure1_SetPoint.Value;
            app.setControlMode("Auto");
            app.highlightSetPointUI("Pressure1");
            app.PID_setpoint_id = "Pressure1";
        end

        % Value changed function: Pressure2_SetPoint
        function Pressure2_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure2_SetPoint.Value;
            app.setControlMode("Auto");
            app.highlightSetPointUI("Pressure2");
            app.PID_setpoint_id = "Pressure2";
        end

        % Value changed function: Pressure3_SetPoint
        function Pressure3_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure3_SetPoint.Value;
            app.setControlMode("Auto");
            app.highlightSetPointUI("Pressure3");
            app.PID_setpoint_id = "Pressure3";
        end

        % Value changed function: KpFlowEditField
        function KpFlowEditFieldValueChanged(app, event)
            app.Kp_flow = app.KpFlowEditField.Value;
        end

        % Value changed function: KiFlowEditField
        function KiFlowEditFieldValueChanged(app, event)
            app.Ki_flow = app.KiFlowEditField.Value;
        end

        % Value changed function: KdFlowEditField
        function KdFlowEditFieldValueChanged(app, event)
            app.Kd_flow = app.KdFlowEditField.Value;
        end

        % Value changed function: KpPressureEditField
        function KpPressureEditFieldValueChanged(app, event)
            app.Kp_pressure = app.KpPressureEditField.Value;
        end

        % Value changed function: KiPressureEditField
        function KiPressureEditFieldValueChanged(app, event)
            app.Ki_pressure = app.KiPressureEditField.Value;
        end

        % Value changed function: KdPressureEditField
        function KdPressureEditFieldValueChanged(app, event)
            app.Kd_pressure = app.KdPressureEditField.Value;
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1228 522];
            app.UIFigure.Name = 'MATLAB App';

            % Create TabGroup2
            app.TabGroup2 = uitabgroup(app.UIFigure);
            app.TabGroup2.Position = [1 -1 1229 524];

            % Create MainTab
            app.MainTab = uitab(app.TabGroup2);
            app.MainTab.Title = 'Main';

            % Create PressureMonitoringControlPanel
            app.PressureMonitoringControlPanel = uipanel(app.MainTab);
            app.PressureMonitoringControlPanel.Title = 'Pressure Monitoring & Control';
            app.PressureMonitoringControlPanel.BackgroundColor = [0.8392 0.8275 0.8588];
            app.PressureMonitoringControlPanel.FontWeight = 'bold';
            app.PressureMonitoringControlPanel.Position = [15 338 298 150];

            % Create Label
            app.Label = uilabel(app.PressureMonitoringControlPanel);
            app.Label.FontSize = 18;
            app.Label.FontWeight = 'bold';
            app.Label.Position = [62 98 25 23];
            app.Label.Text = '';

            % Create Pressure1_SetPoint
            app.Pressure1_SetPoint = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure1_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Pressure1_SetPointValueChanged, true);
            app.Pressure1_SetPoint.Position = [134 77 49 22];

            % Create Pressure2_SetPoint
            app.Pressure2_SetPoint = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure2_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Pressure2_SetPointValueChanged, true);
            app.Pressure2_SetPoint.Position = [134 46 49 22];

            % Create Pressure3_SetPoint
            app.Pressure3_SetPoint = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure3_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Pressure3_SetPointValueChanged, true);
            app.Pressure3_SetPoint.Position = [134 15 50 22];

            % Create CurrentLabel
            app.CurrentLabel = uilabel(app.PressureMonitoringControlPanel);
            app.CurrentLabel.Position = [72 98 45 22];
            app.CurrentLabel.Text = 'Current';

            % Create SetpointLabel
            app.SetpointLabel = uilabel(app.PressureMonitoringControlPanel);
            app.SetpointLabel.Position = [137 99 49 22];
            app.SetpointLabel.Text = 'Setpoint';

            % Create Label_5
            app.Label_5 = uilabel(app.PressureMonitoringControlPanel);
            app.Label_5.HorizontalAlignment = 'center';
            app.Label_5.Position = [17 77 25 22];
            app.Label_5.Text = '1';

            % Create Pressure1_EditField_Current
            app.Pressure1_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure1_EditField_Current.Editable = 'off';
            app.Pressure1_EditField_Current.Position = [68 77 52 22];

            % Create Label_6
            app.Label_6 = uilabel(app.PressureMonitoringControlPanel);
            app.Label_6.HorizontalAlignment = 'center';
            app.Label_6.Position = [17 46 25 22];
            app.Label_6.Text = '2';

            % Create Pressure2_EditField_Current
            app.Pressure2_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure2_EditField_Current.Editable = 'off';
            app.Pressure2_EditField_Current.Position = [68 46 52 22];

            % Create Label_7
            app.Label_7 = uilabel(app.PressureMonitoringControlPanel);
            app.Label_7.HorizontalAlignment = 'center';
            app.Label_7.Position = [17 15 25 22];
            app.Label_7.Text = '3';

            % Create Pressure3_EditField_Current
            app.Pressure3_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure3_EditField_Current.Editable = 'off';
            app.Pressure3_EditField_Current.Position = [68 15 52 22];

            % Create allvaluesinmmHgLabel
            app.allvaluesinmmHgLabel = uilabel(app.PressureMonitoringControlPanel);
            app.allvaluesinmmHgLabel.FontAngle = 'italic';
            app.allvaluesinmmHgLabel.Position = [183 127 114 22];
            app.allvaluesinmmHgLabel.Text = '(all values in mmHg)';

            % Create SensorLabel
            app.SensorLabel = uilabel(app.PressureMonitoringControlPanel);
            app.SensorLabel.FontWeight = 'bold';
            app.SensorLabel.Position = [7 98 45 22];
            app.SensorLabel.Text = 'Sensor';

            % Create GraphLabel
            app.GraphLabel = uilabel(app.PressureMonitoringControlPanel);
            app.GraphLabel.Position = [249 99 45 22];
            app.GraphLabel.Text = 'Graph?';

            % Create Pressure3_GraphEnable
            app.Pressure3_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure3_GraphEnable.Text = '';
            app.Pressure3_GraphEnable.Position = [259 15 25 22];
            app.Pressure3_GraphEnable.Value = true;

            % Create Pressure2_GraphEnable
            app.Pressure2_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure2_GraphEnable.Text = '';
            app.Pressure2_GraphEnable.Position = [259 46 25 22];
            app.Pressure2_GraphEnable.Value = true;

            % Create Pressure1_GraphEnable
            app.Pressure1_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure1_GraphEnable.Text = '';
            app.Pressure1_GraphEnable.Position = [260 79 25 22];
            app.Pressure1_GraphEnable.Value = true;

            % Create Label_8
            app.Label_8 = uilabel(app.PressureMonitoringControlPanel);
            app.Label_8.Position = [206 98 34 22];

            % Create Pressure1_LabelField
            app.Pressure1_LabelField = uieditfield(app.PressureMonitoringControlPanel, 'text');
            app.Pressure1_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure1_LabelFieldValueChanged, true);
            app.Pressure1_LabelField.Position = [194 76 55 22];
            app.Pressure1_LabelField.Value = '1';

            % Create Pressure2_LabelField
            app.Pressure2_LabelField = uieditfield(app.PressureMonitoringControlPanel, 'text');
            app.Pressure2_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure2_LabelFieldValueChanged, true);
            app.Pressure2_LabelField.Position = [194 46 55 22];
            app.Pressure2_LabelField.Value = '2';

            % Create Pressure3_LabelField
            app.Pressure3_LabelField = uieditfield(app.PressureMonitoringControlPanel, 'text');
            app.Pressure3_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure3_LabelFieldValueChanged, true);
            app.Pressure3_LabelField.Position = [194 15 55 22];
            app.Pressure3_LabelField.Value = '3';

            % Create FlowMonitoringControlPanel
            app.FlowMonitoringControlPanel = uipanel(app.MainTab);
            app.FlowMonitoringControlPanel.Title = 'Flow Monitoring & Control';
            app.FlowMonitoringControlPanel.BackgroundColor = [0.7725 0.8314 0.7569];
            app.FlowMonitoringControlPanel.FontWeight = 'bold';
            app.FlowMonitoringControlPanel.Position = [15 206 298 116];

            % Create Label_2
            app.Label_2 = uilabel(app.FlowMonitoringControlPanel);
            app.Label_2.FontSize = 18;
            app.Label_2.FontWeight = 'bold';
            app.Label_2.Position = [67 45 25 23];
            app.Label_2.Text = '';

            % Create Flow1_SetPoint
            app.Flow1_SetPoint = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow1_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Flow1_SetPointValueChanged, true);
            app.Flow1_SetPoint.Position = [134 44 49 22];

            % Create Flow2_SetPoint
            app.Flow2_SetPoint = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow2_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Flow2_SetPointValueChanged, true);
            app.Flow2_SetPoint.Position = [134 13 49 22];

            % Create CurrentLabel_2
            app.CurrentLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.CurrentLabel_2.Position = [71 66 45 22];
            app.CurrentLabel_2.Text = 'Current';

            % Create SetpointLabel_2
            app.SetpointLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.SetpointLabel_2.Position = [134 66 49 22];
            app.SetpointLabel_2.Text = 'Setpoint';

            % Create Label_3
            app.Label_3 = uilabel(app.FlowMonitoringControlPanel);
            app.Label_3.HorizontalAlignment = 'center';
            app.Label_3.Position = [17 45 25 22];
            app.Label_3.Text = '1';

            % Create Flow1_EditField_Current
            app.Flow1_EditField_Current = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow1_EditField_Current.Editable = 'off';
            app.Flow1_EditField_Current.Position = [67 44 53 22];

            % Create Label_4
            app.Label_4 = uilabel(app.FlowMonitoringControlPanel);
            app.Label_4.HorizontalAlignment = 'center';
            app.Label_4.Position = [17 13 25 22];
            app.Label_4.Text = '2';

            % Create Flow2_EditField_Current
            app.Flow2_EditField_Current = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow2_EditField_Current.Editable = 'off';
            app.Flow2_EditField_Current.Position = [67 13 53 22];

            % Create allvaluesinLminLabel
            app.allvaluesinLminLabel = uilabel(app.FlowMonitoringControlPanel);
            app.allvaluesinLminLabel.FontAngle = 'italic';
            app.allvaluesinLminLabel.Position = [189 93 108 22];
            app.allvaluesinLminLabel.Text = '(all values in L/min)';

            % Create Flow2_GraphEnable
            app.Flow2_GraphEnable = uicheckbox(app.FlowMonitoringControlPanel);
            app.Flow2_GraphEnable.Text = '';
            app.Flow2_GraphEnable.Position = [258 13 25 22];
            app.Flow2_GraphEnable.Value = true;

            % Create Flow1_GraphEnable
            app.Flow1_GraphEnable = uicheckbox(app.FlowMonitoringControlPanel);
            app.Flow1_GraphEnable.Text = '';
            app.Flow1_GraphEnable.Position = [258 45 25 22];
            app.Flow1_GraphEnable.Value = true;

            % Create GraphLabel_2
            app.GraphLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.GraphLabel_2.Position = [248 66 45 22];
            app.GraphLabel_2.Text = 'Graph?';

            % Create SensorLabel_2
            app.SensorLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.SensorLabel_2.FontWeight = 'bold';
            app.SensorLabel_2.Position = [7 66 45 22];
            app.SensorLabel_2.Text = 'Sensor';

            % Create Flow1_LabelField
            app.Flow1_LabelField = uieditfield(app.FlowMonitoringControlPanel, 'text');
            app.Flow1_LabelField.ValueChangedFcn = createCallbackFcn(app, @Flow1_LabelFieldValueChanged, true);
            app.Flow1_LabelField.Position = [192 43 55 22];
            app.Flow1_LabelField.Value = '1';

            % Create Flow2_LabelField
            app.Flow2_LabelField = uieditfield(app.FlowMonitoringControlPanel, 'text');
            app.Flow2_LabelField.ValueChangedFcn = createCallbackFcn(app, @Flow2_LabelFieldValueChanged, true);
            app.Flow2_LabelField.Position = [192 13 55 22];
            app.Flow2_LabelField.Value = '2';

            % Create Label_9
            app.Label_9 = uilabel(app.FlowMonitoringControlPanel);
            app.Label_9.Position = [202 66 34 22];

            % Create Panel
            app.Panel = uipanel(app.MainTab);
            app.Panel.BorderColor = [0.651 0.651 0.651];
            app.Panel.BorderWidth = 0;
            app.Panel.BackgroundColor = [0.8392 0.8275 0.8588];
            app.Panel.FontAngle = 'italic';
            app.Panel.Position = [332 154 435 333];

            % Create PressureAxes
            app.PressureAxes = uiaxes(app.Panel);
            title(app.PressureAxes, 'Pressure Data')
            xlabel(app.PressureAxes, 'Time (s)')
            ylabel(app.PressureAxes, 'Pressure (mmHg)')
            zlabel(app.PressureAxes, 'Z')
            app.PressureAxes.FontWeight = 'bold';
            app.PressureAxes.FontSize = 14;
            app.PressureAxes.Position = [12 9 409 312];

            % Create HoldGraphsButton
            app.HoldGraphsButton = uibutton(app.MainTab, 'state');
            app.HoldGraphsButton.Text = 'Hold Graphs';
            app.HoldGraphsButton.BackgroundColor = [0.1569 0.5725 0.8431];
            app.HoldGraphsButton.FontSize = 14;
            app.HoldGraphsButton.FontWeight = 'bold';
            app.HoldGraphsButton.FontColor = [1 1 1];
            app.HoldGraphsButton.Position = [791 40 126 31];

            % Create ConnectButton
            app.ConnectButton = uibutton(app.MainTab, 'push');
            app.ConnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
            app.ConnectButton.BackgroundColor = [0.0824 0.1961 0.2627];
            app.ConnectButton.FontSize = 14;
            app.ConnectButton.FontWeight = 'bold';
            app.ConnectButton.FontColor = [1 1 1];
            app.ConnectButton.Position = [929 79 126 31];
            app.ConnectButton.Text = 'Connect';

            % Create ClearDataButton
            app.ClearDataButton = uibutton(app.MainTab, 'push');
            app.ClearDataButton.ButtonPushedFcn = createCallbackFcn(app, @ClearDataButtonPushed, true);
            app.ClearDataButton.BackgroundColor = [0.1569 0.5725 0.8431];
            app.ClearDataButton.FontSize = 14;
            app.ClearDataButton.FontWeight = 'bold';
            app.ClearDataButton.FontColor = [1 1 1];
            app.ClearDataButton.Position = [653 40 126 31];
            app.ClearDataButton.Text = 'Clear Data';

            % Create ExportDataButton
            app.ExportDataButton = uibutton(app.MainTab, 'push');
            app.ExportDataButton.ButtonPushedFcn = createCallbackFcn(app, @ExportDataButtonPushed, true);
            app.ExportDataButton.BackgroundColor = [0.1569 0.5725 0.8431];
            app.ExportDataButton.FontSize = 14;
            app.ExportDataButton.FontWeight = 'bold';
            app.ExportDataButton.FontColor = [1 1 1];
            app.ExportDataButton.Position = [929 40 126 31];
            app.ExportDataButton.Text = 'Export Data';

            % Create RefreshListButton
            app.RefreshListButton = uibutton(app.MainTab, 'push');
            app.RefreshListButton.ButtonPushedFcn = createCallbackFcn(app, @RefreshListButtonPushed, true);
            app.RefreshListButton.BackgroundColor = [0.0824 0.1961 0.2627];
            app.RefreshListButton.FontSize = 14;
            app.RefreshListButton.FontWeight = 'bold';
            app.RefreshListButton.FontColor = [1 1 1];
            app.RefreshListButton.Position = [791 79 126 31];
            app.RefreshListButton.Text = 'Refresh List';

            % Create GraphOptionsLabel
            app.GraphOptionsLabel = uilabel(app.MainTab);
            app.GraphOptionsLabel.FontSize = 18;
            app.GraphOptionsLabel.FontWeight = 'bold';
            app.GraphOptionsLabel.FontColor = [0.1569 0.5725 0.8431];
            app.GraphOptionsLabel.Position = [505 43 131 28];
            app.GraphOptionsLabel.Text = 'Graph Options';

            % Create COMPortLabel
            app.COMPortLabel = uilabel(app.MainTab);
            app.COMPortLabel.HorizontalAlignment = 'right';
            app.COMPortLabel.FontSize = 18;
            app.COMPortLabel.FontWeight = 'bold';
            app.COMPortLabel.FontColor = [0.0824 0.1961 0.2627];
            app.COMPortLabel.Position = [541 82 93 23];
            app.COMPortLabel.Text = 'COM Port ';

            % Create COMPortDropDown
            app.COMPortDropDown = uidropdown(app.MainTab);
            app.COMPortDropDown.FontSize = 14;
            app.COMPortDropDown.FontWeight = 'bold';
            app.COMPortDropDown.FontColor = [1 1 1];
            app.COMPortDropDown.BackgroundColor = [0.0824 0.1961 0.2627];
            app.COMPortDropDown.Position = [653 79 126 31];

            % Create ConnectedLamp
            app.ConnectedLamp = uilamp(app.MainTab);
            app.ConnectedLamp.Position = [1064 84 20 20];
            app.ConnectedLamp.Color = [0.8 0.8 0.8];

            % Create PumpControlPanel
            app.PumpControlPanel = uipanel(app.MainTab);
            app.PumpControlPanel.Title = 'Pump Control';
            app.PumpControlPanel.BackgroundColor = [0.902 0.902 0.902];
            app.PumpControlPanel.FontWeight = 'bold';
            app.PumpControlPanel.Position = [15 70 298 124];

            % Create PumpPowerSliderLabel
            app.PumpPowerSliderLabel = uilabel(app.PumpControlPanel);
            app.PumpPowerSliderLabel.HorizontalAlignment = 'right';
            app.PumpPowerSliderLabel.FontWeight = 'bold';
            app.PumpPowerSliderLabel.Position = [10 39 95 22];
            app.PumpPowerSliderLabel.Text = 'Pump Power  %';

            % Create PumpPowerSlider
            app.PumpPowerSlider = uislider(app.PumpControlPanel);
            app.PumpPowerSlider.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSliderValueChanged, true);
            app.PumpPowerSlider.Position = [126 48 150 3];

            % Create ControlModeSwitchLabel
            app.ControlModeSwitchLabel = uilabel(app.PumpControlPanel);
            app.ControlModeSwitchLabel.HorizontalAlignment = 'center';
            app.ControlModeSwitchLabel.FontWeight = 'bold';
            app.ControlModeSwitchLabel.Position = [12 70 82 22];
            app.ControlModeSwitchLabel.Text = 'Control Mode';

            % Create ControlModeSwitch
            app.ControlModeSwitch = uiswitch(app.PumpControlPanel, 'slider');
            app.ControlModeSwitch.Items = {'Manual', 'Auto'};
            app.ControlModeSwitch.ValueChangedFcn = createCallbackFcn(app, @ControlModeSwitchValueChanged, true);
            app.ControlModeSwitch.Enable = 'off';
            app.ControlModeSwitch.Tooltip = {'Manual - control pump power using the slider below'; ''; 'Auto - Input a flow or puressure setpoint in the panels above and the system will automatically adjust'};
            app.ControlModeSwitch.Position = [189 71 45 20];
            app.ControlModeSwitch.Value = 'Manual';

            % Create Panel_4
            app.Panel_4 = uipanel(app.MainTab);
            app.Panel_4.BorderWidth = 0;
            app.Panel_4.BackgroundColor = [0.7725 0.8314 0.7569];
            app.Panel_4.Position = [767 154 447 333];

            % Create FlowAxes
            app.FlowAxes = uiaxes(app.Panel_4);
            title(app.FlowAxes, 'Flow Data')
            xlabel(app.FlowAxes, 'Time(s)')
            ylabel(app.FlowAxes, 'Flow (L/min)')
            zlabel(app.FlowAxes, 'Z')
            app.FlowAxes.FontWeight = 'bold';
            app.FlowAxes.FontSize = 14;
            app.FlowAxes.Position = [13 11 419 310];

            % Create SettingsOtherInfoTab
            app.SettingsOtherInfoTab = uitab(app.TabGroup2);
            app.SettingsOtherInfoTab.Title = 'Settings & Other Info.';

            % Create SimulateDataCheckBox
            app.SimulateDataCheckBox = uicheckbox(app.SettingsOtherInfoTab);
            app.SimulateDataCheckBox.Text = 'Simulate Data?';
            app.SimulateDataCheckBox.Position = [180 253 104 22];

            % Create PressureValveControlNotAvailablePanel
            app.PressureValveControlNotAvailablePanel = uipanel(app.SettingsOtherInfoTab);
            app.PressureValveControlNotAvailablePanel.Title = 'Pressure Valve Control (Not Available)';
            app.PressureValveControlNotAvailablePanel.Position = [24 342 260 141];

            % Create MotorStepNumberEditFieldLabel
            app.MotorStepNumberEditFieldLabel = uilabel(app.PressureValveControlNotAvailablePanel);
            app.MotorStepNumberEditFieldLabel.HorizontalAlignment = 'right';
            app.MotorStepNumberEditFieldLabel.Enable = 'off';
            app.MotorStepNumberEditFieldLabel.Position = [17 79 110 22];
            app.MotorStepNumberEditFieldLabel.Text = 'Motor Step Number';

            % Create MotorStepNumberEditField
            app.MotorStepNumberEditField = uieditfield(app.PressureValveControlNotAvailablePanel, 'numeric');
            app.MotorStepNumberEditField.Limits = [0 200];
            app.MotorStepNumberEditField.RoundFractionalValues = 'on';
            app.MotorStepNumberEditField.ValueChangedFcn = createCallbackFcn(app, @MotorStepNumberEditFieldValueChanged, true);
            app.MotorStepNumberEditField.Enable = 'off';
            app.MotorStepNumberEditField.Position = [142 79 100 22];

            % Create ClosedSliderLabel
            app.ClosedSliderLabel = uilabel(app.PressureValveControlNotAvailablePanel);
            app.ClosedSliderLabel.HorizontalAlignment = 'right';
            app.ClosedSliderLabel.Enable = 'off';
            app.ClosedSliderLabel.Position = [9 33 56 22];
            app.ClosedSliderLabel.Text = '% Closed';

            % Create ClosedSlider
            app.ClosedSlider = uislider(app.PressureValveControlNotAvailablePanel);
            app.ClosedSlider.ValueChangedFcn = createCallbackFcn(app, @ClosedSliderValueChanged, true);
            app.ClosedSlider.Enable = 'off';
            app.ClosedSlider.Position = [86 42 150 3];

            % Create PumpAnalogWriteValue0255Label
            app.PumpAnalogWriteValue0255Label = uilabel(app.SettingsOtherInfoTab);
            app.PumpAnalogWriteValue0255Label.HorizontalAlignment = 'right';
            app.PumpAnalogWriteValue0255Label.FontWeight = 'bold';
            app.PumpAnalogWriteValue0255Label.Position = [50 290 153 30];
            app.PumpAnalogWriteValue0255Label.Text = {'Pump Analog Write Value '; '(0-255)'};

            % Create PumpAnalogWriteValue0255EditField
            app.PumpAnalogWriteValue0255EditField = uieditfield(app.SettingsOtherInfoTab, 'numeric');
            app.PumpAnalogWriteValue0255EditField.Limits = [0 255];
            app.PumpAnalogWriteValue0255EditField.RoundFractionalValues = 'on';
            app.PumpAnalogWriteValue0255EditField.ValueChangedFcn = createCallbackFcn(app, @PumpAnalogWriteValue0255EditFieldValueChanged, true);
            app.PumpAnalogWriteValue0255EditField.Position = [218 298 66 22];

            % Create PIDCoefficientsPanel
            app.PIDCoefficientsPanel = uipanel(app.SettingsOtherInfoTab);
            app.PIDCoefficientsPanel.Title = 'PID Coefficients';
            app.PIDCoefficientsPanel.Position = [299 342 469 139];

            % Create Panel_2
            app.Panel_2 = uipanel(app.SettingsOtherInfoTab);
            app.Panel_2.BackgroundColor = [0.8392 0.8275 0.8588];
            app.Panel_2.Position = [311 408 446 42];

            % Create PressureControlLabel
            app.PressureControlLabel = uilabel(app.Panel_2);
            app.PressureControlLabel.FontWeight = 'bold';
            app.PressureControlLabel.Position = [12 10 102 22];
            app.PressureControlLabel.Text = 'Pressure Control';

            % Create KpEditField_2Label
            app.KpEditField_2Label = uilabel(app.Panel_2);
            app.KpEditField_2Label.HorizontalAlignment = 'right';
            app.KpEditField_2Label.Position = [135 10 25 22];
            app.KpEditField_2Label.Text = 'Kp';

            % Create KpPressureEditField
            app.KpPressureEditField = uieditfield(app.Panel_2, 'numeric');
            app.KpPressureEditField.ValueChangedFcn = createCallbackFcn(app, @KpPressureEditFieldValueChanged, true);
            app.KpPressureEditField.Position = [175 10 42 22];

            % Create KiEditField_2Label
            app.KiEditField_2Label = uilabel(app.Panel_2);
            app.KiEditField_2Label.HorizontalAlignment = 'right';
            app.KiEditField_2Label.Position = [240 10 25 22];
            app.KiEditField_2Label.Text = 'Ki';

            % Create KiPressureEditField
            app.KiPressureEditField = uieditfield(app.Panel_2, 'numeric');
            app.KiPressureEditField.ValueChangedFcn = createCallbackFcn(app, @KiPressureEditFieldValueChanged, true);
            app.KiPressureEditField.Position = [280 10 42 22];

            % Create KdPressureEditField
            app.KdPressureEditField = uieditfield(app.Panel_2, 'numeric');
            app.KdPressureEditField.ValueChangedFcn = createCallbackFcn(app, @KdPressureEditFieldValueChanged, true);
            app.KdPressureEditField.Position = [386 10 42 22];

            % Create KdEditField_2Label
            app.KdEditField_2Label = uilabel(app.Panel_2);
            app.KdEditField_2Label.HorizontalAlignment = 'right';
            app.KdEditField_2Label.Position = [346 10 25 22];
            app.KdEditField_2Label.Text = 'Kd';

            % Create Panel_3
            app.Panel_3 = uipanel(app.SettingsOtherInfoTab);
            app.Panel_3.BackgroundColor = [0.7725 0.8314 0.7569];
            app.Panel_3.Position = [311 355 446 42];

            % Create FlowControlLabel
            app.FlowControlLabel = uilabel(app.Panel_3);
            app.FlowControlLabel.FontWeight = 'bold';
            app.FlowControlLabel.Position = [36 10 78 22];
            app.FlowControlLabel.Text = 'Flow Control';

            % Create KpEditFieldLabel
            app.KpEditFieldLabel = uilabel(app.Panel_3);
            app.KpEditFieldLabel.HorizontalAlignment = 'right';
            app.KpEditFieldLabel.Position = [135 10 25 22];
            app.KpEditFieldLabel.Text = 'Kp';

            % Create KpFlowEditField
            app.KpFlowEditField = uieditfield(app.Panel_3, 'numeric');
            app.KpFlowEditField.ValueChangedFcn = createCallbackFcn(app, @KpFlowEditFieldValueChanged, true);
            app.KpFlowEditField.Position = [175 10 42 22];

            % Create KiEditFieldLabel
            app.KiEditFieldLabel = uilabel(app.Panel_3);
            app.KiEditFieldLabel.HorizontalAlignment = 'right';
            app.KiEditFieldLabel.Position = [240 10 25 22];
            app.KiEditFieldLabel.Text = 'Ki';

            % Create KiFlowEditField
            app.KiFlowEditField = uieditfield(app.Panel_3, 'numeric');
            app.KiFlowEditField.ValueChangedFcn = createCallbackFcn(app, @KiFlowEditFieldValueChanged, true);
            app.KiFlowEditField.Position = [280 10 42 22];

            % Create KdFlowEditField
            app.KdFlowEditField = uieditfield(app.Panel_3, 'numeric');
            app.KdFlowEditField.ValueChangedFcn = createCallbackFcn(app, @KdFlowEditFieldValueChanged, true);
            app.KdFlowEditField.Position = [386 10 42 22];

            % Create KdEditFieldLabel
            app.KdEditFieldLabel = uilabel(app.Panel_3);
            app.KdEditFieldLabel.HorizontalAlignment = 'right';
            app.KdEditFieldLabel.Position = [346 10 25 22];
            app.KdEditFieldLabel.Text = 'Kd';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = CCTA_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end