classdef CCTA_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                       matlab.ui.Figure
        TabGroup2                      matlab.ui.container.TabGroup
        MainTab                        matlab.ui.container.Tab
        FlowDataTitleLabelPanel        matlab.ui.container.Panel
        FlowDataTitleLabel             matlab.ui.control.Label
        PressureDataTitleLabelPanel    matlab.ui.container.Panel
        PressureDataTitleLabel         matlab.ui.control.Label
        OptionsLabel                   matlab.ui.control.Label
        ConnectDisconnectButton        matlab.ui.control.Button
        RefreshConnectionsButton       matlab.ui.control.Button
        ConnectedLamp                  matlab.ui.control.Lamp
        ConnectionDropDown             matlab.ui.control.DropDown
        ConnectionDropDownLabel        matlab.ui.control.Label
        OptionsPanel                   matlab.ui.container.Panel
        CalibrateButton                matlab.ui.control.Button
        ExportDataButton               matlab.ui.control.Button
        ClearDataButton                matlab.ui.control.Button
        PauseGraphsButton              matlab.ui.control.StateButton
        FlowGraphPanel                 matlab.ui.container.Panel
        FlowAxesWaitingForConnectionLabel  matlab.ui.control.Label
        FlowAxes                       matlab.ui.control.UIAxes
        PumpControlPanel               matlab.ui.container.Panel
        StartPulsatileFlowButton       matlab.ui.control.StateButton
        ControlModeSwitch              matlab.ui.control.Switch
        ControlModeSwitchLabel         matlab.ui.control.Label
        PumpPowerSlider                matlab.ui.control.Slider
        PumpPowerSliderLabel           matlab.ui.control.Label
        PressureGraphPanel             matlab.ui.container.Panel
        PressureAxesWaitingForConnectionLabel  matlab.ui.control.Label
        PressureAxes                   matlab.ui.control.UIAxes
        Panel_2                        matlab.ui.container.Panel
        Flow2_EditField_Avg            matlab.ui.control.EditField
        Flow1_EditField_Avg            matlab.ui.control.EditField
        AvgSDLabel_2                   matlab.ui.control.Label
        Flow1_LabelField               matlab.ui.control.EditField
        Label_9                        matlab.ui.control.Label
        Flow2_LabelField               matlab.ui.control.EditField
        SensorLabel_2                  matlab.ui.control.Label
        Label_10                       matlab.ui.control.Label
        Flow1_GraphEnable              matlab.ui.control.CheckBox
        Flow2_GraphEnable              matlab.ui.control.CheckBox
        Flow2_EditField_Current        matlab.ui.control.NumericEditField
        Label_8                        matlab.ui.control.Label
        Flow1_EditField_Current        matlab.ui.control.NumericEditField
        Label_7                        matlab.ui.control.Label
        TargetLabel_2                  matlab.ui.control.Label
        CurrentLabel_2                 matlab.ui.control.Label
        Flow2_SetPoint                 matlab.ui.control.NumericEditField
        Flow1_SetPoint                 matlab.ui.control.NumericEditField
        Label_6                        matlab.ui.control.Label
        Panel                          matlab.ui.container.Panel
        Pressure3_EditField_Avg        matlab.ui.control.EditField
        Pressure2_EditField_Avg        matlab.ui.control.EditField
        Pressure1_EditField_Avg        matlab.ui.control.EditField
        AvgSDLabel                     matlab.ui.control.Label
        Pressure3_EditField_SHO        matlab.ui.control.NumericEditField
        Pressure2_EditField_SHO        matlab.ui.control.NumericEditField
        Pressure1_EditField_SHO        matlab.ui.control.NumericEditField
        SHOLabel_Pressure              matlab.ui.control.Label
        Label_11                       matlab.ui.control.Label
        Pressure3_LabelField           matlab.ui.control.EditField
        Pressure2_LabelField           matlab.ui.control.EditField
        Pressure1_LabelField           matlab.ui.control.EditField
        Label_5                        matlab.ui.control.Label
        Pressure1_GraphEnable          matlab.ui.control.CheckBox
        Pressure2_GraphEnable          matlab.ui.control.CheckBox
        Pressure3_GraphEnable          matlab.ui.control.CheckBox
        SensorLabel                    matlab.ui.control.Label
        Pressure3_EditField_Current    matlab.ui.control.NumericEditField
        Label_4                        matlab.ui.control.Label
        Pressure2_EditField_Current    matlab.ui.control.NumericEditField
        Label_3                        matlab.ui.control.Label
        Pressure1_EditField_Current    matlab.ui.control.NumericEditField
        Label_2                        matlab.ui.control.Label
        TargetLabel                    matlab.ui.control.Label
        CurrentLabel                   matlab.ui.control.Label
        Pressure3_SetPoint             matlab.ui.control.NumericEditField
        Pressure2_SetPoint             matlab.ui.control.NumericEditField
        Pressure1_SetPoint             matlab.ui.control.NumericEditField
        SettingsTab                    matlab.ui.container.Tab
        OtherSettingsPanel             matlab.ui.container.Panel
        CalibrationdurationsEditField  matlab.ui.control.NumericEditField
        CalibrationdurationsEditFieldLabel  matlab.ui.control.Label
        CurrentPumpAnalogWriteValueEditField  matlab.ui.control.NumericEditField
        CurrentPumpAnalogWriteValue0255EditFieldLabel  matlab.ui.control.Label
        DeveloperOptionsPanel          matlab.ui.container.Panel
        SimulateDataCheckBox           matlab.ui.control.CheckBox
        PIDCoefficientsPanel           matlab.ui.container.Panel
        FlowCoefficientsPanel          matlab.ui.container.Panel
        KdEditFieldLabel               matlab.ui.control.Label
        KdFlowEditField                matlab.ui.control.NumericEditField
        KiFlowEditField                matlab.ui.control.NumericEditField
        KiEditFieldLabel               matlab.ui.control.Label
        KpFlowEditField                matlab.ui.control.NumericEditField
        KpEditFieldLabel               matlab.ui.control.Label
        FlowControlLabel               matlab.ui.control.Label
        PressureCoefficientsPanel      matlab.ui.container.Panel
        KdEditField_2Label             matlab.ui.control.Label
        KdPressureEditField            matlab.ui.control.NumericEditField
        KiPressureEditField            matlab.ui.control.NumericEditField
        KiEditField_2Label             matlab.ui.control.Label
        KpPressureEditField            matlab.ui.control.NumericEditField
        KpEditField_2Label             matlab.ui.control.Label
        PressureControlLabel           matlab.ui.control.Label
        HelpTab                        matlab.ui.container.Tab
        HTML                           matlab.ui.control.HTML
    end

    % TODO: Add "refresh ports" button

    properties (Access = private)
        plot_raw_data = true;
        F_REFRESH = 20;  % App refresh rate (Hz)
        ARDUINO_ANALOGWRITE_MAX = 255;
        ARDUINO_SERIAL_BITRATE = 115200;
        PID_ERRORS_LENGTH = 100;  % # of error values to use for PID integral calculations
        CALIBRATION_DURATION = 30;

        % Colors
        PRESSURE_COLOR = [0.84 0.83 0.86];
        FLOW_COLOR = [0.77,0.83,0.76];
        BUTTON_COLOR_DEFAULT = [0.94,0.94,0.94];
        BUTTON_COLOR1 = [0.16,0.57,0.84];
        DISABLED_COLOR = [0.5 0.5 0.5];
        COLOR_DISCONNECTED = [1.00,0.00,0.00];
        COLOR_CONNECTED = [0, 1, 0];

        appDir;

        arduinoObj;
        isConnected;
        tempLogFile = fullfile(pwd, 'console_output_temp.txt');  % Define a persistent temporary log file that always stores session history

        t = [];
        setpoint_vals = [];
        setpoint_ids = [];

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

        lastFlowDutyCycle = 128;  % for comparisons
        flowDutyCycle = 128;  % Initialize halfway
        PumpControlMode = 'Manual';


        pulsatileFlowEnabled = false;
        pulsatileFlowValues = [0, 6.8, 4.7, 3.5, 4.7, 5.8, 2.5, 0].*0.7647;  % from Johnson et. al (https://pmc.ncbi.nlm.nih.gov/articles/PMC8757711/#s9), scaled to max ~5 L/min for current pump
        pulsatileFlowTimes = [0, 0.15, 0.25, 0.4, 0.42, 0.55, 0.7, 1];
        pulsatileFlowPeriod;

        SHO_Pressure1 = 0;
        SHO_Pressure2 = 0;
        SHO_Pressure3 = 0;

        PID_setpoint_id = "None";  % Identifier as to which sensor's setpoint is influencing PID control, since only one sensor at a time can do so
        PID_setpoint = "None";
        PID_output = 0;
        PID_errors = [];
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

    methods (Static)
        function dimEditFieldTemporarily(editField)
            editField.BackgroundColor = [0.9, 0.9, 0.9];
            pause(0.5);
            editField.BackgroundColor = [1 1 1];
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
                    pressure_value1 = data(5) - app.SHO_Pressure1;  % apply Static Head Offsets
                    pressure_value_raw2 = data(6);
                    pressure_value2 = data(7) - app.SHO_Pressure2;
                    pressure_value_raw3 = data(8);
                    pressure_value3 = data(9) - app.SHO_Pressure3;
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

            app.setpoint_vals = [];

            app.flow_vals1 = [];  
            app.flow_vals2 = [];

            app.pressure_vals1 = [];
            app.pressure_vals2 = [];
            app.pressure_vals3 = [];

            app.PID_errors = zeros(1,app.PID_ERRORS_LENGTH);
            app.PID_output = 0;
            app.PID_integral = 0;
            app.PID_derivative = 0;
        end
       
        
        function runMainLoop(app, varargin)
            % Check if duration is passed (implying calibration is being
            % done)
            if ~isempty(varargin)
                duration = varargin{1};
                timedMode = true;
                tStart = tic;
                
                % Show loading bar while calibration runs
                d = uiprogressdlg(app.UIFigure, ...
                    'Title', 'Calibrating...', ...
                    'Message', 'Collecting data for calibration (0 seconds remaining...)', ...
                    'Indeterminate', 'off', ...
                    'Value', 0);
            else
                timedMode = false;
            end

            while app.isConnected
                line = app.getArduinoData();

                % Read sensor data
                [tNew, ~, flow_value1, flow_value2, ...
                    ~, pressure_value1, ~, pressure_value2, ...
                    ~, pressure_value3] = app.parseArduinoData(line);

                % Send pulsatile flow signal to Arduino if enabled
                if app.pulsatileFlowEnabled
                    flow = interp1(app.pulsatileFlowTimes, app.pulsatileFlowValues, ...
                        mod(tNew, app.pulsatileFlowPeriod), 'previous');
                end

                % Append new sensor data
                app.t = [app.t tNew];
                app.setpoint_vals = [app.setpoint_vals app.PID_setpoint];
                app.setpoint_ids = [app.setpoint_ids app.PID_setpoint_id];

                app.flow_vals1 = [app.flow_vals1 flow_value1];
                app.flow_vals2 = [app.flow_vals2 flow_value2];

                app.pressure_vals1 = [app.pressure_vals1 pressure_value1];
                app.pressure_vals2 = [app.pressure_vals2 pressure_value2];
                app.pressure_vals3 = [app.pressure_vals3 pressure_value3];

                % Update Edit Fields
                app.Pressure1_EditField_Current.Value = pressure_value1;
                app.Pressure2_EditField_Current.Value = pressure_value2;
                app.Pressure3_EditField_Current.Value = pressure_value3;
                app.Flow1_EditField_Current.Value = flow_value1;
                app.Flow2_EditField_Current.Value = flow_value2;

                app.Pressure1_EditField_Avg.Value = sprintf("%.2f ± %.1f", mean(app.pressure_vals1), std(app.pressure_vals1));
                app.Pressure2_EditField_Avg.Value = sprintf("%.2f ± %.1f", mean(app.pressure_vals2), std(app.pressure_vals2));
                app.Pressure3_EditField_Avg.Value = sprintf("%.2f ± %.1f", mean(app.pressure_vals3), std(app.pressure_vals3));

                app.Flow1_EditField_Avg.Value = sprintf("%.2f ± %.1f", mean(app.flow_vals1), std(app.flow_vals1));
                app.Flow2_EditField_Avg.Value = sprintf("%.2f ± %.1f", mean(app.flow_vals2), std(app.flow_vals2));

                app.updateDataPlots();

                % Update PID if in Auto mode
                if app.PumpControlMode == "Auto"
                    app.updatePIDOutputs();
                end

                % If duration mode, break when time is up, otherwise update
                % loading bar
                if timedMode
                    tElapsed = toc(tStart);
                    if tElapsed >= duration
                        close(d);
                        break
                    else
                        d.Value = tElapsed/duration;
                        format short;
                        d.Message = sprintf('Collecting data for calibration (%d seconds remaining...)', round(duration - tElapsed));
                    end
                end
            end
        end


        function setControlMode(app,mode)
            if lower(mode) == "manual"
                app.PumpControlMode = "Manual";
                app.ControlModeSwitch.Value = "Manual";
                app.ControlModeSwitch.Enable = "off";  % to prevent user from switching toggle when they should just input a setpoint
                highlightSetPointUI(app, "None");
                app.PumpPowerSlider.Enable = "on";
                app.PID_setpoint_id = "None";
            elseif lower(mode) == "pid"
                app.PumpControlMode = "PID";
                app.ControlModeSwitch.Value = "Auto";
                app.ControlModeSwitch.Enable = "on";
                app.PumpPowerSlider.Enable = "off";
            elseif lower(mode) == "pulsatile"
                app.PumpControlMode = "Pulsatile";
                app.ControlModeSwitch.Value = "Auto";
                app.ControlModeSwitch.Enable = "on";
                app.PumpPowerSlider.Enable = "off";
            elseif lower(mode) == "calibration"
                app.PumpControlMode = "Calibration";
                app.ControlModeSwitch.Value = "Auto";
                app.ControlModeSwitch.Enable = "off";
                app.PumpPowerSlider.Enable = "off";
            else
                warning("Invalid control mode selected.");
            end
        end
        
        function sendControlValuesToArduino(app)
            if lower(app.PumpControlMode) == "auto"
                % Update control values based on PID algorithm
                app.flowDutyCycle = app.flowDutyCycle + round(app.PID_output);

                % Place PID output within limits
                if app.flowDutyCycle > app.ARDUINO_ANALOGWRITE_MAX
                    app.flowDutyCycle = app.ARDUINO_ANALOGWRITE_MAX;
                elseif app.flowDutyCycle < 0
                    app.flowDutyCycle = 0;
                end
            elseif lower(app.PumpControlMode) == "manual" || lower(app.PumpControlMode) == "pulsatile"
                % do nothing
            else
                uialert("Invalid pump control mode selected.")
            end

            % Update flow pump control UI
            app.CurrentPumpAnalogWriteValueEditField.Value = app.flowDutyCycle;
            app.PumpPowerSlider.Value = app.flowDutyCycle / app.ARDUINO_ANALOGWRITE_MAX * app.PumpPowerSlider.Limits(2);

            % Send command to Arduino
            if app.SimulateDataCheckBox.Value == false
                command = sprintf("PMP: %d",app.flowDutyCycle);
                writeline(app.arduinoObj,command);
                app.WaitForArduinoMessage();
            end
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
                uialert('Invalid PID setpoint ID selected: %s', app.PID_setpoint_id);
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
                uialert('Unknown PID setpoint ID: %s', app.PID_setpoint_id);
            end

            % Calculate PID output
            app.PID_errors = circshift(app.PID_errors, -1);  % rolling errors array
            
            PID_error = app.PID_setpoint - newValue;
            app.PID_errors(end) = PID_error;
            PID_prev_error = app.PID_errors(end-1);
            
            app.PID_integral = app.PID_integral + PID_error * app.DT_PID;
            app.PID_derivative = (PID_error - PID_prev_error) / app.DT_PID;
            app.PID_output = Kp * PID_error + Ki * app.PID_integral + Kd * app.PID_derivative;

            % Send PID output to Arduino
            app.sendControlValuesToArduino();
        end
        
        function updateSerialPortList(app)
            % Get list of available serial ports
            ports = serialportlist;

            if isempty(ports)
                uialert(app.UIFigure, "Try refreshing the connections list or check your device connection.", "No serial ports detected")
                app.ConnectionDropDown.Enable = "off";
                app.ConnectDisconnectButton.Enable = "off";
                return
            else
                app.ConnectionDropDown.Enable = "on";  % in case it was disabled before
                app.ConnectDisconnectButton.Enable = "on";
            end

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
            app.ConnectionDropDown.Items = portInfo;
            app.ConnectionDropDown.ItemsData = ports;  % To allow proper connections when using serialport() later off of the selected dropdown value
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
                % Get handle to the UI element
                uiElement = setpoints.(fieldNames{i});

                onColor = [0 1 0];
                offColor = [1 1 1];

                if strcmp(fieldNames{i}, setpointID)
                    if isequal(uiElement.BackgroundColor, onColor)
                        uiElement.BackgroundColor = offColor;
                        pause(0.2);  % if it's already green, flash it white briefly to show that a new value is being set
                        uiElement.BackgroundColor = onColor;
                    else
                        uiElement.BackgroundColor = onColor;
                    end
                else
                    uiElement.BackgroundColor = offColor;
                end
            end

        end

        
        function setUpArduinoConnection(app)
            app.arduinoObj = serialport(app.ConnectionDropDown.Value, app.ARDUINO_SERIAL_BITRATE);
            configureTerminator(app.arduinoObj, "LF"); % Assuming newline ('\n') terminates messages
            app.arduinoObj.Timeout = 5; % Timeout for read operations 

            app.WaitForArduinoMessage();
        end
        
        function line = getArduinoData(app)
            if app.SimulateDataCheckBox.Value
                % Simulate data in GUI (for testing)
                line = "NF: N, F1: 1.50 L/min, F2: 2.50 L/min, P1R: 1023, P1: 20 mmHg, P2R: 1023, P2: 50 mmHg, P3R: 1023, P3: 80 mmHg, Pump: 128/255\n";
            else
                if app.arduinoObj.NumBytesAvailable > 0
                    flushinput(app.arduinoObj);
                    readline(app.arduinoObj);  % Read until next newline in case flush clears part of a line
                    line = readline(app.arduinoObj); % Read a line of text
                else
                    line = [];
                end
            end

            pause(0.1); % Small delay to prevent CPU overuse
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
        
        function updateUIColors(app)
            % Color all pressure-related elements
            pressureElements = [app.Panel, ...
                app.PressureGraphPanel, ...
                app.PressureCoefficientsPanel, ...
                app.PressureDataTitleLabelPanel];

            for pressureElement = pressureElements
                pressureElement.BackgroundColor = app.PRESSURE_COLOR;
            end

            % Color all flow-related elements
            flowElements = [app.Panel_2, ...
                app.FlowGraphPanel, ...
                app.FlowCoefficientsPanel, ...
                app.FlowDataTitleLabelPanel];

            for flowElement = flowElements
                flowElement.BackgroundColor = app.FLOW_COLOR;
            end

            % Other things
            app.ConnectDisconnectButton.BackgroundColor = app.COLOR_DISCONNECTED;

            otherOptionsElements = [app.ClearDataButton, app.ExportDataButton, app.PauseGraphsButton];

            for element = otherOptionsElements
                element.BackgroundColor = app.BUTTON_COLOR1;
            end

            app.PressureAxes.Title.Color = app.DISABLED_COLOR;
            app.FlowAxes.Title.Color = app.DISABLED_COLOR;
            
        end
        
        function connectGUI(app)
            % Someone just connected to the serial device

            % Loading bar
            dlg = uiprogressdlg(app.UIFigure, ...
                'Title', 'Loading...', ...
                'Message', 'Please wait...', ...
                'Indeterminate', 'on');

            % Update UI
            app.ConnectDisconnectButton.Text = "Disconnect";
            app.ConnectDisconnectButton.BackgroundColor = app.BUTTON_COLOR_DEFAULT;
            app.ConnectedLamp.Color = app.COLOR_CONNECTED;
            app.setGUIEnabled(true);

            if app.SimulateDataCheckBox.Value == false
                app.setUpArduinoConnection();
            end

            app.ConnectionDropDown.Enable = "off";
            app.RefreshConnectionsButton.Enable = "off";

            % Perform other housekeeping
            app.clearDataArrays();
            app.startLogging();
            app.isConnected = true;

            close(dlg);
            tic;

            app.runMainLoop();
            
        end
        
        function disconnectGUI(app)
            % Loading bar
            dlg = uiprogressdlg(app.UIFigure, ...
                'Title', 'Disconnecting...', ...
                'Message', 'Please wait...', ...
                'Indeterminate', 'on');

            app.arduinoObj = [];
            app.ConnectionDropDown.Enable = "on";
            app.RefreshConnectionsButton.Enable = "on";
            app.ConnectDisconnectButton.Text = "Connect";
            app.ConnectDisconnectButton.BackgroundColor = app.COLOR_DISCONNECTED;
            app.ConnectedLamp.Color = app.COLOR_DISCONNECTED;
            app.isConnected = false;
            app.setGUIEnabled(false);

            close(dlg);
        end

        function setGUIEnabled(app, state)
            % Gather all UI elements to enable/disable
            uiElements = [
                app.Panel;
                app.Panel_2;
                app.PumpControlPanel;
                app.PumpPowerSlider;
                app.PIDCoefficientsPanel;
                app.OptionsLabel;
                app.PressureDataTitleLabelPanel;
                app.FlowDataTitleLabelPanel;
                app.OptionsPanel.Children(:);
                ];

            % Convert logical state to 'on'/'off' string
            stateStr = ternary(state, 'on', 'off');

            % Enable/Disable each UI element
            for uiElement = uiElements'
                if isprop(uiElement, 'Enable')
                    uiElement.Enable = stateStr;
                end
            end

            % Graph formatting based on state
            if state
                graphColor = [1 1 1];
                textColor = [0.15, 0.15, 0.15];  % Default text color
                toolbarVisible = true;
                waitingText = 'Loading...';
            else
                graphColor = [0.90, 0.90, 0.90];
                textColor = [0.50, 0.50, 0.50];  % Disabled text color
                toolbarVisible = false;
                waitingText = 'No connection';
            end

            % Clear graphs if disabling UI
            if ~state
                delete(findall(app.PressureAxes, 'Type', 'Line'));   % Remove lines
                legend(app.PressureAxes, 'off');                    % Remove the legend box

                delete(findall(app.FlowAxes, 'Type', 'Line'));
                legend(app.FlowAxes, 'off'); 
            end

            % Apply formatting to graphs
            for axName = ["PressureAxes", "FlowAxes"]
                ax = app.(axName);
                ax.Color = graphColor;
                ax.Title.Color = textColor;
                ax.YColor = textColor;
                ax.XColor = textColor;
                ax.Toolbar.Visible = toolbarVisible;

                % Dynamically access the corresponding WaitingForConnectionLabel
                labelName = axName + "WaitingForConnectionLabel";
                if isprop(app, labelName)
                    app.(labelName).Visible = "on";
                    app.(labelName).Text = waitingText;
                end

                if state
                    enableDefaultInteractivity(ax);
                else
                    disableDefaultInteractivity(ax);
                end
            end


            pause(2);  % for smoother loading when enabling/disabling
        end

        function setGUIEnabledCalibration(app, state)
            % Convert logical state to 'on'/'off' string
            stateStr = ternary(state, 'on', 'off'); 

            % Disable UI to prevent interference with calibration
            uiElements = [app.Panel, ...
                app.Panel_2, ...
                app.OptionsPanel.Children(:)', ...
                app.PIDCoefficientsPanel, ...
                app.ConnectDisconnectButton];

            for uiElement = uiElements
                if isprop(uiElement, 'Enable')
                    uiElement.Enable = stateStr;
                end
            end
        end
        
        function updateDataPlots(app)
            % Plot data based on UI selections
            if app.PauseGraphsButton.Value == false && app.isConnected
                hold(app.PressureAxes, 'on');
                cla(app.PressureAxes); % Clear previous plots

                if strcmp(app.PressureAxesWaitingForConnectionLabel.Visible,'on')
                    % Hide "Waiting for Connection" labels on graphs if
                    % they're still visible (i.e. first time plotting)
                    app.PressureAxesWaitingForConnectionLabel.Visible = 'off';
                    app.FlowAxesWaitingForConnectionLabel.Visible = 'off';
                end

                lineWidth = 2;

                % Plot pressure data
                if app.Pressure1_GraphEnable.Value
                    plot(app.PressureAxes, app.t, app.pressure_vals1, 'Color', app.PRESSURE_COLOR, 'DisplayName', app.pressure1_label, 'LineWidth', lineWidth);
                end
                if app.Pressure2_GraphEnable.Value
                    plot(app.PressureAxes, app.t, app.pressure_vals2, 'Color', app.PRESSURE_COLOR*0.5, 'DisplayName', app.pressure2_label, 'LineWidth', lineWidth);
                end
                if app.Pressure3_GraphEnable.Value
                    plot(app.PressureAxes, app.t, app.pressure_vals3, 'Color', [0 0 0], 'DisplayName', app.pressure3_label, 'LineWidth', lineWidth);
                end

                if contains(app.PID_setpoint_id,"Pressure")
                    plot(app.PressureAxes, app.t, ones(1,length(app.t))*app.PID_setpoint, '--r', 'DisplayName', 'Target', 'LineWidth', lineWidth)
                end

                hold(app.PressureAxes, 'off');
                xlabel(app.PressureAxes, 'Time (s)');
                ylabel(app.PressureAxes, 'Pressure (mmHg)');
                lgdPressure = legend(app.PressureAxes, 'show','Location','northwest');
                lgdPressure.FontSize = 8;

                % Determine appropriate y-limit
                if contains(app.PID_setpoint_id,"Pressure")
                    pressureMax = max([app.pressure_vals1, app.pressure_vals2, app.pressure_vals3, app.PID_setpoint]);  % if setpoint is for pressure adjust y-limit appropriately
                else
                    pressureMax = max([app.pressure_vals1, app.pressure_vals2, app.pressure_vals3]);
                end
                
                pressureMin = 0;
                pressureDefaultMax = 100;
                pressureMax = max(pressureDefaultMax, pressureMax * 1.1);  % Add 10% headroom
                ylim(app.PressureAxes, [pressureMin pressureMax]);

                % Plot flow data
                hold(app.FlowAxes, 'on');
                cla(app.FlowAxes); % Clear previous plots

                if app.Flow1_GraphEnable.Value
                    plot(app.FlowAxes, app.t, app.flow_vals1, 'Color', app.FLOW_COLOR, 'DisplayName', app.flow1_label, 'LineWidth', lineWidth);
                end
                if app.Flow2_GraphEnable.Value
                    plot(app.FlowAxes, app.t, app.flow_vals2, 'Color', [0 0 0], 'DisplayName', app.flow2_label, 'LineWidth', lineWidth);
                end

                if contains(app.PID_setpoint_id,"Flow")
                    plot(app.FlowAxes, app.t, ones(1,length(app.t))*app.PID_setpoint, '--r', 'DisplayName', 'Target', 'LineWidth', lineWidth)
                end

                hold(app.FlowAxes, 'off');
                xlabel(app.FlowAxes, 'Time (s)');
                ylabel(app.FlowAxes, 'Flow (L/min)');
                ylim(app.FlowAxes, [0 5]);
                lgdFlow = legend(app.FlowAxes, 'show','Location','northwest');
                lgdFlow.FontSize = 8;

                % Determine appropriate y-limit
                
                if contains(app.PID_setpoint_id, "Flow")
                    flowMax = max([app.flow_vals1, app.flow_vals2, app.PID_setpoint]);  % if setpoint is for flow adjust y-limit appropriately
                else
                    flowMax = max([app.flow_vals1, app.flow_vals2]);
                end
                flowMin = 0;
                flowDefaultMax = 5;
                flowMax = max(flowDefaultMax, flowMax * 1.1);  % Add 10% headroom
                ylim(app.FlowAxes, [flowMin flowMax]);
            end

            pause(0.1);  % small pause to prevent CPU overuse
        end
        
        function continueSHOCalibration(app)

            % Run main loop for some time to collect values
            app.clearDataArrays();
            app.runMainLoop(app.CALIBRATION_DURATION);

            % Step 4: Calculate average pressures
            avg1 = mean(app.pressure_vals1);
            avg2 = mean(app.pressure_vals2);
            avg3 = mean(app.pressure_vals3);

            % Step 5: Store averages as static head offset calibration values
            app.SHO_Pressure1 = avg1;
            app.Pressure1_EditField_SHO.Value = avg1;
            app.SHO_Pressure2 = avg2;
            app.Pressure2_EditField_SHO.Value = avg2;
            app.SHO_Pressure3 = avg3;
            app.Pressure3_EditField_SHO.Value = avg3;

            app.clearDataArrays();  % to avoid plots from randomly jumping down

            % Step 6: Show confirmation dialog with the results (nicer formatting)
            app.showCalibrationResults(avg1, avg2, avg3);
        end

        function showCalibrationResults(app, avg1, avg2, avg3)
            % Create custom dialog figure
            fig = uifigure('Name', 'Calibration Complete', 'Position', [500 500 450 300]);

            htmlContent = sprintf([...
                '<div style="font-family: Helvetica, Arial, sans-serif;">' ...
                '<h2 style="color: green;">&#9989; Calibration Complete</h2>' ...
                '<p>The following <b>Static Head Offset</b> values were set:</p>' ...
                '<ul>' ...
                '<li><b>Pressure 1:</b> %.2f mmHg</li>' ...
                '<li><b>Pressure 2:</b> %.2f mmHg</li>' ...
                '<li><b>Pressure 3:</b> %.2f mmHg</li>' ...
                '</ul>' ...
                '</div>'], avg1, avg2, avg3);

            % Add uihtml component for full HTML rendering
            uihtml(fig, ...
                'HTMLSource', htmlContent, ...
                'Position', [80 50 350 220]);

            % Optional: Add close button
            uibutton(fig, 'Text', 'Close', 'Position', [170 10 100 30], 'ButtonPushedFcn', @(btn, event) close(fig));

            % Reset GUI back to normal use
            app.setControlMode("Manual");
            app.setGUIEnabledCalibration(true);
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.appDir = fileparts(mfilename('fullpath')); % Get app directory
            app.startLogging();
            app.updateSerialPortList();
            addpath(fullfile(app.appDir, 'helpers'));  % add helper functions to path
            app.PID_errors = zeros(1, app.PID_ERRORS_LENGTH);
            app.pulsatileFlowPeriod = app.pulsatileFlowTimes(end);

            % Update UI elements
            app.UIFigure.Name = 'Cardiac Catheterization Testing Apparatus (CCTA)';
            disableDefaultInteractivity(app.PressureAxes);
            disableDefaultInteractivity(app.FlowAxes);
            app.updateUIColors();
            app.updateUIPIDCoeffs();
            app.PressureAxes.Visible = "on";  % bug sometimes makes it invisible
            app.FlowAxes.Visible = "on";
        end

        % Button pushed function: ConnectDisconnectButton
        function ConnectDisconnectButtonPushed(app, event)
            
            if app.ConnectDisconnectButton.Text == "Connect"
                app.connectGUI();
            elseif app.ConnectDisconnectButton.Text == "Disconnect"
                app.disconnectGUI();
            else
                uialert("Connect button container contains invalid text.")
            end
        end

        % Button pushed function: ClearDataButton
        function ClearDataButtonPushed(app, event)
            % Empty time and sensor data arrays
            app.clearDataArrays();
            app.runMainLoop();  % Re-run it here from the beginning to reset time array properly
        end

        % Button pushed function: ExportDataButton
        function ExportDataButtonPushed(app, event)
            dlg = uiprogressdlg(app.UIFigure, ...
                'Title', 'Exporting data...', ...
                'Message', 'Please wait...', ...
                'Indeterminate', 'on');

            % Get current timestamp
            timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd-HH-mm-ss'));

            % Create "Data" subfolder if it doesn't exist
            dataFolder = fullfile(app.appDir, 'Data');
            if ~exist(dataFolder, 'dir')
                mkdir(dataFolder);
            end

            % Create the timestamped results folder inside "Data"
            folderName = fullfile(dataFolder, ['CCTA-', timestamp]);
            if ~exist(folderName, 'dir')
                mkdir(folderName);
            end
            
            % Save Pressure and Flow Axes as .fig and .png
            exportgraphics(app.PressureAxes, fullfile(folderName, 'PressurePlot.png'));
            exportgraphics(app.FlowAxes, fullfile(folderName, 'FlowPlot.png'));
            
            % Save data arrays to .mat/.csv files
            data.t = app.t;

            data.setpoint_vals = app.setpoint_vals;
            data.setpoint_ids = app.setpoint_ids;

            data.flow_vals1 = app.flow_vals1;
            data.flow_vals2 = app.flow_vals2;

            data.pressure_vals1 = app.pressure_vals1;
            data.pressure_vals2 = app.pressure_vals2;
            data.pressure_vals3 = app.pressure_vals3;
            
            save(fullfile(folderName, 'ExportedData.mat'), '-struct', 'data');

            csvData = table( ...
                app.t(:), ...
                app.setpoint_vals(:), ...
                app.setpoint_ids(:), ...
                app.flow_vals1(:), ...
                app.flow_vals2(:), ...
                app.pressure_vals1(:), ...
                app.pressure_vals2(:), ...
                app.pressure_vals3(:), ...
                'VariableNames', { ...
                'Time (s)', ...
                'Setpoint Value', ...
                'Setpoint ID', ...
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
            selection = uiconfirm(app.UIFigure, ...
                ['Data has been saved to ', folderName], ...
                'Export Successful', ...
                'Options', {'Open Folder', 'OK'}, ...
                'Icon', 'info');

            if strcmp(selection, 'Open Folder')
                % Open the folder in file explorer
                if ispc
                    winopen(folderName);
                elseif ismac
                    system(['open ', folderName]);
                else
                    system(['xdg-open ', folderName]);
                end
            end

            close(dlg);
        end

        % Button pushed function: RefreshConnectionsButton
        function RefreshConnectionsButtonPushed(app, event)
            dlg = uiprogressdlg(app.UIFigure, ...
                'Title', 'Refreshing serial connections...', ...
                'Message', 'Please wait...', ...
                'Indeterminate', 'on');

            app.updateSerialPortList();

            close(dlg);
        end

        % Value changed function: CurrentPumpAnalogWriteValueEditField
        function CurrentPumpAnalogWriteValueEditFieldValueChanged(app, event)
            value = app.CurrentPumpAnalogWriteValueEditField.Value;
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

        % Value changed function: Flow1_SetPoint
        function Flow1_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Flow1_SetPoint.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Flow1");
            app.PID_setpoint_id = "Flow1";
        end

        % Value changed function: Flow2_SetPoint
        function Flow2_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Flow2_SetPoint.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Flow2");
            app.PID_setpoint_id = "Flow2";
        end

        % Value changed function: Pressure1_SetPoint
        function Pressure1_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure1_SetPoint.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Pressure1");
            app.PID_setpoint_id = "Pressure1";
        end

        % Value changed function: Pressure2_SetPoint
        function Pressure2_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure2_SetPoint.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Pressure2");
            app.PID_setpoint_id = "Pressure2";
        end

        % Value changed function: Pressure3_SetPoint
        function Pressure3_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure3_SetPoint.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Pressure3");
            app.PID_setpoint_id = "Pressure3";
        end

        % Value changed function: KpFlowEditField
        function KpFlowEditFieldValueChanged(app, event)
            app.Kp_flow = app.KpFlowEditField.Value;
            CCTA.dimEditFieldTemporarily(app.KpFlowEditField);  % Change colors temporarily to give user feedback
        end

        % Value changed function: KiFlowEditField
        function KiFlowEditFieldValueChanged(app, event)
            app.Ki_flow = app.KiFlowEditField.Value;
            CCTA.dimEditFieldTemporarily(app.KiFlowEditField);  % Change colors temporarily to give user feedback
        end

        % Value changed function: KdFlowEditField
        function KdFlowEditFieldValueChanged(app, event)
            app.Kd_flow = app.KdFlowEditField.Value;
            CCTA.dimEditFieldTemporarily(app.KdFlowEditField);  % Change colors temporarily to give user feedback
        end

        % Value changed function: KpPressureEditField
        function KpPressureEditFieldValueChanged(app, event)
            app.Kp_pressure = app.KpPressureEditField.Value;
            CCTA.dimEditFieldTemporarily(app.KpPressureEditField);  % Change colors temporarily to give user feedback
        end

        % Value changed function: KiPressureEditField
        function KiPressureEditFieldValueChanged(app, event)
            app.Ki_pressure = app.KiPressureEditField.Value;
            CCTA.dimEditFieldTemporarily(app.KiPressureEditField);  % Change colors temporarily to give user feedback
        end

        % Value changed function: KdPressureEditField
        function KdPressureEditFieldValueChanged(app, event)
            app.Kd_pressure = app.KdPressureEditField.Value;
            CCTA.dimEditFieldTemporarily(app.KdPressureEditField);  % Change colors temporarily to give user feedback
        end

        % Value changed function: PumpPowerSlider
        function PumpPowerSliderValueChanged(app, event)
            app.flowDutyCycle = round(app.PumpPowerSlider.Value/app.PumpPowerSlider.Limits(2)*app.ARDUINO_ANALOGWRITE_MAX);
            app.CurrentPumpAnalogWriteValueEditField.Value = app.flowDutyCycle;
            app.setControlMode("Manual");

            if ~isempty(app.arduinoObj)
                flushoutput(app.arduinoObj);  % in case this was done in the middle of sending another command
            end

            app.sendControlValuesToArduino();
        end

        % Value changed function: ControlModeSwitch
        function ControlModeSwitchValueChanged(app, event)
            value = app.ControlModeSwitch.Value;
            app.setControlMode(value);

            if value == "Manual"
                app.highlightSetPointUI("None");
                app.PID_setpoint = "None";
            end
        end

        % Value changed function: SimulateDataCheckBox
        function SimulateDataCheckBoxValueChanged(app, event)
            value = app.SimulateDataCheckBox.Value;

            if value
                app.ConnectDisconnectButton.Enable = "on";
            else
                app.ConnectDisconnectButton.Enable = "off";
            end
        end

        % Value changed function: StartPulsatileFlowButton
        function StartPulsatileFlowButtonValueChanged(app, event)
            app.pulsatileFlowEnabled = app.StartPulsatileFlowButton.Value;
            app.setControlMode("Pulsatile");
        end

        % Value changed function: Pressure1_EditField_SHO
        function Pressure1_EditField_SHOValueChanged(app, event)
            app.SHO_Pressure1 = app.Pressure1_EditField_SHO.Value;
            app.dimEditFieldTemporarily(app.Pressure1_EditField_SHO);
        end

        % Value changed function: Pressure2_EditField_SHO
        function Pressure2_EditField_SHOValueChanged2(app, event)
            app.SHO_Pressure2 = app.Pressure2_EditField_SHO.Value;
            app.dimEditFieldTemporarily(app.Pressure2_EditField_SHO);
        end

        % Value changed function: Pressure3_EditField_SHO
        function Pressure3_EditField_SHOValueChanged2(app, event)
            app.SHO_Pressure3 = app.Pressure3_EditField_SHO.Value;
            app.dimEditFieldTemporarily(app.Pressure3_EditField_SHO);
        end

        % Button pushed function: CalibrateButton
        function CalibrateButtonPushed(app, event)
            % Optional: make the dialog modal if you want, but it'll still be non-blocking
            d.WindowStyle = 'normal';

            % Set pump power
            app.setControlMode("Calibration");
            calibrationPower = 0.1;
            app.flowDutyCycle = round(calibrationPower * app.ARDUINO_ANALOGWRITE_MAX);
            app.CurrentPumpAnalogWriteValueEditField.Value = app.flowDutyCycle;
            app.PumpPowerSlider.Value = app.flowDutyCycle / app.ARDUINO_ANALOGWRITE_MAX * app.PumpPowerSlider.Limits(2);

            % Disable UI to prevent interference with calibration
            app.setGUIEnabledCalibration(false);

            % Create non-blocking custom dialog
            %
            % Must create after setting other settings above to prevent rest of
            % code from running in the wrong order
            d = uifigure('Name', 'SHO Calibration', 'Position', [500 500 400 200]);

            uilabel(d, ...
                'Text', 'Setting pump to 10% power. Please debubble the system. Once the system has been debubbled, press "Continue" to complete calibration.', ...
                'Position', [20 80 360 80], ...
                'FontSize', 12, ...
                'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', ...
                'WordWrap', 'on');

            uibutton(d, 'Text', 'Continue', ...
                'Position', [90 20 100 30], ...
                'FontSize', 12, ...
                'ButtonPushedFcn', @(btn, event) onDialogResponse(app, d, 'Continue'));

            uibutton(d, 'Text', 'Cancel', ...
                'Position', [210 20 100 30], ...
                'FontSize', 12, ...
                'ButtonPushedFcn', @(btn, event) onDialogResponse(app, d, 'Cancel'));

            % Nested function handles the response
            function onDialogResponse(app, dialogFig, choice)
                delete(dialogFig);  % Close dialog
                if strcmp(choice, 'Continue')
                    app.continueSHOCalibration();
                else
                    disp('Calibration cancelled.');
                    app.setGUIEnabledCalibration(true);
                end
            end
        end

        % Value changed function: CalibrationdurationsEditField
        function CalibrationdurationsEditFieldValueChanged(app, event)
            app.CALIBRATION_DURATION = app.CalibrationdurationsEditField.Value;
            app.dimEditFieldTemporarily(app.CalibrationdurationsEditField);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Get the file path for locating images
            pathToMLAPP = fileparts(mfilename('fullpath'));

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1240 545];
            app.UIFigure.Name = 'MATLAB App';

            % Create TabGroup2
            app.TabGroup2 = uitabgroup(app.UIFigure);
            app.TabGroup2.Position = [1 0 1240 546];

            % Create MainTab
            app.MainTab = uitab(app.TabGroup2);
            app.MainTab.Title = 'Main';

            % Create Panel
            app.Panel = uipanel(app.MainTab);
            app.Panel.Enable = 'off';
            app.Panel.BackgroundColor = [0.8392 0.8275 0.8588];
            app.Panel.FontWeight = 'bold';
            app.Panel.Position = [329 12 435 132];

            % Create Pressure1_SetPoint
            app.Pressure1_SetPoint = uieditfield(app.Panel, 'numeric');
            app.Pressure1_SetPoint.Limits = [0 Inf];
            app.Pressure1_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Pressure1_SetPointValueChanged, true);
            app.Pressure1_SetPoint.HorizontalAlignment = 'center';
            app.Pressure1_SetPoint.Tooltip = {'Target value for PID control'};
            app.Pressure1_SetPoint.Position = [121 77 44 22];

            % Create Pressure2_SetPoint
            app.Pressure2_SetPoint = uieditfield(app.Panel, 'numeric');
            app.Pressure2_SetPoint.Limits = [0 Inf];
            app.Pressure2_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Pressure2_SetPointValueChanged, true);
            app.Pressure2_SetPoint.HorizontalAlignment = 'center';
            app.Pressure2_SetPoint.Tooltip = {'Target value for PID control'};
            app.Pressure2_SetPoint.Position = [121 47 44 22];

            % Create Pressure3_SetPoint
            app.Pressure3_SetPoint = uieditfield(app.Panel, 'numeric');
            app.Pressure3_SetPoint.Limits = [0 Inf];
            app.Pressure3_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Pressure3_SetPointValueChanged, true);
            app.Pressure3_SetPoint.HorizontalAlignment = 'center';
            app.Pressure3_SetPoint.Tooltip = {'Target value for PID control'};
            app.Pressure3_SetPoint.Position = [121 15 44 22];

            % Create CurrentLabel
            app.CurrentLabel = uilabel(app.Panel);
            app.CurrentLabel.Position = [70 99 42 22];
            app.CurrentLabel.Text = 'Current';

            % Create TargetLabel
            app.TargetLabel = uilabel(app.Panel);
            app.TargetLabel.HorizontalAlignment = 'center';
            app.TargetLabel.Position = [121 99 44 22];
            app.TargetLabel.Text = 'Target';

            % Create Label_2
            app.Label_2 = uilabel(app.Panel);
            app.Label_2.HorizontalAlignment = 'center';
            app.Label_2.FontWeight = 'bold';
            app.Label_2.Position = [26 78 25 22];
            app.Label_2.Text = '1';

            % Create Pressure1_EditField_Current
            app.Pressure1_EditField_Current = uieditfield(app.Panel, 'numeric');
            app.Pressure1_EditField_Current.Editable = 'off';
            app.Pressure1_EditField_Current.HorizontalAlignment = 'center';
            app.Pressure1_EditField_Current.Tooltip = {'Current pressure reading (mmHg) for pressure sensor 1 (see label on sensor).'};
            app.Pressure1_EditField_Current.Position = [69 77 44 22];

            % Create Label_3
            app.Label_3 = uilabel(app.Panel);
            app.Label_3.HorizontalAlignment = 'center';
            app.Label_3.FontWeight = 'bold';
            app.Label_3.Position = [26 47 25 22];
            app.Label_3.Text = '2';

            % Create Pressure2_EditField_Current
            app.Pressure2_EditField_Current = uieditfield(app.Panel, 'numeric');
            app.Pressure2_EditField_Current.Editable = 'off';
            app.Pressure2_EditField_Current.HorizontalAlignment = 'center';
            app.Pressure2_EditField_Current.Tooltip = {'Current pressure reading (mmHg) for pressure sensor 2 (see label on sensor).'};
            app.Pressure2_EditField_Current.Position = [70 48 43 22];

            % Create Label_4
            app.Label_4 = uilabel(app.Panel);
            app.Label_4.HorizontalAlignment = 'center';
            app.Label_4.FontWeight = 'bold';
            app.Label_4.Position = [26 16 25 22];
            app.Label_4.Text = '3';

            % Create Pressure3_EditField_Current
            app.Pressure3_EditField_Current = uieditfield(app.Panel, 'numeric');
            app.Pressure3_EditField_Current.Editable = 'off';
            app.Pressure3_EditField_Current.HorizontalAlignment = 'center';
            app.Pressure3_EditField_Current.Tooltip = {'Current pressure reading (mmHg) for pressure sensor 3 (see label on sensor).'};
            app.Pressure3_EditField_Current.Position = [70 16 43 22];

            % Create SensorLabel
            app.SensorLabel = uilabel(app.Panel);
            app.SensorLabel.HorizontalAlignment = 'center';
            app.SensorLabel.FontWeight = 'bold';
            app.SensorLabel.Position = [16 99 45 22];
            app.SensorLabel.Text = 'Sensor';

            % Create Pressure3_GraphEnable
            app.Pressure3_GraphEnable = uicheckbox(app.Panel);
            app.Pressure3_GraphEnable.Text = '';
            app.Pressure3_GraphEnable.Position = [398 14 14 22];
            app.Pressure3_GraphEnable.Value = true;

            % Create Pressure2_GraphEnable
            app.Pressure2_GraphEnable = uicheckbox(app.Panel);
            app.Pressure2_GraphEnable.Text = '';
            app.Pressure2_GraphEnable.Position = [398 45 16 22];
            app.Pressure2_GraphEnable.Value = true;

            % Create Pressure1_GraphEnable
            app.Pressure1_GraphEnable = uicheckbox(app.Panel);
            app.Pressure1_GraphEnable.Text = '';
            app.Pressure1_GraphEnable.Position = [398 77 16 22];
            app.Pressure1_GraphEnable.Value = true;

            % Create Label_5
            app.Label_5 = uilabel(app.Panel);
            app.Label_5.HorizontalAlignment = 'center';
            app.Label_5.Position = [268 99 63 22];

            % Create Pressure1_LabelField
            app.Pressure1_LabelField = uieditfield(app.Panel, 'text');
            app.Pressure1_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure1_LabelFieldValueChanged, true);
            app.Pressure1_LabelField.HorizontalAlignment = 'center';
            app.Pressure1_LabelField.FontAngle = 'italic';
            app.Pressure1_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure1_LabelField.Position = [268 77 63 22];
            app.Pressure1_LabelField.Value = '1';

            % Create Pressure2_LabelField
            app.Pressure2_LabelField = uieditfield(app.Panel, 'text');
            app.Pressure2_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure2_LabelFieldValueChanged, true);
            app.Pressure2_LabelField.HorizontalAlignment = 'center';
            app.Pressure2_LabelField.FontAngle = 'italic';
            app.Pressure2_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure2_LabelField.Position = [269 46 62 22];
            app.Pressure2_LabelField.Value = '2';

            % Create Pressure3_LabelField
            app.Pressure3_LabelField = uieditfield(app.Panel, 'text');
            app.Pressure3_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure3_LabelFieldValueChanged, true);
            app.Pressure3_LabelField.HorizontalAlignment = 'center';
            app.Pressure3_LabelField.FontAngle = 'italic';
            app.Pressure3_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure3_LabelField.Position = [269 15 62 22];
            app.Pressure3_LabelField.Value = '3';

            % Create Label_11
            app.Label_11 = uilabel(app.Panel);
            app.Label_11.HorizontalAlignment = 'center';
            app.Label_11.FontSize = 18;
            app.Label_11.Position = [391 99 30 23];
            app.Label_11.Text = '👁️';

            % Create SHOLabel_Pressure
            app.SHOLabel_Pressure = uilabel(app.Panel);
            app.SHOLabel_Pressure.HorizontalAlignment = 'center';
            app.SHOLabel_Pressure.Tooltip = {'SHO - Static Head Offset. Adjusts for pressure changes caused by fluid height differences in the system.'};
            app.SHOLabel_Pressure.Position = [340 99 43 22];
            app.SHOLabel_Pressure.Text = 'SHO';

            % Create Pressure1_EditField_SHO
            app.Pressure1_EditField_SHO = uieditfield(app.Panel, 'numeric');
            app.Pressure1_EditField_SHO.ValueChangedFcn = createCallbackFcn(app, @Pressure1_EditField_SHOValueChanged, true);
            app.Pressure1_EditField_SHO.HorizontalAlignment = 'center';
            app.Pressure1_EditField_SHO.Tooltip = {'SHO - Static Head Offset. Adjusts for pressure changes caused by fluid height differences in the system.'};
            app.Pressure1_EditField_SHO.Position = [340 77 43 22];

            % Create Pressure2_EditField_SHO
            app.Pressure2_EditField_SHO = uieditfield(app.Panel, 'numeric');
            app.Pressure2_EditField_SHO.ValueChangedFcn = createCallbackFcn(app, @Pressure2_EditField_SHOValueChanged2, true);
            app.Pressure2_EditField_SHO.HorizontalAlignment = 'center';
            app.Pressure2_EditField_SHO.Tooltip = {'SHO - Static Head Offset. Adjusts for pressure changes caused by fluid height differences in the system.'};
            app.Pressure2_EditField_SHO.Position = [340 48 43 22];

            % Create Pressure3_EditField_SHO
            app.Pressure3_EditField_SHO = uieditfield(app.Panel, 'numeric');
            app.Pressure3_EditField_SHO.ValueChangedFcn = createCallbackFcn(app, @Pressure3_EditField_SHOValueChanged2, true);
            app.Pressure3_EditField_SHO.HorizontalAlignment = 'center';
            app.Pressure3_EditField_SHO.Tooltip = {'SHO - Static Head Offset. Adjusts for pressure changes caused by fluid height differences in the system.'};
            app.Pressure3_EditField_SHO.Position = [340 16 43 22];

            % Create AvgSDLabel
            app.AvgSDLabel = uilabel(app.Panel);
            app.AvgSDLabel.HorizontalAlignment = 'center';
            app.AvgSDLabel.Position = [174 99 85 22];
            app.AvgSDLabel.Text = 'Avg ± SD';

            % Create Pressure1_EditField_Avg
            app.Pressure1_EditField_Avg = uieditfield(app.Panel, 'text');
            app.Pressure1_EditField_Avg.HorizontalAlignment = 'center';
            app.Pressure1_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure1_EditField_Avg.Position = [174 77 85 22];

            % Create Pressure2_EditField_Avg
            app.Pressure2_EditField_Avg = uieditfield(app.Panel, 'text');
            app.Pressure2_EditField_Avg.HorizontalAlignment = 'center';
            app.Pressure2_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure2_EditField_Avg.Position = [174 45 85 22];

            % Create Pressure3_EditField_Avg
            app.Pressure3_EditField_Avg = uieditfield(app.Panel, 'text');
            app.Pressure3_EditField_Avg.HorizontalAlignment = 'center';
            app.Pressure3_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure3_EditField_Avg.Position = [174 15 85 22];

            % Create Panel_2
            app.Panel_2 = uipanel(app.MainTab);
            app.Panel_2.Enable = 'off';
            app.Panel_2.BackgroundColor = [0.7686 0.8314 0.7608];
            app.Panel_2.FontWeight = 'bold';
            app.Panel_2.Position = [775 47 448 97];

            % Create Label_6
            app.Label_6 = uilabel(app.Panel_2);
            app.Label_6.FontSize = 18;
            app.Label_6.FontWeight = 'bold';
            app.Label_6.Position = [110 45 25 23];
            app.Label_6.Text = '';

            % Create Flow1_SetPoint
            app.Flow1_SetPoint = uieditfield(app.Panel_2, 'numeric');
            app.Flow1_SetPoint.Limits = [0 Inf];
            app.Flow1_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Flow1_SetPointValueChanged, true);
            app.Flow1_SetPoint.HorizontalAlignment = 'center';
            app.Flow1_SetPoint.Tooltip = {'Target value for PID control'};
            app.Flow1_SetPoint.Position = [161 44 44 22];

            % Create Flow2_SetPoint
            app.Flow2_SetPoint = uieditfield(app.Panel_2, 'numeric');
            app.Flow2_SetPoint.Limits = [0 Inf];
            app.Flow2_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Flow2_SetPointValueChanged, true);
            app.Flow2_SetPoint.HorizontalAlignment = 'center';
            app.Flow2_SetPoint.Tooltip = {'Target value for PID control'};
            app.Flow2_SetPoint.Position = [161 13 44 22];

            % Create CurrentLabel_2
            app.CurrentLabel_2 = uilabel(app.Panel_2);
            app.CurrentLabel_2.Position = [106 66 45 22];
            app.CurrentLabel_2.Text = 'Current';

            % Create TargetLabel_2
            app.TargetLabel_2 = uilabel(app.Panel_2);
            app.TargetLabel_2.HorizontalAlignment = 'center';
            app.TargetLabel_2.Position = [161 66 44 22];
            app.TargetLabel_2.Text = 'Target';

            % Create Label_7
            app.Label_7 = uilabel(app.Panel_2);
            app.Label_7.HorizontalAlignment = 'center';
            app.Label_7.FontWeight = 'bold';
            app.Label_7.Position = [60 45 25 22];
            app.Label_7.Text = '1';

            % Create Flow1_EditField_Current
            app.Flow1_EditField_Current = uieditfield(app.Panel_2, 'numeric');
            app.Flow1_EditField_Current.Editable = 'off';
            app.Flow1_EditField_Current.HorizontalAlignment = 'center';
            app.Flow1_EditField_Current.Tooltip = {'Current flow reading (L/min) for flow sensor 1 (see label on sensor).'};
            app.Flow1_EditField_Current.Position = [107 44 43 22];

            % Create Label_8
            app.Label_8 = uilabel(app.Panel_2);
            app.Label_8.HorizontalAlignment = 'center';
            app.Label_8.FontWeight = 'bold';
            app.Label_8.Position = [60 13 25 22];
            app.Label_8.Text = '2';

            % Create Flow2_EditField_Current
            app.Flow2_EditField_Current = uieditfield(app.Panel_2, 'numeric');
            app.Flow2_EditField_Current.Editable = 'off';
            app.Flow2_EditField_Current.HorizontalAlignment = 'center';
            app.Flow2_EditField_Current.Tooltip = {'Current flow reading (L/min) for flow sensor 2 (see label on sensor).'};
            app.Flow2_EditField_Current.Position = [107 13 43 22];

            % Create Flow2_GraphEnable
            app.Flow2_GraphEnable = uicheckbox(app.Panel_2);
            app.Flow2_GraphEnable.Text = '';
            app.Flow2_GraphEnable.Position = [378 13 15 22];
            app.Flow2_GraphEnable.Value = true;

            % Create Flow1_GraphEnable
            app.Flow1_GraphEnable = uicheckbox(app.Panel_2);
            app.Flow1_GraphEnable.Text = '';
            app.Flow1_GraphEnable.Position = [378 42 14 22];
            app.Flow1_GraphEnable.Value = true;

            % Create Label_10
            app.Label_10 = uilabel(app.Panel_2);
            app.Label_10.HorizontalAlignment = 'center';
            app.Label_10.FontSize = 18;
            app.Label_10.Position = [370 66 30 23];
            app.Label_10.Text = '👁️';

            % Create SensorLabel_2
            app.SensorLabel_2 = uilabel(app.Panel_2);
            app.SensorLabel_2.HorizontalAlignment = 'center';
            app.SensorLabel_2.FontWeight = 'bold';
            app.SensorLabel_2.Position = [50 67 45 22];
            app.SensorLabel_2.Text = 'Sensor';

            % Create Flow2_LabelField
            app.Flow2_LabelField = uieditfield(app.Panel_2, 'text');
            app.Flow2_LabelField.ValueChangedFcn = createCallbackFcn(app, @Flow2_LabelFieldValueChanged, true);
            app.Flow2_LabelField.HorizontalAlignment = 'center';
            app.Flow2_LabelField.FontAngle = 'italic';
            app.Flow2_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow2_LabelField.Position = [297 13 63 22];
            app.Flow2_LabelField.Value = '2';

            % Create Label_9
            app.Label_9 = uilabel(app.Panel_2);
            app.Label_9.HorizontalAlignment = 'center';
            app.Label_9.Position = [297 66 63 22];

            % Create Flow1_LabelField
            app.Flow1_LabelField = uieditfield(app.Panel_2, 'text');
            app.Flow1_LabelField.ValueChangedFcn = createCallbackFcn(app, @Flow1_LabelFieldValueChanged, true);
            app.Flow1_LabelField.HorizontalAlignment = 'center';
            app.Flow1_LabelField.FontAngle = 'italic';
            app.Flow1_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow1_LabelField.Position = [297 43 63 22];
            app.Flow1_LabelField.Value = '1';

            % Create AvgSDLabel_2
            app.AvgSDLabel_2 = uilabel(app.Panel_2);
            app.AvgSDLabel_2.HorizontalAlignment = 'center';
            app.AvgSDLabel_2.Position = [215 65 72 22];
            app.AvgSDLabel_2.Text = 'Avg ± SD';

            % Create Flow1_EditField_Avg
            app.Flow1_EditField_Avg = uieditfield(app.Panel_2, 'text');
            app.Flow1_EditField_Avg.HorizontalAlignment = 'center';
            app.Flow1_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow1_EditField_Avg.Position = [217 43 72 22];

            % Create Flow2_EditField_Avg
            app.Flow2_EditField_Avg = uieditfield(app.Panel_2, 'text');
            app.Flow2_EditField_Avg.HorizontalAlignment = 'center';
            app.Flow2_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow2_EditField_Avg.Position = [217 13 72 22];

            % Create PressureGraphPanel
            app.PressureGraphPanel = uipanel(app.MainTab);
            app.PressureGraphPanel.BorderColor = [0.4902 0.4902 0.4902];
            app.PressureGraphPanel.BackgroundColor = [0.8392 0.8314 0.8588];
            app.PressureGraphPanel.FontAngle = 'italic';
            app.PressureGraphPanel.Position = [330 152 435 329];

            % Create PressureAxes
            app.PressureAxes = uiaxes(app.PressureGraphPanel);
            xlabel(app.PressureAxes, 'Time (s)')
            ylabel(app.PressureAxes, 'Pressure (mmHg)')
            zlabel(app.PressureAxes, 'Z')
            app.PressureAxes.Toolbar.Visible = 'off';
            app.PressureAxes.FontWeight = 'bold';
            app.PressureAxes.XColor = [0.502 0.502 0.502];
            app.PressureAxes.YColor = [0.502 0.502 0.502];
            app.PressureAxes.Color = [0.902 0.902 0.902];
            app.PressureAxes.FontSize = 14;
            app.PressureAxes.Visible = 'off';
            app.PressureAxes.Position = [13 5 409 312];

            % Create PressureAxesWaitingForConnectionLabel
            app.PressureAxesWaitingForConnectionLabel = uilabel(app.PressureGraphPanel);
            app.PressureAxesWaitingForConnectionLabel.HorizontalAlignment = 'center';
            app.PressureAxesWaitingForConnectionLabel.FontSize = 36;
            app.PressureAxesWaitingForConnectionLabel.FontWeight = 'bold';
            app.PressureAxesWaitingForConnectionLabel.FontColor = [0.149 0.149 0.149];
            app.PressureAxesWaitingForConnectionLabel.Enable = 'off';
            app.PressureAxesWaitingForConnectionLabel.Position = [99 141 258 47];
            app.PressureAxesWaitingForConnectionLabel.Text = 'No connection';

            % Create PumpControlPanel
            app.PumpControlPanel = uipanel(app.MainTab);
            app.PumpControlPanel.Enable = 'off';
            app.PumpControlPanel.Title = 'Pump Control';
            app.PumpControlPanel.BackgroundColor = [0.902 0.902 0.902];
            app.PumpControlPanel.FontWeight = 'bold';
            app.PumpControlPanel.Position = [17 171 302 150];

            % Create PumpPowerSliderLabel
            app.PumpPowerSliderLabel = uilabel(app.PumpControlPanel);
            app.PumpPowerSliderLabel.HorizontalAlignment = 'right';
            app.PumpPowerSliderLabel.FontWeight = 'bold';
            app.PumpPowerSliderLabel.Position = [10 57 95 22];
            app.PumpPowerSliderLabel.Text = 'Pump Power  %';

            % Create PumpPowerSlider
            app.PumpPowerSlider = uislider(app.PumpControlPanel);
            app.PumpPowerSlider.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSliderValueChanged, true);
            app.PumpPowerSlider.Tooltip = {'Sends a PWM signal to the pump from 0 to 100% power. 100% power represents the full voltage of your power supply.'};
            app.PumpPowerSlider.Position = [126 66 150 3];
            app.PumpPowerSlider.Value = 50;

            % Create ControlModeSwitchLabel
            app.ControlModeSwitchLabel = uilabel(app.PumpControlPanel);
            app.ControlModeSwitchLabel.HorizontalAlignment = 'center';
            app.ControlModeSwitchLabel.FontWeight = 'bold';
            app.ControlModeSwitchLabel.Position = [15 91 82 22];
            app.ControlModeSwitchLabel.Text = 'Control Mode';

            % Create ControlModeSwitch
            app.ControlModeSwitch = uiswitch(app.PumpControlPanel, 'slider');
            app.ControlModeSwitch.Items = {'Manual', 'Auto'};
            app.ControlModeSwitch.ValueChangedFcn = createCallbackFcn(app, @ControlModeSwitchValueChanged, true);
            app.ControlModeSwitch.Tooltip = {'Manual - control pump power using the slider below'; ''; 'Auto - Input a flow or puressure setpoint in the panels above and the system will automatically adjust'};
            app.ControlModeSwitch.Position = [192 92 45 20];
            app.ControlModeSwitch.Value = 'Manual';

            % Create StartPulsatileFlowButton
            app.StartPulsatileFlowButton = uibutton(app.PumpControlPanel, 'state');
            app.StartPulsatileFlowButton.ValueChangedFcn = createCallbackFcn(app, @StartPulsatileFlowButtonValueChanged, true);
            app.StartPulsatileFlowButton.Enable = 'off';
            app.StartPulsatileFlowButton.Text = 'Start Pulsatile Flow';
            app.StartPulsatileFlowButton.Position = [89 4 118 23];

            % Create FlowGraphPanel
            app.FlowGraphPanel = uipanel(app.MainTab);
            app.FlowGraphPanel.BackgroundColor = [0.7725 0.8314 0.7569];
            app.FlowGraphPanel.Position = [775 152 448 329];

            % Create FlowAxes
            app.FlowAxes = uiaxes(app.FlowGraphPanel);
            xlabel(app.FlowAxes, 'Time(s)')
            ylabel(app.FlowAxes, 'Flow (L/min)')
            zlabel(app.FlowAxes, 'Z')
            app.FlowAxes.Toolbar.Visible = 'off';
            app.FlowAxes.FontWeight = 'bold';
            app.FlowAxes.XColor = [0.502 0.502 0.502];
            app.FlowAxes.YColor = [0.502 0.502 0.502];
            app.FlowAxes.Color = [0.902 0.902 0.902];
            app.FlowAxes.GridColor = [0.149 0.149 0.149];
            app.FlowAxes.FontSize = 14;
            app.FlowAxes.Position = [15 9 419 310];

            % Create FlowAxesWaitingForConnectionLabel
            app.FlowAxesWaitingForConnectionLabel = uilabel(app.FlowGraphPanel);
            app.FlowAxesWaitingForConnectionLabel.HorizontalAlignment = 'center';
            app.FlowAxesWaitingForConnectionLabel.FontSize = 36;
            app.FlowAxesWaitingForConnectionLabel.FontWeight = 'bold';
            app.FlowAxesWaitingForConnectionLabel.FontColor = [0.149 0.149 0.149];
            app.FlowAxesWaitingForConnectionLabel.Enable = 'off';
            app.FlowAxesWaitingForConnectionLabel.Position = [114 141 258 47];
            app.FlowAxesWaitingForConnectionLabel.Text = 'No connection';

            % Create OptionsPanel
            app.OptionsPanel = uipanel(app.MainTab);
            app.OptionsPanel.BorderWidth = 0;
            app.OptionsPanel.Position = [13 328 305 88];

            % Create PauseGraphsButton
            app.PauseGraphsButton = uibutton(app.OptionsPanel, 'state');
            app.PauseGraphsButton.Enable = 'off';
            app.PauseGraphsButton.Icon = fullfile(pathToMLAPP, 'assets', 'pause_icon.png');
            app.PauseGraphsButton.Text = 'Pause Graphs';
            app.PauseGraphsButton.BackgroundColor = [0.1569 0.5725 0.8431];
            app.PauseGraphsButton.FontSize = 14;
            app.PauseGraphsButton.FontWeight = 'bold';
            app.PauseGraphsButton.FontColor = [1 1 1];
            app.PauseGraphsButton.Position = [161 47 145 31];

            % Create ClearDataButton
            app.ClearDataButton = uibutton(app.OptionsPanel, 'push');
            app.ClearDataButton.ButtonPushedFcn = createCallbackFcn(app, @ClearDataButtonPushed, true);
            app.ClearDataButton.Icon = fullfile(pathToMLAPP, 'assets', 'eraser_white.png');
            app.ClearDataButton.BackgroundColor = [0.1569 0.5725 0.8431];
            app.ClearDataButton.FontSize = 14;
            app.ClearDataButton.FontWeight = 'bold';
            app.ClearDataButton.FontColor = [1 1 1];
            app.ClearDataButton.Enable = 'off';
            app.ClearDataButton.Position = [5 12 146 31];
            app.ClearDataButton.Text = 'Clear Data';

            % Create ExportDataButton
            app.ExportDataButton = uibutton(app.OptionsPanel, 'push');
            app.ExportDataButton.ButtonPushedFcn = createCallbackFcn(app, @ExportDataButtonPushed, true);
            app.ExportDataButton.Icon = fullfile(pathToMLAPP, 'assets', 'export.png');
            app.ExportDataButton.BackgroundColor = [0.1569 0.5725 0.8431];
            app.ExportDataButton.FontSize = 14;
            app.ExportDataButton.FontWeight = 'bold';
            app.ExportDataButton.FontColor = [1 1 1];
            app.ExportDataButton.Enable = 'off';
            app.ExportDataButton.Position = [161 12 145 31];
            app.ExportDataButton.Text = 'Export Data';

            % Create CalibrateButton
            app.CalibrateButton = uibutton(app.OptionsPanel, 'push');
            app.CalibrateButton.ButtonPushedFcn = createCallbackFcn(app, @CalibrateButtonPushed, true);
            app.CalibrateButton.Icon = fullfile(pathToMLAPP, 'assets', 'check-circle.png');
            app.CalibrateButton.BackgroundColor = [0.1569 0.5725 0.8431];
            app.CalibrateButton.FontSize = 14;
            app.CalibrateButton.FontWeight = 'bold';
            app.CalibrateButton.FontColor = [1 1 1];
            app.CalibrateButton.Enable = 'off';
            app.CalibrateButton.Position = [5 47 146 31];
            app.CalibrateButton.Text = 'Calibrate';

            % Create ConnectionDropDownLabel
            app.ConnectionDropDownLabel = uilabel(app.MainTab);
            app.ConnectionDropDownLabel.FontSize = 18;
            app.ConnectionDropDownLabel.FontWeight = 'bold';
            app.ConnectionDropDownLabel.FontColor = [0.149 0.149 0.149];
            app.ConnectionDropDownLabel.Position = [18 484 104 23];
            app.ConnectionDropDownLabel.Text = 'Connection';

            % Create ConnectionDropDown
            app.ConnectionDropDown = uidropdown(app.MainTab);
            app.ConnectionDropDown.Items = {};
            app.ConnectionDropDown.FontSize = 14;
            app.ConnectionDropDown.FontWeight = 'bold';
            app.ConnectionDropDown.FontColor = [0.149 0.149 0.149];
            app.ConnectionDropDown.BackgroundColor = [0.9412 0.9412 0.9412];
            app.ConnectionDropDown.Position = [17 449 126 31];
            app.ConnectionDropDown.Value = {};

            % Create ConnectedLamp
            app.ConnectedLamp = uilamp(app.MainTab);
            app.ConnectedLamp.Position = [295 455 20 20];
            app.ConnectedLamp.Color = [1 0 0];

            % Create RefreshConnectionsButton
            app.RefreshConnectionsButton = uibutton(app.MainTab, 'push');
            app.RefreshConnectionsButton.ButtonPushedFcn = createCallbackFcn(app, @RefreshConnectionsButtonPushed, true);
            app.RefreshConnectionsButton.Icon = fullfile(pathToMLAPP, 'assets', 'Refresh_icon.png');
            app.RefreshConnectionsButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RefreshConnectionsButton.FontSize = 14;
            app.RefreshConnectionsButton.FontWeight = 'bold';
            app.RefreshConnectionsButton.FontColor = [0.149 0.149 0.149];
            app.RefreshConnectionsButton.Position = [255 450 31 32];
            app.RefreshConnectionsButton.Text = '';

            % Create ConnectDisconnectButton
            app.ConnectDisconnectButton = uibutton(app.MainTab, 'push');
            app.ConnectDisconnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectDisconnectButtonPushed, true);
            app.ConnectDisconnectButton.BackgroundColor = [1 0 0];
            app.ConnectDisconnectButton.FontSize = 14;
            app.ConnectDisconnectButton.FontWeight = 'bold';
            app.ConnectDisconnectButton.FontColor = [0.149 0.149 0.149];
            app.ConnectDisconnectButton.Enable = 'off';
            app.ConnectDisconnectButton.Position = [149 450 98 31];
            app.ConnectDisconnectButton.Text = 'Connect';

            % Create OptionsLabel
            app.OptionsLabel = uilabel(app.MainTab);
            app.OptionsLabel.FontSize = 18;
            app.OptionsLabel.FontWeight = 'bold';
            app.OptionsLabel.FontColor = [0.149 0.149 0.149];
            app.OptionsLabel.Enable = 'off';
            app.OptionsLabel.Position = [20 410 73 23];
            app.OptionsLabel.Text = 'Options';

            % Create PressureDataTitleLabelPanel
            app.PressureDataTitleLabelPanel = uipanel(app.MainTab);
            app.PressureDataTitleLabelPanel.Enable = 'off';
            app.PressureDataTitleLabelPanel.Position = [330 484 435 30];

            % Create PressureDataTitleLabel
            app.PressureDataTitleLabel = uilabel(app.PressureDataTitleLabelPanel);
            app.PressureDataTitleLabel.HorizontalAlignment = 'center';
            app.PressureDataTitleLabel.FontSize = 18;
            app.PressureDataTitleLabel.FontWeight = 'bold';
            app.PressureDataTitleLabel.FontColor = [0.149 0.149 0.149];
            app.PressureDataTitleLabel.Position = [0 3 434 23];
            app.PressureDataTitleLabel.Text = 'Pressure Data (mmHg)';

            % Create FlowDataTitleLabelPanel
            app.FlowDataTitleLabelPanel = uipanel(app.MainTab);
            app.FlowDataTitleLabelPanel.Enable = 'off';
            app.FlowDataTitleLabelPanel.Position = [775 484 448 30];

            % Create FlowDataTitleLabel
            app.FlowDataTitleLabel = uilabel(app.FlowDataTitleLabelPanel);
            app.FlowDataTitleLabel.HorizontalAlignment = 'center';
            app.FlowDataTitleLabel.FontSize = 18;
            app.FlowDataTitleLabel.FontWeight = 'bold';
            app.FlowDataTitleLabel.FontColor = [0.149 0.149 0.149];
            app.FlowDataTitleLabel.Position = [0 4 447 23];
            app.FlowDataTitleLabel.Text = 'Flow Data (L/min)';

            % Create SettingsTab
            app.SettingsTab = uitab(app.TabGroup2);
            app.SettingsTab.Title = 'Settings';

            % Create PIDCoefficientsPanel
            app.PIDCoefficientsPanel = uipanel(app.SettingsTab);
            app.PIDCoefficientsPanel.Enable = 'off';
            app.PIDCoefficientsPanel.Title = 'PID Coefficients';
            app.PIDCoefficientsPanel.FontWeight = 'bold';
            app.PIDCoefficientsPanel.Position = [17 365 469 139];

            % Create PressureCoefficientsPanel
            app.PressureCoefficientsPanel = uipanel(app.PIDCoefficientsPanel);
            app.PressureCoefficientsPanel.BackgroundColor = [0.8392 0.8275 0.8588];
            app.PressureCoefficientsPanel.FontWeight = 'bold';
            app.PressureCoefficientsPanel.Position = [12 69 446 42];

            % Create PressureControlLabel
            app.PressureControlLabel = uilabel(app.PressureCoefficientsPanel);
            app.PressureControlLabel.FontWeight = 'bold';
            app.PressureControlLabel.Position = [12 10 102 22];
            app.PressureControlLabel.Text = 'Pressure Control';

            % Create KpEditField_2Label
            app.KpEditField_2Label = uilabel(app.PressureCoefficientsPanel);
            app.KpEditField_2Label.HorizontalAlignment = 'right';
            app.KpEditField_2Label.FontWeight = 'bold';
            app.KpEditField_2Label.Position = [135 10 25 22];
            app.KpEditField_2Label.Text = 'Kp';

            % Create KpPressureEditField
            app.KpPressureEditField = uieditfield(app.PressureCoefficientsPanel, 'numeric');
            app.KpPressureEditField.ValueChangedFcn = createCallbackFcn(app, @KpPressureEditFieldValueChanged, true);
            app.KpPressureEditField.FontWeight = 'bold';
            app.KpPressureEditField.Position = [175 10 42 22];

            % Create KiEditField_2Label
            app.KiEditField_2Label = uilabel(app.PressureCoefficientsPanel);
            app.KiEditField_2Label.HorizontalAlignment = 'right';
            app.KiEditField_2Label.FontWeight = 'bold';
            app.KiEditField_2Label.Position = [240 10 25 22];
            app.KiEditField_2Label.Text = 'Ki';

            % Create KiPressureEditField
            app.KiPressureEditField = uieditfield(app.PressureCoefficientsPanel, 'numeric');
            app.KiPressureEditField.ValueChangedFcn = createCallbackFcn(app, @KiPressureEditFieldValueChanged, true);
            app.KiPressureEditField.FontWeight = 'bold';
            app.KiPressureEditField.Position = [280 10 42 22];

            % Create KdPressureEditField
            app.KdPressureEditField = uieditfield(app.PressureCoefficientsPanel, 'numeric');
            app.KdPressureEditField.ValueChangedFcn = createCallbackFcn(app, @KdPressureEditFieldValueChanged, true);
            app.KdPressureEditField.FontWeight = 'bold';
            app.KdPressureEditField.Position = [386 10 42 22];

            % Create KdEditField_2Label
            app.KdEditField_2Label = uilabel(app.PressureCoefficientsPanel);
            app.KdEditField_2Label.HorizontalAlignment = 'right';
            app.KdEditField_2Label.FontWeight = 'bold';
            app.KdEditField_2Label.Position = [346 10 25 22];
            app.KdEditField_2Label.Text = 'Kd';

            % Create FlowCoefficientsPanel
            app.FlowCoefficientsPanel = uipanel(app.PIDCoefficientsPanel);
            app.FlowCoefficientsPanel.BackgroundColor = [0.7725 0.8314 0.7569];
            app.FlowCoefficientsPanel.FontWeight = 'bold';
            app.FlowCoefficientsPanel.Position = [12 13 446 42];

            % Create FlowControlLabel
            app.FlowControlLabel = uilabel(app.FlowCoefficientsPanel);
            app.FlowControlLabel.FontWeight = 'bold';
            app.FlowControlLabel.Position = [36 10 78 22];
            app.FlowControlLabel.Text = 'Flow Control';

            % Create KpEditFieldLabel
            app.KpEditFieldLabel = uilabel(app.FlowCoefficientsPanel);
            app.KpEditFieldLabel.HorizontalAlignment = 'right';
            app.KpEditFieldLabel.FontWeight = 'bold';
            app.KpEditFieldLabel.Position = [135 10 25 22];
            app.KpEditFieldLabel.Text = 'Kp';

            % Create KpFlowEditField
            app.KpFlowEditField = uieditfield(app.FlowCoefficientsPanel, 'numeric');
            app.KpFlowEditField.ValueChangedFcn = createCallbackFcn(app, @KpFlowEditFieldValueChanged, true);
            app.KpFlowEditField.FontWeight = 'bold';
            app.KpFlowEditField.Position = [175 10 42 22];

            % Create KiEditFieldLabel
            app.KiEditFieldLabel = uilabel(app.FlowCoefficientsPanel);
            app.KiEditFieldLabel.HorizontalAlignment = 'right';
            app.KiEditFieldLabel.FontWeight = 'bold';
            app.KiEditFieldLabel.Position = [240 10 25 22];
            app.KiEditFieldLabel.Text = 'Ki';

            % Create KiFlowEditField
            app.KiFlowEditField = uieditfield(app.FlowCoefficientsPanel, 'numeric');
            app.KiFlowEditField.ValueChangedFcn = createCallbackFcn(app, @KiFlowEditFieldValueChanged, true);
            app.KiFlowEditField.FontWeight = 'bold';
            app.KiFlowEditField.Position = [280 10 42 22];

            % Create KdFlowEditField
            app.KdFlowEditField = uieditfield(app.FlowCoefficientsPanel, 'numeric');
            app.KdFlowEditField.ValueChangedFcn = createCallbackFcn(app, @KdFlowEditFieldValueChanged, true);
            app.KdFlowEditField.FontWeight = 'bold';
            app.KdFlowEditField.Position = [386 10 42 22];

            % Create KdEditFieldLabel
            app.KdEditFieldLabel = uilabel(app.FlowCoefficientsPanel);
            app.KdEditFieldLabel.HorizontalAlignment = 'right';
            app.KdEditFieldLabel.FontWeight = 'bold';
            app.KdEditFieldLabel.Position = [346 10 25 22];
            app.KdEditFieldLabel.Text = 'Kd';

            % Create DeveloperOptionsPanel
            app.DeveloperOptionsPanel = uipanel(app.SettingsTab);
            app.DeveloperOptionsPanel.Title = 'Developer Options';
            app.DeveloperOptionsPanel.FontWeight = 'bold';
            app.DeveloperOptionsPanel.Position = [499 365 260 140];

            % Create SimulateDataCheckBox
            app.SimulateDataCheckBox = uicheckbox(app.DeveloperOptionsPanel);
            app.SimulateDataCheckBox.ValueChangedFcn = createCallbackFcn(app, @SimulateDataCheckBoxValueChanged, true);
            app.SimulateDataCheckBox.Tooltip = {'Simulates data without requiring a connection to a microcontroller. Mostly useful for testing that the GUI works.'};
            app.SimulateDataCheckBox.Text = 'Simulate Data?';
            app.SimulateDataCheckBox.Position = [14 88 160 22];

            % Create CurrentPumpAnalogWriteValue0255EditFieldLabel
            app.CurrentPumpAnalogWriteValue0255EditFieldLabel = uilabel(app.SettingsTab);
            app.CurrentPumpAnalogWriteValue0255EditFieldLabel.HorizontalAlignment = 'right';
            app.CurrentPumpAnalogWriteValue0255EditFieldLabel.FontWeight = 'bold';
            app.CurrentPumpAnalogWriteValue0255EditFieldLabel.Position = [773 472 201 30];
            app.CurrentPumpAnalogWriteValue0255EditFieldLabel.Text = {'Current Pump Analog Write Value '; '(0-255)'};

            % Create CurrentPumpAnalogWriteValueEditField
            app.CurrentPumpAnalogWriteValueEditField = uieditfield(app.SettingsTab, 'numeric');
            app.CurrentPumpAnalogWriteValueEditField.Limits = [0 255];
            app.CurrentPumpAnalogWriteValueEditField.RoundFractionalValues = 'on';
            app.CurrentPumpAnalogWriteValueEditField.ValueChangedFcn = createCallbackFcn(app, @CurrentPumpAnalogWriteValueEditFieldValueChanged, true);
            app.CurrentPumpAnalogWriteValueEditField.Editable = 'off';
            app.CurrentPumpAnalogWriteValueEditField.Position = [989 480 66 22];

            % Create OtherSettingsPanel
            app.OtherSettingsPanel = uipanel(app.SettingsTab);
            app.OtherSettingsPanel.Title = 'Other Settings';
            app.OtherSettingsPanel.Position = [17 279 469 73];

            % Create CalibrationdurationsEditFieldLabel
            app.CalibrationdurationsEditFieldLabel = uilabel(app.OtherSettingsPanel);
            app.CalibrationdurationsEditFieldLabel.HorizontalAlignment = 'right';
            app.CalibrationdurationsEditFieldLabel.Position = [116 13 126 22];
            app.CalibrationdurationsEditFieldLabel.Text = 'Calibration duration (s)';

            % Create CalibrationdurationsEditField
            app.CalibrationdurationsEditField = uieditfield(app.OtherSettingsPanel, 'numeric');
            app.CalibrationdurationsEditField.Limits = [0 Inf];
            app.CalibrationdurationsEditField.ValueChangedFcn = createCallbackFcn(app, @CalibrationdurationsEditFieldValueChanged, true);
            app.CalibrationdurationsEditField.Position = [257 13 100 22];
            app.CalibrationdurationsEditField.Value = 30;

            % Create HelpTab
            app.HelpTab = uitab(app.TabGroup2);
            app.HelpTab.Title = 'Help';

            % Create HTML
            app.HTML = uihtml(app.HelpTab);
            app.HTML.HTMLSource = './assets/help.html';
            app.HTML.Position = [16 51 1201 456];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = CCTA_exported

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.UIFigure)

                % Execute the startup function
                runStartupFcn(app, @startupFcn)
            else

                % Focus the running singleton app
                figure(runningApp.UIFigure)

                app = runningApp;
            end

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