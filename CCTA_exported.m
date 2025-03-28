classdef CCTA_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                       matlab.ui.Figure
        MainTabGroup                   matlab.ui.container.TabGroup
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
        PumpPowerSpinner               matlab.ui.control.Spinner
        StartPulsatileFlowButton       matlab.ui.control.StateButton
        ControlModeSwitch              matlab.ui.control.Switch
        ControlModeSwitchLabel         matlab.ui.control.Label
        PumpPowerSlider                matlab.ui.control.Slider
        PumpPowerSliderLabel           matlab.ui.control.Label
        PressureGraphPanel             matlab.ui.container.Panel
        PressureAxesWaitingForConnectionLabel  matlab.ui.control.Label
        PressureAxes                   matlab.ui.control.UIAxes
        FlowDetailsPanel               matlab.ui.container.Panel
        Flow1_EditField_Avg            matlab.ui.control.EditField
        Flow_ResetAllLabelButton       matlab.ui.control.Button
        Flow_ResetAllTargetButton      matlab.ui.control.Button
        Flow2_EditField_Avg            matlab.ui.control.EditField
        RollAvgSDLabel_2               matlab.ui.control.Label
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
        Flow2_EditField_Target         matlab.ui.control.NumericEditField
        Flow1_EditField_Target         matlab.ui.control.NumericEditField
        Label_6                        matlab.ui.control.Label
        PressureDetailsPanel           matlab.ui.container.Panel
        Pressure_ResetAllSHOButton     matlab.ui.control.Button
        Pressure_ResetAllLabelButton   matlab.ui.control.Button
        Pressure_ResetAllTargetButton  matlab.ui.control.Button
        Pressure3_EditField_Avg        matlab.ui.control.EditField
        Pressure2_EditField_Avg        matlab.ui.control.EditField
        Pressure1_EditField_Avg        matlab.ui.control.EditField
        RollAvgSDLabel                 matlab.ui.control.Label
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
        Pressure3_EditField_Target     matlab.ui.control.NumericEditField
        Pressure2_EditField_Target     matlab.ui.control.NumericEditField
        Pressure1_EditField_Target     matlab.ui.control.NumericEditField
        SettingsTab                    matlab.ui.container.Tab
        OtherSettingsPanel             matlab.ui.container.Panel
        RollingaveragedurationEditField  matlab.ui.control.NumericEditField
        RollingaveragedurationEditFieldLabel  matlab.ui.control.Label
        CalibrationdurationsEditField  matlab.ui.control.NumericEditField
        CalibrationdurationsEditFieldLabel  matlab.ui.control.Label
        CurrentPumpAnalogWriteValueEditField  matlab.ui.control.NumericEditField
        CurrentPumpAnalogWriteValue0255Label  matlab.ui.control.Label
        DeveloperToolsPanel            matlab.ui.container.Panel
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
        % Constants
        plot_raw_data = true;
        F_REFRESH = 20;  % App refresh rate (Hz)
        ARDUINO_ANALOGWRITE_MAX = 255;
        ARDUINO_SERIAL_BITRATE = 115200;
        PID_ERRORS_LENGTH = 75;  % # of error values to use for PID integral calculations, determined empirically to minimize settling time
        CALIBRATION_DURATION = 20;
        ROLLING_AVERAGE_DURATION = 10;  % # of seconds for rolling average
        PUMP_LOW_POWER = 15;
        DT_PID = 0.1;  % Time step for integral/derivative PID calculations (adjust based on system response)

        % Colors
        COLOR_PRESSURE = [0.84 0.83 0.86];
        COLOR_PRESSURE_2 = [161, 138, 219]./255;
        FLOW_COLOR = [0.77,0.83,0.76];
        BUTTON_COLOR_DEFAULT = [0.94,0.94,0.94];
        BUTTON_COLOR1 = [0.16,0.57,0.84];
        DISABLED_COLOR = [0.5 0.5 0.5];
        COLOR_DISCONNECTED = [1.00,0.00,0.00];
        COLOR_CONNECTED = [0, 1, 0];
        COLOR_ERROR = [255, 161, 130]./255;
        COLOR_WARNING = [255, 190, 130]./255;
        COLOR_NORMAL = [1 1 1];
        COLOR_EDITFIELD_DIMMED = [0.75 0.75 0.75];

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
        Kp_flow = 5;  % PID coefficients when controlling flow
        Ki_flow = 0.5;  % Default values defined here
        Kd_flow = 0;

        Kp_pressure = 0.2;  % PID coefficients when controlling pressure
        Ki_pressure = 0.1;  % Default values defined here
        Kd_pressure = 0.05;

        lastFlowDutyCycle = 0;  % for comparisons
        flowDutyCycle = 0;  % Initialize halfway
        PumpControlMode = 'Manual';

        % Other variables

        rollingAverageCutoffTime;

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

    methods (Access = private)

        function dimEditFieldTemporarily(app, editField)
            editField.BackgroundColor = app.COLOR_EDITFIELD_DIMMED;
            pause(0.5);
            editField.BackgroundColor = app.COLOR_NORMAL;
        end

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

                % Update Edit Field Values
                app.Pressure1_EditField_Current.Value = pressure_value1;
                app.Pressure2_EditField_Current.Value = pressure_value2;
                app.Pressure3_EditField_Current.Value = pressure_value3;
                app.Flow1_EditField_Current.Value = flow_value1;
                app.Flow2_EditField_Current.Value = flow_value2;

                % Update edit field background colors if pressure/flow
                % values are negative
                fields = {
                    'Pressure1', pressure_value1;
                    'Pressure2', pressure_value2;
                    'Pressure3', pressure_value3;
                    'Flow1',     flow_value1;
                    'Flow2',     flow_value2
                    };

                for i = 1:size(fields, 1)
                    name = fields{i, 1};
                    val  = fields{i, 2};
                    fieldHandle = app.([name '_EditField_Current']);
                    fieldHandle.Value = val;
                    if val < 0
                        fieldHandle.BackgroundColor = app.COLOR_ERROR;
                    else
                        fieldHandle.BackgroundColor = app.COLOR_NORMAL;
                    end
                end

                % Update sensor rolling average values
                app.updateSensorAverages();

                % Update pressure/flow plots
                app.updateDataPlots();

                % Update PID if in Auto mode
                if app.PumpControlMode == "PID"
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
                highlightSetPointUI(app, "None");  % clear any setpoint UI from any prior PID control
                app.PumpControlMode = "Manual";
                app.ControlModeSwitch.Value = "Manual";
                app.ControlModeSwitch.Enable = "off";  % to prevent user from switching toggle when they should just input a setpoint
                app.PumpPowerSlider.Enable = "on";
                app.PumpPowerSpinner.Enable = "on";
                app.PID_setpoint_id = "None";
            elseif lower(mode) == "pid"
                app.PumpControlMode = "PID";
                app.ControlModeSwitch.Value = "Auto";
                app.ControlModeSwitch.Enable = "on";
                app.PumpPowerSlider.Enable = "off";
                app.PumpPowerSpinner.Enable = "off";
            elseif lower(mode) == "pulsatile"
                highlightSetPointUI(app, "None");  % clear any setpoint UI from any prior PID control
                app.PumpControlMode = "Pulsatile";
                app.ControlModeSwitch.Value = "Auto";
                app.ControlModeSwitch.Enable = "on";
                app.PumpPowerSlider.Enable = "off";
                app.PumpPowerSpinner.Enable = "off";
            elseif lower(mode) == "calibration"
                highlightSetPointUI(app, "None");  % clear any setpoint UI from any prior PID control
                app.PumpControlMode = "Calibration";
                app.ControlModeSwitch.Value = "Auto";
                app.ControlModeSwitch.Enable = "off";
                app.PumpPowerSlider.Enable = "off";
                app.PumpPowerSpinner.Enable = "off";
            else
                warning("Invalid control mode selected.");
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

            % Update control value (flow duty cycle) based on PID algorithm
            app.flowDutyCycle = app.flowDutyCycle + round(app.PID_output);

            % Place PID output within limits
            if app.flowDutyCycle > app.ARDUINO_ANALOGWRITE_MAX
                app.flowDutyCycle = app.ARDUINO_ANALOGWRITE_MAX;
            elseif app.flowDutyCycle < 0
                app.flowDutyCycle = 0;
            end

            app.setPumpPower(app.flowDutyCycle/app.ARDUINO_ANALOGWRITE_MAX*100);
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
            'Pressure1', app.Pressure1_EditField_Target, ...
            'Pressure2', app.Pressure2_EditField_Target, ...
            'Pressure3', app.Pressure3_EditField_Target, ...
            'Flow1', app.Flow1_EditField_Target, ...
            'Flow2', app.Flow2_EditField_Target ...
            );

            % Iterate through all setpoints and update colors
            fieldNames = fieldnames(setpoints);

            for i = 1:length(fieldNames)
                % Get handle to the UI element
                uiElement = setpoints.(fieldNames{i});

                onColor = app.COLOR_CONNECTED;
                offColor = app.COLOR_NORMAL;

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
            pressureElements = [app.PressureDetailsPanel, ...
                app.PressureGraphPanel, ...
                app.PressureCoefficientsPanel, ...
                app.PressureDataTitleLabelPanel];

            for pressureElement = pressureElements
                pressureElement.BackgroundColor = app.COLOR_PRESSURE;
            end

            % Color all flow-related elements
            flowElements = [app.FlowDetailsPanel, ...
                app.FlowGraphPanel, ...
                app.FlowCoefficientsPanel, ...
                app.FlowDataTitleLabelPanel];

            for flowElement = flowElements
                flowElement.BackgroundColor = app.FLOW_COLOR;
            end

            % Other things
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
            app.setPumpPower(50);  % start at 50% as a signal the system started working, without pushing too much water
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

            app.setPumpPower(0);  % turn off pump
            app.arduinoObj = [];
            app.ConnectionDropDown.Enable = "on";
            app.RefreshConnectionsButton.Enable = "on";
            app.ConnectDisconnectButton.Text = "Connect";
            app.ConnectedLamp.Color = app.COLOR_DISCONNECTED;
            app.isConnected = false;
            app.SimulateDataCheckBox.Value = false;  % to allow user to re-select it if they want to simulate again
            app.setGUIEnabled(false);

            % Pressure/Flow Values
            app.Pressure1_EditField_Current.Value = 0;
            app.Pressure1_EditField_Avg.Value = '0';
            app.Pressure2_EditField_Current.Value = 0;
            app.Pressure2_EditField_Avg.Value = '0';
            app.Pressure3_EditField_Current.Value = 0;
            app.Pressure3_EditField_Avg.Value = '0';
            app.Pressure_ResetAllTargetButtonPushed();
            app.Pressure_ResetAllLabelButtonPushed();
            app.Pressure_ResetAllSHOButtonPushed();

            app.Flow1_EditField_Current.Value = 0;
            app.Flow1_EditField_Avg.Value = '0';
            app.Flow2_EditField_Current.Value = 0;
            app.Flow2_EditField_Avg.Value = '0';
            app.Flow_ResetAllTargetButtonPushed();
            app.Flow_ResetAllLabelButtonPushed();

            % Close loading bar
            close(dlg);
        end

        function setGUIEnabled(app, state)
            % Gather all UI elements to enable/disable
            uiElements = [
                app.PressureDetailsPanel;
                app.PressureGraphPanel;
                app.FlowDetailsPanel;
                app.FlowGraphPanel;
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
                graphColor = app.COLOR_NORMAL;
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
            uiElements = [app.PressureDetailsPanel, ...
                app.FlowDetailsPanel, ...
                app.OptionsPanel.Children(:)', ...
                app.PIDCoefficientsPanel, ...
                app.ConnectDisconnectButton];

            for uiElement = uiElements
                if isprop(uiElement, 'Enable')
                    uiElement.Enable = stateStr;
                end
            end
        end
        
        % cutoffTime - cutoff for rolling average, to be visualized via
        %              shading on graph
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

                % Collect time values for rolling average shading
                currentTime = app.t(end);
                cutoffTime = app.rollingAverageCutoffTime;

                %% Plot pressure data
                if app.Pressure1_GraphEnable.Value
                    plot(app.PressureAxes, app.t, app.pressure_vals1, 'Color', app.COLOR_PRESSURE_2, 'DisplayName', app.pressure1_label, 'LineWidth', lineWidth);
                end
                if app.Pressure2_GraphEnable.Value
                    plot(app.PressureAxes, app.t, app.pressure_vals2, 'Color', app.COLOR_PRESSURE*0.5, 'DisplayName', app.pressure2_label, 'LineWidth', lineWidth);
                end
                if app.Pressure3_GraphEnable.Value
                    plot(app.PressureAxes, app.t, app.pressure_vals3, 'Color', [0 0 0], 'DisplayName', app.pressure3_label, 'LineWidth', lineWidth);
                end

                if contains(app.PID_setpoint_id,"Pressure")
                    plot(app.PressureAxes, app.t, ones(1,length(app.t))*app.PID_setpoint, '--r', 'DisplayName', 'Target', 'LineWidth', lineWidth)
                end

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

                % Shade area corresponding to rolling average
                ylims = ylim(app.PressureAxes);
                x_fill = [cutoffTime, currentTime, currentTime, cutoffTime];
                y_fill = [ylims(1), ylims(1), ylims(2), ylims(2)];
                fill(app.PressureAxes, x_fill, y_fill, app.COLOR_PRESSURE, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Roll. Avg.');
                hold(app.PressureAxes, 'off');

                % Axes labels and legend
                xlabel(app.PressureAxes, 'Time (s)');
                ylabel(app.PressureAxes, 'Pressure (mmHg)');
                lgdPressure = legend(app.PressureAxes, 'show','Orientation','horizontal');
                lgdPressure.Position = [0.1809    0.0426    0.6998    0.0520];
                lgdPressure.FontSize = 8;

                %% Plot flow data
                hold(app.FlowAxes, 'on');
                cla(app.FlowAxes); % Clear previous plots

                if app.Flow1_GraphEnable.Value
                    plot(app.FlowAxes, app.t, app.flow_vals1, 'Color', app.FLOW_COLOR*0.7, 'DisplayName', app.flow1_label, 'LineWidth', lineWidth);
                end
                if app.Flow2_GraphEnable.Value
                    plot(app.FlowAxes, app.t, app.flow_vals2, 'Color', [0 0 0], 'DisplayName', app.flow2_label, 'LineWidth', lineWidth);
                end

                if contains(app.PID_setpoint_id,"Flow")
                    plot(app.FlowAxes, app.t, ones(1,length(app.t))*app.PID_setpoint, '--r', 'DisplayName', 'Target', 'LineWidth', lineWidth)
                end

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

                % Shade area corresponding to rolling average
                ylims = ylim(app.FlowAxes);
                x_fill = [cutoffTime, currentTime, currentTime, cutoffTime];
                y_fill = [ylims(1), ylims(1), ylims(2), ylims(2)];
                fill(app.FlowAxes, x_fill, y_fill, app.FLOW_COLOR, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Roll. Avg.');
                hold(app.FlowAxes, 'off');

                % Axes labels and legend
                hold(app.FlowAxes, 'off');
                xlabel(app.FlowAxes, 'Time (s)');
                ylabel(app.FlowAxes, 'Flow (L/min)');
                ylim(app.FlowAxes, [0 5]);
                lgdFlow = legend(app.FlowAxes, 'show','Orientation','Horizontal');
                lgdFlow.Position = [0.2496    0.0424    0.5516    0.0520];
                lgdFlow.FontSize = 8;
            end

            pause(0.1);  % small pause to prevent CPU overuse
        end
        
        function continueSHOCalibration(app)
            app.setGUIEnabledCalibration(false);  % Disable UI to prevent interference with calibration

            % Run main loop for some time to collect values
            app.clearDataArrays();
            app.setPumpPower(0);  % static head offset is measured when the system is static (i.e. off)
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
        
        function updateSensorAverages(app)
            % Get the current time and compute the cutoff time
            currentTime = app.t(end);
            app.rollingAverageCutoffTime = max([currentTime - app.ROLLING_AVERAGE_DURATION,0]);

            % Find the index of the first value within the last N seconds
            idx = find(app.t >= app.rollingAverageCutoffTime, 1);

            % If the array is too short or cutoff not found, use entire array
            if isempty(idx)
                idx = 1;
            end

            % Compute averages and standard deviations for the last N seconds
            p1 = app.pressure_vals1(idx:end);
            p2 = app.pressure_vals2(idx:end);
            p3 = app.pressure_vals3(idx:end);
            f1 = app.flow_vals1(idx:end);
            f2 = app.flow_vals2(idx:end);

            % Calculate means and standard deviations
            avgStruct = struct( ...
                'Pressure1', [mean(p1), std(p1)], ...
                'Pressure2', [mean(p2), std(p2)], ...
                'Pressure3', [mean(p3), std(p3)], ...
                'Flow1',     [mean(f1), std(f1)], ...
                'Flow2',     [mean(f2), std(f2)] ...
                );

            % Update UI fields
            app.Pressure1_EditField_Avg.Value = sprintf("%.2f ± %.1f", avgStruct.Pressure1);
            app.Pressure2_EditField_Avg.Value = sprintf("%.2f ± %.1f", avgStruct.Pressure2);
            app.Pressure3_EditField_Avg.Value = sprintf("%.2f ± %.1f", avgStruct.Pressure3);
            app.Flow1_EditField_Avg.Value     = sprintf("%.2f ± %.1f", avgStruct.Flow1);
            app.Flow2_EditField_Avg.Value     = sprintf("%.2f ± %.1f", avgStruct.Flow2);

            % Reset all backgrounds to normal
            app.Pressure1_EditField_Avg.BackgroundColor = app.COLOR_NORMAL;
            app.Pressure2_EditField_Avg.BackgroundColor = app.COLOR_NORMAL;
            app.Pressure3_EditField_Avg.BackgroundColor = app.COLOR_NORMAL;
            app.Flow1_EditField_Avg.BackgroundColor     = app.COLOR_NORMAL;
            app.Flow2_EditField_Avg.BackgroundColor     = app.COLOR_NORMAL;

            % If no setpoint selected, skip highlight
            if ~strcmp(app.PID_setpoint_id, "None")
                if isfield(avgStruct, app.PID_setpoint_id)
                    % Get setpoint value and mean/std
                    sp = app.PID_setpoint;
                    stats = avgStruct.(app.PID_setpoint_id);
                    mu = stats(1);
                    sigma = stats(2);

                    % Check if within mean ± std
                    if sp >= (mu - sigma) && sp <= (mu + sigma)
                        color = app.COLOR_CONNECTED;
                    else
                        color = app.COLOR_WARNING;
                    end

                    % Set background color for corresponding Avg field
                    avgField = app.PID_setpoint_id + "_EditField_Avg";
                    app.(avgField).BackgroundColor = color;
                end
            end

        end

        
        function setPumpPower(app,percentPower)
            % Calculate flow duty cycle value
            app.flowDutyCycle = round(percentPower/100*app.ARDUINO_ANALOGWRITE_MAX);

            % Update UI
            app.PumpPowerSpinner.Value = percentPower;
            alpha = percentPower/100;
            app.PumpPowerSpinner.BackgroundColor = (1 - alpha) * app.COLOR_NORMAL + alpha * app.COLOR_WARNING;
            app.PumpPowerSlider.Value = percentPower;
            app.CurrentPumpAnalogWriteValueEditField.Value = app.flowDutyCycle;

            % Send pump power command to Arduino
            if ~isempty(app.arduinoObj)
                flushoutput(app.arduinoObj);  % in case this was done in the middle of sending another command
            end

            % Send command to Arduino
            if app.SimulateDataCheckBox.Value == false
                command = sprintf("PMP: %d",app.flowDutyCycle);
                writeline(app.arduinoObj,command);
                app.WaitForArduinoMessage();
            end
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
            app.UIFigure.Resize = 'off';

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
            dlg = uiprogressdlg(app.UIFigure, ...
                'Title', 'Clearing data...', ...
                'Message', 'Please wait...', ...
                'Indeterminate', 'on');

            tic;
            app.clearDataArrays();

            if toc < 1
                pause(1);  % Pause 1 second if data was cleared quickly, to give user feedback
            end

            tic;  % reset timer so graph data doesn't have a time shift
            close(dlg);

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

        % Value changed function: Flow1_EditField_Target
        function Flow1_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Flow1_EditField_Target.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Flow1");
            app.PID_setpoint_id = "Flow1";
        end

        % Value changed function: Flow2_EditField_Target
        function Flow2_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Flow2_EditField_Target.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Flow2");
            app.PID_setpoint_id = "Flow2";
        end

        % Value changed function: Pressure1_EditField_Target
        function Pressure1_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Pressure1_EditField_Target.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Pressure1");
            app.PID_setpoint_id = "Pressure1";
        end

        % Value changed function: Pressure2_EditField_Target
        function Pressure2_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Pressure2_EditField_Target.Value;
            app.setControlMode("PID");
            app.highlightSetPointUI("Pressure2");
            app.PID_setpoint_id = "Pressure2";
        end

        % Value changed function: Pressure3_EditField_Target
        function Pressure3_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Pressure3_EditField_Target.Value;
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
            app.setControlMode("Manual");
            pumpPowerPercent = app.PumpPowerSlider.Value;
            app.setPumpPower(pumpPowerPercent);
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
            app.setControlMode("Manual");
            app.setPumpPower(app.PUMP_LOW_POWER);  % to help with debubbling

            % Create non-blocking custom dialog
            %
            % Must create after setting other settings above to prevent rest of
            % code from running in the wrong order
            d = uifigure('Name', 'SHO Calibration', 'Position', [500 500 400 200]);
            d.WindowStyle = 'normal';

            uilabel(d, ...
                'Text', 'System has been set to low power and manual pump control. Please debubble the system (you may continue changing the pump power as needed), then press "Continue" to complete calibration.', ...
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

        % Button pushed function: Pressure_ResetAllTargetButton
        function Pressure_ResetAllTargetButtonPushed(app, event)
            app.PID_setpoint = [];
            app.setControlMode("Manual");
            app.highlightSetPointUI("None");
            app.PID_setpoint_id = "None";

            app.Pressure1_EditField_Target.Value = 0;
            app.Pressure2_EditField_Target.Value = 0;
            app.Pressure3_EditField_Target.Value = 0;
        end

        % Button pushed function: Flow_ResetAllTargetButton
        function Flow_ResetAllTargetButtonPushed(app, event)
            app.PID_setpoint = [];
            app.setControlMode("Manual");
            app.highlightSetPointUI("None");
            app.PID_setpoint_id = "None";

            app.Flow1_EditField_Target.Value = 0;
            app.Flow2_EditField_Target.Value = 0;
        end

        % Button pushed function: Pressure_ResetAllLabelButton
        function Pressure_ResetAllLabelButtonPushed(app, event)
            app.pressure1_label = "1";
            app.Pressure1_LabelField.Value = "1";

            app.pressure2_label = "2";
            app.Pressure2_LabelField.Value = "2";

            app.pressure3_label = "3";
            app.Pressure3_LabelField.Value = "3";
        end

        % Button pushed function: Pressure_ResetAllSHOButton
        function Pressure_ResetAllSHOButtonPushed(app, event)
            app.SHO_Pressure1 = 0;
            app.Pressure1_EditField_SHO.Value = 0;

            app.SHO_Pressure2 = 0;
            app.Pressure2_EditField_SHO.Value = 0;

            app.SHO_Pressure3 = 0;
            app.Pressure3_EditField_SHO.Value = 0;
        end

        % Button pushed function: Flow_ResetAllLabelButton
        function Flow_ResetAllLabelButtonPushed(app, event)
            app.flow1_label = "1";
            app.Flow1_LabelField.Value = "1";

            app.flow2_label = "2";
            app.Flow2_LabelField.Value = "2";
        end

        % Value changed function: PauseGraphsButton
        function PauseGraphsButtonValueChanged(app, event)
            value = app.PauseGraphsButton.Value;
            
            if value == true
                app.PauseGraphsButton.BackgroundColor = app.COLOR_DISCONNECTED;
                app.PauseGraphsButton.Text = "Unpause Graphs";
                app.PauseGraphsButton.Icon = "assets/play-button-arrowhead.png";
            else
                app.PauseGraphsButton.BackgroundColor = app.BUTTON_COLOR1;
                app.PauseGraphsButton.Text = "Pause Graphs";
                app.PauseGraphsButton.Icon = "assets/pause_icon.png";
            end
        end

        % Callback function: not associated with a component
        function STOPPUMPButtonValueChanged(app, event)
            value = app.STOPPUMPButton.Value;
            
            if value
                app.flowDutyCycle = 0;
                app.CurrentPumpAnalogWriteValueEditField.Value = app.flowDutyCycle;
                app.setControlMode("Manual");
                app.pumpStopped = true;
            else
                app.pumpStopped = false;
            end
        end

        % Value changed function: RollingaveragedurationEditField
        function RollingaveragedurationEditFieldValueChanged(app, event)
            app.ROLLING_AVERAGE_DURATION = app.RollingaveragedurationEditField.Value;
            app.dimEditFieldTemporarily(app.RollingaveragedurationEditField);
        end

        % Value changed function: PumpPowerSpinner
        function PumpPowerSpinnerValueChanged(app, event)
            pumpPowerPercent = app.PumpPowerSpinner.Value;
            app.flowDutyCycle = round(pumpPowerPercent/100*app.ARDUINO_ANALOGWRITE_MAX);
            app.CurrentPumpAnalogWriteValueEditField.Value = app.flowDutyCycle;
            app.PumpPowerSlider.Value = pumpPowerPercent;
            app.setControlMode("Manual");

            if ~isempty(app.arduinoObj)
                flushoutput(app.arduinoObj);  % in case this was done in the middle of sending another command
            end

            app.sendControlValuesToArduino();
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
            app.UIFigure.Position = [100 100 1240 564];
            app.UIFigure.Name = 'MATLAB App';

            % Create MainTabGroup
            app.MainTabGroup = uitabgroup(app.UIFigure);
            app.MainTabGroup.Position = [1 1 1240 564];

            % Create MainTab
            app.MainTab = uitab(app.MainTabGroup);
            app.MainTab.Title = 'Main';

            % Create PressureDetailsPanel
            app.PressureDetailsPanel = uipanel(app.MainTab);
            app.PressureDetailsPanel.Enable = 'off';
            app.PressureDetailsPanel.BackgroundColor = [0.8392 0.8275 0.8588];
            app.PressureDetailsPanel.FontWeight = 'bold';
            app.PressureDetailsPanel.Position = [329 12 435 150];

            % Create Pressure1_EditField_Target
            app.Pressure1_EditField_Target = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure1_EditField_Target.Limits = [0 Inf];
            app.Pressure1_EditField_Target.ValueChangedFcn = createCallbackFcn(app, @Pressure1_EditField_TargetValueChanged, true);
            app.Pressure1_EditField_Target.HorizontalAlignment = 'center';
            app.Pressure1_EditField_Target.Tooltip = {'Target value for PID control'};
            app.Pressure1_EditField_Target.Position = [121 91 44 22];

            % Create Pressure2_EditField_Target
            app.Pressure2_EditField_Target = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure2_EditField_Target.Limits = [0 Inf];
            app.Pressure2_EditField_Target.ValueChangedFcn = createCallbackFcn(app, @Pressure2_EditField_TargetValueChanged, true);
            app.Pressure2_EditField_Target.HorizontalAlignment = 'center';
            app.Pressure2_EditField_Target.Tooltip = {'Target value for PID control'};
            app.Pressure2_EditField_Target.Position = [121 61 44 22];

            % Create Pressure3_EditField_Target
            app.Pressure3_EditField_Target = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure3_EditField_Target.Limits = [0 Inf];
            app.Pressure3_EditField_Target.ValueChangedFcn = createCallbackFcn(app, @Pressure3_EditField_TargetValueChanged, true);
            app.Pressure3_EditField_Target.HorizontalAlignment = 'center';
            app.Pressure3_EditField_Target.Tooltip = {'Target value for PID control'};
            app.Pressure3_EditField_Target.Position = [121 29 44 22];

            % Create CurrentLabel
            app.CurrentLabel = uilabel(app.PressureDetailsPanel);
            app.CurrentLabel.Position = [70 117 42 22];
            app.CurrentLabel.Text = 'Current';

            % Create TargetLabel
            app.TargetLabel = uilabel(app.PressureDetailsPanel);
            app.TargetLabel.HorizontalAlignment = 'center';
            app.TargetLabel.Position = [121 117 44 22];
            app.TargetLabel.Text = 'Target';

            % Create Label_2
            app.Label_2 = uilabel(app.PressureDetailsPanel);
            app.Label_2.HorizontalAlignment = 'center';
            app.Label_2.FontWeight = 'bold';
            app.Label_2.Position = [26 92 25 22];
            app.Label_2.Text = '1';

            % Create Pressure1_EditField_Current
            app.Pressure1_EditField_Current = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure1_EditField_Current.Editable = 'off';
            app.Pressure1_EditField_Current.HorizontalAlignment = 'center';
            app.Pressure1_EditField_Current.Tooltip = {'Current pressure reading (mmHg) for pressure sensor 1 (see label on sensor).'};
            app.Pressure1_EditField_Current.Position = [69 91 44 22];

            % Create Label_3
            app.Label_3 = uilabel(app.PressureDetailsPanel);
            app.Label_3.HorizontalAlignment = 'center';
            app.Label_3.FontWeight = 'bold';
            app.Label_3.Position = [26 61 25 22];
            app.Label_3.Text = '2';

            % Create Pressure2_EditField_Current
            app.Pressure2_EditField_Current = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure2_EditField_Current.Editable = 'off';
            app.Pressure2_EditField_Current.HorizontalAlignment = 'center';
            app.Pressure2_EditField_Current.Tooltip = {'Current pressure reading (mmHg) for pressure sensor 2 (see label on sensor).'};
            app.Pressure2_EditField_Current.Position = [70 62 43 22];

            % Create Label_4
            app.Label_4 = uilabel(app.PressureDetailsPanel);
            app.Label_4.HorizontalAlignment = 'center';
            app.Label_4.FontWeight = 'bold';
            app.Label_4.Position = [26 30 25 22];
            app.Label_4.Text = '3';

            % Create Pressure3_EditField_Current
            app.Pressure3_EditField_Current = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure3_EditField_Current.Editable = 'off';
            app.Pressure3_EditField_Current.HorizontalAlignment = 'center';
            app.Pressure3_EditField_Current.Tooltip = {'Current pressure reading (mmHg) for pressure sensor 3 (see label on sensor).'};
            app.Pressure3_EditField_Current.Position = [70 30 43 22];

            % Create SensorLabel
            app.SensorLabel = uilabel(app.PressureDetailsPanel);
            app.SensorLabel.HorizontalAlignment = 'center';
            app.SensorLabel.FontWeight = 'bold';
            app.SensorLabel.Position = [16 117 45 22];
            app.SensorLabel.Text = 'Sensor';

            % Create Pressure3_GraphEnable
            app.Pressure3_GraphEnable = uicheckbox(app.PressureDetailsPanel);
            app.Pressure3_GraphEnable.Text = '';
            app.Pressure3_GraphEnable.Position = [398 28 14 22];
            app.Pressure3_GraphEnable.Value = true;

            % Create Pressure2_GraphEnable
            app.Pressure2_GraphEnable = uicheckbox(app.PressureDetailsPanel);
            app.Pressure2_GraphEnable.Text = '';
            app.Pressure2_GraphEnable.Position = [398 59 16 22];
            app.Pressure2_GraphEnable.Value = true;

            % Create Pressure1_GraphEnable
            app.Pressure1_GraphEnable = uicheckbox(app.PressureDetailsPanel);
            app.Pressure1_GraphEnable.Text = '';
            app.Pressure1_GraphEnable.Position = [398 91 16 22];
            app.Pressure1_GraphEnable.Value = true;

            % Create Label_5
            app.Label_5 = uilabel(app.PressureDetailsPanel);
            app.Label_5.HorizontalAlignment = 'center';
            app.Label_5.Position = [268 117 63 22];

            % Create Pressure1_LabelField
            app.Pressure1_LabelField = uieditfield(app.PressureDetailsPanel, 'text');
            app.Pressure1_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure1_LabelFieldValueChanged, true);
            app.Pressure1_LabelField.HorizontalAlignment = 'center';
            app.Pressure1_LabelField.FontAngle = 'italic';
            app.Pressure1_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure1_LabelField.Position = [268 91 63 22];
            app.Pressure1_LabelField.Value = '1';

            % Create Pressure2_LabelField
            app.Pressure2_LabelField = uieditfield(app.PressureDetailsPanel, 'text');
            app.Pressure2_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure2_LabelFieldValueChanged, true);
            app.Pressure2_LabelField.HorizontalAlignment = 'center';
            app.Pressure2_LabelField.FontAngle = 'italic';
            app.Pressure2_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure2_LabelField.Position = [269 60 62 22];
            app.Pressure2_LabelField.Value = '2';

            % Create Pressure3_LabelField
            app.Pressure3_LabelField = uieditfield(app.PressureDetailsPanel, 'text');
            app.Pressure3_LabelField.ValueChangedFcn = createCallbackFcn(app, @Pressure3_LabelFieldValueChanged, true);
            app.Pressure3_LabelField.HorizontalAlignment = 'center';
            app.Pressure3_LabelField.FontAngle = 'italic';
            app.Pressure3_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure3_LabelField.Position = [269 29 62 22];
            app.Pressure3_LabelField.Value = '3';

            % Create Label_11
            app.Label_11 = uilabel(app.PressureDetailsPanel);
            app.Label_11.HorizontalAlignment = 'center';
            app.Label_11.FontSize = 18;
            app.Label_11.Position = [391 117 30 23];
            app.Label_11.Text = '👁️';

            % Create SHOLabel_Pressure
            app.SHOLabel_Pressure = uilabel(app.PressureDetailsPanel);
            app.SHOLabel_Pressure.HorizontalAlignment = 'center';
            app.SHOLabel_Pressure.Tooltip = {'SHO - Static Head Offset. Adjusts for pressure changes caused by fluid height differences in the system.'};
            app.SHOLabel_Pressure.Position = [340 117 43 22];
            app.SHOLabel_Pressure.Text = 'SHO';

            % Create Pressure1_EditField_SHO
            app.Pressure1_EditField_SHO = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure1_EditField_SHO.ValueChangedFcn = createCallbackFcn(app, @Pressure1_EditField_SHOValueChanged, true);
            app.Pressure1_EditField_SHO.HorizontalAlignment = 'center';
            app.Pressure1_EditField_SHO.Tooltip = {'SHO - Static Head Offset. Adjusts for pressure changes caused by fluid height differences in the system.'};
            app.Pressure1_EditField_SHO.Position = [340 91 43 22];

            % Create Pressure2_EditField_SHO
            app.Pressure2_EditField_SHO = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure2_EditField_SHO.ValueChangedFcn = createCallbackFcn(app, @Pressure2_EditField_SHOValueChanged2, true);
            app.Pressure2_EditField_SHO.HorizontalAlignment = 'center';
            app.Pressure2_EditField_SHO.Tooltip = {'SHO - Static Head Offset. Adjusts for pressure changes caused by fluid height differences in the system.'};
            app.Pressure2_EditField_SHO.Position = [340 62 43 22];

            % Create Pressure3_EditField_SHO
            app.Pressure3_EditField_SHO = uieditfield(app.PressureDetailsPanel, 'numeric');
            app.Pressure3_EditField_SHO.ValueChangedFcn = createCallbackFcn(app, @Pressure3_EditField_SHOValueChanged2, true);
            app.Pressure3_EditField_SHO.HorizontalAlignment = 'center';
            app.Pressure3_EditField_SHO.Tooltip = {'SHO - Static Head Offset. Adjusts for pressure changes caused by fluid height differences in the system.'};
            app.Pressure3_EditField_SHO.Position = [340 30 43 22];

            % Create RollAvgSDLabel
            app.RollAvgSDLabel = uilabel(app.PressureDetailsPanel);
            app.RollAvgSDLabel.HorizontalAlignment = 'center';
            app.RollAvgSDLabel.Position = [174 117 85 22];
            app.RollAvgSDLabel.Text = 'Roll. Avg ± SD';

            % Create Pressure1_EditField_Avg
            app.Pressure1_EditField_Avg = uieditfield(app.PressureDetailsPanel, 'text');
            app.Pressure1_EditField_Avg.Editable = 'off';
            app.Pressure1_EditField_Avg.HorizontalAlignment = 'center';
            app.Pressure1_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure1_EditField_Avg.Position = [174 91 85 22];
            app.Pressure1_EditField_Avg.Value = '0';

            % Create Pressure2_EditField_Avg
            app.Pressure2_EditField_Avg = uieditfield(app.PressureDetailsPanel, 'text');
            app.Pressure2_EditField_Avg.Editable = 'off';
            app.Pressure2_EditField_Avg.HorizontalAlignment = 'center';
            app.Pressure2_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure2_EditField_Avg.Position = [174 59 85 22];
            app.Pressure2_EditField_Avg.Value = '0';

            % Create Pressure3_EditField_Avg
            app.Pressure3_EditField_Avg = uieditfield(app.PressureDetailsPanel, 'text');
            app.Pressure3_EditField_Avg.Editable = 'off';
            app.Pressure3_EditField_Avg.HorizontalAlignment = 'center';
            app.Pressure3_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Pressure3_EditField_Avg.Position = [174 29 85 22];
            app.Pressure3_EditField_Avg.Value = '0';

            % Create Pressure_ResetAllTargetButton
            app.Pressure_ResetAllTargetButton = uibutton(app.PressureDetailsPanel, 'push');
            app.Pressure_ResetAllTargetButton.ButtonPushedFcn = createCallbackFcn(app, @Pressure_ResetAllTargetButtonPushed, true);
            app.Pressure_ResetAllTargetButton.BackgroundColor = [0.902 0.902 0.902];
            app.Pressure_ResetAllTargetButton.FontSize = 8;
            app.Pressure_ResetAllTargetButton.Position = [120 8 47 15];
            app.Pressure_ResetAllTargetButton.Text = 'Reset All';

            % Create Pressure_ResetAllLabelButton
            app.Pressure_ResetAllLabelButton = uibutton(app.PressureDetailsPanel, 'push');
            app.Pressure_ResetAllLabelButton.ButtonPushedFcn = createCallbackFcn(app, @Pressure_ResetAllLabelButtonPushed, true);
            app.Pressure_ResetAllLabelButton.BackgroundColor = [0.902 0.902 0.902];
            app.Pressure_ResetAllLabelButton.FontSize = 8;
            app.Pressure_ResetAllLabelButton.Position = [269 8 62 15];
            app.Pressure_ResetAllLabelButton.Text = 'Reset All';

            % Create Pressure_ResetAllSHOButton
            app.Pressure_ResetAllSHOButton = uibutton(app.PressureDetailsPanel, 'push');
            app.Pressure_ResetAllSHOButton.ButtonPushedFcn = createCallbackFcn(app, @Pressure_ResetAllSHOButtonPushed, true);
            app.Pressure_ResetAllSHOButton.BackgroundColor = [0.902 0.902 0.902];
            app.Pressure_ResetAllSHOButton.FontSize = 8;
            app.Pressure_ResetAllSHOButton.Position = [340 8 43 15];
            app.Pressure_ResetAllSHOButton.Text = 'Reset All';

            % Create FlowDetailsPanel
            app.FlowDetailsPanel = uipanel(app.MainTab);
            app.FlowDetailsPanel.Enable = 'off';
            app.FlowDetailsPanel.BackgroundColor = [0.7686 0.8314 0.7608];
            app.FlowDetailsPanel.FontWeight = 'bold';
            app.FlowDetailsPanel.Position = [775 50 448 112];

            % Create Label_6
            app.Label_6 = uilabel(app.FlowDetailsPanel);
            app.Label_6.FontSize = 18;
            app.Label_6.FontWeight = 'bold';
            app.Label_6.Position = [110 60 25 23];
            app.Label_6.Text = '';

            % Create Flow1_EditField_Target
            app.Flow1_EditField_Target = uieditfield(app.FlowDetailsPanel, 'numeric');
            app.Flow1_EditField_Target.Limits = [0 Inf];
            app.Flow1_EditField_Target.ValueChangedFcn = createCallbackFcn(app, @Flow1_EditField_TargetValueChanged, true);
            app.Flow1_EditField_Target.HorizontalAlignment = 'center';
            app.Flow1_EditField_Target.Tooltip = {'Target value for PID control'};
            app.Flow1_EditField_Target.Position = [161 59 44 22];

            % Create Flow2_EditField_Target
            app.Flow2_EditField_Target = uieditfield(app.FlowDetailsPanel, 'numeric');
            app.Flow2_EditField_Target.Limits = [0 Inf];
            app.Flow2_EditField_Target.ValueChangedFcn = createCallbackFcn(app, @Flow2_EditField_TargetValueChanged, true);
            app.Flow2_EditField_Target.HorizontalAlignment = 'center';
            app.Flow2_EditField_Target.Tooltip = {'Target value for PID control'};
            app.Flow2_EditField_Target.Position = [161 28 44 22];

            % Create CurrentLabel_2
            app.CurrentLabel_2 = uilabel(app.FlowDetailsPanel);
            app.CurrentLabel_2.Position = [106 81 45 22];
            app.CurrentLabel_2.Text = 'Current';

            % Create TargetLabel_2
            app.TargetLabel_2 = uilabel(app.FlowDetailsPanel);
            app.TargetLabel_2.HorizontalAlignment = 'center';
            app.TargetLabel_2.Position = [161 81 44 22];
            app.TargetLabel_2.Text = 'Target';

            % Create Label_7
            app.Label_7 = uilabel(app.FlowDetailsPanel);
            app.Label_7.HorizontalAlignment = 'center';
            app.Label_7.FontWeight = 'bold';
            app.Label_7.Position = [60 60 25 22];
            app.Label_7.Text = '1';

            % Create Flow1_EditField_Current
            app.Flow1_EditField_Current = uieditfield(app.FlowDetailsPanel, 'numeric');
            app.Flow1_EditField_Current.Editable = 'off';
            app.Flow1_EditField_Current.HorizontalAlignment = 'center';
            app.Flow1_EditField_Current.Tooltip = {'Current flow reading (L/min) for flow sensor 1 (see label on sensor).'};
            app.Flow1_EditField_Current.Position = [107 59 43 22];

            % Create Label_8
            app.Label_8 = uilabel(app.FlowDetailsPanel);
            app.Label_8.HorizontalAlignment = 'center';
            app.Label_8.FontWeight = 'bold';
            app.Label_8.Position = [60 28 25 22];
            app.Label_8.Text = '2';

            % Create Flow2_EditField_Current
            app.Flow2_EditField_Current = uieditfield(app.FlowDetailsPanel, 'numeric');
            app.Flow2_EditField_Current.Editable = 'off';
            app.Flow2_EditField_Current.HorizontalAlignment = 'center';
            app.Flow2_EditField_Current.Tooltip = {'Current flow reading (L/min) for flow sensor 2 (see label on sensor).'};
            app.Flow2_EditField_Current.Position = [107 28 43 22];

            % Create Flow2_GraphEnable
            app.Flow2_GraphEnable = uicheckbox(app.FlowDetailsPanel);
            app.Flow2_GraphEnable.Text = '';
            app.Flow2_GraphEnable.Position = [393 30 15 22];
            app.Flow2_GraphEnable.Value = true;

            % Create Flow1_GraphEnable
            app.Flow1_GraphEnable = uicheckbox(app.FlowDetailsPanel);
            app.Flow1_GraphEnable.Text = '';
            app.Flow1_GraphEnable.Position = [393 59 14 22];
            app.Flow1_GraphEnable.Value = true;

            % Create Label_10
            app.Label_10 = uilabel(app.FlowDetailsPanel);
            app.Label_10.HorizontalAlignment = 'center';
            app.Label_10.FontSize = 18;
            app.Label_10.Position = [385 83 30 23];
            app.Label_10.Text = '👁️';

            % Create SensorLabel_2
            app.SensorLabel_2 = uilabel(app.FlowDetailsPanel);
            app.SensorLabel_2.HorizontalAlignment = 'center';
            app.SensorLabel_2.FontWeight = 'bold';
            app.SensorLabel_2.Position = [50 82 45 22];
            app.SensorLabel_2.Text = 'Sensor';

            % Create Flow2_LabelField
            app.Flow2_LabelField = uieditfield(app.FlowDetailsPanel, 'text');
            app.Flow2_LabelField.ValueChangedFcn = createCallbackFcn(app, @Flow2_LabelFieldValueChanged, true);
            app.Flow2_LabelField.HorizontalAlignment = 'center';
            app.Flow2_LabelField.FontAngle = 'italic';
            app.Flow2_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow2_LabelField.Position = [312 30 63 22];
            app.Flow2_LabelField.Value = '2';

            % Create Label_9
            app.Label_9 = uilabel(app.FlowDetailsPanel);
            app.Label_9.HorizontalAlignment = 'center';
            app.Label_9.Position = [312 83 63 22];

            % Create Flow1_LabelField
            app.Flow1_LabelField = uieditfield(app.FlowDetailsPanel, 'text');
            app.Flow1_LabelField.ValueChangedFcn = createCallbackFcn(app, @Flow1_LabelFieldValueChanged, true);
            app.Flow1_LabelField.HorizontalAlignment = 'center';
            app.Flow1_LabelField.FontAngle = 'italic';
            app.Flow1_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow1_LabelField.Position = [312 60 63 22];
            app.Flow1_LabelField.Value = '1';

            % Create RollAvgSDLabel_2
            app.RollAvgSDLabel_2 = uilabel(app.FlowDetailsPanel);
            app.RollAvgSDLabel_2.HorizontalAlignment = 'center';
            app.RollAvgSDLabel_2.Position = [219 81 82 22];
            app.RollAvgSDLabel_2.Text = 'Roll. Avg ± SD';

            % Create Flow2_EditField_Avg
            app.Flow2_EditField_Avg = uieditfield(app.FlowDetailsPanel, 'text');
            app.Flow2_EditField_Avg.Editable = 'off';
            app.Flow2_EditField_Avg.HorizontalAlignment = 'center';
            app.Flow2_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow2_EditField_Avg.Position = [217 28 85 22];
            app.Flow2_EditField_Avg.Value = '0';

            % Create Flow_ResetAllTargetButton
            app.Flow_ResetAllTargetButton = uibutton(app.FlowDetailsPanel, 'push');
            app.Flow_ResetAllTargetButton.ButtonPushedFcn = createCallbackFcn(app, @Flow_ResetAllTargetButtonPushed, true);
            app.Flow_ResetAllTargetButton.BackgroundColor = [0.902 0.902 0.902];
            app.Flow_ResetAllTargetButton.FontSize = 8;
            app.Flow_ResetAllTargetButton.Position = [159 7 47 15];
            app.Flow_ResetAllTargetButton.Text = 'Reset All';

            % Create Flow_ResetAllLabelButton
            app.Flow_ResetAllLabelButton = uibutton(app.FlowDetailsPanel, 'push');
            app.Flow_ResetAllLabelButton.ButtonPushedFcn = createCallbackFcn(app, @Flow_ResetAllLabelButtonPushed, true);
            app.Flow_ResetAllLabelButton.BackgroundColor = [0.902 0.902 0.902];
            app.Flow_ResetAllLabelButton.FontSize = 8;
            app.Flow_ResetAllLabelButton.Position = [313 9 62 15];
            app.Flow_ResetAllLabelButton.Text = 'Reset All';

            % Create Flow1_EditField_Avg
            app.Flow1_EditField_Avg = uieditfield(app.FlowDetailsPanel, 'text');
            app.Flow1_EditField_Avg.Editable = 'off';
            app.Flow1_EditField_Avg.HorizontalAlignment = 'center';
            app.Flow1_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow1_EditField_Avg.Position = [217 60 85 22];
            app.Flow1_EditField_Avg.Value = '0';

            % Create PressureGraphPanel
            app.PressureGraphPanel = uipanel(app.MainTab);
            app.PressureGraphPanel.BorderColor = [0.4902 0.4902 0.4902];
            app.PressureGraphPanel.Enable = 'off';
            app.PressureGraphPanel.BackgroundColor = [0.8392 0.8314 0.8588];
            app.PressureGraphPanel.FontAngle = 'italic';
            app.PressureGraphPanel.Position = [330 168 435 329];

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
            app.PressureAxes.Position = [13 44 409 273];

            % Create PressureAxesWaitingForConnectionLabel
            app.PressureAxesWaitingForConnectionLabel = uilabel(app.PressureGraphPanel);
            app.PressureAxesWaitingForConnectionLabel.HorizontalAlignment = 'center';
            app.PressureAxesWaitingForConnectionLabel.FontSize = 36;
            app.PressureAxesWaitingForConnectionLabel.FontWeight = 'bold';
            app.PressureAxesWaitingForConnectionLabel.FontColor = [0.149 0.149 0.149];
            app.PressureAxesWaitingForConnectionLabel.Enable = 'off';
            app.PressureAxesWaitingForConnectionLabel.Position = [99 163 258 47];
            app.PressureAxesWaitingForConnectionLabel.Text = 'No connection';

            % Create PumpControlPanel
            app.PumpControlPanel = uipanel(app.MainTab);
            app.PumpControlPanel.Enable = 'off';
            app.PumpControlPanel.Title = 'Pump Control';
            app.PumpControlPanel.BackgroundColor = [0.902 0.902 0.902];
            app.PumpControlPanel.FontWeight = 'bold';
            app.PumpControlPanel.Position = [17 153 302 186];

            % Create PumpPowerSliderLabel
            app.PumpPowerSliderLabel = uilabel(app.PumpControlPanel);
            app.PumpPowerSliderLabel.HorizontalAlignment = 'right';
            app.PumpPowerSliderLabel.FontWeight = 'bold';
            app.PumpPowerSliderLabel.Position = [10 93 95 22];
            app.PumpPowerSliderLabel.Text = 'Pump Power  %';

            % Create PumpPowerSlider
            app.PumpPowerSlider = uislider(app.PumpControlPanel);
            app.PumpPowerSlider.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSliderValueChanged, true);
            app.PumpPowerSlider.Tooltip = {'Sends a PWM signal to the pump from 0 to 100% power. 100% power represents the full voltage of your power supply.'};
            app.PumpPowerSlider.Position = [126 102 150 3];

            % Create ControlModeSwitchLabel
            app.ControlModeSwitchLabel = uilabel(app.PumpControlPanel);
            app.ControlModeSwitchLabel.HorizontalAlignment = 'center';
            app.ControlModeSwitchLabel.FontWeight = 'bold';
            app.ControlModeSwitchLabel.Position = [15 127 82 22];
            app.ControlModeSwitchLabel.Text = 'Control Mode';

            % Create ControlModeSwitch
            app.ControlModeSwitch = uiswitch(app.PumpControlPanel, 'slider');
            app.ControlModeSwitch.Items = {'Manual', 'Auto'};
            app.ControlModeSwitch.ValueChangedFcn = createCallbackFcn(app, @ControlModeSwitchValueChanged, true);
            app.ControlModeSwitch.Tooltip = {'Manual - control pump power using the slider below'; ''; 'Auto - Input a flow or pressure target value in the appropriate box underneath the graphs, and the system will automatically adjust'};
            app.ControlModeSwitch.Position = [192 128 45 20];
            app.ControlModeSwitch.Value = 'Manual';

            % Create StartPulsatileFlowButton
            app.StartPulsatileFlowButton = uibutton(app.PumpControlPanel, 'state');
            app.StartPulsatileFlowButton.ValueChangedFcn = createCallbackFcn(app, @StartPulsatileFlowButtonValueChanged, true);
            app.StartPulsatileFlowButton.Enable = 'off';
            app.StartPulsatileFlowButton.Text = 'Start Pulsatile Flow';
            app.StartPulsatileFlowButton.Position = [170 8 118 23];

            % Create PumpPowerSpinner
            app.PumpPowerSpinner = uispinner(app.PumpControlPanel);
            app.PumpPowerSpinner.RoundFractionalValues = 'on';
            app.PumpPowerSpinner.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSpinnerValueChanged, true);
            app.PumpPowerSpinner.Position = [15 76 92 17];

            % Create FlowGraphPanel
            app.FlowGraphPanel = uipanel(app.MainTab);
            app.FlowGraphPanel.Enable = 'off';
            app.FlowGraphPanel.BackgroundColor = [0.7725 0.8314 0.7569];
            app.FlowGraphPanel.Position = [775 168 448 329];

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
            app.FlowAxes.Position = [15 44 419 275];

            % Create FlowAxesWaitingForConnectionLabel
            app.FlowAxesWaitingForConnectionLabel = uilabel(app.FlowGraphPanel);
            app.FlowAxesWaitingForConnectionLabel.HorizontalAlignment = 'center';
            app.FlowAxesWaitingForConnectionLabel.FontSize = 36;
            app.FlowAxesWaitingForConnectionLabel.FontWeight = 'bold';
            app.FlowAxesWaitingForConnectionLabel.FontColor = [0.149 0.149 0.149];
            app.FlowAxesWaitingForConnectionLabel.Enable = 'off';
            app.FlowAxesWaitingForConnectionLabel.Position = [117 163 258 47];
            app.FlowAxesWaitingForConnectionLabel.Text = 'No connection';

            % Create OptionsPanel
            app.OptionsPanel = uipanel(app.MainTab);
            app.OptionsPanel.BorderWidth = 0;
            app.OptionsPanel.Position = [13 346 305 88];

            % Create PauseGraphsButton
            app.PauseGraphsButton = uibutton(app.OptionsPanel, 'state');
            app.PauseGraphsButton.ValueChangedFcn = createCallbackFcn(app, @PauseGraphsButtonValueChanged, true);
            app.PauseGraphsButton.Enable = 'off';
            app.PauseGraphsButton.Tooltip = {'Pause graphs (useful if you want to zoom into graphs and interact with them, since that does not work well while the graphs are live).'};
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
            app.ConnectionDropDownLabel.Position = [18 502 104 23];
            app.ConnectionDropDownLabel.Text = 'Connection';

            % Create ConnectionDropDown
            app.ConnectionDropDown = uidropdown(app.MainTab);
            app.ConnectionDropDown.Items = {};
            app.ConnectionDropDown.FontSize = 14;
            app.ConnectionDropDown.FontWeight = 'bold';
            app.ConnectionDropDown.FontColor = [0.149 0.149 0.149];
            app.ConnectionDropDown.BackgroundColor = [0.9412 0.9412 0.9412];
            app.ConnectionDropDown.Position = [17 467 126 31];
            app.ConnectionDropDown.Value = {};

            % Create ConnectedLamp
            app.ConnectedLamp = uilamp(app.MainTab);
            app.ConnectedLamp.Position = [295 473 20 20];
            app.ConnectedLamp.Color = [1 0 0];

            % Create RefreshConnectionsButton
            app.RefreshConnectionsButton = uibutton(app.MainTab, 'push');
            app.RefreshConnectionsButton.ButtonPushedFcn = createCallbackFcn(app, @RefreshConnectionsButtonPushed, true);
            app.RefreshConnectionsButton.Icon = fullfile(pathToMLAPP, 'assets', 'Refresh_icon.png');
            app.RefreshConnectionsButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RefreshConnectionsButton.FontSize = 14;
            app.RefreshConnectionsButton.FontWeight = 'bold';
            app.RefreshConnectionsButton.FontColor = [0.149 0.149 0.149];
            app.RefreshConnectionsButton.Position = [255 468 31 32];
            app.RefreshConnectionsButton.Text = '';

            % Create ConnectDisconnectButton
            app.ConnectDisconnectButton = uibutton(app.MainTab, 'push');
            app.ConnectDisconnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectDisconnectButtonPushed, true);
            app.ConnectDisconnectButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.ConnectDisconnectButton.FontSize = 14;
            app.ConnectDisconnectButton.FontWeight = 'bold';
            app.ConnectDisconnectButton.FontColor = [0.149 0.149 0.149];
            app.ConnectDisconnectButton.Enable = 'off';
            app.ConnectDisconnectButton.Position = [149 468 98 31];
            app.ConnectDisconnectButton.Text = 'Connect';

            % Create OptionsLabel
            app.OptionsLabel = uilabel(app.MainTab);
            app.OptionsLabel.FontSize = 18;
            app.OptionsLabel.FontWeight = 'bold';
            app.OptionsLabel.FontColor = [0.149 0.149 0.149];
            app.OptionsLabel.Enable = 'off';
            app.OptionsLabel.Position = [20 428 73 23];
            app.OptionsLabel.Text = 'Options';

            % Create PressureDataTitleLabelPanel
            app.PressureDataTitleLabelPanel = uipanel(app.MainTab);
            app.PressureDataTitleLabelPanel.Enable = 'off';
            app.PressureDataTitleLabelPanel.Position = [330 502 435 30];

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
            app.FlowDataTitleLabelPanel.Position = [775 502 448 30];

            % Create FlowDataTitleLabel
            app.FlowDataTitleLabel = uilabel(app.FlowDataTitleLabelPanel);
            app.FlowDataTitleLabel.HorizontalAlignment = 'center';
            app.FlowDataTitleLabel.FontSize = 18;
            app.FlowDataTitleLabel.FontWeight = 'bold';
            app.FlowDataTitleLabel.FontColor = [0.149 0.149 0.149];
            app.FlowDataTitleLabel.Position = [0 4 447 23];
            app.FlowDataTitleLabel.Text = 'Flow Data (L/min)';

            % Create SettingsTab
            app.SettingsTab = uitab(app.MainTabGroup);
            app.SettingsTab.Title = 'Settings';

            % Create PIDCoefficientsPanel
            app.PIDCoefficientsPanel = uipanel(app.SettingsTab);
            app.PIDCoefficientsPanel.Enable = 'off';
            app.PIDCoefficientsPanel.Title = 'PID Coefficients';
            app.PIDCoefficientsPanel.FontWeight = 'bold';
            app.PIDCoefficientsPanel.Position = [17 383 469 139];

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

            % Create DeveloperToolsPanel
            app.DeveloperToolsPanel = uipanel(app.SettingsTab);
            app.DeveloperToolsPanel.Title = 'Developer Tools';
            app.DeveloperToolsPanel.FontWeight = 'bold';
            app.DeveloperToolsPanel.Position = [499 383 260 140];

            % Create SimulateDataCheckBox
            app.SimulateDataCheckBox = uicheckbox(app.DeveloperToolsPanel);
            app.SimulateDataCheckBox.ValueChangedFcn = createCallbackFcn(app, @SimulateDataCheckBoxValueChanged, true);
            app.SimulateDataCheckBox.Tooltip = {'Simulates data without requiring a connection to a microcontroller. Mostly useful for testing that the GUI works. '; ''; 'To simulate, check the checkbox then go to the ''Main'' tab and click ''Connect''. Don''t worry about what device is listed on the dropdown there.'};
            app.SimulateDataCheckBox.Text = 'Simulate Data?';
            app.SimulateDataCheckBox.Position = [14 88 160 22];

            % Create CurrentPumpAnalogWriteValue0255Label
            app.CurrentPumpAnalogWriteValue0255Label = uilabel(app.SettingsTab);
            app.CurrentPumpAnalogWriteValue0255Label.HorizontalAlignment = 'right';
            app.CurrentPumpAnalogWriteValue0255Label.FontWeight = 'bold';
            app.CurrentPumpAnalogWriteValue0255Label.Position = [513 421 117 30];
            app.CurrentPumpAnalogWriteValue0255Label.Text = {'Current Pump '; 'Analog Write Value '};

            % Create CurrentPumpAnalogWriteValueEditField
            app.CurrentPumpAnalogWriteValueEditField = uieditfield(app.SettingsTab, 'numeric');
            app.CurrentPumpAnalogWriteValueEditField.Limits = [0 255];
            app.CurrentPumpAnalogWriteValueEditField.RoundFractionalValues = 'on';
            app.CurrentPumpAnalogWriteValueEditField.ValueChangedFcn = createCallbackFcn(app, @CurrentPumpAnalogWriteValueEditFieldValueChanged, true);
            app.CurrentPumpAnalogWriteValueEditField.Editable = 'off';
            app.CurrentPumpAnalogWriteValueEditField.Position = [645 425 96 22];

            % Create OtherSettingsPanel
            app.OtherSettingsPanel = uipanel(app.SettingsTab);
            app.OtherSettingsPanel.Title = 'Other Settings';
            app.OtherSettingsPanel.FontWeight = 'bold';
            app.OtherSettingsPanel.Position = [17 246 469 124];

            % Create CalibrationdurationsEditFieldLabel
            app.CalibrationdurationsEditFieldLabel = uilabel(app.OtherSettingsPanel);
            app.CalibrationdurationsEditFieldLabel.HorizontalAlignment = 'right';
            app.CalibrationdurationsEditFieldLabel.Position = [116 64 126 22];
            app.CalibrationdurationsEditFieldLabel.Text = 'Calibration duration (s)';

            % Create CalibrationdurationsEditField
            app.CalibrationdurationsEditField = uieditfield(app.OtherSettingsPanel, 'numeric');
            app.CalibrationdurationsEditField.Limits = [0 Inf];
            app.CalibrationdurationsEditField.ValueChangedFcn = createCallbackFcn(app, @CalibrationdurationsEditFieldValueChanged, true);
            app.CalibrationdurationsEditField.Position = [257 64 100 22];
            app.CalibrationdurationsEditField.Value = 30;

            % Create RollingaveragedurationEditFieldLabel
            app.RollingaveragedurationEditFieldLabel = uilabel(app.OtherSettingsPanel);
            app.RollingaveragedurationEditFieldLabel.HorizontalAlignment = 'right';
            app.RollingaveragedurationEditFieldLabel.Position = [107 33 135 22];
            app.RollingaveragedurationEditFieldLabel.Text = 'Rolling average duration';

            % Create RollingaveragedurationEditField
            app.RollingaveragedurationEditField = uieditfield(app.OtherSettingsPanel, 'numeric');
            app.RollingaveragedurationEditField.Limits = [0 Inf];
            app.RollingaveragedurationEditField.ValueChangedFcn = createCallbackFcn(app, @RollingaveragedurationEditFieldValueChanged, true);
            app.RollingaveragedurationEditField.Position = [257 33 100 22];
            app.RollingaveragedurationEditField.Value = 10;

            % Create HelpTab
            app.HelpTab = uitab(app.MainTabGroup);
            app.HelpTab.Title = 'Help';

            % Create HTML
            app.HTML = uihtml(app.HelpTab);
            app.HTML.HTMLSource = './assets/help.html';
            app.HTML.Position = [16 69 1201 456];

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