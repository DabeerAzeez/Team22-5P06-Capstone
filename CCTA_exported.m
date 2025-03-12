classdef CCTA_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        TabGroup2                       matlab.ui.container.TabGroup
        MainTab                         matlab.ui.container.Tab
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
        FlowAxes                        matlab.ui.control.UIAxes
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
        SettingsTab                     matlab.ui.container.Tab
        FlowPumpControlPanel            matlab.ui.container.Panel
        ControlSwitch                   matlab.ui.control.Switch
        ControlSwitchLabel              matlab.ui.control.Label
        PumpPowerSlider                 matlab.ui.control.Slider
        PumpPowerSliderLabel            matlab.ui.control.Label
        AnalogWriteValueEditField       matlab.ui.control.NumericEditField
        AnalogWriteValueEditFieldLabel  matlab.ui.control.Label
        OPENVALVEButton                 matlab.ui.control.Button
        PUMPOFFButton                   matlab.ui.control.Button
        PUMPONButton                    matlab.ui.control.Button
        PressureValveControlPanel       matlab.ui.container.Panel
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

        Kp = 0;  % No PID control by default
        Ki = 0;
        Kd = 0;

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

                    if app.Pressure1_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals1, 'r', 'DisplayName', app.pressure1_label, 'LineWidth', 1.5);
                    end
                    if app.Pressure2_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals2, 'g', 'DisplayName', app.pressure2_label, 'LineWidth', 1.5);
                    end
                    if app.Pressure3_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals3, 'b', 'DisplayName', app.pressure3_label, 'LineWidth', 1.5);
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

                    if app.Flow1_GraphEnable.Value
                        plot(app.FlowAxes, app.t, app.flow_vals1, 'r', 'DisplayName', app.flow1_label, 'LineWidth', 2);
                    end
                    if app.Flow2_GraphEnable.Value
                        plot(app.FlowAxes, app.t, app.flow_vals2, 'g', 'DisplayName', app.flow2_label, 'LineWidth', 2);
                    end

                    hold(app.FlowAxes, 'off');
                    title(app.FlowAxes, 'Flow Data');
                    xlabel(app.FlowAxes, 'Time (s)');
                    ylabel(app.FlowAxes, 'Flow (L/min)');
                    ylim(app.FlowAxes, [0 5]);
                    legend(app.FlowAxes, 'show','Location','northwest');
                end

                % writeline(app.arduinoObj, sprintf("Pressure Motor Step Number: %d, Pump Duty Cycle: %d",100,randi([0,255])));
                % app.WaitForArduinoMessage();

                % Update PID Outputs
                if app.PumpControlMode == "PID"
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
                app.ControlSwitch.Value = "Manual";
            elseif lower(mode) == "pid"
                app.PumpControlMode = "PID";
                app.ControlSwitch.Value = "PID";
            else
                warning("Invalid control mode selected.");
            end
        end
        
        function sendControlValuesToArduino(app)
            app.MotorStepNumberEditField.Value = app.pressureMotorPosition;

            if lower(app.PumpControlMode) == "pid"
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
            app.AnalogWriteValueEditField.Value = app.flowDutyCycle;
            app.PumpPowerSlider.Value = app.flowDutyCycle / app.ARDUINO_ANALOGWRITE_MAX * app.PumpPowerSlider.Limits(2);

            % Send command to Arduino
            % command = sprintf("Pressure Motor Step Number: %d, Pump Duty Cycle: %d",app.pressureMotorPosition,app.flowDutyCycle);
            % writeline(app.arduinoObj,command);
            % app.WaitForArduinoMessage();

            f = parfeval(@testAsync, 0);
        end

        function testAsync(app)
            % Send command to Arduino
            command = sprintf("Pressure Motor Step Number: %d, Pump Duty Cycle: %d",app.pressureMotorPosition,app.flowDutyCycle);
            writeline(app.arduinoObj,command);
            app.WaitForArduinoMessage();
        end

        function setPIDCoefficients(app,inputVariable)
            if inputVariable == "Flow"
                app.Kp = app.Kp_flow;
                app.Ki = app.Ki_flow;
                app.Kd = app.Kd_flow;
            elseif inputVariable == "Pressure"
                app.Kp = app.Kp_pressure;
                app.Ki = app.Ki_pressure;
                app.Kd = app.Kd_pressure;
            else
                app.Kp = 0;
                app.Ki = 0;
                app.Kd = 0;
                disp("Invalid input variable inputted when setting PID coefficients. PID coefficients have been reset to zero.")
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
                error('Invalid field name: %s', app.PID_setpoint_id);
            end

            % Calculate PID output
            PID_error = app.PID_setpoint - newValue;
            app.PID_integral = app.PID_integral + PID_error * app.DT_PID;
            app.PID_derivative = (PID_error - app.PID_prev_error) / app.DT_PID;
            app.PID_output = app.Kp * PID_error + app.Ki * app.PID_integral + app.Kd * app.PID_derivative;
            app.PID_prev_error = PID_error;

            % fprintf("Setpoint / New Value / Error / Output / DC: %.2f, %.2f, %.2f, %.2f \n",app.PID_setpoint, newValue, error, app.PID_output, app.flowDutyCycle);

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
            children = app.PressureValveControlPanel.Children;
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
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.startLogging();
            app.updateSerialPortList();

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

        % Value changed function: Pressure1_SetPoint
        function Pressure1_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure1_SetPoint.Value;
            app.setControlMode("PID");
            app.setPIDCoefficients("Pressure");
            app.highlightSetPointUI("Pressure1");
            app.PID_setpoint_id = "Pressure1";
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
            app.AnalogWriteValueEditField.Value = app.flowDutyCycle;
            app.setControlMode("Manual");

            flushoutput(app.arduinoObj);  % in case this was done in the middle of sending another command
            app.sendControlValuesToArduino();
        end

        % Value changed function: AnalogWriteValueEditField
        function AnalogWriteValueEditFieldValueChanged(app, event)
            value = app.AnalogWriteValueEditField.Value;
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
            app.setPIDCoefficients("Flow");
            app.highlightSetPointUI("Flow1");
            app.PID_setpoint_id = "Flow1";
        end

        % Value changed function: ControlSwitch
        function ControlSwitchValueChanged(app, event)
            app.setControlMode(app.ControlSwitch.Value);
            app.highlightSetPointUI("None");
        end

        % Value changed function: Flow2_SetPoint
        function Flow2_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Flow2_SetPoint.Value;
            app.setControlMode("PID");
            app.setPIDCoefficients("Flow");
            app.highlightSetPointUI("Flow2");
            app.PID_setpoint_id = "Flow2";
        end

        % Value changed function: Pressure2_SetPoint
        function Pressure2_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure2_SetPoint.Value;
            app.setControlMode("PID");
            app.setPIDCoefficients("Pressure");
            app.highlightSetPointUI("Pressure2");
            app.PID_setpoint_id = "Pressure2";
        end

        % Value changed function: Pressure3_SetPoint
        function Pressure3_SetPointValueChanged(app, event)
            app.PID_setpoint = app.Pressure3_SetPoint.Value;
            app.setControlMode("PID");
            app.setPIDCoefficients("Pressure");
            app.highlightSetPointUI("Pressure3");
            app.PID_setpoint_id = "Pressure3";
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
            app.PressureMonitoringControlPanel.BackgroundColor = [0.902 0.902 0.902];
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
            app.FlowMonitoringControlPanel.BackgroundColor = [0.902 0.902 0.902];
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
            app.Panel.BackgroundColor = [0.8 0.8 0.8];
            app.Panel.FontAngle = 'italic';
            app.Panel.Position = [332 154 883 333];

            % Create PressureAxes
            app.PressureAxes = uiaxes(app.Panel);
            title(app.PressureAxes, 'Pressure Data')
            xlabel(app.PressureAxes, 'Time (s)')
            ylabel(app.PressureAxes, 'Pressure (mmHg)')
            zlabel(app.PressureAxes, 'Z')
            app.PressureAxes.FontWeight = 'bold';
            app.PressureAxes.FontSize = 14;
            app.PressureAxes.Position = [12 9 409 312];

            % Create FlowAxes
            app.FlowAxes = uiaxes(app.Panel);
            title(app.FlowAxes, 'Flow Data')
            xlabel(app.FlowAxes, 'Time(s)')
            ylabel(app.FlowAxes, 'Flow (L/min)')
            zlabel(app.FlowAxes, 'Z')
            app.FlowAxes.FontWeight = 'bold';
            app.FlowAxes.FontSize = 14;
            app.FlowAxes.Position = [435 9 437 312];

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
            app.ConnectButton.BackgroundColor = [0.9529 0.5725 0.2157];
            app.ConnectButton.FontSize = 14;
            app.ConnectButton.FontWeight = 'bold';
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
            app.RefreshListButton.BackgroundColor = [0.9529 0.5725 0.2157];
            app.RefreshListButton.FontSize = 14;
            app.RefreshListButton.FontWeight = 'bold';
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
            app.COMPortLabel.FontColor = [0.9529 0.5725 0.2157];
            app.COMPortLabel.Position = [541 82 93 23];
            app.COMPortLabel.Text = 'COM Port ';

            % Create COMPortDropDown
            app.COMPortDropDown = uidropdown(app.MainTab);
            app.COMPortDropDown.FontSize = 14;
            app.COMPortDropDown.FontWeight = 'bold';
            app.COMPortDropDown.BackgroundColor = [0.9529 0.5725 0.2157];
            app.COMPortDropDown.Position = [653 79 126 31];

            % Create ConnectedLamp
            app.ConnectedLamp = uilamp(app.MainTab);
            app.ConnectedLamp.Position = [1064 84 20 20];
            app.ConnectedLamp.Color = [0.8 0.8 0.8];

            % Create SettingsTab
            app.SettingsTab = uitab(app.TabGroup2);
            app.SettingsTab.Title = 'Settings';

            % Create SimulateDataCheckBox
            app.SimulateDataCheckBox = uicheckbox(app.SettingsTab);
            app.SimulateDataCheckBox.Text = 'Simulate Data?';
            app.SimulateDataCheckBox.Position = [26 459 104 22];

            % Create PressureValveControlPanel
            app.PressureValveControlPanel = uipanel(app.SettingsTab);
            app.PressureValveControlPanel.Title = 'Pressure Valve Control';
            app.PressureValveControlPanel.Position = [26 296 260 141];

            % Create MotorStepNumberEditFieldLabel
            app.MotorStepNumberEditFieldLabel = uilabel(app.PressureValveControlPanel);
            app.MotorStepNumberEditFieldLabel.HorizontalAlignment = 'right';
            app.MotorStepNumberEditFieldLabel.Enable = 'off';
            app.MotorStepNumberEditFieldLabel.Position = [17 79 110 22];
            app.MotorStepNumberEditFieldLabel.Text = 'Motor Step Number';

            % Create MotorStepNumberEditField
            app.MotorStepNumberEditField = uieditfield(app.PressureValveControlPanel, 'numeric');
            app.MotorStepNumberEditField.Limits = [0 200];
            app.MotorStepNumberEditField.RoundFractionalValues = 'on';
            app.MotorStepNumberEditField.ValueChangedFcn = createCallbackFcn(app, @MotorStepNumberEditFieldValueChanged, true);
            app.MotorStepNumberEditField.Enable = 'off';
            app.MotorStepNumberEditField.Position = [142 79 100 22];

            % Create ClosedSliderLabel
            app.ClosedSliderLabel = uilabel(app.PressureValveControlPanel);
            app.ClosedSliderLabel.HorizontalAlignment = 'right';
            app.ClosedSliderLabel.Enable = 'off';
            app.ClosedSliderLabel.Position = [9 33 56 22];
            app.ClosedSliderLabel.Text = '% Closed';

            % Create ClosedSlider
            app.ClosedSlider = uislider(app.PressureValveControlPanel);
            app.ClosedSlider.ValueChangedFcn = createCallbackFcn(app, @ClosedSliderValueChanged, true);
            app.ClosedSlider.Enable = 'off';
            app.ClosedSlider.Position = [86 42 150 3];

            % Create PUMPONButton
            app.PUMPONButton = uibutton(app.SettingsTab, 'push');
            app.PUMPONButton.BackgroundColor = [0.4667 0.6745 0.1882];
            app.PUMPONButton.FontSize = 14;
            app.PUMPONButton.FontWeight = 'bold';
            app.PUMPONButton.Enable = 'off';
            app.PUMPONButton.Position = [25 248 130 28];
            app.PUMPONButton.Text = 'PUMP ON';

            % Create PUMPOFFButton
            app.PUMPOFFButton = uibutton(app.SettingsTab, 'push');
            app.PUMPOFFButton.BackgroundColor = [1 0 0];
            app.PUMPOFFButton.FontSize = 14;
            app.PUMPOFFButton.FontWeight = 'bold';
            app.PUMPOFFButton.Enable = 'off';
            app.PUMPOFFButton.Position = [170 248 129 29];
            app.PUMPOFFButton.Text = 'PUMP OFF';

            % Create OPENVALVEButton
            app.OPENVALVEButton = uibutton(app.SettingsTab, 'push');
            app.OPENVALVEButton.BackgroundColor = [0.9294 0.6941 0.1255];
            app.OPENVALVEButton.FontSize = 14;
            app.OPENVALVEButton.FontWeight = 'bold';
            app.OPENVALVEButton.Enable = 'off';
            app.OPENVALVEButton.Position = [23 205 129 29];
            app.OPENVALVEButton.Text = 'OPEN VALVE';

            % Create FlowPumpControlPanel
            app.FlowPumpControlPanel = uipanel(app.SettingsTab);
            app.FlowPumpControlPanel.Title = 'Flow Pump Control';
            app.FlowPumpControlPanel.Position = [313 268 306 170];

            % Create AnalogWriteValueEditFieldLabel
            app.AnalogWriteValueEditFieldLabel = uilabel(app.FlowPumpControlPanel);
            app.AnalogWriteValueEditFieldLabel.HorizontalAlignment = 'right';
            app.AnalogWriteValueEditFieldLabel.FontWeight = 'bold';
            app.AnalogWriteValueEditFieldLabel.Position = [13 73 113 22];
            app.AnalogWriteValueEditFieldLabel.Text = 'Analog Write Value';

            % Create AnalogWriteValueEditField
            app.AnalogWriteValueEditField = uieditfield(app.FlowPumpControlPanel, 'numeric');
            app.AnalogWriteValueEditField.Limits = [0 255];
            app.AnalogWriteValueEditField.RoundFractionalValues = 'on';
            app.AnalogWriteValueEditField.ValueChangedFcn = createCallbackFcn(app, @AnalogWriteValueEditFieldValueChanged, true);
            app.AnalogWriteValueEditField.Position = [141 73 100 22];

            % Create PumpPowerSliderLabel
            app.PumpPowerSliderLabel = uilabel(app.FlowPumpControlPanel);
            app.PumpPowerSliderLabel.HorizontalAlignment = 'right';
            app.PumpPowerSliderLabel.FontWeight = 'bold';
            app.PumpPowerSliderLabel.Position = [13 40 95 22];
            app.PumpPowerSliderLabel.Text = 'Pump Power  %';

            % Create PumpPowerSlider
            app.PumpPowerSlider = uislider(app.FlowPumpControlPanel);
            app.PumpPowerSlider.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSliderValueChanged, true);
            app.PumpPowerSlider.Position = [129 49 150 3];

            % Create ControlSwitchLabel
            app.ControlSwitchLabel = uilabel(app.FlowPumpControlPanel);
            app.ControlSwitchLabel.HorizontalAlignment = 'center';
            app.ControlSwitchLabel.FontWeight = 'bold';
            app.ControlSwitchLabel.Position = [17 116 48 22];
            app.ControlSwitchLabel.Text = 'Control';

            % Create ControlSwitch
            app.ControlSwitch = uiswitch(app.FlowPumpControlPanel, 'slider');
            app.ControlSwitch.Items = {'Manual', 'PID'};
            app.ControlSwitch.ValueChangedFcn = createCallbackFcn(app, @ControlSwitchValueChanged, true);
            app.ControlSwitch.Position = [123 117 45 20];
            app.ControlSwitch.Value = 'Manual';

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