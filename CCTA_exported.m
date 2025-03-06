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
        ClearSetpointsButton_2          matlab.ui.control.Button
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
        ClearSetpointsButton            matlab.ui.control.Button
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

        Kp = 1;
        Ki = 0;
        Kd = 0;
        
        pressureMotorPosition = 0;
        MAX_PRESSURE_MOTOR_POSITION = -200*7;  % 7 rotations, 200 steps per rotation; TODO: read in max stepper value from Arduino instead of hard-coding to 7 rotations
        flowDutyCycle = 0;

        PID_setpoint_id = "None";  % Identifier as to which sensor's setpoint is influencing PID control, since only one sensor at a time can do so
        PID_setpoint = 0;
        PID_output = 0;
        PID_prev_error = 0;
        PID_integral = 0;
        PID_derivative = 0;
    end

    properties (Dependent)
        dt_refresh
        pressure1_Setpoint
    end

    methods
        function value = get.dt_refresh(app)
            value = 1 / app.F_REFRESH;  % Compute refresh interval dynamically
        end

        function value = get.pressure1_Setpoint(app)
             value = app.Pressure1_SetPoint.Value;
        end
    end

    methods (Access = private)

        function [t, newFlowIndicator, flow_value_raw1, flow_value1, ...
                flow_value_raw2, flow_value2, pressure_value_raw1, ...
                pressure_value1, pressure_value_raw2, pressure_value2, ...
                pressure_value_raw3, pressure_value3] = readSensorData(app)

            % Reads and parses serial data from the Arduino serial port.
            % Inputs:
            %   serialObj - MATLAB serial object for Arduino communication.
            % Outputs:
            %   flow_value_raw - Raw ADC value for flow sensor.
            %   flow_value - Processed flow rate in L/min.
            %   pressure_value_raw - Raw ADC value for pressure sensor.
            %   pressure_value - Processed pressure in mmHg.

            if app.SimulateDataCheckBox.Value
                % Simulate data (for testing)
                line = "New Flow?: N, Flow Raw 1: 0, Flow 1: 1.50 L/min, Flow Raw 2: 0, Flow 2: 2.50 L/min, Pressure Raw 1: 1023, Pressure 1: 20 mmHg, Pressure Raw 2: 1023, Pressure 2: 50 mmHg, Pressure Raw 3: 1023, Pressure 3: 80 mmHg\n";
            else
                % Read data from Arduino; flush serial buffer by reading
                % three times to improve results
                %
                % TODO: See if waiting for arduinoObj.NumBytesAvailable > 0
                % replaces the need for this for loop
                for j = 1:3
                    line = readline(app.arduinoObj);
                end
            end

            t = toc;
            pause(app.dt_refresh);  % pause to allow app to process other user inputs (e.g. button presses)

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
                % Expected format:
                % "New Flow?: Y, Flow Raw 1: 120, Flow 1: 3.50 L/min, Flow Raw 2: 110, Flow 2: 3.20 L/min,
                % Pressure Raw 1: 512, Pressure 1: 75.3 mmHg, Pressure Raw 2: 523, Pressure 2: 76.1 mmHg,
                % Pressure Raw 3: 540, Pressure 3: 78.4 mmHg"

                % Extract values using sscanf
                data = sscanf(line, 'New Flow?: %c, Flow Raw 1: %d, Flow 1: %f L/min, Flow Raw 2: %d, Flow 2: %f L/min, Pressure Raw 1: %d, Pressure 1: %f mmHg, Pressure Raw 2: %d, Pressure 2: %f mmHg, Pressure Raw 3: %d, Pressure 3: %f mmHg\n');

                % Ensure we received all expected values
                if numel(data) == 11
                    newFlowIndicator = char(data(1));   % 'Y' or 'N'
                    flow_value_raw1 = data(2);
                    flow_value1 = data(3);
                    flow_value_raw2 = data(4);
                    flow_value2 = data(5);
                    pressure_value_raw1 = data(6);
                    pressure_value1 = data(7);
                    pressure_value_raw2 = data(8);
                    pressure_value2 = data(9);
                    pressure_value_raw3 = data(10);
                    pressure_value3 = data(11);

                    % Get timestamp
                    timestamp = datetime("now");

                    % Format the new sensor data as a string
                    newLogEntry = sprintf(['===== Sensor Data (%s) =====\n', ...
                        'New Flow?: %c\n', ...
                        'Flow Raw 1: %d, Flow 1: %.2f L/min\n', ...
                        'Flow Raw 2: %d, Flow 2: %.2f L/min\n', ...
                        'Pressure Raw 1: %d, Pressure 1: %.2f mmHg\n', ...
                        'Pressure Raw 2: %d, Pressure 2: %.2f mmHg\n', ...
                        'Pressure Raw 3: %d, Pressure 3: %.2f mmHg\n\n'], ...
                        timestamp, newFlowIndicator, ...
                        flow_value_raw1, flow_value1, ...
                        flow_value_raw2, flow_value2, ...
                        pressure_value_raw1, pressure_value1, ...
                        pressure_value_raw2, pressure_value2, ...
                        pressure_value_raw3, pressure_value3);

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
                % Read sensor data
                [tNew, ~, ~, flow_value1, ~, flow_value2, ...
                    ~, pressure_value1, ~, pressure_value2, ...
                    ~, pressure_value3] = app.readSensorData();

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
                        plot(app.PressureAxes, app.t, app.pressure_vals1, 'r', 'DisplayName', 'Pressure 1');
                    end
                    if app.Pressure2_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals2, 'g', 'DisplayName', 'Pressure 2');
                    end
                    if app.Pressure3_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals3, 'b', 'DisplayName', 'Pressure 3');
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
                        plot(app.FlowAxes, app.t, app.flow_vals1, 'r', 'DisplayName', 'Flow 1');
                    end
                    if app.Flow2_GraphEnable.Value
                        plot(app.FlowAxes, app.t, app.flow_vals2, 'g', 'DisplayName', 'Flow 2');
                    end

                    hold(app.FlowAxes, 'off');
                    title(app.FlowAxes, 'Flow Data');
                    xlabel(app.FlowAxes, 'Time (s)');
                    ylabel(app.FlowAxes, 'Flow (L/min)');
                    ylim(app.FlowAxes, [0 5]);
                    legend(app.FlowAxes, 'show','Location','northwest');
                end

                % Update PID Outputs (WIP)
                % app.updatePIDOutputs(pressure_value1, flow_value1);  

                % Check whether disconnect button has been clicked
                if app.ConnectButton.Text == "CONNECT"
                    break
                end

                % Pause to prevent crashes
                pause(app.dt_refresh);
            end
        end
        
        function sendPIDOutputsToArduino(app)
            app.MotorStepNumberEditField.Value = app.pressureMotorPosition;
            line = sprintf("Pressure Motor Step Number: %d, Pump Duty Cycle: %d",app.pressureMotorPosition,app.flowDutyCycle);
            app.arduinoObj.writeline(line)

            % TODO: based on setpoint ID
            % if app.PID_setpoint_id == "Pressure1"
                % stepper_position = round(pid_output); % Convert PID output to a valid stepper motor position
                % stepper_position = max(0, min(app.MAX_PRESSURE_MOTOR_POSITION, stepper_position)); % Constrain range;
                % app.pressureMotorPosition = stepper_position;
            % end
        end
        
        function updatePIDOutputs(app, newValue)
            % Calculate PID output
            error = app.PID_setpoint - newValue;
            app.PID_integral = app.PID_integral + error * app.DT_PID;
            app.PID_derivative = (error - prev_error) / app.DT_PID;
            app.PID_output = app.Kp * error + app.Ki * app.PID_integral + app.Kd * app.PID_derivative;
            app.PID_prev_error = error;

            % Send PID output to Arduino
            app.sendPIDOutputsToArduino();
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
                    app.arduinoObj = serialport(app.COMPortDropDown.Value, 9600);
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

        % Callback function
        function ClearDataButtonValueChanged(app, event)
            % Empty time and sensor data arrays
            app.clearDataArrays();
        end

        % Button pushed function: ClearDataButton
        function ClearDataButtonPushed(app, event)
            % Empty time and sensor data arrays
            app.clearDataArrays();
        end

        % Button pushed function: ExportDataButton
        function ExportDataButtonPushed(app, event)
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

            csvData = table(app.t(:), app.flow_vals1(:), app.flow_vals2(:), app.pressure_vals1(:), app.pressure_vals2(:), app.pressure_vals3(:), ...
                'VariableNames', {'Time (s)', 'Flow_SVC (L/min)', 'Flow_IVC (L/min)', 'Pressure_PA (mmHg)', 'Pressure_SVC (mmHg)', 'Pressure_IVC (mmHg)'});
            
            writetable(csvData, fullfile(folderName, 'ExportedData.csv'));

            % Save console output to text file
            app.saveLog(fullfile(folderName, 'console_output.txt'));
            
            % Display dialog box
            uialert(app.UIFigure, ['Data has been saved to ', folderName], 'Export Successful', 'Icon', 'info');
        end

        % Button pushed function: ClearSetpointsButton
        function ClearSetpointsButtonPushed(app, event)
            app.Pressure1_SetPoint.Value = 0;
        end

        % Value changed function: Pressure1_SetPoint
        function Pressure1_SetPointValueChanged(app, event)
            app.PID_setpoint_id = "Pressure1";
            app.PID_setpoint = app.pressure1_Setpoint;
        end

        % Value changed function: MotorStepNumberEditField
        function MotorStepNumberEditFieldValueChanged(app, event)
            value = app.MotorStepNumberEditField.Value;
            app.ClosedSlider.Value = value/app.MAX_PRESSURE_MOTOR_POSITION * app.ClosedSlider.Limits(2);  % update slider

            app.pressureMotorPosition = value;
            app.sendPIDOutputsToArduino();
        end

        % Value changed function: ClosedSlider
        function ClosedSliderValueChanged(app, event)
            percent_closed = app.ClosedSlider.Value/app.ClosedSlider.Limits(2);  % negative because gear rotates valve the other way
            app.pressureMotorPosition = round(app.MAX_PRESSURE_MOTOR_POSITION * percent_closed);
            app.sendPIDOutputsToArduino();
        end

        % Button pushed function: RefreshListButton
        function RefreshListButtonPushed(app, event)
            app.updateSerialPortList();
        end

        % Value changed function: PumpPowerSlider
        function PumpPowerSliderValueChanged(app, event)
            app.flowDutyCycle = round(app.PumpPowerSlider.Value/app.PumpPowerSlider.Limits(2)*app.ARDUINO_ANALOGWRITE_MAX);
            app.sendPIDOutputsToArduino();
        end

        % Value changed function: AnalogWriteValueEditField
        function AnalogWriteValueEditFieldValueChanged(app, event)
            value = app.AnalogWriteValueEditField.Value;

            app.flowDutyCycle = value;
            app.PumpPowerSlider.Value = value/app.ARDUINO_ANALOGWRITE_MAX * app.PumpPowerSlider.Limits(2);  % update slider
            app.sendPIDOutputsToArduino();
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
            app.PressureMonitoringControlPanel.Position = [15 307 298 181];

            % Create Label
            app.Label = uilabel(app.PressureMonitoringControlPanel);
            app.Label.FontSize = 18;
            app.Label.FontWeight = 'bold';
            app.Label.Position = [62 129 25 23];
            app.Label.Text = '';

            % Create Pressure1_SetPoint
            app.Pressure1_SetPoint = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure1_SetPoint.ValueChangedFcn = createCallbackFcn(app, @Pressure1_SetPointValueChanged, true);
            app.Pressure1_SetPoint.Enable = 'off';
            app.Pressure1_SetPoint.Position = [154 108 84 22];

            % Create Pressure2_SetPoint
            app.Pressure2_SetPoint = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure2_SetPoint.Enable = 'off';
            app.Pressure2_SetPoint.Position = [154 77 84 22];

            % Create Pressure3_SetPoint
            app.Pressure3_SetPoint = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure3_SetPoint.Enable = 'off';
            app.Pressure3_SetPoint.Position = [154 46 84 22];

            % Create CurrentLabel
            app.CurrentLabel = uilabel(app.PressureMonitoringControlPanel);
            app.CurrentLabel.FontWeight = 'bold';
            app.CurrentLabel.Position = [81 130 48 22];
            app.CurrentLabel.Text = 'Current';

            % Create SetpointLabel
            app.SetpointLabel = uilabel(app.PressureMonitoringControlPanel);
            app.SetpointLabel.FontWeight = 'bold';
            app.SetpointLabel.Position = [171 130 53 22];
            app.SetpointLabel.Text = 'Setpoint';

            % Create Label_5
            app.Label_5 = uilabel(app.PressureMonitoringControlPanel);
            app.Label_5.HorizontalAlignment = 'right';
            app.Label_5.Position = [28 108 25 22];
            app.Label_5.Text = '1';

            % Create Pressure1_EditField_Current
            app.Pressure1_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure1_EditField_Current.Editable = 'off';
            app.Pressure1_EditField_Current.Position = [68 108 70 22];

            % Create Label_6
            app.Label_6 = uilabel(app.PressureMonitoringControlPanel);
            app.Label_6.HorizontalAlignment = 'right';
            app.Label_6.Position = [28 77 25 22];
            app.Label_6.Text = '2';

            % Create Pressure2_EditField_Current
            app.Pressure2_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure2_EditField_Current.Editable = 'off';
            app.Pressure2_EditField_Current.Position = [68 77 70 22];

            % Create Label_7
            app.Label_7 = uilabel(app.PressureMonitoringControlPanel);
            app.Label_7.HorizontalAlignment = 'right';
            app.Label_7.Position = [28 46 25 22];
            app.Label_7.Text = '3';

            % Create Pressure3_EditField_Current
            app.Pressure3_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure3_EditField_Current.Editable = 'off';
            app.Pressure3_EditField_Current.Position = [68 46 70 22];

            % Create allvaluesinmmHgLabel
            app.allvaluesinmmHgLabel = uilabel(app.PressureMonitoringControlPanel);
            app.allvaluesinmmHgLabel.FontAngle = 'italic';
            app.allvaluesinmmHgLabel.Position = [183 159 114 22];
            app.allvaluesinmmHgLabel.Text = '(all values in mmHg)';

            % Create SensorLabel
            app.SensorLabel = uilabel(app.PressureMonitoringControlPanel);
            app.SensorLabel.FontWeight = 'bold';
            app.SensorLabel.Position = [5 130 57 22];
            app.SensorLabel.Text = 'Sensor #';

            % Create GraphLabel
            app.GraphLabel = uilabel(app.PressureMonitoringControlPanel);
            app.GraphLabel.Position = [249 130 45 22];
            app.GraphLabel.Text = 'Graph?';

            % Create Pressure3_GraphEnable
            app.Pressure3_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure3_GraphEnable.Text = '';
            app.Pressure3_GraphEnable.Position = [259 46 25 22];
            app.Pressure3_GraphEnable.Value = true;

            % Create Pressure2_GraphEnable
            app.Pressure2_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure2_GraphEnable.Text = '';
            app.Pressure2_GraphEnable.Position = [259 77 25 22];
            app.Pressure2_GraphEnable.Value = true;

            % Create Pressure1_GraphEnable
            app.Pressure1_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure1_GraphEnable.Text = '';
            app.Pressure1_GraphEnable.Position = [260 110 25 22];
            app.Pressure1_GraphEnable.Value = true;

            % Create ClearSetpointsButton
            app.ClearSetpointsButton = uibutton(app.PressureMonitoringControlPanel, 'push');
            app.ClearSetpointsButton.ButtonPushedFcn = createCallbackFcn(app, @ClearSetpointsButtonPushed, true);
            app.ClearSetpointsButton.FontSize = 10;
            app.ClearSetpointsButton.Position = [154 11 84 23];
            app.ClearSetpointsButton.Text = 'Clear Setpoints';

            % Create FlowMonitoringControlPanel
            app.FlowMonitoringControlPanel = uipanel(app.MainTab);
            app.FlowMonitoringControlPanel.Title = 'Flow Monitoring & Control';
            app.FlowMonitoringControlPanel.BackgroundColor = [0.902 0.902 0.902];
            app.FlowMonitoringControlPanel.FontWeight = 'bold';
            app.FlowMonitoringControlPanel.Position = [15 153 298 143];

            % Create Label_2
            app.Label_2 = uilabel(app.FlowMonitoringControlPanel);
            app.Label_2.FontSize = 18;
            app.Label_2.FontWeight = 'bold';
            app.Label_2.Position = [67 72 25 23];
            app.Label_2.Text = '';

            % Create Flow1_SetPoint
            app.Flow1_SetPoint = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow1_SetPoint.Enable = 'off';
            app.Flow1_SetPoint.Position = [153 71 84 22];

            % Create Flow2_SetPoint
            app.Flow2_SetPoint = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow2_SetPoint.Enable = 'off';
            app.Flow2_SetPoint.Position = [153 40 84 22];

            % Create CurrentLabel_2
            app.CurrentLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.CurrentLabel_2.FontWeight = 'bold';
            app.CurrentLabel_2.Position = [80 93 48 22];
            app.CurrentLabel_2.Text = 'Current';

            % Create SetpointLabel_2
            app.SetpointLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.SetpointLabel_2.FontWeight = 'bold';
            app.SetpointLabel_2.Position = [170 93 53 22];
            app.SetpointLabel_2.Text = 'Setpoint';

            % Create Label_3
            app.Label_3 = uilabel(app.FlowMonitoringControlPanel);
            app.Label_3.HorizontalAlignment = 'right';
            app.Label_3.Position = [27 71 25 22];
            app.Label_3.Text = '1';

            % Create Flow1_EditField_Current
            app.Flow1_EditField_Current = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow1_EditField_Current.Editable = 'off';
            app.Flow1_EditField_Current.Position = [67 71 70 22];

            % Create Label_4
            app.Label_4 = uilabel(app.FlowMonitoringControlPanel);
            app.Label_4.HorizontalAlignment = 'right';
            app.Label_4.Position = [27 40 25 22];
            app.Label_4.Text = '2';

            % Create Flow2_EditField_Current
            app.Flow2_EditField_Current = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow2_EditField_Current.Editable = 'off';
            app.Flow2_EditField_Current.Position = [67 40 70 22];

            % Create allvaluesinLminLabel
            app.allvaluesinLminLabel = uilabel(app.FlowMonitoringControlPanel);
            app.allvaluesinLminLabel.FontAngle = 'italic';
            app.allvaluesinLminLabel.Position = [189 121 108 22];
            app.allvaluesinLminLabel.Text = '(all values in L/min)';

            % Create Flow2_GraphEnable
            app.Flow2_GraphEnable = uicheckbox(app.FlowMonitoringControlPanel);
            app.Flow2_GraphEnable.Text = '';
            app.Flow2_GraphEnable.Position = [258 40 25 22];
            app.Flow2_GraphEnable.Value = true;

            % Create Flow1_GraphEnable
            app.Flow1_GraphEnable = uicheckbox(app.FlowMonitoringControlPanel);
            app.Flow1_GraphEnable.Text = '';
            app.Flow1_GraphEnable.Position = [258 72 25 22];
            app.Flow1_GraphEnable.Value = true;

            % Create GraphLabel_2
            app.GraphLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.GraphLabel_2.Position = [248 93 45 22];
            app.GraphLabel_2.Text = 'Graph?';

            % Create SensorLabel_2
            app.SensorLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.SensorLabel_2.FontWeight = 'bold';
            app.SensorLabel_2.Position = [11 93 56 22];
            app.SensorLabel_2.Text = 'Sensor #';

            % Create ClearSetpointsButton_2
            app.ClearSetpointsButton_2 = uibutton(app.FlowMonitoringControlPanel, 'push');
            app.ClearSetpointsButton_2.FontSize = 10;
            app.ClearSetpointsButton_2.Position = [154 9 84 23];
            app.ClearSetpointsButton_2.Text = 'Clear Setpoints';

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
            app.FlowPumpControlPanel.Position = [313 297 306 141];

            % Create AnalogWriteValueEditFieldLabel
            app.AnalogWriteValueEditFieldLabel = uilabel(app.FlowPumpControlPanel);
            app.AnalogWriteValueEditFieldLabel.HorizontalAlignment = 'right';
            app.AnalogWriteValueEditFieldLabel.Enable = 'off';
            app.AnalogWriteValueEditFieldLabel.Position = [58 78 107 22];
            app.AnalogWriteValueEditFieldLabel.Text = 'Analog Write Value';

            % Create AnalogWriteValueEditField
            app.AnalogWriteValueEditField = uieditfield(app.FlowPumpControlPanel, 'numeric');
            app.AnalogWriteValueEditField.Limits = [0 255];
            app.AnalogWriteValueEditField.RoundFractionalValues = 'on';
            app.AnalogWriteValueEditField.ValueChangedFcn = createCallbackFcn(app, @AnalogWriteValueEditFieldValueChanged, true);
            app.AnalogWriteValueEditField.Enable = 'off';
            app.AnalogWriteValueEditField.Position = [180 78 100 22];

            % Create PumpPowerSliderLabel
            app.PumpPowerSliderLabel = uilabel(app.FlowPumpControlPanel);
            app.PumpPowerSliderLabel.HorizontalAlignment = 'right';
            app.PumpPowerSliderLabel.Enable = 'off';
            app.PumpPowerSliderLabel.Position = [17 33 91 22];
            app.PumpPowerSliderLabel.Text = 'Pump Power  %';

            % Create PumpPowerSlider
            app.PumpPowerSlider = uislider(app.FlowPumpControlPanel);
            app.PumpPowerSlider.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSliderValueChanged, true);
            app.PumpPowerSlider.Enable = 'off';
            app.PumpPowerSlider.Position = [129 42 150 3];

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