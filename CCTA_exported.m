classdef CCTA_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        TabGroup2                       matlab.ui.container.TabGroup
        MainTab                         matlab.ui.container.Tab
        ExportDataButton                matlab.ui.control.Button
        ClearDataButton                 matlab.ui.control.Button
        COMPortDropDown                 matlab.ui.control.DropDown
        COMPortDropDownLabel            matlab.ui.control.Label
        CONNECTButton                   matlab.ui.control.Button
        HoldGraphsButton                matlab.ui.control.StateButton
        PUMPOFFButton                   matlab.ui.control.Button
        PUMPONButton                    matlab.ui.control.Button
        FlowMonitoringControlPanel      matlab.ui.container.Panel
        GraphLabel_2                    matlab.ui.control.Label
        Flow_IVC_GraphEnable            matlab.ui.control.CheckBox
        Flow_SVC_GraphEnable            matlab.ui.control.CheckBox
        AllvaluesinLminLabel            matlab.ui.control.Label
        Flow_SVC_EditField_Current      matlab.ui.control.NumericEditField
        SVCEditField_2Label             matlab.ui.control.Label
        Flow_IVC_EditField_Current      matlab.ui.control.NumericEditField
        IVCEditField_2Label             matlab.ui.control.Label
        SetpointLabel_2                 matlab.ui.control.Label
        CurrentLabel_2                  matlab.ui.control.Label
        EditField_5                     matlab.ui.control.NumericEditField
        EditField_4                     matlab.ui.control.NumericEditField
        Label_2                         matlab.ui.control.Label
        PressureMonitoringControlPanel  matlab.ui.container.Panel
        Pressure_IVC_GraphEnable        matlab.ui.control.CheckBox
        Pressure_SVC_GraphEnable        matlab.ui.control.CheckBox
        Pressure_PA_GraphEnable         matlab.ui.control.CheckBox
        GraphLabel                      matlab.ui.control.Label
        LocationLabel                   matlab.ui.control.Label
        AllvaluesinmmHgLabel            matlab.ui.control.Label
        Pressure_PA_EditField_Current   matlab.ui.control.NumericEditField
        PAEditFieldLabel                matlab.ui.control.Label
        Pressure_SVC_EditField_Current  matlab.ui.control.NumericEditField
        SVCEditFieldLabel               matlab.ui.control.Label
        Pressure_IVC_EditField_Current  matlab.ui.control.NumericEditField
        IVCEditFieldLabel               matlab.ui.control.Label
        SetpointLabel                   matlab.ui.control.Label
        CurrentLabel                    matlab.ui.control.Label
        EditField_3                     matlab.ui.control.NumericEditField
        EditField_2                     matlab.ui.control.NumericEditField
        EditField                       matlab.ui.control.NumericEditField
        Label                           matlab.ui.control.Label
        FlowAxes                        matlab.ui.control.UIAxes
        PressureAxes                    matlab.ui.control.UIAxes
        DeveloperOptionsTab             matlab.ui.container.Tab
        SimulateDataCheckBox            matlab.ui.control.CheckBox
    end


    properties (Access = private)
        plot_raw_data = true;
        f_refresh = 20;  % App refresh rate (Hz)

        arduinoObj;
        tempLogFile = fullfile(pwd, 'console_output_temp.txt');  % Define a persistent temporary log file that always stores session history

        t = [];
        flow_vals1 = [];  % Containers for sensor data
        flow_vals2 = [];

        pressure_vals1 = [];
        pressure_vals2 = [];
        pressure_vals3 = [];
    end

    properties (Dependent)
        dt_refresh
    end

    methods
        function value = get.dt_refresh(app)
            value = 1 / app.f_refresh;  % Compute refresh interval dynamically
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
                for j = 1:3
                    line = readline(app.arduinoObj);
                end
            end

            t = toc;
            pause(app.dt_refresh);  % pause to allow app to process other user inputs (e.g. button presses)

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
                    % fprintf('%s', newLogEntry);
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
                disp(app.t);
                app.t = [app.t tNew];
                app.flow_vals1 = [app.flow_vals1 flow_value1];
                app.flow_vals2 = [app.flow_vals2 flow_value2];

                app.pressure_vals1 = [app.pressure_vals1 pressure_value1];
                app.pressure_vals2 = [app.pressure_vals2 pressure_value2];
                app.pressure_vals3 = [app.pressure_vals3 pressure_value3];

                % Update Edit Fields with the latest sensor data
                app.Pressure_PA_EditField_Current.Value = pressure_value1;
                app.Pressure_SVC_EditField_Current.Value = pressure_value2;
                app.Pressure_IVC_EditField_Current.Value = pressure_value3;

                app.Flow_IVC_EditField_Current.Value = flow_value1;
                app.Flow_SVC_EditField_Current.Value = flow_value2;

                % Plot pressure data based on checkbox selections
                % as long as "hold graph" is not pressed
                if app.HoldGraphsButton.Value == false
                    hold(app.PressureAxes, 'on');
                    cla(app.PressureAxes); % Clear previous plots

                    if app.Pressure_PA_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals1, 'r', 'DisplayName', 'Pressure PA');
                    end
                    if app.Pressure_SVC_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals2, 'g', 'DisplayName', 'Pressure SVC');
                    end
                    if app.Pressure_IVC_GraphEnable.Value
                        plot(app.PressureAxes, app.t, app.pressure_vals3, 'b', 'DisplayName', 'Pressure IVC');
                    end

                    hold(app.PressureAxes, 'off');
                    title(app.PressureAxes, 'Pressure Data');
                    xlabel(app.PressureAxes, 'Time (s)');
                    ylabel(app.PressureAxes, 'Pressure (mmHg)');
                    ylim(app.PressureAxes, [0 200]);
                    legend(app.PressureAxes, 'show','Location','northwest');

                    % Plot flow data based on checkbox selections
                    hold(app.FlowAxes, 'on');
                    cla(app.FlowAxes); % Clear previous plots

                    if app.Flow_IVC_GraphEnable.Value
                        plot(app.FlowAxes, app.t, app.flow_vals1, 'r', 'DisplayName', 'Flow IVC');
                    end
                    if app.Flow_SVC_GraphEnable.Value
                        plot(app.FlowAxes, app.t, app.flow_vals2, 'g', 'DisplayName', 'Flow SVC');
                    end

                    hold(app.FlowAxes, 'off');
                    title(app.FlowAxes, 'Flow Data');
                    xlabel(app.FlowAxes, 'Time (s)');
                    ylabel(app.FlowAxes, 'Flow (L/min)');
                    ylim(app.FlowAxes, [0 5]);
                    legend(app.FlowAxes, 'show','Location','northwest');
                end

                if app.CONNECTButton.Text == "CONNECT"
                    break
                end

                % Pause to prevent crashes
                pause(app.dt_refresh);
            end
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.startLogging();
            
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

        % Callback function
        function SavelogfileButtonPushed(app, event)

        end

        % Button pushed function: CONNECTButton
        function CONNECTButtonPushed(app, event)
            
            if app.CONNECTButton.Text == "CONNECT"
                % Someone just connected to the serial device
                app.CONNECTButton.Text = "DISCONNECT";
                app.CONNECTButton.BackgroundColor = 'red';

                if app.SimulateDataCheckBox.Value == false
                    app.arduinoObj = serialport(app.COMPortDropDown.Value, 9600);
                end
                app.clearDataArrays();
    
                tic;
                app.startLogging();
                app.runMainLoop();

            elseif app.CONNECTButton.Text == "DISCONNECT"
                % Someone just disconnected from the serial device
                app.arduinoObj = [];
                app.CONNECTButton.Text = "CONNECT";
                app.CONNECTButton.BackgroundColor = 'green';
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
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1228 463];
            app.UIFigure.Name = 'MATLAB App';

            % Create TabGroup2
            app.TabGroup2 = uitabgroup(app.UIFigure);
            app.TabGroup2.Position = [1 3 1229 461];

            % Create MainTab
            app.MainTab = uitab(app.TabGroup2);
            app.MainTab.Title = 'Main';

            % Create PressureAxes
            app.PressureAxes = uiaxes(app.MainTab);
            title(app.PressureAxes, 'Pressure Data')
            xlabel(app.PressureAxes, 'Time (s)')
            ylabel(app.PressureAxes, 'Pressure (mmHg)')
            zlabel(app.PressureAxes, 'Z')
            app.PressureAxes.Position = [360 127 373 270];

            % Create FlowAxes
            app.FlowAxes = uiaxes(app.MainTab);
            title(app.FlowAxes, 'Flow Data')
            xlabel(app.FlowAxes, 'Time(s)')
            ylabel(app.FlowAxes, 'Flow (L/min)')
            zlabel(app.FlowAxes, 'Z')
            app.FlowAxes.Position = [788 127 410 270];

            % Create PressureMonitoringControlPanel
            app.PressureMonitoringControlPanel = uipanel(app.MainTab);
            app.PressureMonitoringControlPanel.Title = 'Pressure Monitoring & Control';
            app.PressureMonitoringControlPanel.FontWeight = 'bold';
            app.PressureMonitoringControlPanel.Position = [38 234 298 181];

            % Create Label
            app.Label = uilabel(app.PressureMonitoringControlPanel);
            app.Label.FontSize = 18;
            app.Label.FontWeight = 'bold';
            app.Label.Position = [62 100 25 23];
            app.Label.Text = '';

            % Create EditField
            app.EditField = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.EditField.Enable = 'off';
            app.EditField.Position = [154 79 84 22];

            % Create EditField_2
            app.EditField_2 = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.EditField_2.Enable = 'off';
            app.EditField_2.Position = [154 48 84 22];

            % Create EditField_3
            app.EditField_3 = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.EditField_3.Enable = 'off';
            app.EditField_3.Position = [154 17 84 22];

            % Create CurrentLabel
            app.CurrentLabel = uilabel(app.PressureMonitoringControlPanel);
            app.CurrentLabel.FontWeight = 'bold';
            app.CurrentLabel.Position = [81 101 48 22];
            app.CurrentLabel.Text = 'Current';

            % Create SetpointLabel
            app.SetpointLabel = uilabel(app.PressureMonitoringControlPanel);
            app.SetpointLabel.FontWeight = 'bold';
            app.SetpointLabel.Position = [171 101 53 22];
            app.SetpointLabel.Text = 'Setpoint';

            % Create IVCEditFieldLabel
            app.IVCEditFieldLabel = uilabel(app.PressureMonitoringControlPanel);
            app.IVCEditFieldLabel.HorizontalAlignment = 'right';
            app.IVCEditFieldLabel.Position = [28 79 25 22];
            app.IVCEditFieldLabel.Text = 'IVC';

            % Create Pressure_IVC_EditField_Current
            app.Pressure_IVC_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure_IVC_EditField_Current.Position = [68 79 70 22];

            % Create SVCEditFieldLabel
            app.SVCEditFieldLabel = uilabel(app.PressureMonitoringControlPanel);
            app.SVCEditFieldLabel.HorizontalAlignment = 'right';
            app.SVCEditFieldLabel.Position = [23 48 30 22];
            app.SVCEditFieldLabel.Text = 'SVC';

            % Create Pressure_SVC_EditField_Current
            app.Pressure_SVC_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure_SVC_EditField_Current.Position = [68 48 70 22];

            % Create PAEditFieldLabel
            app.PAEditFieldLabel = uilabel(app.PressureMonitoringControlPanel);
            app.PAEditFieldLabel.HorizontalAlignment = 'right';
            app.PAEditFieldLabel.Position = [28 17 25 22];
            app.PAEditFieldLabel.Text = 'PA';

            % Create Pressure_PA_EditField_Current
            app.Pressure_PA_EditField_Current = uieditfield(app.PressureMonitoringControlPanel, 'numeric');
            app.Pressure_PA_EditField_Current.Position = [68 17 70 22];

            % Create AllvaluesinmmHgLabel
            app.AllvaluesinmmHgLabel = uilabel(app.PressureMonitoringControlPanel);
            app.AllvaluesinmmHgLabel.FontAngle = 'italic';
            app.AllvaluesinmmHgLabel.Position = [97 129 108 22];
            app.AllvaluesinmmHgLabel.Text = 'All values in mmHg';

            % Create LocationLabel
            app.LocationLabel = uilabel(app.PressureMonitoringControlPanel);
            app.LocationLabel.FontWeight = 'bold';
            app.LocationLabel.Position = [13 101 55 22];
            app.LocationLabel.Text = 'Location';

            % Create GraphLabel
            app.GraphLabel = uilabel(app.PressureMonitoringControlPanel);
            app.GraphLabel.Position = [249 101 45 22];
            app.GraphLabel.Text = 'Graph?';

            % Create Pressure_PA_GraphEnable
            app.Pressure_PA_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure_PA_GraphEnable.Text = '';
            app.Pressure_PA_GraphEnable.Position = [259 17 25 22];
            app.Pressure_PA_GraphEnable.Value = true;

            % Create Pressure_SVC_GraphEnable
            app.Pressure_SVC_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure_SVC_GraphEnable.Text = '';
            app.Pressure_SVC_GraphEnable.Position = [259 48 25 22];
            app.Pressure_SVC_GraphEnable.Value = true;

            % Create Pressure_IVC_GraphEnable
            app.Pressure_IVC_GraphEnable = uicheckbox(app.PressureMonitoringControlPanel);
            app.Pressure_IVC_GraphEnable.Text = '';
            app.Pressure_IVC_GraphEnable.Position = [260 81 25 22];
            app.Pressure_IVC_GraphEnable.Value = true;

            % Create FlowMonitoringControlPanel
            app.FlowMonitoringControlPanel = uipanel(app.MainTab);
            app.FlowMonitoringControlPanel.Title = 'Flow Monitoring & Control';
            app.FlowMonitoringControlPanel.FontWeight = 'bold';
            app.FlowMonitoringControlPanel.Position = [38 70 298 143];

            % Create Label_2
            app.Label_2 = uilabel(app.FlowMonitoringControlPanel);
            app.Label_2.FontSize = 18;
            app.Label_2.FontWeight = 'bold';
            app.Label_2.Position = [68 46 25 23];
            app.Label_2.Text = '';

            % Create EditField_4
            app.EditField_4 = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.EditField_4.Enable = 'off';
            app.EditField_4.Position = [154 45 84 22];

            % Create EditField_5
            app.EditField_5 = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.EditField_5.Enable = 'off';
            app.EditField_5.Position = [154 14 84 22];

            % Create CurrentLabel_2
            app.CurrentLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.CurrentLabel_2.Position = [81 67 45 22];
            app.CurrentLabel_2.Text = 'Current';

            % Create SetpointLabel_2
            app.SetpointLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.SetpointLabel_2.Position = [171 67 49 22];
            app.SetpointLabel_2.Text = 'Setpoint';

            % Create IVCEditField_2Label
            app.IVCEditField_2Label = uilabel(app.FlowMonitoringControlPanel);
            app.IVCEditField_2Label.HorizontalAlignment = 'right';
            app.IVCEditField_2Label.Position = [28 45 25 22];
            app.IVCEditField_2Label.Text = 'IVC';

            % Create Flow_IVC_EditField_Current
            app.Flow_IVC_EditField_Current = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow_IVC_EditField_Current.Position = [68 45 70 22];

            % Create SVCEditField_2Label
            app.SVCEditField_2Label = uilabel(app.FlowMonitoringControlPanel);
            app.SVCEditField_2Label.HorizontalAlignment = 'right';
            app.SVCEditField_2Label.Position = [23 14 30 22];
            app.SVCEditField_2Label.Text = 'SVC';

            % Create Flow_SVC_EditField_Current
            app.Flow_SVC_EditField_Current = uieditfield(app.FlowMonitoringControlPanel, 'numeric');
            app.Flow_SVC_EditField_Current.Position = [68 14 70 22];

            % Create AllvaluesinLminLabel
            app.AllvaluesinLminLabel = uilabel(app.FlowMonitoringControlPanel);
            app.AllvaluesinLminLabel.FontAngle = 'italic';
            app.AllvaluesinLminLabel.Position = [100 95 102 22];
            app.AllvaluesinLminLabel.Text = 'All values in L/min';

            % Create Flow_SVC_GraphEnable
            app.Flow_SVC_GraphEnable = uicheckbox(app.FlowMonitoringControlPanel);
            app.Flow_SVC_GraphEnable.Text = '';
            app.Flow_SVC_GraphEnable.Position = [259 14 25 22];
            app.Flow_SVC_GraphEnable.Value = true;

            % Create Flow_IVC_GraphEnable
            app.Flow_IVC_GraphEnable = uicheckbox(app.FlowMonitoringControlPanel);
            app.Flow_IVC_GraphEnable.Text = '';
            app.Flow_IVC_GraphEnable.Position = [259 46 25 22];
            app.Flow_IVC_GraphEnable.Value = true;

            % Create GraphLabel_2
            app.GraphLabel_2 = uilabel(app.FlowMonitoringControlPanel);
            app.GraphLabel_2.Position = [249 67 45 22];
            app.GraphLabel_2.Text = 'Graph?';

            % Create PUMPONButton
            app.PUMPONButton = uibutton(app.MainTab, 'push');
            app.PUMPONButton.BackgroundColor = [0.4667 0.6745 0.1882];
            app.PUMPONButton.FontWeight = 'bold';
            app.PUMPONButton.Enable = 'off';
            app.PUMPONButton.Position = [39 32 80 22];
            app.PUMPONButton.Text = 'PUMP ON';

            % Create PUMPOFFButton
            app.PUMPOFFButton = uibutton(app.MainTab, 'push');
            app.PUMPOFFButton.BackgroundColor = [1 0 0];
            app.PUMPOFFButton.FontWeight = 'bold';
            app.PUMPOFFButton.Enable = 'off';
            app.PUMPOFFButton.Position = [257 32 79 21];
            app.PUMPOFFButton.Text = 'PUMP OFF';

            % Create HoldGraphsButton
            app.HoldGraphsButton = uibutton(app.MainTab, 'state');
            app.HoldGraphsButton.Text = 'Hold Graphs';
            app.HoldGraphsButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.HoldGraphsButton.FontSize = 14;
            app.HoldGraphsButton.FontWeight = 'bold';
            app.HoldGraphsButton.Position = [807 63 125 30];

            % Create CONNECTButton
            app.CONNECTButton = uibutton(app.MainTab, 'push');
            app.CONNECTButton.ButtonPushedFcn = createCallbackFcn(app, @CONNECTButtonPushed, true);
            app.CONNECTButton.BackgroundColor = [0 1 0];
            app.CONNECTButton.FontSize = 14;
            app.CONNECTButton.FontWeight = 'bold';
            app.CONNECTButton.Position = [447 30 102 25];
            app.CONNECTButton.Text = 'CONNECT';

            % Create COMPortDropDownLabel
            app.COMPortDropDownLabel = uilabel(app.MainTab);
            app.COMPortDropDownLabel.HorizontalAlignment = 'right';
            app.COMPortDropDownLabel.FontSize = 14;
            app.COMPortDropDownLabel.FontWeight = 'bold';
            app.COMPortDropDownLabel.Position = [361 71 70 22];
            app.COMPortDropDownLabel.Text = 'COM Port';

            % Create COMPortDropDown
            app.COMPortDropDown = uidropdown(app.MainTab);
            app.COMPortDropDown.FontSize = 14;
            app.COMPortDropDown.FontWeight = 'bold';
            app.COMPortDropDown.Position = [446 71 100 22];

            % Create ClearDataButton
            app.ClearDataButton = uibutton(app.MainTab, 'push');
            app.ClearDataButton.ButtonPushedFcn = createCallbackFcn(app, @ClearDataButtonPushed, true);
            app.ClearDataButton.FontSize = 14;
            app.ClearDataButton.FontWeight = 'bold';
            app.ClearDataButton.Position = [957 63 127 30];
            app.ClearDataButton.Text = 'Clear Data';

            % Create ExportDataButton
            app.ExportDataButton = uibutton(app.MainTab, 'push');
            app.ExportDataButton.ButtonPushedFcn = createCallbackFcn(app, @ExportDataButtonPushed, true);
            app.ExportDataButton.FontSize = 14;
            app.ExportDataButton.FontWeight = 'bold';
            app.ExportDataButton.Position = [661 63 127 30];
            app.ExportDataButton.Text = 'Export Data';

            % Create DeveloperOptionsTab
            app.DeveloperOptionsTab = uitab(app.TabGroup2);
            app.DeveloperOptionsTab.Title = 'Developer Options';

            % Create SimulateDataCheckBox
            app.SimulateDataCheckBox = uicheckbox(app.DeveloperOptionsTab);
            app.SimulateDataCheckBox.Text = 'Simulate Data?';
            app.SimulateDataCheckBox.Position = [26 396 104 22];

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