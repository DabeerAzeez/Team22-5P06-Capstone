classdef CCTA_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                       matlab.ui.Figure
        MainTabGroup                   matlab.ui.container.TabGroup
        MainTab                        matlab.ui.container.Tab
        PumpControlLabel               matlab.ui.control.Label
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
        PulseBPMLabel                  matlab.ui.control.Label
        PulseBPMSpinner                matlab.ui.control.Spinner
        PulseBPMSlider                 matlab.ui.control.Slider
        PumpControlModeDropDown        matlab.ui.control.DropDown
        PumpPowerLamp                  matlab.ui.control.Lamp
        PumpControlModeDropdownLabel   matlab.ui.control.Label
        PumpPowerPercentLabel          matlab.ui.control.Label
        PumpPowerSwitch                matlab.ui.control.Switch
        PumpPowerSwitchLabel           matlab.ui.control.Label
        PumpPowerSpinner               matlab.ui.control.Spinner
        PumpPowerSlider                matlab.ui.control.Slider
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
        RollingaveragedurationsEditField  matlab.ui.control.NumericEditField
        RollingaveragedurationsEditFieldLabel  matlab.ui.control.Label
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
        ConsoleTab                     matlab.ui.container.Tab
        ConsoleTextArea                matlab.ui.control.TextArea
    end

    properties (Access = private)
        % Constants
        % TODO: Add more UI elements to be able to change these through UI
        ARDUINO_ANALOGWRITE_MAX = 255;
        ARDUINO_SERIAL_BITRATE = 115200;
        PUMP_MIN_POWER_PID = 0.10;       % minimum pump power before flow stops - used for PID limits
        PUMP_MIN_DUTY_CYCLE;
        PID_ERRORS_LENGTH = 75;         % # of error values to use for PID integral calculations, determined empirically to minimize settling time
        CALIBRATION_DURATION = 20;      % # of seconds to run calibration procedure
        ROLLING_AVERAGE_DURATION = 10;  % # of seconds for rolling average
        CALIBRATION_REST_PERIOD = 5;    % # of seconds to rest before collecting static heads for calibration
        PID_SCALING_FACTOR = 10000;
        SERIAL_COMMS_TIMEOUT = 5;  % # of seconds to wait for the next message over serial before showing a uialert

        PULSATILE_DEFAULT_BPM = 40;
        PULSATILE_DEFAULT_POWER = 100;

        PAUSE_LENGTH = 0.1;  % For small pauses to prevent CPU overuse, duration of pause (s)

        % Colors
        COLOR_PRESSURE = [0.84 0.83 0.86];
        COLOR_PRESSURE_2 = [161, 138, 219]./255;
        COLOR_FLOW = [0.77,0.83,0.76];
        COLOR_BTN_DEFAULT = [0.94,0.94,0.94];
        COLOR_BTN_1 = [0.16,0.57,0.84];
        COLOR_DISABLED = [0.5 0.5 0.5];
        COLOR_DISCONNECTED = [1.00,0.00,0.00];
        COLOR_CONNECTED = [0, 1, 0];
        COLOR_ERROR = [1 0 0];
        COLOR_WARNING = [245, 220, 59]./255;
        COLOR_NORMAL = [1 1 1];
        COLOR_LAMP_OFF = [0.50,0.50,0.50];
        COLOR_EDITFIELD_DIMMED = [0.75 0.75 0.75];

        % Graphing
        pressure1Line 
        pressure2Line 
        pressure3Line 
        pressureTargetLine
        pressureFill
        flow1Line
        flow2Line
        flow1MarkerLine
        flow2MarkerLine
        flowTargetLine
        flowFill

        % PID Variables
        Kp_flow = 5;  % PID coefficients when controlling flow
        Ki_flow = 0.5;  % Default values defined here
        Kd_flow = 0;

        Kp_pressure = 0.2;  % PID coefficients when controlling pressure
        Ki_pressure = 0.1;  % Default values defined here
        Kd_pressure = 0.05;

        lastFlowDutyCycle = 0;  % for comparisons
        pumpDutyCycle = 0;
        pumpStopped = true;
        PumpControlMode = 'Manual';

        % Containers for sensor data
        t = [];
        mainLoopTime = [];
        setpoint_vals = [];
        setpoint_ids = [];

        flow_vals1 = [];  
        flow_vals2 = [];
        flow_markers1 = [];  % boolean arrays to track whether flow value is new or old
        flow_markers2 = [];

        pressure_vals1 = [];
        pressure_vals2 = [];
        pressure_vals3 = [];

        flow_vals1_avgs = string.empty;
        flow_vals2_avgs = string.empty;

        pressure_vals1_avgs = string.empty;
        pressure_vals2_avgs = string.empty;
        pressure_vals3_avgs = string.empty;

        % Other variables
        appDir;

        arduinoObj;
        ports;
        portsData;
        isConnected;
        tempLogFile = fullfile(pwd, 'console_output_temp.txt');  % Define a persistent temporary log file that always stores session history

        rollingAverageCutoffTime;

        PUMP_MAX_FLOW = 5;  % L/min
        pulsatileFlowEnabled = false;
        pulsatileFlowValues;
        pulsatileFlowTimes;
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

    methods (Access = private)

        function dimEditFieldTemporarily(app, editField)
            %DIMEDITFIELDTEMPORARILY Temporarily dims an edit field’s background color.
            %
            %   DIMEDITFIELDTEMPORARILY(EDITFIELD) changes the background color of
            %   the specified EDITFIELD to a dimmed version, waits briefly, and then
            %   restores the original color. This provides visual feedback for user input.
            %
            %   Inputs:
            %       EDITFIELD - The edit field UI component to temporarily dim
            %
            %   Outputs:
            %       None
            %
            %   Example:
            %       app.dimEditFieldTemporarily(app.InputField)

            alpha = 0.5;  % Amount of dimming (0 = no dim, 1 = full dim)

            originalColor = editField.BackgroundColor;
            dimmedTarget = app.COLOR_EDITFIELD_DIMMED;  % Light gray dim target (can adjust)

            % Blend original color with dimmed target
            dimmedColor = (1 - alpha) * originalColor + alpha * dimmedTarget;

            % Apply dimming
            editField.BackgroundColor = dimmedColor;
            pause(0.5);

            % Revert to original
            editField.BackgroundColor = originalColor;
        end

        function [newFlow, flow_value1, flow_value2, ...
                pressure_value1, pressure_value2, pressure_value3] = parseArduinoData(app, line)

            % PARSEARDUINODATA Parses a line of serial data from the Arduino.
            %
            %   [NEWFLOWINDICATOR, FLOW1, FLOW2, P1, P2, P3] = PARSEARDUINODATA(LINE)
            %   Extracts flow and pressure values from the given LINE of serial data sent
            %   by the Arduino. Applies static head offset corrections and logs the data.
            %   
            %   Inputs:
            %       LINE - A string containing the raw serial message from the Arduino
            %
            %   Outputs:
            %       NEWFLOWINDICATOR   - True/false whether a new flow
            %                            value was received
            %       FLOW1              - Flow rate from sensor 1 (L/min)
            %       FLOW2              - Flow rate from sensor 2 (L/min)
            %       P1                 - Calibrated pressure from sensor 1 (mmHg)
            %       P2                 - Calibrated pressure from sensor 2 (mmHg)
            %       P3                 - Calibrated pressure from sensor 3 (mmHg)

            % Handle special messages first
            if ~isempty(line)
                if contains(line, "ERROR")
                    uialert(app.UIFigure, ...
                        sprintf("Error message received from Arduino: %s", line), ...
                        "Arduino Error Received");
                    return;
                elseif contains(line, "RECEIVED")
                    % Reuse last known values to prevent random dips in data plots
                    newFlow = 'N';
                    flow_value1 = app.flow_vals1(end);
                    flow_value2 = app.flow_vals2(end);
                    pressure_value1 = app.pressure_vals1(end);
                    pressure_value2 = app.pressure_vals2(end);
                    pressure_value3 = app.pressure_vals3(end);

                    disp(line);
                    app.printToConsole(line);
                    return;
                end
            end

            try
                % Raise error if line is empty
                if isempty(line)
                    error("Empty serial line received.");
                end

                % Parse the incoming line using textscan
                tokens = textscan(line, ...
                    ['NF: %c, F1: %f, F2: %f, ' ...
                    'P1R: %d, P1: %f, ' ...
                    'P2R: %d, P2: %f, ' ...
                    'P3R: %d, P3: %f ' ...
                    '| MODE: %s ' ...
                    '| PMP: %d ' ...
                    '| PID_ID: %s SP: %f, Kp: %f, Ki: %f, Kd: %f ' ...
                    '| BPM: %d, AMP: %d']);

                % Expected number of fields
                nExpectedDataPoints = 18;

                % Check that all fields were successfully parsed
                nActual = numel(tokens);
                if nActual ~= nExpectedDataPoints
                    error("Serial parsing failed. Expected %d values but got %d. Line: %s", nExpectedDataPoints, nActual, line);
                end

                % Only unpack relevant fields
                newFlowIndicator = char(tokens{1});              % 'Y' or 'N'
                flow_value1      = tokens{2};
                flow_value2      = tokens{3};
                pressure_value1  = tokens{5} - app.SHO_Pressure1;
                pressure_value2  = tokens{7} - app.SHO_Pressure2;
                pressure_value3  = tokens{9} - app.SHO_Pressure3;

                if strcmp(newFlowIndicator,'Y')
                    newFlow = true;
                elseif strcmp(newFlowIndicator,'N')
                    newFlow = false;
                else
                    uialert(app.UIFigure, sprintf("Unrecognized new flow indicator received: %s",newFlowIndicator), ...
                        "Unrecognized message");
                    newFlow = false;
                end

                % Update pump power UI under PID control to show
                % functionality to user
                if app.PumpControlMode == "Auto"
                    flowDutyCycle = tokens{11};
                    app.setPumpPower(flowDutyCycle,'dutyCycle','uiOnly');
                end

                % Optional logging
                disp(line);  % TODO: Add timestamp here if needed
                app.printToConsole(line);

            catch ME
                % Assign default error values
                newFlow = false;
                flow_value1 = -1;
                flow_value2 = -1;
                pressure_value1 = -1;
                pressure_value2 = -1;
                pressure_value3 = -1;

                % Display error message
                warning(ME.message);
                app.printToConsole(ME.message);
            end
        end

        function startLogging(app)
            %STARTLOGGING Begins logging console output to a temporary file.
            %
            %   STARTLOGGING() activates MATLAB's diary feature and redirects all
            %   command window output to the temporary log file specified in the app.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

            diary(app.tempLogFile);
            diary on;

            % Notify user
            fprintf('Logging started. All session logs will be recorded in: %s\n', app.tempLogFile);
        end

        function saveLog(app, filename)
            %SAVELOG Saves the current log file and restarts session logging.
            %
            %   SAVELOG(FILENAME) stops current logging, copies the session's temporary
            %   log file to the specified FILENAME, and restarts logging to the temporary file.
            %
            %   Inputs:
            %       FILENAME - Destination filename for the saved log file
            %
            %   Outputs:
            %       None

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
            %CLEARDATAARRAYS Resets data arrays and PID state for a fresh session.
            %
            %   CLEARDATAARRAYS() clears stored sensor and setpoint data, resets PID
            %   controller values, and restarts the session timer.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

            tic;
            app.t = [];

            app.setpoint_vals = [];

            app.flow_vals1 = [];  
            app.flow_vals2 = [];
            app.flow_markers1 = [];
            app.flow_markers2 = [];

            app.pressure_vals1 = [];
            app.pressure_vals2 = [];
            app.pressure_vals3 = [];

            app.PID_errors = zeros(1,app.PID_ERRORS_LENGTH);
            app.PID_output = 0;
            app.PID_integral = 0;
            app.PID_derivative = 0;
        end
       
        
        function runMainLoop(app, varargin)
            %RUNMAINLOOP Main loop for real-time data acquisition and control.
            %
            %   RUNMAINLOOP() continuously reads sensor data from the Arduino, updates
            %   UI components, plots, and PID control output while the app remains
            %   connected. If a duration is passed as an optional input, the loop runs
            %   in timed mode for calibration purposes and displays a progress dialog.
            %
            %   Inputs:
            %       VARARGIN - Optional duration (in seconds) to run the loop in timed mode
            %
            %   Outputs:
            %       None

            % Check if duration argument is passed (implying calibration is 
            % being done)
            if ~isempty(varargin)
                duration = varargin{1};
                timedMode = true;
                
                % Show loading bar while calibration runs
                d = uiprogressdlg(app.UIFigure, ...
                    'Title', 'Calibrating...', ...
                    'Message', 'Collecting data for calibration (0 seconds remaining...)', ...
                    'Indeterminate', 'off', ...
                    'Value', 0);
            else
                timedMode = false;
            end

            app.mainLoopTime = tic;
            app.setUpDataPlots();

            while app.isConnected
                [tNew, line, success] = app.getArduinoData();

                if ~success
                    return;
                end

                % Read sensor data
                [newFlow, flow_value1, flow_value2, ...
                    pressure_value1, pressure_value2, pressure_value3] = app.parseArduinoData(line);

                % Append new sensor data
                app.t = [app.t tNew];
                app.setpoint_vals = [app.setpoint_vals app.PID_setpoint];
                app.setpoint_ids = [app.setpoint_ids app.PID_setpoint_id];

                app.flow_vals1 = [app.flow_vals1 flow_value1];  
                app.flow_vals2 = [app.flow_vals2 flow_value2];
                app.flow_markers1 = [app.flow_markers1 newFlow];
                app.flow_markers2 = [app.flow_markers2 newFlow];

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

                % If duration mode, break when time is up, otherwise update
                % loading bar
                if timedMode
                    tElapsed = toc(app.mainLoopTime);
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

        function setPumpControlMode(app,mode)
            %SETPUMPCONTROLMODE Configures the pump control mode and updates the UI accordingly.
            %
            %   SETPUMPCONTROLMODE(MODE) sets the current pump control mode to one of
            %   the supported options: 'manual', 'pid', 'pulsatile', or 'calibration'.
            %   It adjusts the UI components to reflect the selected mode and disables
            %   or enables manual controls as needed.
            %
            %   Inputs:
            %       MODE - A string specifying the control mode ('manual', 'pid', 'pulsatile', or 'calibration')
            %
            %   Outputs:
            %       None

            if mode == "Manual"
                app.PID_setpoint_id = "None";
                app.highlightSetPointUI("None");  % clear any setpoint UI from any prior PID control
                app.PumpControlMode = "Manual";
                app.PumpControlModeDropDown.Value = "Manual";
                app.PumpPowerSlider.Enable = "on";
                app.PumpPowerSpinner.Enable = "on";

                app.setPumpPower(50);  % let Arduino know we're back to manual control

            elseif mode == "Auto"  % PID Control
                app.PumpControlMode = "Auto";
                app.PumpControlModeDropDown.Value = "Auto";
                app.PumpPowerSlider.Enable = "off";
                app.PumpPowerSpinner.Enable = "off";

                app.sendPIDConfigToArduino();

            elseif mode == "Pulsatile"
                app.PID_setpoint_id = "None";
                app.highlightSetPointUI("None");  % clear any setpoint UI from any prior PID control
                app.PumpControlMode = "Pulsatile";
                app.PumpControlModeDropDown.Enable = "on";

                app.PulseBPMLabel.Enable = "on";
                app.PulseBPMSpinner.Enable = "on";
                app.PulseBPMSlider.Enable = "on";

                app.setPumpPower(app.PULSATILE_DEFAULT_POWER);  % to set default pulsatile flow power
                app.PulseBPMSliderValueChanged();  % send first pulsatile command

            elseif mode == "Calibration"
                app.PID_setpoint_id = "None";
                app.highlightSetPointUI("None");  % clear any setpoint UI from any prior PID control
                app.PumpControlMode = "Calibration";
                app.PumpControlModeDropDown.Enable = "off";
                app.PumpPowerSlider.Enable = "off";
                app.PumpPowerSpinner.Enable = "off";
            else
                warning("Invalid control mode selected.");
            end

            if mode ~= "Pulsatile"
                % Reset pulsatile flow UI
                app.PulseBPMLabel.Enable = "off";
                app.PulseBPMSpinner.Enable = "off";
                app.PulseBPMSlider.Enable = "off";
                app.PulseBPMSpinner.Value = app.PULSATILE_DEFAULT_BPM;
                app.PulseBPMSlider.Value = app.PULSATILE_DEFAULT_BPM;
            end
        end
        
        function updatePIDOutputs(app)
            %UPDATEPIDOUTPUTS Computes and applies PID control output for pump power.
            %
            %   UPDATEPIDOUTPUTS() calculates the PID control output based on the
            %   current setpoint and sensor feedback, using the appropriate PID
            %   coefficients for pressure or flow. It updates the pump duty cycle
            %   accordingly and applies saturation limits.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

            if app.PID_setpoint_id == "None"
                return;  % In case this function gets accidentally called when no PID setpoint is set
            end

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
                uialert(app.UIFigure, ...
                    sprintf("Invalid PID setpoint ID selected: %s", app.PID_setpoint_id), ...
                    "Invalid PID Setpoint");
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
            app.pumpDutyCycle = app.pumpDutyCycle + round(app.PID_output);

            % Place PID output within limits
            if app.pumpDutyCycle > app.ARDUINO_ANALOGWRITE_MAX
                app.pumpDutyCycle = app.ARDUINO_ANALOGWRITE_MAX;
            elseif app.pumpDutyCycle < app.PUMP_MIN_DUTY_CYCLE
                app.pumpDutyCycle = app.PUMP_MIN_DUTY_CYCLE;
            end

            app.setPumpPower(app.pumpDutyCycle, 'dutyCycle');
        end
        
        function updateSerialPortList(app)
            %UPDATESERIALPORTLIST Scans and populates available serial ports in the UI.
            %
            %   UPDATESERIALPORTLIST() detects connected serial devices, retrieves
            %   their friendly names (on Windows), and updates the app's dropdown menu
            %   for device selection. Enables or disables UI controls based on port availability.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

            % Get list of available serial ports
            app.portsData = serialportlist;

            if isempty(app.portsData)
                uialert(app.UIFigure, "Try refreshing the connections list or check your device connection.", "No serial ports detected")
                app.ConnectionDropDown.Enable = "off";
                app.ConnectDisconnectButton.Enable = "off";
                return
            else
                app.ConnectionDropDown.Enable = "on";  % in case it was disabled before
                app.ConnectDisconnectButton.Enable = "on";
            end

            % Read the "friendly names" of the COM Port devices using Powershell

            app.ports = {};

            % Use system command to get device descriptions (Windows)
            if ispc
                [~, cmdout] = system("powershell -Command ""Get-WMIObject Win32_PnPEntity | Where-Object { $_.Name -match '\(COM[0-9]+\)' } | ForEach-Object { $_.Name }""");
                deviceList = splitlines(strtrim(cmdout));
            else
                deviceList = repmat("Unknown Device", size(app.portsData));
            end

            for i = 1:length(app.portsData)
                matchIdx = find(contains(deviceList, app.portsData(i)), 1);
                if ~isempty(matchIdx)
                    idn = deviceList{matchIdx};
                    idn = strtrim(erase(idn, ['(', app.portsData(i), ')'])); % Strip duplicate COM port info from the name
                else
                    idn = "Unknown Device";
                end

                app.ports{i} = sprintf('%s: %s', app.portsData(i), idn);
            end

            % Update dropdown with formatted port names
            app.ConnectionDropDown.Items = app.ports;
            app.ConnectionDropDown.ItemsData = app.portsData;  % To allow proper connections when using serialport() later off of the selected dropdown value
        end
        
        function highlightSetPointUI(app, setpointID)
            %HIGHLIGHTSETPOINTUI Highlights the active setpoint field in the UI.
            %
            %   HIGHLIGHTSETPOINTUI(SETPOINTID) updates the background color of all
            %   setpoint edit fields. The field matching SETPOINTID is highlighted to
            %   indicate it is the active target. Other fields are reset to normal color.
            %
            %   Inputs:
            %       SETPOINTID - Name of the active setpoint (e.g., 'Pressure1', 'Flow2')
            %
            %   Outputs:
            %       None

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

        
        function success = setUpArduinoConnection(app, dlg)
            %SETUPARDUINOCONNECTION Initializes serial connection with the Arduino.
            %
            %   SETUPARDUINOCONNECTION() creates and configures a serialport object for
            %   communication with the Arduino based on the selected port and bitrate.
            %
            %   Inputs:
            %       DLG - loading bar for connection status
            %
            %   Outputs:
            %       None

            try
                app.arduinoObj = serialport(app.ConnectionDropDown.Value, app.ARDUINO_SERIAL_BITRATE);
                configureTerminator(app.arduinoObj, "LF"); % Assuming newline ('\n') terminates messages
                app.arduinoObj.Timeout = 5; % Timeout for read operations
            catch
                close(dlg);
                uialert(app.UIFigure, ...
                    sprintf("Unable to connect to the selected serialport device. Verify that the device is connected and try re-connecting."), ...
                    "Serial Connection Error", ...
                    'CloseFcn', @(~,~) uiresume(app.UIFigure));
                uiwait(app.UIFigure);
                success = false;
                return;
            end

            success = app.waitForArduinoMessage(dlg);
        end
        
        function [t, line, success] = getArduinoData(app)
            %GETARDUINODATA Retrieves a line of sensor data from the Arduino.
            %
            %   LINE = GETARDUINODATA() reads one line of serial data from the Arduino.
            %   If simulation mode is enabled, it returns simulated data instead.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       LINE - A string of raw data from the Arduino or simulated data
            
            t = toc(app.mainLoopTime);

            if app.SimulateDataCheckBox.Value
                %% Simulate data values
                % Simulate once-per-second "NF: Y"
                NF = string(abs(mod(t, 1)) < 0.05);
                NF = replace(NF, {'true','false'}, {'Y','N'});

                % Simulate time-varying sinusoidal data
                freq = 0.3; % Hz for all signals
                ampF = 1; % Amplitude for flow (L/min)
                ampP = 20;  % Amplitude for pressure (mmHg)

                F1 = 1 + ampF * sin(2 * pi * freq * t);
                F2 = 3 + ampF * cos(2 * pi * freq * t);
                P1 = 20 + ampP * sin(2 * pi * freq * t);
                P2 = 40 + ampP * cos(2 * pi * freq * t);
                P3 = 60 + ampP * sin(2 * pi * freq * t + pi/4);

                % Generate a fake serial line
                line = sprintf("NF: %s, F1: %.2f, F2: %.2f, P1R: 1023, P1: %.2f, P2R: 740, P2: %.2f, P3R: 350, P3: %.2f | MODE: Manual | PMP: 128 | PID_ID: Pressure1 SP: 60.00, Kp: 1.00, Ki: 0.10, Kd: 0.01 | BPM: 75, AMP: 80", ...
                    NF, F1, F2, P1, P2, P3);
                success = true;
            else
                %% Read data in from Arduino
                if app.isConnected
                    try
                        success = app.waitForArduinoMessage();
                        if success
                            flushinput(app.arduinoObj);
                            readline(app.arduinoObj);  % Read until next newline in case flush clears part of a line
                            line = readline(app.arduinoObj); % Read a line of text
                        else
                            line = [];
                            success = false;
                        end
                    catch err
                        uialert(app.UIFigure, ...
                            sprintf("Serial communication error encountered while retrieving Arduino Data: %s", err.message), ...
                            "Serial Communication Error")
                    end
                end
            end
        end
        
        function success = waitForArduinoMessage(app, dlg)
            %WAITFORARDUINOMESSAGE Waits for the Arduino to send serial data.
            %
            %   WAITFORARDUINOMESSAGE() pauses execution until the Arduino has
            %   transmitted at least one line of data over the serial connection.
            %
            %   Inputs:
            %       DLG - loading bar for connection status
            %
            %   Outputs:
            %       None

            success = true;

            tMessage = tic;
            messageReceived = false;

            while toc(tMessage) < app.SERIAL_COMMS_TIMEOUT
                if app.arduinoObj.NumBytesAvailable ~= 0
                    messageReceived = true;
                    break;
                end
                pause(0.0001);  % prevent busy-waiting
            end

            if ~messageReceived
                close(dlg);
                
                uialert(app.UIFigure, ...
                    sprintf("No message received from serial connection after %d seconds. System will now disconnect. Try re-setting the connection.", app.SERIAL_COMMS_TIMEOUT), ...
                    "No Messages Received", ...
                    'CloseFcn', @(~,~) uiresume(app.UIFigure));

                uiwait(app.UIFigure);
                success = false;
            end
        end

        
        function updateUIPIDCoeffs(app)
            %UPDATEUIPIDCOEFFS Updates UI fields with current PID coefficient values.
            %
            %   UPDATEUIPIDCOEFFS() sets the UI input fields for PID coefficients
            %   (Kp, Ki, Kd) using the current values for flow and pressure control.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

            app.KpFlowEditField.Value = app.Kp_flow;  % PID coefficients when controlling flow
            app.KiFlowEditField.Value = app.Ki_flow;
            app.KdFlowEditField.Value = app.Kd_flow;

            app.KpPressureEditField.Value = app.Kp_pressure;  % PID coefficients when controlling pressure
            app.KiPressureEditField.Value = app.Ki_pressure;
            app.KdPressureEditField.Value = app.Kd_pressure; 
        end
        
        function updateUIColors(app)
            %UPDATEUICOLORS Applies consistent color themes to UI components.
            %
            %   UPDATEUICOLORS() sets the background and font colors of pressure-related,
            %   flow-related, and control-related UI panels and elements according to
            %   the app’s defined color scheme.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

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
                flowElement.BackgroundColor = app.COLOR_FLOW;
            end

            % Other things
            otherOptionsElements = [app.ClearDataButton, app.ExportDataButton, app.PauseGraphsButton];

            for element = otherOptionsElements
                element.BackgroundColor = app.COLOR_BTN_1;
            end

            app.PressureAxes.Title.Color = app.COLOR_DISABLED;
            app.FlowAxes.Title.Color = app.COLOR_DISABLED;

            app.PumpPowerSwitch.FontColor = app.COLOR_ERROR;
            app.PumpPowerSwitchLabel.FontColor = app.COLOR_ERROR;
            
        end

        function success = setGUIConnectionState(app, state)
            %SETGUICONNECTIONSTATE Connects or disconnects the GUI and device state.
            %
            %   SETGUICONNECTIONSTATE(ISCONNECTED) handles all steps to either connect
            %   or disconnect the GUI from the Arduino device. It updates UI controls,
            %   enables or disables user interaction, initializes or clears data, and
            %   manages graphs and device connection status.
            %
            %   Inputs:
            %       ISCONNECTED - true to connect, false to disconnect
            %
            %   Outputs:
            %       SUCCESS - flag to show whether connection was
            %       successful

            dlgTitle = ternary(state, 'Loading...', 'Disconnecting...');
            dlgMessage = 'Please wait...';

            dlg = uiprogressdlg(app.UIFigure, ...
                'Title', dlgTitle, ...
                'Message', dlgMessage, ...
                'Indeterminate', 'on');

            success = true;

            if state
                % UI and Device Setup
                app.ConnectDisconnectButton.Text = "Disconnect";
                app.ConnectDisconnectButton.BackgroundColor = app.COLOR_BTN_DEFAULT;
                app.ConnectedLamp.Color = app.COLOR_CONNECTED;

                % Disable dropdowns after connection
                app.ConnectionDropDown.Enable = "off";
                app.RefreshConnectionsButton.Enable = "off";

                % Set up Arduino if not simulating
                if ~app.SimulateDataCheckBox.Value
                    success = app.setUpArduinoConnection(dlg);

                    if ~success
                        return;
                    end
                else
                    app.SimulateDataCheckBox.Enable = "off";  % prevent user from trying to disable simulation while simulating
                end

                % Clear, log, and connect
                app.clearDataArrays();
                app.startLogging();
                app.isConnected = true;
            else
                % Disconnect logic
                app.setPumpPower(0);
                app.arduinoObj = [];
                app.ConnectionDropDown.Enable = "on";
                app.RefreshConnectionsButton.Enable = "on";
                app.ConnectDisconnectButton.Text = "Connect";
                app.ConnectedLamp.Color = app.COLOR_ERROR;
                app.isConnected = false;
                app.SimulateDataCheckBox.Enable = "on";  % in case it was disabled before
                app.PauseGraphsButton.Value = false;
                app.PauseGraphsButtonValueChanged();

                % Reset pressure and flow fields
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
            end

            % Enable/Disable UI panels and components
            uiElements = [
                app.PressureDetailsPanel;
                app.PressureGraphPanel;
                app.FlowDetailsPanel;
                app.FlowGraphPanel;
                app.PumpControlPanel;
                app.PIDCoefficientsPanel;
                app.OptionsLabel;
                app.PumpControlLabel;
                app.PressureDataTitleLabelPanel;
                app.FlowDataTitleLabelPanel;
                app.OptionsPanel.Children(:);
                ];

            stateStr = ternary(state, 'on', 'off');

            for uiElement = uiElements'
                if isprop(uiElement, 'Enable')
                    uiElement.Enable = stateStr;
                end
            end

            % Graph styling and interactivity
            if state
                graphColor = app.COLOR_NORMAL;
                textColor = [0.15, 0.15, 0.15];
                toolbarVisible = true;
                waitingText = 'Loading...';
            else
                graphColor = [0.90, 0.90, 0.90];
                textColor = [0.50, 0.50, 0.50];
                toolbarVisible = false;
                waitingText = 'No connection';

                delete(findall(app.PressureAxes, 'Type', 'Line'));
                legend(app.PressureAxes, 'off');

                delete(findall(app.FlowAxes, 'Type', 'Line'));
                legend(app.FlowAxes, 'off');
            end

            for axName = ["PressureAxes", "FlowAxes"]
                ax = app.(axName);
                ax.Color = graphColor;
                ax.Title.Color = textColor;
                ax.YColor = textColor;
                ax.XColor = textColor;
                ax.Toolbar.Visible = toolbarVisible;

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

            pause(2);  % For smoother UX, esp. on slower computers
            close(dlg);

            if state
                % Run acquisition
                tic;
                app.runMainLoop();
            end

        end


        function setGUICalibrationState(app, state)
            %SETGUICALIBRATIONSTATE Enables or disables UI components during calibration.
            %
            %   SETGUICALIBRATIONSTATE(STATE) temporarily disables or enables a subset
            %   of the GUI elements to prevent user interference during calibration.
            %
            %   Inputs:
            %       STATE - Logical true to enable, false to disable relevant UI elements
            %
            %   Outputs:
            %       None

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
        

        function updateDataPlots(app)
            %UPDATEDATAPLOTS Updates pressure and flow plots with current data.
            %
            %   UPDATEDATAPLOTS() updates existing plot data instead of redrawing axes.
            %   Only updates legends if visibility toggles have changed.
            %
            %   Inputs:
            %       NEWFLOW - Indicator ('Y' or 'N') whether a new or old 
            %           flow value has been sent

            if app.PauseGraphsButton.Value == true || ~app.isConnected
                drawnow;
                return;
            end

            currentTime = app.t(end);
            cutoffTime = app.rollingAverageCutoffTime;

            %% PRESSURE AXES
            % Update pressure lines
            if app.Pressure1_GraphEnable.Value
                set(app.pressure1Line, 'XData', app.t, 'YData', app.pressure_vals1);
            else
                set(app.pressure1Line, 'XData', nan, 'YData', nan);
            end
            if app.Pressure2_GraphEnable.Value
                set(app.pressure2Line, 'XData', app.t, 'YData', app.pressure_vals2);
            else
                set(app.pressure2Line, 'XData', nan, 'YData', nan);
            end
            if app.Pressure3_GraphEnable.Value
                set(app.pressure3Line, 'XData', app.t, 'YData', app.pressure_vals3);
            else
                set(app.pressure3Line, 'XData', nan, 'YData', nan);
            end

            if contains(app.PID_setpoint_id, "Pressure")
                set(app.pressureTargetLine, 'XData', app.t, 'YData', ones(1,length(app.t))*app.PID_setpoint);
            else
                set(app.pressureTargetLine, 'XData', nan, 'YData', nan);
            end

            % Determine appropriate y-limit
            if contains(app.PID_setpoint_id,"Pressure")
                pressureMax = max([app.pressure_vals1, app.pressure_vals2, app.pressure_vals3, app.PID_setpoint]);  % if setpoint is for pressure adjust y-limit appropriately
            else
                pressureMax = max([app.pressure_vals1, app.pressure_vals2, app.pressure_vals3]);
            end

            % Determine appropriate y-limit
            pressureMin = 0;
            pressureDefaultMax = 100;
            pressureMax = max(pressureDefaultMax, pressureMax * 1.1);  % Add 10% headroom
            ylim(app.PressureAxes, [pressureMin pressureMax]);

            ylims = ylim(app.PressureAxes);
            x_fill = [cutoffTime, currentTime, currentTime, cutoffTime];
            y_fill = [ylims(1), ylims(1), ylims(2), ylims(2)];
            set(app.pressureFill, 'XData', x_fill, 'YData', y_fill);

            % Update flow lines
            if app.Flow1_GraphEnable.Value
                % Line without markers
                set(app.flow1Line, 'XData', app.t, 'YData', app.flow_vals1);

                % Markers
                markerIndices1 = find(app.flow_markers1);
                set(app.flow1MarkerLine, 'XData', app.t(markerIndices1), ...
                    'YData', app.flow_vals1(markerIndices1));
            else
                set(app.flow1Line, 'XData', nan, 'YData', nan);
                set(app.flow1MarkerLine, 'XData', nan, 'YData', nan);
            end

            % Update flow2 line
            if app.Flow2_GraphEnable.Value
                % Line without markers
                set(app.flow2Line, 'XData', app.t, 'YData', app.flow_vals2);

                % Markers
                markerIndices2 = find(app.flow_markers2);
                set(app.flow2MarkerLine, 'XData', app.t(markerIndices2), ...
                    'YData', app.flow_vals2(markerIndices2));
            else
                set(app.flow2Line, 'XData', nan, 'YData', nan);
                set(app.flow2MarkerLine, 'XData', nan, 'YData', nan);
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
            set(app.flowFill, 'XData', x_fill, 'YData', y_fill);

            try
                drawnow limitrate;
            catch
                % Drawnow limitrate sometimes crashes after a while, need
                % to debug why.
                pause(app.PAUSE_LENGTH);
            end
        end
        
        function continueCalibration(app)
            %CONTINUECALIBRATION Performs static head offset calibration.
            %
            %   CONTINUECALIBRATION() disables the UI, powers off the pump to allow
            %   pressure stabilization, collects pressure data, calculates average values,
            %   stores them as static head offsets, and updates the UI.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

            app.setGUICalibrationState(false);  % Disable UI to prevent interference with calibration

            % Turn pump off and let system come to a rest
            d = uiprogressdlg(app.UIFigure, ...
                    'Title', 'Turning pump off...', ...
                    'Message', 'Turning pump off to let system come to a rest', ...
                    'Indeterminate', 'off', ...
                    'Value', 0);

            app.setPumpPower(0);  % static head offset is measured when the system is static (i.e. off)
            for i = 1:app.CALIBRATION_REST_PERIOD
                pause(1)
                d.Value = i/app.CALIBRATION_REST_PERIOD;
            end

            close(d);

            % Run main loop for some time to collect values
            app.clearDataArrays();
            app.runMainLoop(app.CALIBRATION_DURATION);

            % Calculate average pressures
            avg1 = mean(app.pressure_vals1);
            avg2 = mean(app.pressure_vals2);
            avg3 = mean(app.pressure_vals3);

            % Store averages as static head offset calibration values
            app.SHO_Pressure1 = avg1;
            app.Pressure1_EditField_SHO.Value = avg1;
            app.SHO_Pressure2 = avg2;
            app.Pressure2_EditField_SHO.Value = avg2;
            app.SHO_Pressure3 = avg3;
            app.Pressure3_EditField_SHO.Value = avg3;

            app.clearDataArrays();  % to avoid plots from randomly jumping down

            % Show confirmation dialog with the results (nicer formatting)
            app.showCalibrationResults(avg1, avg2, avg3);
        end

        function showCalibrationResults(app, avg1, avg2, avg3)
            %SHOWCALIBRATIONRESULTS Displays a summary dialog of calibration results.
            %
            %   SHOWCALIBRATIONRESULTS(AVG1, AVG2, AVG3) shows a formatted HTML-based
            %   confirmation dialog with the calculated static head offset values for
            %   each pressure sensor. Also resets the app to manual control mode.
            %
            %   Inputs:
            %       AVG1 - Static head offset for Pressure 1
            %       AVG2 - Static head offset for Pressure 2
            %       AVG3 - Static head offset for Pressure 3
            %
            %   Outputs:
            %       None

            % Create custom dialog figure
            mainPos = app.UIFigure.Position;  % Get position of main app window

            figWidth = 450;  % Define size of the new figure
            figHeight = 300;

            figLeft = mainPos(1) + (mainPos(3) - figWidth) / 2;  % Calculate centered position
            figBottom = mainPos(2) + (mainPos(4) - figHeight) / 2;

            fig = uifigure('Name', 'Calibration Complete', ...
                'Position', [figLeft figBottom figWidth figHeight]);  % Create the figure centered over the main app

            % Add uihtml component for full HTML rendering of results
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

            uihtml(fig, ...
                'HTMLSource', htmlContent, ...
                'Position', [80 50 350 220]);

            % Optional: Add close button
            uibutton(fig, 'Text', 'Close', 'Position', [170 10 100 30], 'ButtonPushedFcn', @(btn, event) close(fig));

            % Reset GUI back to normal use
            app.setPumpControlMode("Manual");
            app.setGUICalibrationState(true);
        end
        
        function updateSensorAverages(app)
            %UPDATESENSORAVERAGES Computes rolling averages and updates UI feedback.
            %
            %   UPDATESENSORAVERAGES() calculates the mean and standard deviation of
            %   recent pressure and flow values within a defined time window. It updates
            %   the text in the UI average value fields as well as the
            %   background color if it is that sensor being targetted by PID control
            %   and the mean value is within uncertainty to the current setpoint.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

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

            % Compute mean and std
            [mu1, s1] = mu_sigma(p1);
            [mu2, s2] = mu_sigma(p2);
            [mu3, s3] = mu_sigma(p3);
            [mu4, s4] = mu_sigma(f1);
            [mu5, s5] = mu_sigma(f2);

            % Store in struct
            avgStruct = struct( ...
                'Pressure1', struct('mu', mu1, 'sigma', s1, 'str', sprintf("%.2f ± %.1f", mu1, s1)), ...
                'Pressure2', struct('mu', mu2, 'sigma', s2, 'str', sprintf("%.2f ± %.1f", mu2, s2)), ...
                'Pressure3', struct('mu', mu3, 'sigma', s3, 'str', sprintf("%.2f ± %.1f", mu3, s3)), ...
                'Flow1',     struct('mu', mu4, 'sigma', s4, 'str', sprintf("%.2f ± %.1f", mu4, s4)), ...
                'Flow2',     struct('mu', mu5, 'sigma', s5, 'str', sprintf("%.2f ± %.1f", mu5, s5)) ...
                );

            % Append to log arrays
            app.pressure_vals1_avgs(end+1) = avgStruct.Pressure1.str;
            app.pressure_vals2_avgs(end+1) = avgStruct.Pressure2.str;
            app.pressure_vals3_avgs(end+1) = avgStruct.Pressure3.str;
            app.flow_vals1_avgs(end+1)     = avgStruct.Flow1.str;
            app.flow_vals2_avgs(end+1)     = avgStruct.Flow2.str;

            % Update UI fields
            app.Pressure1_EditField_Avg.Value = avgStruct.Pressure1.str;
            app.Pressure2_EditField_Avg.Value = avgStruct.Pressure2.str;
            app.Pressure3_EditField_Avg.Value = avgStruct.Pressure3.str;
            app.Flow1_EditField_Avg.Value     = avgStruct.Flow1.str;
            app.Flow2_EditField_Avg.Value     = avgStruct.Flow2.str;

            % Highlight appropriate average edit field depending on whether
            % PID control is enabled and has reached target within uncertainty
            % or not
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
                    mu = stats.mu;
                    sigma = stats.sigma;

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

            function [mu, sigma] = mu_sigma(v)
                mu = mean(v);
                sigma = std(v);
            end
        end

        
        function setPumpPower(app, value, varargin)
            % SETPUMPPOWER Sets the pump power using percent or duty cycle.
            %
            %   SETPUMPPOWER(VALUE) sets pump power as a percentage (0–100).
            %   SETPUMPPOWER(VALUE, 'dutyCycle') interprets VALUE as a raw duty cycle.
            %   SETPUMPPOWER(VALUE, 'uiOnly') only updates UI (useful for
            %       UI responsiveness under PID control)
            %
            %   Inputs:
            %       VALUE - Either percent power (0–100) or duty cycle (0–max)
            %       Optional flags: 'dutyCycle', 'uiOnly'
            %
            %   Outputs:
            %       None

            value = double(value);  % convert to double to prevent errors with integers when updating UI

            % Handle optional flags
            isDutyCycle = any(strcmpi(varargin, 'dutyCycle'));
            isUiOnly    = any(strcmpi(varargin, 'uiOnly'));

            % Convert if duty cycle
            if isDutyCycle
                percentPower = value / app.ARDUINO_ANALOGWRITE_MAX * 100;
            else
                percentPower = value;
            end

            % Validate percentPower
            if percentPower < 0 || percentPower > 100
                uialert(app.UIFigure, ...
                    sprintf("Invalid pump power: %.2f. Must be between 0 and 100.", percentPower), ...
                    "Invalid Input");
                return;
            end

            % Update pump duty cycle value for record
            app.pumpDutyCycle = round(percentPower/100 * app.ARDUINO_ANALOGWRITE_MAX);

            if percentPower == 0
                app.PumpPowerSpinner.BackgroundColor = app.COLOR_ERROR;
            else
                alpha = percentPower / 100;
                app.PumpPowerSpinner.BackgroundColor = ...
                    (1 - alpha) * app.COLOR_NORMAL + alpha * app.COLOR_WARNING;
            end

            app.PumpPowerSpinner.Value = percentPower;
            app.PumpPowerSlider.Value = percentPower;
            app.CurrentPumpAnalogWriteValueEditField.Value = app.pumpDutyCycle;

            % Only send update to Arduino if not uiOnly mode
            if ~isUiOnly
                % Send command to Arduino
                command = sprintf("PMP: %d", app.pumpDutyCycle);
                app.sendArduinoMessage(command);
            end
        end

        
        function setUpDataPlots(app)
            % Hide "Waiting for Connection" labels if visible
            if strcmp(app.PressureAxesWaitingForConnectionLabel.Visible, 'on')
                app.PressureAxesWaitingForConnectionLabel.Visible = 'off';
                app.FlowAxesWaitingForConnectionLabel.Visible = 'off';
            end

            cla(app.PressureAxes);
            cla(app.FlowAxes);
            legend(app.PressureAxes,'off');
            legend(app.FlowAxes,'off');

            hold(app.PressureAxes, 'on');
            hold(app.FlowAxes, 'on');

            lineWidth = 2;

            % Initialize pressure plot lines and fill with NaNs
            app.pressure1Line = plot(app.PressureAxes, nan, nan, 'Color', app.COLOR_PRESSURE_2, 'DisplayName', app.pressure1_label, 'LineWidth', lineWidth);
            app.pressure2Line = plot(app.PressureAxes, nan, nan, 'Color', app.COLOR_PRESSURE*0.5, 'DisplayName', app.pressure2_label, 'LineWidth', lineWidth);
            app.pressure3Line = plot(app.PressureAxes, nan, nan, 'Color', [0 0 0], 'DisplayName', app.pressure3_label, 'LineWidth', lineWidth);
            app.pressureTargetLine = plot(app.PressureAxes, nan, nan, 'r--', 'DisplayName', 'Target', 'LineWidth', lineWidth);
            app.pressureFill = fill(app.PressureAxes, nan(1,4), nan(1,4), app.COLOR_PRESSURE*0.6, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Roll. Avg.');

            xlabel(app.PressureAxes, 'Time (s)');
            ylabel(app.PressureAxes, 'Pressure (mmHg)');

            lgdPressure = legend(app.PressureAxes, 'show','Orientation','horizontal');
            lgdPressure.Position = [0.1809    0.0426    0.6998    0.0520];
            lgdPressure.FontSize = 8;

            % Initialize flow plot lines and fill with NaNs
            app.flow1Line = plot(app.FlowAxes, nan, nan, 'Color', app.COLOR_FLOW*0.7, 'DisplayName', app.flow1_label, 'LineWidth', lineWidth);
            app.flow2Line = plot(app.FlowAxes, nan, nan, 'Color', [0 0 0], 'DisplayName', app.flow2_label, 'LineWidth', lineWidth);
            if ~isempty(app.flow1MarkerLine) && isgraphics(app.flow1MarkerLine)
                delete(app.flow1MarkerLine);  % Forces delete of Flow1MarkerLine because cla() doesn't work when HandleVisibility is 'off'
            end
            app.flow1MarkerLine = plot(app.FlowAxes, nan, nan, 'o', 'MarkerSize', 5, 'MarkerEdgeColor', app.COLOR_FLOW*0.7, 'LineStyle', 'none', 'HandleVisibility', 'off');  % marker lines to highlight new flow values
            app.flow2MarkerLine = plot(app.FlowAxes, nan, nan, 'o', 'MarkerSize', 5, 'MarkerEdgeColor', [0 0 0], 'LineStyle', 'none', 'DisplayName', 'New Values');  % Only add one marker line (black one) to legend for clarity

            app.flowTargetLine = plot(app.FlowAxes, nan, nan, 'r--', 'DisplayName', 'Target', 'LineWidth', lineWidth);
            app.flowFill = fill(app.FlowAxes, nan(1,4), nan(1,4), app.COLOR_FLOW*0.6, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Roll. Avg.');

            xlabel(app.FlowAxes, 'Time (s)');
            ylabel(app.FlowAxes, 'Flow (L/min)');

            lgdFlow = legend(app.FlowAxes, 'show','Orientation','Horizontal');
            lgdFlow.Position = [0.2496    0.0424    0.5516    0.0520];
            lgdFlow.FontSize = 8;

            lgdPressure.AutoUpdate = "off";
            lgdFlow.AutoUpdate = "off";
            
        end
        

        function setPulsatileFlow(app, bpm, percentPower)
            % SETPULSATILEFLOW Sends a pulsatile flow command to the Arduino.
            %
            %   SETPULSATILEFLOW(BPM, PERCENTPOWER) sends a pulsatile command with the
            %   given BPM and amplitude (as percent power) to the Arduino.
            %
            %   Inputs:
            %       BPM           - Pulsatile frequency in beats per minute (scalar)
            %       PERCENTPOWER  - Amplitude of flow oscillation in percent (0–100)
            %
            %   Outputs:
            %       None

            % Validate percentPower
            if percentPower < 0 || percentPower > 100
                uialert(app.UIFigure, ...
                    sprintf("Invalid pulsatile power was attempted to be set: %.2f. Value must be between 0 and 100.", percentPower), ...
                    "Invalid Input");
                return;
            elseif bpm < 0
                uialert(app.UIFigure, ...
                    sprintf("Invalid pulsatile bpm was attempted to be set: %.2f. Value must be greater than 0.", bpm), ...
                    "Invalid Input");
                return;
            end

            % Update UI
            app.PulseBPMSpinner.Value = bpm;
            app.PulseBPMSlider.Value = bpm;

            % Calculate amplitude as a fraction
            amplitude = round(percentPower / 100 * app.ARDUINO_ANALOGWRITE_MAX);

            command = sprintf("PULSE: %d, %d", bpm, amplitude);
            app.sendArduinoMessage(command);
        end

        
        function sendArduinoMessage(app, message)
            if app.isConnected
                try
                    % Send pump power command to Arduino
                    if ~isempty(app.arduinoObj)
                        flushoutput(app.arduinoObj);  % in case this was done in the middle of sending another command
                    end

                    % Send command to Arduino
                    if app.SimulateDataCheckBox.Value == false && ~app.pumpStopped
                        writeline(app.arduinoObj,message);
                        app.waitForArduinoMessage();  % Helps prevent crashing
                    end
                catch err
                    uialert(app.UIFigure, ...
                        sprintf("Error encountered while sending message to Arduino: %s \n\n Attempted message: %s", err.message, message), ...
                        "Serial Communication Error");
                end
            end
        end
        
        function sendPIDConfigToArduino(app)
            % SENDPIDCONFIGTOARDUINO Sends PID configuration to the Arduino.
            %
            %   This function formats and transmits a command string to the Arduino to
            %   configure PID control settings. The command includes the target sensor,
            %   setpoint value, and PID coefficients (Kp, Ki, Kd). The Arduino is expected
            %   to parse this command and apply the PID control logic internally.

            % Select correct PID coefficients based on setpoint ID
            if strcmp(app.PID_setpoint_id, "Flow1") || strcmp(app.PID_setpoint_id, "Flow2")
                Kp = app.Kp_flow;
                Ki = app.Ki_flow;
                Kd = app.Kd_flow;
            else
                Kp = app.Kp_pressure;
                Ki = app.Ki_pressure;
                Kd = app.Kd_pressure;
            end

            % Scale all floats by 10000 and convert to integers
            cmd = sprintf("PID: %s, %d, %d, %d, %d", ...
                app.PID_setpoint_id, ...
                round(app.PID_setpoint * app.PID_SCALING_FACTOR), ...
                round(Kp * app.PID_SCALING_FACTOR), ...
                round(Ki * app.PID_SCALING_FACTOR), ...
                round(Kd * app.PID_SCALING_FACTOR));

            app.sendArduinoMessage(cmd);
        end
        
        function printToConsole(app, message)
            app.ConsoleTextArea.Value{end+1} = char(message);
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            %STARTUPFCN Executes once when the app starts.
            %
            %   Initializes default UI states, sets component values,
            %   and prepares the environment for user interaction.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       None

            app.appDir = fileparts(mfilename('fullpath')); % Get app directory
            app.startLogging();
            app.updateSerialPortList();
            addpath(fullfile(app.appDir, 'helpers'));  % add helper functions to path
            app.PID_errors = zeros(1, app.PID_ERRORS_LENGTH);
            app.UIFigure.Resize = 'off';
            app.PUMP_MIN_DUTY_CYCLE = app.PUMP_MIN_POWER_PID * app.ARDUINO_ANALOGWRITE_MAX;
            app.PumpPowerSwitchValueChanged();

            %% Update UI elements
            app.UIFigure.Name = 'Cardiac Catheterization Testing Apparatus (CCTA)';
            disableDefaultInteractivity(app.PressureAxes);
            disableDefaultInteractivity(app.FlowAxes);
            app.updateUIColors();
            app.updateUIPIDCoeffs();
            app.PressureAxes.Visible = "on";  % bug sometimes makes it invisible if this isn't set
            app.FlowAxes.Visible = "on";
            app.PulseBPMSpinner.Value = app.PULSATILE_DEFAULT_BPM;
            app.PulseBPMSlider.Value = app.PULSATILE_DEFAULT_POWER;
        end

        % Button pushed function: ConnectDisconnectButton
        function ConnectDisconnectButtonPushed(app, event)
            %CONNECTDISCONNECTBUTTONPUSHED Handles connect/disconnect button logic.
            %
            %   CONNECTDISCONNECTBUTTONPUSHED() toggles between connecting and
            %   disconnecting the system based on the current button label, and calls
            %   the appropriate connection handler.

            if app.ConnectDisconnectButton.Text == "Connect"
                success = app.setGUIConnectionState(true);
                if ~success
                    app.setGUIConnectionState(false);
                end
            elseif app.ConnectDisconnectButton.Text == "Disconnect"
                app.setGUIConnectionState(false);
            end
        end

        % Button pushed function: ClearDataButton
        function ClearDataButtonPushed(app, event)
            %CLEARDATABUTTONPUSHED Clears sensor data arrays and resets the loop.
            %
            %   CLEARDATABUTTONPUSHED() clears all logged sensor data, shows a
            %   brief loading dialog, and restarts the main loop to reset the time array.

            % Empty time and sensor data arrays
            dlg = uiprogressdlg(app.UIFigure, ...
                'Title', 'Clearing data...', ...
                'Message', 'Please wait...', ...
                'Indeterminate', 'on');

            tic;
            app.clearDataArrays();
            app.setUpDataPlots();

            if toc < 1
                pause(1);  % Pause 1 second if data was cleared quickly, to give user feedback
            end

            tic;  % reset timer so graph data doesn't have a time shift
            close(dlg);

            app.runMainLoop();  % Re-run it here from the beginning to reset time array properly
        end

        % Button pushed function: ExportDataButton
        function ExportDataButtonPushed(app, event)
            %EXPORTDATABUTTONPUSHED Handles exporting of plots, data, and logs.
            %
            %   EXPORTDATABUTTONPUSHED() saves pressure and flow plots, sensor data,
            %   and console output to a timestamped folder. It also generates an Excel
            %   file with logged values and constants, and provides user confirmation.

            dlg = uiprogressdlg(app.UIFigure, ...
                'Title', 'Exporting data...', ...
                'Message', 'Please wait...', ...
                'Indeterminate', 'on');

            % Create a timestamped results folder inside the data folder
            timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd-HH-mm-ss'));
            dataFolder = fullfile(app.appDir, '..', 'data');
            folderName = fullfile(dataFolder, ['CCTA-', timestamp]);
            if ~exist(folderName, 'dir')
                mkdir(folderName);
            end

            % Save Pressure and Flow Axes as images
            exportgraphics(app.PressureAxes, fullfile(folderName, 'PressurePlot.png'));
            exportgraphics(app.FlowAxes, fullfile(folderName, 'FlowPlot.png'));

            % Save data arrays
            data.t = app.t;
            data.setpoint_vals = app.setpoint_vals;
            data.setpoint_ids = app.setpoint_ids;

            data.flow_vals1 = app.flow_vals1;
            data.flow_vals2 = app.flow_vals2;
            data.flow_vals1_avgs = app.flow_vals1_avgs;
            data.flow_vals2_avgs = app.flow_vals2_avgs;

            data.pressure_vals1 = app.pressure_vals1;
            data.pressure_vals2 = app.pressure_vals2;
            data.pressure_vals3 = app.pressure_vals3;
            data.pressure_vals1_avgs = app.pressure_vals1_avgs;
            data.pressure_vals2_avgs = app.pressure_vals2_avgs;
            data.pressure_vals3_avgs = app.pressure_vals3_avgs;

            save(fullfile(folderName, 'ExportedData.mat'), '-struct', 'data');

            % Generate column headers
            flow1_col  = createColumnHeader("Flow1", app.flow1_label);
            flow2_col  = createColumnHeader("Flow2", app.flow2_label);
            press1_col = createColumnHeader("Pressure1", app.pressure1_label);
            press2_col = createColumnHeader("Pressure2", app.pressure2_label);
            press3_col = createColumnHeader("Pressure3", app.pressure3_label);

            % Create data table
            xlsxData = table( ...
                app.t(:), ...
                app.setpoint_vals(:), ...
                app.setpoint_ids(:), ...
                app.flow_vals1(:), ...
                app.flow_vals2(:), ...
                app.pressure_vals1(:), ...
                app.pressure_vals2(:), ...
                app.pressure_vals3(:), ...
                app.flow_vals1_avgs(:), ...
                app.flow_vals2_avgs(:), ...
                app.pressure_vals1_avgs(:), ...
                app.pressure_vals2_avgs(:), ...
                app.pressure_vals3_avgs(:), ...
                'VariableNames', { ...
                'Time (s)', ...
                'Setpoint Value', ...
                'Setpoint ID', ...
                flow1_col, ...
                flow2_col, ...
                press1_col, ...
                press2_col, ...
                press3_col, ...
                [flow1_col, ' Avg (mean ± std)'], ...
                [flow2_col, ' Avg (mean ± std)'], ...
                [press1_col, ' Avg (mean ± std)'], ...
                [press2_col, ' Avg (mean ± std)'], ...
                [press3_col, ' Avg (mean ± std)'], ...
                });

            % Write Excel file with two sheets
            xlsxFile = fullfile(folderName, 'ExportedData.xlsx');
            writetable(xlsxData, xlsxFile, 'Sheet', 'Data');

            % Create constants sheet
            constants = table( ...
                ["Rolling Average Duration (s)"]', ...
                [app.ROLLING_AVERAGE_DURATION]', ...
                'VariableNames', {'Name', 'Value'} ...
                );
            writetable(constants, xlsxFile, 'Sheet', 'Constants');

            % Save console output
            app.saveLog(fullfile(folderName, 'console_output.txt'));

            % User confirmation
            uialert(app.UIFigure, ['Data has been saved to ', folderName], 'Export Successful', 'Icon', 'info');
            selection = uiconfirm(app.UIFigure, ...
                ['Data has been saved to ', folderName], ...
                'Export Successful', ...
                'Options', {'Open Folder', 'OK'}, ...
                'Icon', 'info');

            if strcmp(selection, 'Open Folder')
                if ispc
                    winopen(folderName);
                elseif ismac
                    system(['open ', folderName]);
                else
                    system(['xdg-open ', folderName]);
                end
            end

            close(dlg);

            % --- Nested helper function ---
            function colHeader = createColumnHeader(sensorID, label)
                num = regexp(sensorID, '\d+', 'match', 'once');
                type = regexp(sensorID, '[A-Za-z]+', 'match', 'once');

                if label == string(num)
                    colHeader = sprintf('%s %s', type, num);
                else
                    colHeader = sprintf('%s %s - %s', type, num, label);
                end

                if strcmp(type, 'Flow')
                    colHeader = colHeader + " (L/min)";
                else
                    colHeader = colHeader + " (mmHg)";
                end

                colHeader = char(colHeader);
            end
        end

        % Button pushed function: RefreshConnectionsButton
        function RefreshConnectionsButtonPushed(app, event)
            %REFRESHCONNECTIONSBUTTONPUSHED Refreshes available serial connections.
            %
            %   REFRESHCONNECTIONSBUTTONPUSHED() scans for available serial ports,
            %   updates the dropdown menu with results, and re-enables connection options.


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
            app.setPumpPower(value,'dutyCycle');
            app.setPumpControlMode("Manual");
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
            app.highlightSetPointUI("Flow1");
            app.PID_setpoint_id = "Flow1";
            app.setPumpControlMode("Auto");
        end

        % Value changed function: Flow2_EditField_Target
        function Flow2_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Flow2_EditField_Target.Value;
            app.highlightSetPointUI("Flow2");
            app.PID_setpoint_id = "Flow2";
            app.setPumpControlMode("Auto");
        end

        % Value changed function: Pressure1_EditField_Target
        function Pressure1_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Pressure1_EditField_Target.Value;
            app.highlightSetPointUI("Pressure1");
            app.PID_setpoint_id = "Pressure1";
            app.setPumpControlMode("Auto");
        end

        % Value changed function: Pressure2_EditField_Target
        function Pressure2_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Pressure2_EditField_Target.Value;
            app.highlightSetPointUI("Pressure2");
            app.PID_setpoint_id = "Pressure2";
            app.setPumpControlMode("Auto");
        end

        % Value changed function: Pressure3_EditField_Target
        function Pressure3_EditField_TargetValueChanged(app, event)
            app.PID_setpoint = app.Pressure3_EditField_Target.Value;
            app.highlightSetPointUI("Pressure3");
            app.PID_setpoint_id = "Pressure3";
            app.setPumpControlMode("Auto");
        end

        % Value changed function: KpFlowEditField
        function KpFlowEditFieldValueChanged(app, event)
            app.Kp_flow = app.KpFlowEditField.Value;
            app.dimEditFieldTemporarily(app.KpFlowEditField);
            app.sendPIDConfigToArduino();
        end

        % Value changed function: KiFlowEditField
        function KiFlowEditFieldValueChanged(app, event)
            app.Ki_flow = app.KiFlowEditField.Value;
            app.dimEditFieldTemporarily(app.KiFlowEditField);
            app.sendPIDConfigToArduino();
        end

        % Value changed function: KdFlowEditField
        function KdFlowEditFieldValueChanged(app, event)
            app.Kd_flow = app.KdFlowEditField.Value;
            app.dimEditFieldTemporarily(app.KdFlowEditField);
            app.sendPIDConfigToArduino();
        end

        % Value changed function: KpPressureEditField
        function KpPressureEditFieldValueChanged(app, event)
            app.Kp_pressure = app.KpPressureEditField.Value;
            app.dimEditFieldTemporarily(app.KpPressureEditField); 
            app.sendPIDConfigToArduino();
        end

        % Value changed function: KiPressureEditField
        function KiPressureEditFieldValueChanged(app, event)
            app.Ki_pressure = app.KiPressureEditField.Value;
            app.dimEditFieldTemporarily(app.KiPressureEditField);  
            app.sendPIDConfigToArduino();
        end

        % Value changed function: KdPressureEditField
        function KdPressureEditFieldValueChanged(app, event)
            app.Kd_pressure = app.KdPressureEditField.Value;
            app.dimEditFieldTemporarily(app.KdPressureEditField); 
            app.sendPIDConfigToArduino();
        end

        % Value changed function: SimulateDataCheckBox
        function SimulateDataCheckBoxValueChanged(app, event)
            value = app.SimulateDataCheckBox.Value;

            % Replaces connection dropdown list with a "SIMULATE" entry
            % to clarify the system is not connected to a physical system
            if value
                app.ConnectDisconnectButton.Enable = "on";
                app.ConnectionDropDown.Items = "SIMULATE";
                app.ConnectionDropDown.ItemsData = [];
            else
                if ~isempty(app.portsData)
                    % If ports were available before simulation, return
                    % dropdown back to prior state
                    app.ConnectionDropDown.Items = app.ports;
                    app.ConnectionDropDown.ItemsData = app.portsData;
                else
                    app.ConnectDisconnectButton.Enable = "off";
                end
            end
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
            %% Prepare system for calibration
            app.setPumpControlMode("Manual");

            %% Create non-blocking custom dialog to confirm details before continuing calibration
            %
            % Must create after setting other settings above to prevent rest of
            % code from running in the wrong order

            mainPos = app.UIFigure.Position;  % Get position of main app window

            figWidth = 550;  % Define size of the new figure
            figHeight = 200;

            figLeft = mainPos(1) + (mainPos(3) - figWidth) / 2;  % Calculate centered position
            figBottom = mainPos(2) + (mainPos(4) - figHeight) / 2;

            d = uifigure('Name', 'SHO Calibration', 'Position', [figLeft figBottom figWidth figHeight]);
            d.WindowStyle = 'normal';

            uilabel(d, ...
                'Text', 'Please debubble the pressure sensors (pump control has been set back to manual mode for you), then click the button below to confirm sensors have been debubbled and to start calibration.', ...
                'Position', [20 80 360 80], ...
                'FontSize', 12, ...
                'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', ...
                'WordWrap', 'on');

            uibutton(d, 'Text', 'Confirm', ...
                'Position', [90 20 100 30], ...
                'FontSize', 12, ...
                'ButtonPushedFcn', @(btn, event) onDialogResponse(app, d, 'Confirm'));

            uibutton(d, 'Text', 'Cancel', ...
                'Position', [210 20 100 30], ...
                'FontSize', 12, ...
                'ButtonPushedFcn', @(btn, event) onDialogResponse(app, d, 'Cancel'));

            % Nested function handles the response
            function onDialogResponse(app, dialogFig, choice)
                delete(dialogFig);  % Close dialog
                if strcmp(choice, 'Confirm')
                    app.continueCalibration();
                else
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
            app.setPumpControlMode("Manual");
            app.highlightSetPointUI("None");
            app.PID_setpoint_id = "None";

            app.Pressure1_EditField_Target.Value = 0;
            app.Pressure2_EditField_Target.Value = 0;
            app.Pressure3_EditField_Target.Value = 0;
        end

        % Button pushed function: Flow_ResetAllTargetButton
        function Flow_ResetAllTargetButtonPushed(app, event)
            app.PID_setpoint = [];
            app.setPumpControlMode("Manual");
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

            drawnow;
            
            if value == true
                app.PauseGraphsButton.BackgroundColor = app.COLOR_DISCONNECTED;
                app.PauseGraphsButton.Text = "Unpause Graphs";
                app.PauseGraphsButton.Icon = "assets/play-button-arrowhead.png";
            else
                app.PauseGraphsButton.BackgroundColor = app.COLOR_BTN_1;
                app.PauseGraphsButton.Text = "Pause Graphs";
                app.PauseGraphsButton.Icon = "assets/pause_icon.png";
            end
        end

        % Value changed function: RollingaveragedurationsEditField
        function RollingaveragedurationsEditFieldValueChanged(app, event)
            app.ROLLING_AVERAGE_DURATION = app.RollingaveragedurationsEditField.Value;
            app.dimEditFieldTemporarily(app.RollingaveragedurationsEditField);
        end

        % Value changed function: PumpPowerSlider
        function PumpPowerSliderValueChanged(app, event)
            percentPower = app.PumpPowerSlider.Value;
            app.PumpPowerSpinner.Value = percentPower;
            controlMode = app.PumpControlModeDropDown.Value;
            app.dimEditFieldTemporarily(app.PumpPowerSpinner);
            
            if controlMode == "Manual"
                app.setPumpPower(percentPower);
            elseif controlMode == "Pulsatile"
                bpm = app.PulseBPMSlider.Value;  % TODO: use a separate variable instead of pulling values from the UI
                app.setPulsatileFlow(bpm,percentPower);
            end

        end

        % Value changed function: PumpPowerSpinner
        function PumpPowerSpinnerValueChanged(app, event)
            pumpPowerPercent = app.PumpPowerSpinner.Value;
            app.PumpPowerSlider.Value = pumpPowerPercent;  % set slider to appropriate value
            app.PumpPowerSliderValueChanged();  % pretend slider was just changed instead
        end

        % Value changed function: PumpPowerSwitch
        function PumpPowerSwitchValueChanged(app, event)
            % For safety, return system to manual mode with the pump off
            app.setPumpControlMode("Manual");
            app.setPumpPower(0);

            if app.PumpPowerSwitch.Value == "Off"
                app.PumpPowerSpinner.Enable = "off";
                app.PumpPowerSlider.Enable = "off";
                app.PumpPowerPercentLabel.Enable = "off";
                app.PumpControlModeDropDown.Enable = "off";
                app.PumpControlModeDropdownLabel.Enable = "off";
                app.PumpPowerLamp.Color = app.COLOR_LAMP_OFF;
                app.pumpStopped = true;
            else
                app.PumpPowerSpinner.Enable = "on";
                app.PumpPowerSlider.Enable = "on";
                app.PumpPowerPercentLabel.Enable = "on";
                app.PumpControlModeDropDown.Enable = "on";
                app.PumpControlModeDropdownLabel.Enable = "on";
                app.PumpPowerLamp.Color = app.COLOR_CONNECTED;
                app.pumpStopped = false;
            end
        end

        % Value changed function: PumpControlModeDropDown
        function PumpControlModeDropDownValueChanged(app, event)
            value = app.PumpControlModeDropDown.Value;

            if value == "Auto"
                uialert(app.UIFigure, ...
                    "Please set a target pressure or flow rate by inputting a value in the appropriate ""Target"" box instead of selecting automatic control through this dropdown. The system has been switched back to Manual mode.", ...
                    "Permission Denied", ...
                    'Icon', 'warning');
                app.setPumpControlMode("Manual");
            else
                app.setPumpControlMode(value);
            end
        end

        % Value changed function: PulseBPMSpinner
        function PulseBPMSpinnerValueChanged(app, event)
            bpm = app.PulseBPMSpinner.Value;
            percentPower = app.PumpPowerSlider.Value;
            app.setPulsatileFlow(bpm, percentPower);
        end

        % Value changed function: PulseBPMSlider
        function PulseBPMSliderValueChanged(app, event)
            bpm = round(app.PulseBPMSlider.Value);
            percentPower = app.PumpPowerSlider.Value;
            app.setPulsatileFlow(bpm, percentPower);
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
            app.UIFigure.Position = [100 100 1240 567];
            app.UIFigure.Name = 'MATLAB App';

            % Create MainTabGroup
            app.MainTabGroup = uitabgroup(app.UIFigure);
            app.MainTabGroup.Position = [1 0 1240 568];

            % Create MainTab
            app.MainTab = uitab(app.MainTabGroup);
            app.MainTab.Title = 'Main';

            % Create PressureDetailsPanel
            app.PressureDetailsPanel = uipanel(app.MainTab);
            app.PressureDetailsPanel.Enable = 'off';
            app.PressureDetailsPanel.BackgroundColor = [0.8392 0.8275 0.8588];
            app.PressureDetailsPanel.FontWeight = 'bold';
            app.PressureDetailsPanel.Position = [329 16 435 150];

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
            app.FlowDetailsPanel.Position = [775 16 448 150];

            % Create Label_6
            app.Label_6 = uilabel(app.FlowDetailsPanel);
            app.Label_6.FontSize = 18;
            app.Label_6.FontWeight = 'bold';
            app.Label_6.Position = [110 95 25 23];
            app.Label_6.Text = '';

            % Create Flow1_EditField_Target
            app.Flow1_EditField_Target = uieditfield(app.FlowDetailsPanel, 'numeric');
            app.Flow1_EditField_Target.Limits = [0 Inf];
            app.Flow1_EditField_Target.ValueChangedFcn = createCallbackFcn(app, @Flow1_EditField_TargetValueChanged, true);
            app.Flow1_EditField_Target.HorizontalAlignment = 'center';
            app.Flow1_EditField_Target.Tooltip = {'Target value for PID control'};
            app.Flow1_EditField_Target.Position = [161 94 44 22];

            % Create Flow2_EditField_Target
            app.Flow2_EditField_Target = uieditfield(app.FlowDetailsPanel, 'numeric');
            app.Flow2_EditField_Target.Limits = [0 Inf];
            app.Flow2_EditField_Target.ValueChangedFcn = createCallbackFcn(app, @Flow2_EditField_TargetValueChanged, true);
            app.Flow2_EditField_Target.HorizontalAlignment = 'center';
            app.Flow2_EditField_Target.Tooltip = {'Target value for PID control'};
            app.Flow2_EditField_Target.Position = [161 63 44 22];

            % Create CurrentLabel_2
            app.CurrentLabel_2 = uilabel(app.FlowDetailsPanel);
            app.CurrentLabel_2.Position = [106 116 45 22];
            app.CurrentLabel_2.Text = 'Current';

            % Create TargetLabel_2
            app.TargetLabel_2 = uilabel(app.FlowDetailsPanel);
            app.TargetLabel_2.HorizontalAlignment = 'center';
            app.TargetLabel_2.Position = [161 116 44 22];
            app.TargetLabel_2.Text = 'Target';

            % Create Label_7
            app.Label_7 = uilabel(app.FlowDetailsPanel);
            app.Label_7.HorizontalAlignment = 'center';
            app.Label_7.FontWeight = 'bold';
            app.Label_7.Position = [60 95 25 22];
            app.Label_7.Text = '1';

            % Create Flow1_EditField_Current
            app.Flow1_EditField_Current = uieditfield(app.FlowDetailsPanel, 'numeric');
            app.Flow1_EditField_Current.Editable = 'off';
            app.Flow1_EditField_Current.HorizontalAlignment = 'center';
            app.Flow1_EditField_Current.Tooltip = {'Current flow reading (L/min) for flow sensor 1 (see label on sensor).'};
            app.Flow1_EditField_Current.Position = [107 94 43 22];

            % Create Label_8
            app.Label_8 = uilabel(app.FlowDetailsPanel);
            app.Label_8.HorizontalAlignment = 'center';
            app.Label_8.FontWeight = 'bold';
            app.Label_8.Position = [60 63 25 22];
            app.Label_8.Text = '2';

            % Create Flow2_EditField_Current
            app.Flow2_EditField_Current = uieditfield(app.FlowDetailsPanel, 'numeric');
            app.Flow2_EditField_Current.Editable = 'off';
            app.Flow2_EditField_Current.HorizontalAlignment = 'center';
            app.Flow2_EditField_Current.Tooltip = {'Current flow reading (L/min) for flow sensor 2 (see label on sensor).'};
            app.Flow2_EditField_Current.Position = [107 63 43 22];

            % Create Flow2_GraphEnable
            app.Flow2_GraphEnable = uicheckbox(app.FlowDetailsPanel);
            app.Flow2_GraphEnable.Text = '';
            app.Flow2_GraphEnable.Position = [393 65 15 22];
            app.Flow2_GraphEnable.Value = true;

            % Create Flow1_GraphEnable
            app.Flow1_GraphEnable = uicheckbox(app.FlowDetailsPanel);
            app.Flow1_GraphEnable.Text = '';
            app.Flow1_GraphEnable.Position = [393 94 14 22];
            app.Flow1_GraphEnable.Value = true;

            % Create Label_10
            app.Label_10 = uilabel(app.FlowDetailsPanel);
            app.Label_10.HorizontalAlignment = 'center';
            app.Label_10.FontSize = 18;
            app.Label_10.Position = [385 118 30 23];
            app.Label_10.Text = '👁️';

            % Create SensorLabel_2
            app.SensorLabel_2 = uilabel(app.FlowDetailsPanel);
            app.SensorLabel_2.HorizontalAlignment = 'center';
            app.SensorLabel_2.FontWeight = 'bold';
            app.SensorLabel_2.Position = [50 117 45 22];
            app.SensorLabel_2.Text = 'Sensor';

            % Create Flow2_LabelField
            app.Flow2_LabelField = uieditfield(app.FlowDetailsPanel, 'text');
            app.Flow2_LabelField.ValueChangedFcn = createCallbackFcn(app, @Flow2_LabelFieldValueChanged, true);
            app.Flow2_LabelField.HorizontalAlignment = 'center';
            app.Flow2_LabelField.FontAngle = 'italic';
            app.Flow2_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow2_LabelField.Position = [312 65 63 22];
            app.Flow2_LabelField.Value = '2';

            % Create Label_9
            app.Label_9 = uilabel(app.FlowDetailsPanel);
            app.Label_9.HorizontalAlignment = 'center';
            app.Label_9.Position = [312 118 63 22];

            % Create Flow1_LabelField
            app.Flow1_LabelField = uieditfield(app.FlowDetailsPanel, 'text');
            app.Flow1_LabelField.ValueChangedFcn = createCallbackFcn(app, @Flow1_LabelFieldValueChanged, true);
            app.Flow1_LabelField.HorizontalAlignment = 'center';
            app.Flow1_LabelField.FontAngle = 'italic';
            app.Flow1_LabelField.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow1_LabelField.Position = [312 95 63 22];
            app.Flow1_LabelField.Value = '1';

            % Create RollAvgSDLabel_2
            app.RollAvgSDLabel_2 = uilabel(app.FlowDetailsPanel);
            app.RollAvgSDLabel_2.HorizontalAlignment = 'center';
            app.RollAvgSDLabel_2.Position = [219 116 82 22];
            app.RollAvgSDLabel_2.Text = 'Roll. Avg ± SD';

            % Create Flow2_EditField_Avg
            app.Flow2_EditField_Avg = uieditfield(app.FlowDetailsPanel, 'text');
            app.Flow2_EditField_Avg.Editable = 'off';
            app.Flow2_EditField_Avg.HorizontalAlignment = 'center';
            app.Flow2_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow2_EditField_Avg.Position = [217 63 85 22];
            app.Flow2_EditField_Avg.Value = '0';

            % Create Flow_ResetAllTargetButton
            app.Flow_ResetAllTargetButton = uibutton(app.FlowDetailsPanel, 'push');
            app.Flow_ResetAllTargetButton.ButtonPushedFcn = createCallbackFcn(app, @Flow_ResetAllTargetButtonPushed, true);
            app.Flow_ResetAllTargetButton.BackgroundColor = [0.902 0.902 0.902];
            app.Flow_ResetAllTargetButton.FontSize = 8;
            app.Flow_ResetAllTargetButton.Position = [159 42 47 15];
            app.Flow_ResetAllTargetButton.Text = 'Reset All';

            % Create Flow_ResetAllLabelButton
            app.Flow_ResetAllLabelButton = uibutton(app.FlowDetailsPanel, 'push');
            app.Flow_ResetAllLabelButton.ButtonPushedFcn = createCallbackFcn(app, @Flow_ResetAllLabelButtonPushed, true);
            app.Flow_ResetAllLabelButton.BackgroundColor = [0.902 0.902 0.902];
            app.Flow_ResetAllLabelButton.FontSize = 8;
            app.Flow_ResetAllLabelButton.Position = [313 44 62 15];
            app.Flow_ResetAllLabelButton.Text = 'Reset All';

            % Create Flow1_EditField_Avg
            app.Flow1_EditField_Avg = uieditfield(app.FlowDetailsPanel, 'text');
            app.Flow1_EditField_Avg.Editable = 'off';
            app.Flow1_EditField_Avg.HorizontalAlignment = 'center';
            app.Flow1_EditField_Avg.Tooltip = {'Graph label for this sensor (changes legend entry).'};
            app.Flow1_EditField_Avg.Position = [217 95 85 22];
            app.Flow1_EditField_Avg.Value = '0';

            % Create PressureGraphPanel
            app.PressureGraphPanel = uipanel(app.MainTab);
            app.PressureGraphPanel.BorderColor = [0.4902 0.4902 0.4902];
            app.PressureGraphPanel.Enable = 'off';
            app.PressureGraphPanel.BackgroundColor = [0.8392 0.8314 0.8588];
            app.PressureGraphPanel.FontAngle = 'italic';
            app.PressureGraphPanel.Position = [330 172 435 329];

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
            app.PumpControlPanel.BackgroundColor = [0.902 0.902 0.902];
            app.PumpControlPanel.FontWeight = 'bold';
            app.PumpControlPanel.Position = [15 108 302 195];

            % Create PumpPowerSlider
            app.PumpPowerSlider = uislider(app.PumpControlPanel);
            app.PumpPowerSlider.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSliderValueChanged, true);
            app.PumpPowerSlider.Enable = 'off';
            app.PumpPowerSlider.Tooltip = {'Sends a signal to the pump to operate from 0 to 100% power. 100% power represents 100% duty cycle, at the voltage of your power supply.'; ''; 'Note: In ''Pulsatile'' mode, "Pump Power" instead represents the amplitude of oscillations.'};
            app.PumpPowerSlider.Position = [129 91 150 3];

            % Create PumpPowerSpinner
            app.PumpPowerSpinner = uispinner(app.PumpControlPanel);
            app.PumpPowerSpinner.Limits = [0 100];
            app.PumpPowerSpinner.RoundFractionalValues = 'on';
            app.PumpPowerSpinner.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSpinnerValueChanged, true);
            app.PumpPowerSpinner.Tooltip = {'Sends a signal to the pump to operate from 0 to 100% power. 100% power represents 100% duty cycle, at the voltage of your power supply.'; ''; 'Note: In ''Pulsatile'' mode, "Pump Power" instead represents the amplitude of oscillations.'};
            app.PumpPowerSpinner.Position = [18 65 92 17];

            % Create PumpPowerSwitchLabel
            app.PumpPowerSwitchLabel = uilabel(app.PumpControlPanel);
            app.PumpPowerSwitchLabel.HorizontalAlignment = 'right';
            app.PumpPowerSwitchLabel.FontWeight = 'bold';
            app.PumpPowerSwitchLabel.Position = [31 158 78 22];
            app.PumpPowerSwitchLabel.Text = 'Pump Power';

            % Create PumpPowerSwitch
            app.PumpPowerSwitch = uiswitch(app.PumpControlPanel, 'slider');
            app.PumpPowerSwitch.ValueChangedFcn = createCallbackFcn(app, @PumpPowerSwitchValueChanged, true);
            app.PumpPowerSwitch.FontWeight = 'bold';
            app.PumpPowerSwitch.Position = [184 159 45 20];

            % Create PumpPowerPercentLabel
            app.PumpPowerPercentLabel = uilabel(app.PumpControlPanel);
            app.PumpPowerPercentLabel.FontWeight = 'bold';
            app.PumpPowerPercentLabel.Position = [15 84 100 22];
            app.PumpPowerPercentLabel.Text = 'Pump Power (%)';

            % Create PumpControlModeDropdownLabel
            app.PumpControlModeDropdownLabel = uilabel(app.PumpControlPanel);
            app.PumpControlModeDropdownLabel.HorizontalAlignment = 'right';
            app.PumpControlModeDropdownLabel.FontWeight = 'bold';
            app.PumpControlModeDropdownLabel.Position = [26 118 82 22];
            app.PumpControlModeDropdownLabel.Text = 'Control Mode';

            % Create PumpPowerLamp
            app.PumpPowerLamp = uilamp(app.PumpControlPanel);
            app.PumpPowerLamp.Position = [271 160 20 20];
            app.PumpPowerLamp.Color = [0.502 0.502 0.502];

            % Create PumpControlModeDropDown
            app.PumpControlModeDropDown = uidropdown(app.PumpControlPanel);
            app.PumpControlModeDropDown.Items = {'Manual', 'Auto', 'Pulsatile'};
            app.PumpControlModeDropDown.ValueChangedFcn = createCallbackFcn(app, @PumpControlModeDropDownValueChanged, true);
            app.PumpControlModeDropDown.Tooltip = {'Choose how to control the pump:'; '- Manual: Set pump power yourself'; '- Auto: Use PID control to achieve targets'; '- Pulsatile: Simulate pulsatile flow'};
            app.PumpControlModeDropDown.Position = [146 118 117 22];
            app.PumpControlModeDropDown.Value = 'Manual';

            % Create PulseBPMSlider
            app.PulseBPMSlider = uislider(app.PumpControlPanel);
            app.PulseBPMSlider.Limits = [1 200];
            app.PulseBPMSlider.MajorTicks = [1 50 100 150 200];
            app.PulseBPMSlider.ValueChangedFcn = createCallbackFcn(app, @PulseBPMSliderValueChanged, true);
            app.PulseBPMSlider.Enable = 'off';
            app.PulseBPMSlider.Tooltip = {'Sends a PWM signal to the pump from 0 to 100% power. 100% power represents the full voltage of your power supply.'};
            app.PulseBPMSlider.Position = [128 40 150 3];
            app.PulseBPMSlider.Value = 60;

            % Create PulseBPMSpinner
            app.PulseBPMSpinner = uispinner(app.PumpControlPanel);
            app.PulseBPMSpinner.Limits = [1 200];
            app.PulseBPMSpinner.RoundFractionalValues = 'on';
            app.PulseBPMSpinner.ValueChangedFcn = createCallbackFcn(app, @PulseBPMSpinnerValueChanged, true);
            app.PulseBPMSpinner.Enable = 'off';
            app.PulseBPMSpinner.Position = [17 14 92 17];
            app.PulseBPMSpinner.Value = 60;

            % Create PulseBPMLabel
            app.PulseBPMLabel = uilabel(app.PumpControlPanel);
            app.PulseBPMLabel.HorizontalAlignment = 'center';
            app.PulseBPMLabel.FontWeight = 'bold';
            app.PulseBPMLabel.Position = [14 33 96 22];
            app.PulseBPMLabel.Text = 'Pulse BPM';

            % Create FlowGraphPanel
            app.FlowGraphPanel = uipanel(app.MainTab);
            app.FlowGraphPanel.Enable = 'off';
            app.FlowGraphPanel.BackgroundColor = [0.7725 0.8314 0.7569];
            app.FlowGraphPanel.Position = [775 172 448 329];

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
            app.OptionsPanel.Position = [11 350 305 88];

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
            app.ConnectionDropDownLabel.Position = [16 506 104 23];
            app.ConnectionDropDownLabel.Text = 'Connection';

            % Create ConnectionDropDown
            app.ConnectionDropDown = uidropdown(app.MainTab);
            app.ConnectionDropDown.Items = {};
            app.ConnectionDropDown.FontSize = 14;
            app.ConnectionDropDown.FontWeight = 'bold';
            app.ConnectionDropDown.FontColor = [0.149 0.149 0.149];
            app.ConnectionDropDown.BackgroundColor = [0.9412 0.9412 0.9412];
            app.ConnectionDropDown.Position = [15 471 126 31];
            app.ConnectionDropDown.Value = {};

            % Create ConnectedLamp
            app.ConnectedLamp = uilamp(app.MainTab);
            app.ConnectedLamp.Position = [293 477 20 20];
            app.ConnectedLamp.Color = [1 0 0];

            % Create RefreshConnectionsButton
            app.RefreshConnectionsButton = uibutton(app.MainTab, 'push');
            app.RefreshConnectionsButton.ButtonPushedFcn = createCallbackFcn(app, @RefreshConnectionsButtonPushed, true);
            app.RefreshConnectionsButton.Icon = fullfile(pathToMLAPP, 'assets', 'Refresh_icon.png');
            app.RefreshConnectionsButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RefreshConnectionsButton.FontSize = 14;
            app.RefreshConnectionsButton.FontWeight = 'bold';
            app.RefreshConnectionsButton.FontColor = [0.149 0.149 0.149];
            app.RefreshConnectionsButton.Position = [253 472 31 32];
            app.RefreshConnectionsButton.Text = '';

            % Create ConnectDisconnectButton
            app.ConnectDisconnectButton = uibutton(app.MainTab, 'push');
            app.ConnectDisconnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectDisconnectButtonPushed, true);
            app.ConnectDisconnectButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.ConnectDisconnectButton.FontSize = 14;
            app.ConnectDisconnectButton.FontWeight = 'bold';
            app.ConnectDisconnectButton.FontColor = [0.149 0.149 0.149];
            app.ConnectDisconnectButton.Enable = 'off';
            app.ConnectDisconnectButton.Position = [147 472 98 31];
            app.ConnectDisconnectButton.Text = 'Connect';

            % Create OptionsLabel
            app.OptionsLabel = uilabel(app.MainTab);
            app.OptionsLabel.FontSize = 18;
            app.OptionsLabel.FontWeight = 'bold';
            app.OptionsLabel.FontColor = [0.149 0.149 0.149];
            app.OptionsLabel.Enable = 'off';
            app.OptionsLabel.Position = [18 432 73 23];
            app.OptionsLabel.Text = 'Options';

            % Create PressureDataTitleLabelPanel
            app.PressureDataTitleLabelPanel = uipanel(app.MainTab);
            app.PressureDataTitleLabelPanel.Enable = 'off';
            app.PressureDataTitleLabelPanel.Position = [330 506 435 30];

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
            app.FlowDataTitleLabelPanel.Position = [775 506 448 30];

            % Create FlowDataTitleLabel
            app.FlowDataTitleLabel = uilabel(app.FlowDataTitleLabelPanel);
            app.FlowDataTitleLabel.HorizontalAlignment = 'center';
            app.FlowDataTitleLabel.FontSize = 18;
            app.FlowDataTitleLabel.FontWeight = 'bold';
            app.FlowDataTitleLabel.FontColor = [0.149 0.149 0.149];
            app.FlowDataTitleLabel.Position = [0 4 447 23];
            app.FlowDataTitleLabel.Text = 'Flow Data (L/min)';

            % Create PumpControlLabel
            app.PumpControlLabel = uilabel(app.MainTab);
            app.PumpControlLabel.FontSize = 18;
            app.PumpControlLabel.FontWeight = 'bold';
            app.PumpControlLabel.FontColor = [0.149 0.149 0.149];
            app.PumpControlLabel.Enable = 'off';
            app.PumpControlLabel.Position = [18 314 124 23];
            app.PumpControlLabel.Text = 'Pump Control';

            % Create SettingsTab
            app.SettingsTab = uitab(app.MainTabGroup);
            app.SettingsTab.Title = 'Settings';

            % Create PIDCoefficientsPanel
            app.PIDCoefficientsPanel = uipanel(app.SettingsTab);
            app.PIDCoefficientsPanel.Enable = 'off';
            app.PIDCoefficientsPanel.Title = 'PID Coefficients';
            app.PIDCoefficientsPanel.FontWeight = 'bold';
            app.PIDCoefficientsPanel.Position = [17 387 469 139];

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
            app.DeveloperToolsPanel.Position = [499 387 260 140];

            % Create SimulateDataCheckBox
            app.SimulateDataCheckBox = uicheckbox(app.DeveloperToolsPanel);
            app.SimulateDataCheckBox.ValueChangedFcn = createCallbackFcn(app, @SimulateDataCheckBoxValueChanged, true);
            app.SimulateDataCheckBox.Tooltip = {'Simulates data without requiring a connection to a microcontroller. Mostly useful for testing that the GUI works. '; ''; 'To simulate, check the checkbox then go to the ''Main'' tab and click ''Connect''.'};
            app.SimulateDataCheckBox.Text = 'Simulate Data?';
            app.SimulateDataCheckBox.Position = [14 88 160 22];

            % Create CurrentPumpAnalogWriteValue0255Label
            app.CurrentPumpAnalogWriteValue0255Label = uilabel(app.SettingsTab);
            app.CurrentPumpAnalogWriteValue0255Label.HorizontalAlignment = 'right';
            app.CurrentPumpAnalogWriteValue0255Label.FontWeight = 'bold';
            app.CurrentPumpAnalogWriteValue0255Label.Position = [513 425 117 30];
            app.CurrentPumpAnalogWriteValue0255Label.Text = {'Current Pump '; 'Analog Write Value '};

            % Create CurrentPumpAnalogWriteValueEditField
            app.CurrentPumpAnalogWriteValueEditField = uieditfield(app.SettingsTab, 'numeric');
            app.CurrentPumpAnalogWriteValueEditField.Limits = [0 255];
            app.CurrentPumpAnalogWriteValueEditField.RoundFractionalValues = 'on';
            app.CurrentPumpAnalogWriteValueEditField.ValueChangedFcn = createCallbackFcn(app, @CurrentPumpAnalogWriteValueEditFieldValueChanged, true);
            app.CurrentPumpAnalogWriteValueEditField.Editable = 'off';
            app.CurrentPumpAnalogWriteValueEditField.Position = [645 429 96 22];

            % Create OtherSettingsPanel
            app.OtherSettingsPanel = uipanel(app.SettingsTab);
            app.OtherSettingsPanel.Title = 'Other Settings';
            app.OtherSettingsPanel.FontWeight = 'bold';
            app.OtherSettingsPanel.Position = [17 250 469 124];

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

            % Create RollingaveragedurationsEditFieldLabel
            app.RollingaveragedurationsEditFieldLabel = uilabel(app.OtherSettingsPanel);
            app.RollingaveragedurationsEditFieldLabel.HorizontalAlignment = 'right';
            app.RollingaveragedurationsEditFieldLabel.Position = [90 33 152 22];
            app.RollingaveragedurationsEditFieldLabel.Text = 'Rolling average duration (s)';

            % Create RollingaveragedurationsEditField
            app.RollingaveragedurationsEditField = uieditfield(app.OtherSettingsPanel, 'numeric');
            app.RollingaveragedurationsEditField.Limits = [0 Inf];
            app.RollingaveragedurationsEditField.ValueChangedFcn = createCallbackFcn(app, @RollingaveragedurationsEditFieldValueChanged, true);
            app.RollingaveragedurationsEditField.Position = [257 33 100 22];
            app.RollingaveragedurationsEditField.Value = 10;

            % Create HelpTab
            app.HelpTab = uitab(app.MainTabGroup);
            app.HelpTab.Title = 'Help';

            % Create HTML
            app.HTML = uihtml(app.HelpTab);
            app.HTML.HTMLSource = './assets/help.html';
            app.HTML.Position = [16 73 1201 456];

            % Create ConsoleTab
            app.ConsoleTab = uitab(app.MainTabGroup);
            app.ConsoleTab.Title = 'Console';

            % Create ConsoleTextArea
            app.ConsoleTextArea = uitextarea(app.ConsoleTab);
            app.ConsoleTextArea.WordWrap = 'off';
            app.ConsoleTextArea.FontName = 'Consolas';
            app.ConsoleTextArea.Tooltip = {'Console showing important messages from the Arduino as well as other errors and warnings.'};
            app.ConsoleTextArea.Position = [16 16 1206 514];

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