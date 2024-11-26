%% Capstone Project: Flow and Pressure Sensor Data Visualization
% This script reads flow and pressure sensor data from an Arduino and plots it in real-time.
% The data is obtained from a YF‐S201 Water Flow Sensor and a Deltran-01 pressure sensor from Utah Medical connected to the Arduino.
% The script provides three visualization options:
%   1. Flow rate over time (if enabled with the 'plot_raw_data' toggle).
%   2. Raw pressure sensor data over time.
%   3. Fitted pressure data over time.
% The 'plot_raw_data' boolean variable at the top of the script allows easy enabling/disabling of the flow rate plot.
% The user can adjust the COM port as needed.

% Constants
PLOT_RAW_DATA = true;  % Toggle to enable/disable raw data plot
COM_PORT = "COM5";

load("./Data/fitdata25-Apr.mat")

close all;
if exist("arduinoObj", "var")
    arduinoObj = [];
    clear arduinoObj;
end

fs = 100;  % Current sampling rate: 100Hz
ts = 1/fs;

fig = figure;
sgtitle(fig, 'Group 22 5P06 Capstone Data Visualization', 'FontSize', 14, 'FontWeight', 'bold');

arduinoObj = serialport(COM_PORT, 9600); % Ensure you adjust COM port as needed
a = zeros(750, 1);
tic;
t = [];
raw_flow = [];
raw_pressure = [];
fitted_pressure = [];

while(1)
    % Clear the input buffer to get the most recent data
    flush(arduinoObj);
    
    % Read a couple times (the first few values after flushing are not
    % consistent so we need to read a few times)
    for j = 1:3
        line = readline(arduinoObj);
    end
    
    % Parse the flow and pressure values from the line
    data = sscanf(line, 'Flow: %f L/min ; Pressure: %d');
    if numel(data) == 2
        flow_value = data(1);
        pressure_value = data(2);
    else
        continue; % Skip iteration if data is not correctly parsed
    end
    
    disp(['Flow: ' num2str(flow_value) ', Pressure: ' num2str(pressure_value)]);

    % Store and process the values
    raw_flow = [raw_flow flow_value];
    raw_pressure = [raw_pressure pressure_value];
    fitted_pressure = [fitted_pressure polyval(p, pressure_value)];
    a = [a(2:end); polyval(p, pressure_value)];
    t = [t toc];

    % Plotting
    if PLOT_RAW_DATA
        subplot(1, 3, 1);
        plot(t, raw_flow, 'b');
        title('Flow Sensor Data');
        xlabel('Time (s)');
        ylabel('Flow (L/min)');
        
        subplot(1, 3, 2);
        plot(t, raw_pressure, 'g');
        title('Raw Pressure Sensor Data');
        xlabel('Time (s)');
        ylabel('Pressure Value');
        ylim([0 1024]);  % 10-bit ADC on the Arduino

        subplot(1, 3, 3);
        plot(t, fitted_pressure, 'r');
        title('Fitted Pressure Data');
        xlabel('Time (s)');
        ylabel('Pressure (mmHg)');
        ylim([-50 400]);  % Output range of the sensor
    else
        subplot(1, 2, 1);
        plot(t, raw_flow, 'b');
        title('Flow Sensor Data');
        xlabel('Time (s)');
        ylabel('Flow (L/min)');

        subplot(1, 2, 2);
        plot(t, fitted_pressure, 'r');
        title('Fitted Pressure Data');
        xlabel('Time (s)');
        ylabel('Pressure (mmHg)');
        ylim([-50 400]);  % Output range of the sensor
    end
    
    % Pause for 0.05 seconds to allow plotting
    pause(0.05);
end
