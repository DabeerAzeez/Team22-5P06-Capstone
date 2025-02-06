%% Capstone Project: Flow and Pressure Sensor Data Visualization
% This script reads flow and pressure sensor data from an Arduino and plots it in real-time.
% The pressure sensor values are converted from raw ADC values (0-1023) to voltage (0-5V),
% and then interpolated to pressure (mmHg) using given data points.

PLOT_RAW_DATA = true; 
COM_PORT = serialportlist;
fs = 100;  % Sampling rate: 100Hz
ts = 1/fs;

% pressure vs. voltage data points for interpolation
pressure_mmHg = [0, 26.2, 48.7, 74.2, 99.0, 124.5, 150];
voltage_V = [0.131, 0.850, 1.435, 2.1407, 2.8055, 3.1573, 3.25];

close all;
if exist("arduinoObj", "var")
    arduinoObj = [];
    clear arduinoObj;
end

fig = figure;


arduinoObj = serialport(COM_PORT, 9600); 
t = [];
raw_flow = [];
raw_voltage = [];
converted_pressure = [];

tic;
while(1)
    flush(arduinoObj);
   
    for j = 1:3
        line = readline(arduinoObj);
    end
    
    % Parse the flow and pressure values
    data = sscanf(line, 'Flow: %f L/min ; Pressure: %d');
    if numel(data) == 2
        flow_value = data(1);
        raw_adc_value = data(2); 
    else
        continue; 
    end
    
    % Convert raw ADC to voltage (assuming 10-bit ADC, 5V reference)
    voltage_value = (raw_adc_value / 1023) * 5;
    
    pressure_value = interp1(voltage_V, pressure_mmHg, voltage_value, 'linear', 'extrap');
    %pressure_value = pressure_value - 2;
    disp(['Flow: ' num2str(flow_value) ', Voltage: ' num2str(voltage_value) ' V, Pressure: ' num2str(pressure_value) ' mmHg']);

    raw_flow = [raw_flow flow_value];
    raw_voltage = [raw_voltage voltage_value];
    converted_pressure = [converted_pressure pressure_value];
    t = [t toc];

   
    if PLOT_RAW_DATA
        subplot(3, 1, 1);
        plot(t, raw_flow, 'b');
        title('Flow Sensor Data');
        xlabel('Time (s)');
        ylabel('Flow (L/min)');
        
        subplot(3, 1, 2);
        plot(t, raw_voltage, 'g');
        title('Raw Voltage Data');
        xlabel('Time (s)');
        ylabel('Voltage (V)');
        ylim([0 5]);

        subplot(3, 1, 3);
        plot(t, converted_pressure, 'r');
        title('Pressure Data');
        xlabel('Time (s)');
        ylabel('Pressure (mmHg)');
        ylim([0 200]);
    else
        subplot(2, 1, 1);
        plot(t, raw_flow, 'b');
        title('Flow Sensor Data');
        xlabel('Time (s)');
        ylabel('Flow (L/min)');

        subplot(2, 1, 2);
        plot(t, converted_pressure, 'r');
        title('Pressure Data');
        xlabel('Time (s)');
        ylabel('Pressure (mmHg)');
        % ylim([0 200]);
    end
    
    pause(0.05); 
end
