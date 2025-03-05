% MATLAB Script to Test Arduino Serial Communication
clear;

% Define serial port (change COMx to match your system)
arduinoPort = "COM5";  % Update this to match your Arduino's port
baudRate = 9600;

% Open serial connection
serialObj = serialport(arduinoPort, baudRate);
configureTerminator(serialObj, "LF");  % Use newline as terminator
flush(serialObj);  % Clear any old data

pause(3);

% Example command string to send
% pick step number from 0 to 1400 (200 per rev. 7 revs max)

% Send command to Arduino
writeline(serialObj, "Pressure Motor Step Number: 200, Flow Motor Voltage: 50");
pause(0.5);
writeline(serialObj, "Pressure Motor Step Number: -200, Flow Motor Voltage: 50");
pause(0.5);
writeline(serialObj, "Pressure Motor Step Number: 400, Flow Motor Voltage: 50");
pause(0.5);

% Continuous loop to read responses from Arduino
disp("Waiting for response from Arduino... Press Ctrl+C to stop.");

while true
    if serialObj.NumBytesAvailable > 0  % Check if data is available
        data = readline(serialObj);  % Read incoming data
        % data = read(serialObj, serialObj.NumBytesAvailable, "char"); % Read all available bytes
        disp("Arduino says: " + data);  % Display received message
    end
    pause(0.1);  % Small delay to avoid overloading CPU
end
