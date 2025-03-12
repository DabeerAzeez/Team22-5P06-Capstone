% Define serial port (adjust COM port for Windows or use '/dev/ttyUSB0' for Linux/Mac)
serialObj = serialport("COM5", 9600); % Change "COM3" to your port

% Configure serial object
configureTerminator(serialObj, "LF"); % Assuming newline ('\n') terminates messages
serialObj.Timeout = 5; % Timeout for read operations

disp("Reading from serial port...");

% Read and display incoming data in a loop
while true
    if serialObj.NumBytesAvailable > 0
        data = readline(serialObj); % Read a line of text
        disp("Received: " + data); % Display received data
    end
    pause(0.1); % Small delay to prevent CPU overuse
end

% Cleanup (Use Ctrl+C to stop script, then run this manually)
% clear serialObj;
