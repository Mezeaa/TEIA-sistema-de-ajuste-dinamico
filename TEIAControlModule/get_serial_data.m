function get_serial_data(serial_port_name, baudrate, number_of_samples, ...
    number_of_signals, filename)

% Initialize serial port object
close all;
clc;
sampled_data = zeros(number_of_signals, number_of_samples);
time = zeros(1, number_of_samples);
delete(instrfind({'Port'},{serial_port_name}));
serial_port = serial(serial_port_name, 'BaudRate', baudrate);
warning('off', 'MATLAB:serial:fscanf:unsuccessfulRead');

% Open serial port
fopen(serial_port);

% Data vector index
current_sample = 1;

% Send sync signal
fprintf(serial_port, 'k');

% Display start message
disp('Acquiring data...');

% Read data from port
while current_sample <= number_of_samples
    tic;
    fprintf(serial_port, 'c');
    data_read = fscanf(serial_port, '%d')';
    for i = 1:number_of_signals
        %sampled_data(i, current_sample) = typecast(data_read(i), 'double');
        sampled_data(i, current_sample) = data_read(i);
    end
    current_sample = current_sample + 1;
    time(current_sample - 1) = toc;
end

% Send stop message
fprintf(serial_port, 's');

% Normalize data
sampled_data = sampled_data' / 65536;

% Generate time vector
time_index = 2;
while time_index <= number_of_samples
    time(time_index) = time(time_index) + time(time_index - 1);
    time_index = time_index + 1;
end
time = time - time(1);

% Save data to file
save(strcat(filename, '.mat'), 'sampled_data', 'time');

% Display success message
disp('Done. Data saved to:');
disp(strcat(filename, '.mat'));

% Close serial port
fclose(serial_port);

end

