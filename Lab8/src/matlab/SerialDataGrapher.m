function SerialDataGrapher
clear all; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% @author Ian Wilkey (iwilkey)
%%% @course ECE 322
%%% @assignment Final Lab
%%% @brief This MATLAB script graphs incoming serial data from an Arduino
%%% Nano using a text file buffer.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define where the data is being piped into...
DATA_PATH = "/Users/ianwilkey/Desktop/ECE-322/Lab8/SerialDataBuffer";

figure("Name", "ECE 322 Final Project MAX30102 Real-Time Data Graphing");
while(true)
    
    % It's possible that, through electrical interference, serial data
    % loss, ect, MATLAB cannot process the text file buffer. In this
    % case, it will just wait until it can.
    try
        % Open data file...
        file_handle = fopen(DATA_PATH);
        % Scan the contents-- expecting an integer...
        data_cell = textscan(file_handle, "%d");
        % Dispose of current frame data...
        fclose(file_handle);
        
        % Convert current frame data into a matrix for plotting...
        data = cell2mat(data_cell);
    
        % Render new(ish) data...
        plot(data(max(1, length(data) - 500) : ...
            length(data) - 1), ...
            "LineWidth", 2, ...
            "Color", "red");
        % Label...
        xlabel("Sample");
        ylabel("MAX30102 Filtered Value");
    catch
        warning("Data buffer was not formatted in a way MATLAB could understand. Please reset the Nano.");
    end

    % Simulate a steady frame rate...
    pause(1 / 10);

end
