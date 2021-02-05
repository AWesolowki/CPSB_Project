clear all;
close all;
clc


device = serialport("COM11", 115200);

figure;

dane = [];
i = 1;

while(1)
    
    fprintf('Number of bytes available to read : %i\n', device.get.NumBytesAvailable);
    
    tmp = strsplit(readline(device));
    
    dane(ceil(i/3), mod(i, 3) + 1) = str2double(tmp(2));
    i = i + 1;
    
    subplot(2,1,1);
    plot(dane(:, 1));
    
    subplot(2,1,2);
    plot(dane(:, 2));
    
end







