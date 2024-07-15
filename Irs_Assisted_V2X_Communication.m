clc;
clear all;
close all;
 
% Define simulation parameters
num_vehicles = 2; % Number of vehicles
num_rsus = 5; % Number of roadside units
simulation_time = 20; % Simulation time (seconds)
communication_range = 50; % Communication range between vehicles and RSUs (meters)
vehicle_step_size = 10; % Step size for vehicle movement
irs_positions = zeros(num_rsus, 2);
 
% Initialize RSU positions in a straight line
for i = 1:num_rsus
    irs_positions(i, :) = [50 + (i-1)*100, 100];
end
 
% Initialize vehicle positions
vehicle_positions = [30, 60; 100, 140]; % Different initial positions for vehicles
 
% Initialize vehicle connections
vehicle_connections = zeros(num_vehicles, 1); % Initially, no vehicle is connected to any RSU
 
% Plotting setup
figure;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Simulation of V2I Communication');
grid on;
axis equal;
xlim([0, 600]);
ylim([0, 200]);
hold on;
 
% Plot RSU positions
rsu_scatter = scatter(irs_positions(:, 1), irs_positions(:, 2), 'rs', 'filled');
rsu_text = text(irs_positions(:, 1), irs_positions(:, 2), cellstr(num2str((1:num_rsus)')),'HorizontalAlignment','center', 'VerticalAlignment','middle');
 
% Main simulation loop
for t = 1:simulation_time
    % Clear previous vehicle connections
    fprintf('\nSimulation Time: %d\n', t);
    
    % Clear previous vehicle connections
    delete(findobj('Type', 'line', 'Color', 'g'));
    
    % Move vehicles in a straight line
    vehicle_positions(:, 1) = vehicle_positions(:, 1) + vehicle_step_size;
    
    % Loop through each vehicle
    for v = 1:num_vehicles
        % Check distance to each RSU
        distances = vecnorm(vehicle_positions(v, :) - irs_positions, 2, 2);
        [min_distance, min_index] = min(distances);
        
        if min_distance <= communication_range
            % Vehicle is within communication range of an RSU
            if vehicle_connections(v) ~= min_index
                % Vehicle is not currently connected to this RSU, establish connection
                fprintf('Vehicle %d connected to RSU %d\n', v, min_index);
                vehicle_connections(v) = min_index;
            end
        else
            % Vehicle is out of range of all RSUs, disconnect
            if vehicle_connections(v) ~= 0
                % Vehicle was previously connected, disconnect
                fprintf('Vehicle %d disconnected from RSU %d\n', v, vehicle_connections(v));
                vehicle_connections(v) = 0;
            end
        end
    end
    
    % Check if any RSU is inactive
    if all(vehicle_connections == 0)
        fprintf('All RSUs are inactive.\n');
    end
    
    % Plot vehicle connections
    for v = 1:num_vehicles
        if vehicle_connections(v) ~= 0
            plot([vehicle_positions(v, 1), irs_positions(vehicle_connections(v), 1)], [vehicle_positions(v, 2), irs_positions(vehicle_connections(v), 2)], 'g');
        end
    end
    
    % Plot vehicle positions
    if exist('vehicle_scatter', 'var')
        delete(vehicle_scatter);
    end
    vehicle_scatter = scatter(vehicle_positions(:, 1), vehicle_positions(:, 2), 'bo', 'filled');
    text(vehicle_positions(:, 1), vehicle_positions(:, 2), cellstr(num2str((1:num_vehicles)')),'HorizontalAlignment','center', 'VerticalAlignment','middle');
 
    pause(1); % Pause to observe the animation
end
 
% Parameters (adjusted with increased noise at direct path and higher IRS passive elements' index)
fc = 2.4e9; % Carrier frequency (2.4 GHz for WiFi)
c = physconst('LightSpeed'); % Speed of light
lambda = c / fc; % Wavelength
d_TR = 100; % Distance between transmitter and receiver (m)
d_TI = 5; % Distance between transmitter and IRS (m)
d_IR = 20; % Distance between IRS and receiver (m)
h_BS = 10; % Height of transmitter/receiver (m)
h_IRS = 10; % Height of IRS (m)
num_IRS_elements = 300; % Increased Number of IRS elements
 
% Define noise power (dBm) - increased for the direct path
noise_power_dBm_direct = -170; % Increased noise power at direct path
noise_power_dBm_irs = -200; % Reduced noise power at IRS path
 
% Bandwidth (10 MHz for WiFi)
B = 33e6; % 10 MHz
 
% Pre-allocate arrays for results
direct_path_snr_dB = zeros(1000, 1); % Store 1000 samples
irs_path_snr_dB = zeros(1000, 1); % Store 1000 samples
 
% Calculate data rate and spectrum efficiency
data_rate_direct = zeros(1000, 1);
data_rate_irs = zeros(1000, 1);
spectrum_efficiency_direct = zeros(1000, 1);
spectrum_efficiency_irs = zeros(1000, 1);
 
% Blocking probability counters
num_blocked_direct = 0;
num_blocked_irs = 0;
 
% Calculate latency
latency_direct = d_TR / c; % Direct path latency
latency_irs = (d_TI + d_IR) / c; % Latency with IRS
 
% Store mean SNR values for every 100 samples
mean_snr_direct = zeros(10, 1);
mean_snr_irs = zeros(10, 1);
 
% Counter for mean SNR calculation
mean_counter = 1;
 
for i = 1:1000
    % Channel modeling (path loss and shadowing) - consider shadowing only for demonstration
    shadowing_dB = 4 * randn; % Random shadowing (dB)
 
    % Direct path gain (T-R)
    PL_TR = fspl(d_TR, lambda); % Path loss
    direct_path_gain_linear = db2pow(-PL_TR + shadowing_dB); % Apply shadowing
 
    % IRS path gain (T-I-R)
    PL_TI = fspl(d_TI, lambda); % Path loss (T-I)
    PL_IR = fspl(d_IR, lambda); % Path loss (I-R)
    irs_path_gain_linear = db2pow(-PL_TI - PL_IR + shadowing_dB); % Apply shadowing
 
    % Define IRS reflection coefficients (adjust for enhanced reflection properties)
    reflection_coefficients = exp(1j*pi/2); % Example: Reflect with 90-degree phase shift (can be complex for amplitude and phase control)
    reflection_coefficients = repmat(reflection_coefficients, num_IRS_elements, 1); % Same reflection coefficient for all elements
    reflection_coefficients = reflection_coefficients * 1.5; % Increase reflection coefficient
 
    % Calculate phase shifts due to IRS element positions (placeholder)
    % This part requires further development based on IRS geometry and wave propagation
    phase_shifts = zeros(num_IRS_elements, 1); % Placeholder for phase shifts
 
    % Combine reflected paths (considering path loss, shadowing, reflection coefficients, and phase shifts)
    irs_reflected_gain_linear = sum(reflection_coefficients .* irs_path_gain_linear .* exp(1j*(phase_shifts + 2*pi*d_TI/lambda)));
 
    % Total IRS path gain (including reflection)
    irs_path_gain_linear = irs_reflected_gain_linear + irs_path_gain_linear; % Assuming perfect reflection for simplicity
 
    % Simulate AWGN noise
    noise_power_linear_direct = db2pow(noise_power_dBm_direct);
    noise_power_linear_irs = db2pow(noise_power_dBm_irs);
    noise_direct = sqrt(noise_power_linear_direct/2) * (randn(1) + 1j*randn(1));
    noise_irs = sqrt(noise_power_linear_irs/2) * (randn(1) + 1j*randn(1));
 
    % Calculate SNR for direct path
    direct_path_snr_linear = abs(direct_path_gain_linear)^2 / noise_power_linear_direct;
    direct_path_snr_dB(i) = pow2db(direct_path_snr_linear);
 
    % Calculate SNR for IRS path
    irs_path_snr_linear = abs(irs_path_gain_linear)^2 / noise_power_linear_irs;
    irs_path_snr_dB(i) = pow2db(irs_path_snr_linear);
    
    % Calculate data rate and spectrum efficiency
    data_rate_direct(i) = B * log2(1 + direct_path_snr_linear);
    data_rate_irs(i) = B * log2(1 + irs_path_snr_linear);
    spectrum_efficiency_direct(i) = data_rate_direct(i) / B;
    spectrum_efficiency_irs(i) = data_rate_irs(i) / B;
    
    % Calculate blocking probability
    if direct_path_snr_dB(i) < 0
        num_blocked_direct = num_blocked_direct + 1;
    end
    
    if irs_path_snr_dB(i) < 0
        num_blocked_irs = num_blocked_irs + 1;
    end
    
    % Calculate mean SNR values for every 100 samples
    if mod(i, 100) == 0
        mean_snr_direct(mean_counter) = mean(direct_path_snr_dB(i-99:i));
        mean_snr_irs(mean_counter) = mean(irs_path_snr_dB(i-99:i));
        mean_counter = mean_counter + 1;
    end
end
 
% Blocking probabilities
blockage_prob_direct = num_blocked_direct / 1000;
blockage_prob_irs = num_blocked_irs / 1000;
 
% Calculate average data rate and spectrum efficiency
avg_data_rate_direct = mean(data_rate_direct);
avg_data_rate_irs = mean(data_rate_irs);
avg_spectrum_efficiency_direct = mean(spectrum_efficiency_direct);
avg_spectrum_efficiency_irs = mean(spectrum_efficiency_irs);
 
% Calculate mean and standard deviation of data rate
mean_data_rate_direct = mean(data_rate_direct);
mean_data_rate_irs = mean(data_rate_irs);
std_data_rate_direct = std(data_rate_direct);
std_data_rate_irs = std(data_rate_irs);
 
% Calculate mean and standard deviation of spectrum efficiency
mean_spectrum_efficiency_direct = mean(spectrum_efficiency_direct);
mean_spectrum_efficiency_irs = mean(spectrum_efficiency_irs);
std_spectrum_efficiency_direct = std(spectrum_efficiency_direct);
std_spectrum_efficiency_irs = std(spectrum_efficiency_irs);
 
% Display results
fprintf('Direct Path SNR (dB): Mean=%.2f, Median=%.2f, Std=%.2f\n', mean(direct_path_snr_dB), median(direct_path_snr_dB), std(direct_path_snr_dB));
fprintf('IRS Path SNR (dB): Mean=%.2f, Median=%.2f, Std=%.2f\n', mean(irs_path_snr_dB), median(irs_path_snr_dB), std(irs_path_snr_dB));
fprintf('Direct Path Latency (s): %.10f\n', latency_direct);
fprintf('Latency with IRS (s): %.10f\n', latency_irs);
fprintf('Blocking Probability for Direct Path: %.2f%%\n', blockage_prob_direct * 100);
fprintf('Blocking Probability with IRS: %.2f%%\n', blockage_prob_irs * 100);
fprintf('Average Data Rate without IRS (Mbps): %.2f\n', avg_data_rate_direct / 1e6);
fprintf('Average Data Rate with IRS (Mbps): %.2f\n', avg_data_rate_irs / 1e6);
fprintf('Average Spectrum Efficiency without IRS (bps/Hz): %.2f\n', avg_spectrum_efficiency_direct);
fprintf('Average Spectrum Efficiency with IRS (bps/Hz): %.2f\n', avg_spectrum_efficiency_irs);
 
% % Display mean and standard deviation of data rate and spectrum efficiency
% fprintf('Mean Data Rate without IRS (Mbps): %.2f\n', mean_data_rate_direct / 1e6);
% fprintf('Standard Deviation of Data Rate without IRS (Mbps): %.2f\n', std_data_rate_direct / 1e6);
% fprintf('Mean Data Rate with IRS (Mbps): %.2f\n', mean_data_rate_irs / 1e6);
% fprintf('Standard Deviation of Data Rate with IRS (Mbps): %.2f\n', std_data_rate_irs / 1e6);
% fprintf('Mean Spectrum Efficiency without IRS (bps/Hz): %.2f\n', mean_spectrum_efficiency_direct);
% fprintf('Standard Deviation of Spectrum Efficiency without IRS (bps/Hz): %.2f\n', std_spectrum_efficiency_direct);
% fprintf('Mean Spectrum Efficiency with IRS (bps/Hz): %.2f\n', mean_spectrum_efficiency_irs);
% fprintf('Standard Deviation of Spectrum Efficiency with IRS (bps/Hz): %.2f\n', std_spectrum_efficiency_irs);
 
% Plot SNR for all 1000 samples
figure;
subplot(2, 1, 1);
plot(1:1000, direct_path_snr_dB, 'b-', 'LineWidth', 1);
xlabel('Sample Number');
ylabel('SNR (dB)');
title('SNR for Direct Path (All Samples)');
grid on;
 
subplot(2, 1, 2);
plot(1:1000, irs_path_snr_dB, 'r-', 'LineWidth', 1);
xlabel('Sample Number');
ylabel('SNR (dB)');
title('SNR for IRS Path (All Samples)');
grid on;
 
% Plot mean SNR values for both cases in a single plot
figure;
subplot(2, 1, 1);
plot(1:100:1000, mean_snr_direct, 'b-o', 'LineWidth', 1);
xlabel('Sample Number');
ylabel('SNR (dB)');
title('Mean SNR for Direct Path');
grid on;
 
subplot(2, 1, 2);
plot(1:100:1000, mean_snr_irs, 'r-o', 'LineWidth', 1);
xlabel('Sample Number');
ylabel('SNR (dB)');
title('Mean SNR for IRS Path');
grid on;
 
% Plot average data rate for both cases and standard deviation
figure;
 
% Plot average data rate
subplot(2, 1, 1);
bar([avg_data_rate_direct, avg_data_rate_irs] / 1e6);
xticklabels({'Without IRS', 'With IRS'});
ylabel('Mbps');
title('Average Data Rate with and without IRS');
grid on;
 
%% Plot standard deviation of data rate
subplot(2, 1, 2);
bar([std_data_rate_direct / 1e6, std_data_rate_irs / 1e6]);
xticks(1:2);
xticklabels({'Without IRS', 'With IRS'});
ylabel('Mbps');
title('Standard Deviation of Data Rate ');
grid on;
 
 
 
 
 
% Plot average spectrum efficiency for both cases and spectral efficiency of data rate
figure;
 
% Plot average spectrum efficiency
subplot(2, 1, 1);
bar([avg_spectrum_efficiency_direct, avg_spectrum_efficiency_irs]);
xticklabels({'Without IRS', 'With IRS'});
ylabel('bps/Hz');
title('Average Spectrum Efficiency with and without IRS');
grid on;
 
% Plot standard deviation of spectrum efficiency
subplot(2, 1, 2);
bar([std_spectrum_efficiency_direct, std_spectrum_efficiency_irs]);
xticklabels({'Without IRS', 'With IRS'});
ylabel('bps/Hz');
title('Standard Deviation of Spectrum Efficiency');
grid on;
 
% Plot blockage probability
figure;
subplot(2, 1, 1);
bar([blockage_prob_direct, blockage_prob_irs]);
xticklabels({'Without IRS', 'With IRS'});
ylabel('Blockage Probability');
title('Blockage Probability with and without IRS');
grid on;
 
% Set y-axis limits to percentage
ylim([0, 1]);
yticks(0:0.1:1);
yticklabels({'0%', '10%', '20%', '30%', '40%', '50%', '60%', '70%', '80%', '90%', '100%'});
 
% Add text labels
text(1, blockage_prob_direct, sprintf('%.2f%%', blockage_prob_direct * 100),...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(2, blockage_prob_irs, sprintf('%.2f%%', blockage_prob_irs * 100),...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
