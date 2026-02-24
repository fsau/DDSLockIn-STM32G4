% DDSLI Control Script
% Controls DDSLI device with frequency sweep functionality

% Configuration variables
initial_meas_freq_a = 33760000; # mHz
initial_meas_freq_b = 33760000; 
final_meas_freq_a = 33790000;
final_meas_freq_b = 33790000;
sweep_meas_time = 10;
sweep_pre_time = 1;

% Update sweep frequencies ranges for including pre-measure time

sweep_time = sweep_meas_time + sweep_pre_time;
sweep_delta_a = final_meas_freq_a - initial_meas_freq_a;
sweep_delta_b = final_meas_freq_b - initial_meas_freq_b;
sweep_inactive_ratio = (sweep_pre_time/sweep_time);

initial_freq_a = initial_meas_freq_a - sweep_delta_a * sweep_inactive_ratio;
initial_freq_b = initial_meas_freq_b - sweep_delta_b * sweep_inactive_ratio;
final_freq_a = final_meas_freq_a;
final_freq_b = final_meas_freq_b;

% Calculate sweep rates (Hz/s * 2^32)
% Sweep rate = (final_freq - initial_freq) / sweep_time
% Convert mHz to Hz: divide by 1000
% Then multiply by 2^32 for integer representation

sweep_rate_a = int64(((final_freq_a - initial_freq_a) / 1000) / sweep_time * 2^32);
sweep_rate_b = int64(((final_freq_b - initial_freq_b) / 1000) / sweep_time * 2^32);

ddsli_open();

% Command format: f<freq_a>af<freq_b>bf<sweep_rate_a>qf<sweep_rate_b>gu
cmd = sprintf("####f%daf%dbf%dqf%dgu####", 
              initial_freq_a, 
              initial_freq_b, 
              sweep_rate_a, 
              sweep_rate_b);

printf("Sending command: %s\n", cmd);
ret = ddsli_send_cmd(cmd);
pause(0.1);

printf("Starting output stream...\n");
ddsli_toggle_out_stream();
ddsli_read_blocks_matrix();  # clear buffer

printf("Waiting %.1f seconds for sweep...\n", sweep_time);
pause(sweep_time);

printf("Stopping output stream...\n");
ddsli_toggle_out_stream();

printf("Reading data blocks...\n");
blocks = ddsli_read_blocks_matrix();

ddsli_close();

figure(1)
plot(blocks.chA');
figure(2)
plot(blocks.chB');