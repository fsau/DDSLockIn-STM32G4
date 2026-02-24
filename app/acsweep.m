% DDSLI Control Script
% Controls DDSLI device with frequency sweep functionality

% Configuration variables
initial_freq_a = 33790000; # mHz
initial_freq_b = 33760000; 
final_freq_a = 33760000;
final_freq_b = 33790000;
sweep_time = 5;             % Sweep duration in seconds

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

printf("Starting output stream...\n");
ddsli_toggle_out_stream();

printf("Waiting %.1f seconds for sweep...\n", sweep_time);
pause(sweep_time);

printf("Stopping output stream...\n");
ddsli_toggle_out_stream();

printf("Reading data blocks...\n");
blocks = ddsli_read_blocks_matrix();

ddsli_close();