pkg load instrument-control

function out = parse_ddsli_block(block)
    % block: uint8 array of length 56
    if length(block) ~= 56
        error("Block must be 56 bytes");
    endif

    block = uint8(block(:)); % ensure column vector

    % Helper for int64 (Q32.32)
    int64_from_bytes = @(b) typecast(b, 'int64');

    % frequency (phase state)
    out.frequency.phase = int64_from_bytes(block(1:8));
    out.frequency.phase_inc = int64_from_bytes(block(9:16));
    out.frequency.phase_inc_delta = int64_from_bytes(block(17:24));

    % DDS amplitude
    out.ddsli_amplitude.A1 = typecast(block(25:26), 'int16');
    out.ddsli_amplitude.A2 = typecast(block(27:28), 'int16');

    % chA floats
    out.chA = zeros(1, 3);

    for k = 1:3
        start_idx = 29 + (k - 1) * 4;
        out.chA(k) = typecast(block(start_idx:start_idx + 3), 'single');
    endfor

    % chB floats
    out.chB = zeros(1, 3);

    for k = 1:3
        start_idx = 41 + (k - 1) * 4;
        out.chB(k) = typecast(block(start_idx:start_idx + 3), 'single');
    endfor

endfunction

function [packets, remaining] = extract_packets(buffer)
    % EXTRACT_PACKETS - Extract valid packets from a byte buffer
    %
    % [packets, remaining] = extract_packets(buffer)
    %
    % Headers: 0x55 0x55 0x55 {0x00,0x10,0x20}
    % Terminator: 0xAA
    % Blocks: 52 bytes, separated by 0x77 (for type 0x20)

    packets = {};
    buffer = uint8(buffer(:).'); % row vector
    pos = 1;
    buflen = length(buffer);
    oldpos = pos;

    HEADER = uint8([0x55 0x55 0x55]);
    VALID_TYPES = uint8([0x00 0x10 0x20]);

    while pos + 3 <= buflen
        % Find next header
        idx = strfind(char(buffer(pos:end)), char(HEADER));

        if isempty(idx)
            % disp("No package header found.")
            break;
        endif

        start = pos + idx(1) - 1;

        if start + 3 > buflen
            disp("Incomplete header.")
            break; % incomplete header
        endif

        type = buffer(start + 3);

        if ~ismember(type, VALID_TYPES)
            pos = start + 1;
            continue;
        endif

        cursor = start + 4; % after header

        switch type
            case 0x00
                % fixed 4096 points → we assume one big block
                % disp("ADC data received.")
                expected_len = 4096 - 1;

                if cursor + expected_len > buflen
                    break; % incomplete packet
                endif

                newpacket = typecast(buffer(start:cursor + expected_len),'uint16');
                packets{end + 1} = [newpacket(3:2:end) newpacket(4:2:end)];
                pos = cursor + expected_len + 1;

            case 0x10
                % disp("AUX data received.")
                % fixed 32 points
                expected_len = 32 - 1;

                if cursor + expected_len > buflen
                    break; % incomplete packet
                endif

                packets{end + 1} = buffer(start:cursor + expected_len);
                pos = cursor + expected_len + 1;

            case 0x20
                % disp("LI data received.")
                block_start = cursor;

                break_loop = false;
                while true
                    % Check if full 56-byte block + next byte exists
                    if block_start + 56 - 1 > buflen
                        % Incomplete block → wait for more data
                        break_loop = true;
                        break;
                    endif

                    block_end = block_start + 56 - 1;
                    next_byte_pos = block_end + 1;

                    % If next byte doesn't exist yet → incomplete
                    if next_byte_pos > buflen
                        break_loop = true;
                        break;
                    endif

                    next_byte = buffer(next_byte_pos);

                    % Only accept 0x77 or 0xAA as next byte
                    if next_byte ~= 0x77 && next_byte ~= 0xAA
                        % Invalid packet → discard first byte after header and resync
                        pos = start;
                        break_loop = true;
                        break;
                    endif

                    % Valid 56-byte block → parse
                    packets{end + 1} = parse_ddsli_block(buffer(block_start:block_end));

                    if next_byte == 0x77
                        block_start = next_byte_pos + 1; % move to next block
                    else
                        % next_byte == 0xAA → end of packet
                        pos = next_byte_pos + 1;
                        break_loop = true;
                        break;
                    endif
                endwhile

                if break_loop == 1
                    break;
                endif

        endswitch

    endwhile

    remaining = buffer(pos:end);
endfunction

[~, serialfname] = system('echo -n /dev/ttyACM*');
ser = serialport(serialfname, 'timeout', 0.001);

x=[];
disp('Capturing...')
write(ser,'cp');
tic();
while length(x) < 2000000
    pause(0.01);
    x=[x read(ser,10000)];
endwhile
toc();

write(ser,'cp');

disp('Processing...')
tic();
[a,b]=extract_packets(x);
as = [a];

while !isempty(b)
    [a,b]=extract_packets(b);
    as = [as a];
endwhile
toc();

fs = As = [];
for i = 1:length(as)
    fs(i) = double(as{1}.frequency.phase_inc)/2^64*1e6;
    vs(i) = as{i}.chA(1);
endfor

for i = 1:length(as)
phs(i) = angle(as{i}.chB(1) + 1i*as{i}.chB(2));
endfor

figure(1);
plot(fs);
figure(2);
plot(vs);
figure(3);
plot(phs);

% pkg load signal  % for plotting and real-time updates

% % === Initialize plots ===
% figure(1);
% h1 = plot(nan, nan, 'r', nan, nan, 'b');  % Oscilloscope: 2 channels
% xlabel('Sample'); ylabel('Amplitude'); title('Oscilloscope');
% ylim([-1 1]); grid on;

% figure(2);
% h2 = plot(nan, nan, 'bo');  % Impedance plot
% xlabel('|Z|'); ylabel('Phase (rad)'); title('Impedance');
% grid on;

% % === Buffers for plotting ===
% osc_buffer_A = [];
% osc_buffer_B = [];
% imp_z = [];
% imp_phase = [];

% % Timing control
% t_last_m = tic();
% m_interval = 1;  % 10 Hz
% t_last_p = tic();
% p_interval = 0.1;    % as fast as possible

% % Serial read buffer
% serial_buf = uint8([]);

% % === Main loop ===
% while true
%     % --- Send 'm' ~10 Hz ---
%     if toc(t_last_m) >= m_interval
%         write(ser, 'm');
%         t_last_m = tic();
%     endif

%     % --- Send 'p' as fast as possible ---
%     if toc(t_last_p) >= p_interval
%         write(ser, 'p');
%         t_last_p = tic();
%     endif

%     % --- Read from serial port ---
%     % read() will return immediately with up to 1024 bytes
%     len = 1024;
%     try
%         new_bytes = read(ser, len);
%         serial_buf = [serial_buf; new_bytes];
%     catch
%         % ignore read errors
%     end_try_catch

%     % --- Extract packets from buffer ---
%     [packets, serial_buf] = extract_packets(serial_buf);

%     % --- Process packets ---
%     for k = 1:length(packets)
%         pkt = packets{k};

%         if isnumeric(pkt)
%             % === 0x00 Oscilloscope data ===
%             chA = double(pkt(1:2:end));
%             chB = double(pkt(2:2:end));
%             osc_buffer_A = [osc_buffer_A chA];
%             osc_buffer_B = [osc_buffer_B chB];

%             % Keep only last N points
%             N = 1024;
%             if length(osc_buffer_A) > N
%                 osc_buffer_A = osc_buffer_A(end-N+1:end);
%                 osc_buffer_B = osc_buffer_B(end-N+1:end);
%             endif

%             % Update plot 1
%             set(h1(1), 'YData', osc_buffer_A, 'XData', 1:length(osc_buffer_A));
%             set(h1(2), 'YData', osc_buffer_B, 'XData', 1:length(osc_buffer_B));
%             drawnow limitrate;

%         elseif isstruct(pkt)
%             % === 0x20 DDS output ===
%             A = pkt.chA;  % [I Q DC]
%             B = pkt.chB;

%             % Compute impedance: Z = V/I
%             v_complex = complex(B(1), B(2));
%             i_complex = complex(A(1), A(2));
%             Z = v_complex / i_complex;

%             imp_z = [imp_z abs(Z)];
%             imp_phase = [imp_phase angle(Z)];

%             % Keep only last N points
%             N = 500;
%             if length(imp_z) > N
%                 imp_z = imp_z(end-N+1:end);
%                 imp_phase = imp_phase(end-N+1:end);
%             endif

%             % Update plot 2
%             set(h2, 'XData', imp_z, 'YData', imp_phase);
%             drawnow limitrate;
%         endif
%     endfor

% endwhile
