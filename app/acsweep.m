% Configuration variables
initial_meas_freq_a = 32500000; # mHz
initial_meas_freq_b = 32500000; 
final_meas_freq_a = 32800000;
final_meas_freq_b = 32800000;
sweep_meas_time = 5;
sweep_pre_time = 2;

sweep_time = sweep_meas_time + sweep_pre_time;
sweep_delta_a = final_meas_freq_a - initial_meas_freq_a;
sweep_delta_b = final_meas_freq_b - initial_meas_freq_b;
sweep_inactive_ratio = (sweep_pre_time/sweep_meas_time);

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
              int64(initial_freq_a), 
              int64(initial_freq_b), 
              sweep_rate_a, 
              sweep_rate_b);

printf("Sending command: %s\nand waiting %.1f seconds...\n", cmd,sweep_pre_time);
ret = ddsli_send_cmd(cmd);
pause(sweep_pre_time);

printf("Starting output stream.\n");
ddsli_toggle_out_stream();

printf("Waiting %.1f seconds for measurement...\n", sweep_meas_time);
pause(sweep_meas_time);

printf("Stopping output stream.\n");
ddsli_toggle_out_stream();

printf("Reading data blocks.\n");
blocks = ddsli_read_blocks_matrix();

printf("Closing device.\n");
ddsli_close();

vcal = 301/586; % mV per unit

fs = double(blocks.phase_inc)/2^64*1e6;

% ---- deconvolved admittance and BVD fit (poster method) ----
rate = 1e6/256;            % 1 MSa/s ADC, 256-sample lock-in blocks
n = length(fs);
t = (0:n-1)'/rate;
w = 2*pi*fs(:);
dw = mean(diff(w))*rate;

z  = -double(blocks.chA(1,:))' + 1i*double(blocks.chA(2,:))';
zr = -double(blocks.chB(1,:))' + 1i*double(blocks.chB(2,:))';

% chirp compression (swept-sine deconvolution)
te = linspace(-max(t)/2, max(t)*1.5, 2*n);
sweep_rate_hz = abs(dw)/2/pi;
kernel_width = 2e3/max(sweep_rate_hz, eps);
cb = exp(-0.5i*dw*(te-max(t)/2).^2) .* (abs(te-max(t)/2) < kernel_width);
dec  = conv(z, cb, 'same') / sum(cb);
decr = conv(zr, cb, 'same') / sum(cb);
Y = dec ./ decr;
[~, i_res] = max(abs(Y));

% BVD fit on the deconvolved admittance (simple charge-amp model)
Cf = 10e-12;
f = fs(:);
y = conj(Y(:));
f0g = f(i_res);
amp = abs(y(i_res));
bw_hz = sum(abs(y) > amp/2) * abs(mean(diff(f)));
Q0 = max(f0g/max(bw_hz, 1), 10);
p0 = [f0g, log(Q0), log(2e9), log(1.25e-12), 0];

sm = @(p) ( 1./( 1i*(2*pi*f).*exp(p(3))./(2*pi*p(1)) ...
                 + 1./(1i*(2*pi*f).*(1./(2*pi*p(1).*exp(p(3))))) ...
                 + exp(p(3))./exp(p(2)) ) ...
             + 1i*(2*pi*f).*exp(p(4)) ) ...
         ./ (-1 ./ (1 ./ (1i*(2*pi*f)*Cf))) .* exp(-1i*p(5));
fl = max(abs(y))*1e-3;
obj = @(p) sum(abs((sm(p)-y)./max(abs(y),fl)).^2);
p = fminsearch(obj, p0, optimset('MaxFunEvals',30000,'MaxIter',30000,'Display','off'));

ym = sm(p);
SSE = sum(abs(ym-y).^2);
SST = sum(abs(y-mean(y)).^2);
R2 = 1 - SSE/SST;
bvd = [p(1), exp(p(2)), exp(p(3)), exp(p(4)), Cf, p(5)];
Yfit = conj(ym);

printf('BVD fit: f0 = %.4f kHz, Q = %.4g, Z0 = %.4g MOhm, R = %.4g kOhm, C0 = %.3f pF, Cfb = %.3f pF, phi = %.2f deg, R2 = %.5f\n', ...
       bvd(1)/1e3, bvd(2), bvd(3)/1e6, bvd(3)/bvd(2)/1e3, ...
       bvd(4)*1e12, bvd(5)*1e12, bvd(6)*180/pi, R2);

% graphs: demodulated response and deconvolved admittance
adm_scale = 2*pi*32e3*Cf*1e6;   % same uS scale as the poster
A_uS  = abs(Y)*adm_scale;
Af_uS = abs(Yfit)*adm_scale;
ph  = -angle(1i*Y)*180/pi;
phf = -angle(1i*Yfit)*180/pi;
f_khz = f/1e3;

figure(1);
set(gcf, 'Position', [100 100 1200 480]);

subplot(1,2,1);
plot(t, double(blocks.chA(2,:))'*vcal, 'r', ...
     t, double(blocks.chA(1,:))'*vcal, 'b');
xlabel 't (s)'
ylabel 'lock-in voltage (mV_{rms})'
legend('I/Re', 'Q/Im')
title 'Demodulated chirp response'
grid on; grid minor;

subplot(1,2,2);
[ax, h1, h2] = plotyy(f_khz, A_uS, f_khz, ph);
set(h1, 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 2);
set(h2, 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 2);
set(ax(1), 'YScale', 'log');
hold(ax(1), 'on');
plot(ax(1), f_khz, Af_uS, 'k--', 'LineWidth', 1.4);
hold(ax(1), 'off');
hold(ax(2), 'on');
plot(ax(2), f_khz, phf, 'k--', 'LineWidth', 1.0);
hold(ax(2), 'off');
set(ax(2), 'YLim', [-180 180], 'YTick', -180:90:180);
ylabel(ax(1), '|Y| (\muS)')
ylabel(ax(2), 'phase (deg)')
xlabel(ax(1), 'f (kHz)')
title(ax(1), 'Deconvolved admittance')
legend(ax(1), 'measured', 'BVD fit')
grid(ax(1), 'on'); grid(ax(2), 'on');
