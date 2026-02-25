% Configuration variables
initial_meas_freq_a = 32750000; # mHz
initial_meas_freq_b = 32750000; 
final_meas_freq_a = 32810000;
final_meas_freq_b = 32810000;
sweep_meas_time = 240;
sweep_pre_time = 20;

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

figure(1)
plot(fs - fs(1),blocks.chA'*vcal);
xlabel "Frequency delta (Hz)"
ylabel "RMS Voltage (mV)"
legend("I","Q","DC");
title "Channel A"
grid on; grid minor;

figure(2)
plot(fs - fs(1),blocks.chB'*vcal);
xlabel "Frequency delta (Hz)"
ylabel "RMS Voltage (mV)"
legend("I","Q","DC");
title "Channel B"
grid on; grid minor;

figure(3)

pkg load signal

QI = decimate(blocks.chA(1,:),25,4);
QQ = decimate(blocks.chA(2,:),25,4);
VI = decimate(blocks.chB(1,:),25,4);
VQ = decimate(blocks.chB(2,:),25,4);
fsd = fs(1:25:end);
Q = movmean(QI+1i*QQ,200);
V = movmean(VI+1i*VQ,200);
C = 10.5e-12;
R = 2e9;
ZC = 1./(1i*2*pi*fsd*C);
ZRC = 1./(1./R+1./ZC);
ZDUT = -(V./Q).*ZRC;
h=plotyy(fsd - fsd(1),abs(ZDUT),fsd - fsd(1),-180-unwrap(angle(ZDUT))*180/pi,@semilogy,@plot);
axis(h(1),[0,max(fsd-fsd(1)),7e3,5e9]);
title "Impedance"
xlabel "Frequency delta (Hz)"
ylabel(h(1),'|Z|')
ylabel(h(2),'Phase (°)')
grid on; grid minor;

% pkg load optim

% function Z = bvd_impedance(p, f)
%   % p = [Rm, Lm, Cm, C0]
%   Rm = p(1)*1e3;
%   Lm = p(2)*1e3;
%   Cm = p(3)*1e-15;
%   C0 = p(4)*1e-12;
%   Cm = max(p(3)*1e-15, 1e-18);

%   w = 2*pi*f;

%   Zm = Rm + 1i*w.*Lm + 1./(1i*w.*Cm);
%   Y  = 1./Zm + 1i*w.*C0;
%   Z  = 1./Y;
% endfunction

% function y = bvd_model_ri(p, f)
%   Z = bvd_impedance(p, f);
%   mag = abs(Z);
%   mag = max(mag, 1e-12);   % floor to avoid log(0)
%   y = [log(abs(Z)),angle(Z)];
% endfunction

% y_meas = [log(abs(ZDUT)),unwrap(-angle(ZDUT))-pi];

% % ---------------- INITIAL GUESS ----------------

% fs_guess = fsd(find(abs(ZDUT) == min(abs(ZDUT)), 1));
% w0 = 2*pi*fs_guess;

% Rm0 = min(abs(ZDUT))/1e3-2;
% Cm0 = 3;
% Lm0 = 1/(w0^2 * Cm0*1e-15)/1e3;
% C00 = 1.75;

% p0 = [Rm0; Lm0; Cm0; C00]';   % column vector

% % ---------------- BOUNDS ----------------
% % lb = [1,    4,   1,  1];
% % ub = [100,  10,   6,  3];

% lb = p0 + [-0.5, -0.001, -0.001, -0.2];
% ub = lb + [1, 0.002, 0.002, 0.3];

% % ---------------- FIT ----------------

% options = optimset(
%   "TolX", 1e-12,
%   "TolFun", 1e-12,
%   "MaxIter", 2000,
%   "MaxFunEvals", 5000
% );

% [p_fit, resnorm, resid, exitflag, output] = ...
%   lsqcurvefit(
%     @(p,f) bvd_model_ri(p,f),
%     p0,
%     fsd,
%     y_meas,
%     lb,
%     ub,
%     options
%   );

% % ---------------- RESULTS ----------------
% Rm = p_fit(1);
% Lm = p_fit(2);
% Cm = p_fit(3);
% C0 = p_fit(4);

% printf("BVD fit results:\n");
% printf("Rm = %.6g ohm\n", Rm);
% printf("Lm = %.6g H\n",   Lm);
% printf("Cm = %.6g F\n",   Cm);
% printf("C0 = %.6g F\n",   C0);
