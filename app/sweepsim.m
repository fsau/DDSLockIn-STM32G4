pkg load signal

% % # Parameters
% L = 6740;
% C = 3.5e-15;
% R = 2e4;
% C0 = 3e-12;

% dt = 1e-6;
% tm = 20;

% ff = 32760;
% f0 = 32810;
% tau_ramp = 0.1;  # 100 ms ramp

% t = (0:dt:tm)';

% # --- Chirp definition ---
% k = (ff - f0)/tm;
% phi = 2*pi*(f0*t + 0.5*k*t.^2);
% env = 1 - exp(-t/tau_ramp);

% F = env .* sin(phi);

% # --- System (motional branch only) ---
% num = [1 0];
% den = [L R 1/C];
% sys = tf(num, den);

% i_motional = lsim(sys, F, t);

% dphi_dt = 2*pi*(f0 + k*t);   # instantaneous angular frequency
% dFdt = cos(phi) .* dphi_dt;

% i_c0 = C0 * dFdt;

% % # --- Total current ---
% i = i_motional + i_c0;

% N = 256;
% nb = floor(length(t)/N);

% fs = zeros(nb,1);
% X = zeros(nb,1);
% Y = zeros(nb,1);
% DC = zeros(nb,1);
% t_blk = zeros(nb,1);

% for kblk = 1:nb

%   idx = (kblk-1)*N + (1:N);

%   tt = t(idx);
%   ph = phi(idx);
%   y  = i(idx);

%   # Basis functions
%   c = cos(ph);
%   s = sin(ph);
%   one = ones(size(ph));

%   # Design matrix
%   M = [c s one];

%   # Least-squares solve: y ≈ A*c + B*s + C
%   abc = M \ y;

%   A = abc(1);
%   B = abc(2);
%   C = abc(3);

%   # Store
%   X(kblk) = A;
%   Y(kblk) = B;
%   DC(kblk) = C;

%   t_blk(kblk) = mean(tt);

% endfor

% # --- Amplitude & phase ---
% A = sqrt(X.^2 + Y.^2);
% theta = atan2(Y, X);

% # --- Instantaneous frequency ---
% f_inst = f0 + k*t;
% f_blk = f0 + k*t_blk;

# --- Plots ---
% figure(1);

% subplot(2,2,1);
% plot(t, i);
% title("Total current i(t)");

% subplot(2,2,2);
% plot((f_blk), abs(X-2*pi*f_blk*C0+1i*Y));
% xlabel("Frequency (Hz)");
% ylabel("Amplitude");
% title("Lock-in amplitude");

% subplot(2,2,3);
% plot((f_blk), X-2*pi*f_blk*C0);
% hold on
% plot((f_blk),Y);
% hold off
% xlabel("Frequency (Hz)");
% ylabel("Amplitude");
% title("Lock-in components");

% subplot(2,2,4);
% plot((f_blk), unwrap(angle(X-2*pi*f_blk*C0+1i*Y)));
% xlabel("Frequency (Hz)");
% ylabel("Phase (rad)");
% title("Lock-in phase");

% figure(2);
% plot(X-2*pi*f_blk*C0,Y);
% axis "square"
% grid on
% grid minor


% % # Parameters
L = 6740;
C = 3.5e-15;
R = 2e4;
C0 = 2.5e-12;

dt = 1e-6;
tm = 10;

ff = 32600;
f0 = 32900;
t = (0:dt:tm)';
w = 2*pi*linspace(f0,ff,length(t));
dw = 2*pi*(ff - f0)/tm;

tic
[z,dz] = rlc_chirp(dt,tm,f0,ff,L,R,C,C0);
toc

figure(1);
plot(t,[real(z),imag(z)]);
legend('Real','Imag');

figure(2);
plotyy(w/2/pi,abs(z),w/2/pi,angle(z));

% function dzdt = f(z, dz, w, dw)
%     L = 6740;
%     C = 3.5e-15;
%     R = 2e4;
%     w0 = 1/sqrt(L*C);
%     dzdt = ( ...
%         (1/L * (w^2*L - 1/C - 1i*dw*L - 1i*w*R)) * z ...
%         - 1/L * ( R + 2i * w * L ) * dz ...
%         - 1i * w ...
%     );
% end

% for st = 2:length(t)

%   z0 = z(st-1);
%   dz0 = dz(st-1);
%   w0 = w(st-1);

%   k1z = dz0;
%   k1dz = f(z0, dz0, w0, dw);

%   k2z = dz0 + 0.5*dt*k1dz;
%   k2dz = f(z0 + 0.5*dt*k1z, dz0 + 0.5*dt*k1dz, w0, dw);

%   k3z = dz0 + 0.5*dt*k2dz;
%   k3dz = f(z0 + 0.5*dt*k2z, dz0 + 0.5*dt*k2dz, w0, dw);

%   k4z = dz0 + dt*k3dz;
%   k4dz = f(z0 + dt*k3z, dz0 + dt*k3dz, w0, dw);

%   z(st)  = z0  + dt/6*(k1z  + 2*k2z  + 2*k3z  + k4z);
%   dz(st) = dz0 + dt/6*(k1dz + 2*k2dz + 2*k3dz + k4dz);

% end