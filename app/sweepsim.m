pkg load signal

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