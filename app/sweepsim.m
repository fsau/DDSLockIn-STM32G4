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
