pkg load signal

% % # Parameters
L = 6741.02;
C = 3.5e-15;
R = 30e3;
C0 = 12.00e-12;

dt = 1e-6;
tm = 7;

ff = 32720;
f0 = 32780;
t = (0:dt:tm)';
w = 2*pi*linspace(f0,ff,length(t));
dw = 2*pi*(ff - f0)/tm;

w0 = 1/sqrt(L*C);
Q = w0*L/R;

tic
[z,dz] = rlc_chirp(dt,tm,f0,ff,L,R,C,C0);
toc

z=z*1i;

function y = blockmean(x, N)
    x = x(:);  % make column

    M = floor(length(x)/N);   % number of full blocks
    x = x(1:M*N);             % trim excess

    X = reshape(x, N, M);     % each column = one block
    y = mean(X, 1);           % mean over rows

    y = y.';  % return as column (optional)
end

z = blockmean(z,256);
dz = blockmean(dz,256);
t = blockmean(t,256);
w = blockmean(w,256);

figure(1);
plot(t,[real(z),imag(z)]);
legend('Real','Imag');
grid on; grid minor;

figure(2);


ti = find(abs(z)>0.6*max(abs(z)),1);
tfi = find(abs(dz(ti:end))<3*min(abs(dz(ti:end))),1) + ti - 1;
plotyy(w(ti:tfi)/2/pi,abs((dz(ti:tfi))),
       w(ti:tfi)/2/pi,unwrap(angle((dz(ti:tfi)))),@semilogy,@plot);
grid on; grid minor;

pf = polyfit(w((ti):(tfi))/2/pi-w(1)/2/pi,unwrap(angle((dz((ti):tfi)))),2);
f_meas = -pf(2)/(2*pf(1))+w(1)/2/pi;
disp([f_meas, w0/2/pi])

[~,i_res] = min(abs(w-2*pi*f_meas));
t_res = t(i_res);

tppk = find(abs(dz(ti:end)) < 0.3*max(abs(dz(ti:end))),1) + ti - 1;
tef = find(abs(dz(ti:end)) < 0.1*max(abs(dz(ti:end))),1) + ti - 1;

wd = w - 2*pi*f_meas*t;
pa = polyfit((t(tppk:tef)-t_res),log(abs((dz(tppk:tef)))./abs(wd(tppk:tef))),1);
figure(3);
plot((t(tppk:tef)-t_res),log(abs((dz(tppk:tef)))./abs(wd(tppk:tef))));
% plot((t(tppk:tef)-t_res),log(abs((dz(tppk:tef)))./abs(wd(tppk:tef))));

k = pa(1);
A = exp(pa(2));
Q_meas = 2*pi*f_meas/(-2*k);
disp([Q_meas Q]);

