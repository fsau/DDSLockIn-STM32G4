pkg load signal

% % # Parameters

% L = 6740.02;
% C = 3.5e-15;
% R = 80e3;

C0 = 1.75e-12;

omega0 = 2*pi*32800; % 1/sqrt(L*C)
Z_0      = 1.2e9; % sqrt(L/C)
Q_factor = 1e3; % w_0*L/R

L = Z_0 / omega0;
C = 1 / (omega0 * Z_0);
R = Z_0 / Q_factor;

dt = 1e-6;
tm = 0.1;

ff = 31000;
f0 = 34000;
t = (0:dt:tm)';
w = 2*pi*linspace(f0,ff,length(t));
dw = 2*pi*(ff - f0)/tm;

w0 = 1/sqrt(L*C);
Q = w0*L/R;

tic
[z,dz] = rlc_chirp(dt,tm,f0,ff,L,R,C,C0);
toc

% z=z*1i;

function y = blockmean(x, N)
    x = x(:);  % make column

    M = floor(length(x)/N);   % number of full blocks
    x = x(1:M*N);             % trim excess

    X = reshape(x, N, M);     % each column = one block
    y = mean(X, 1);           % mean over rows

    y = y.';  % return as column (optional)
end

z = z + randn(size(z))*1e-8.*exp(1i*rand(size(z))*2*pi);
% dz = dz + rand(size(dz))*1e-6.*exp(1i*rand(size(dz))*2*pi)/dt;

z = blockmean(z,256);
% dz = blockmean(dz,256);
t = blockmean(t,256);
w = blockmean(w,256);

z = z - 1i*imag(median(z.*w)/median(w));

figure(1);
plot(t,[real(z),imag(z)]);
legend('Real','Imag');
grid on; grid minor;

te = linspace(-tm/2,tm*1.5,2*length(t));

deswept = conv(z,exp(-0.5i*dw*(te-tm/2).^2),"same");
% deswept = conv(imag(z),real(exp(-0.5i*dw*(te-tm/2).^2)),"same");

figure(2)
plot(w/2/pi,abs(deswept))
grid on; grid minor;
[~,max_i] = max(deswept);
f_meas = w(max_i)/2/pi;
printf("f_meas = %f, f_res = %f,\n error = %f\n",f_meas,w0/2/pi,f_meas-w0/2/pi);


figure(3);
plot(t-t(max_i),[abs(z.*exp(-0.5i*dw*(t-t(max_i)).^2))]);
grid on; grid minor;
