fadc = fopen("badc.bin");
fdac = fopen("bdac.bin");
flpf = fopen("blpf.bin");
fpha = fopen("bpha.bin");
fsin = fopen("bsin.bin");
adc2 = fopen("adcmixdump.bin");
sin2 = fopen("sinmixdump.bin");
dadc2 = uint8(fread(adc2));
dsin2 = uint8(fread(sin2));
dadc = uint8(fread(fadc));
ddac = uint8(fread(fdac));
dlpf = uint8(fread(flpf));
dpha = uint8(fread(fpha));
dsin = uint8(fread(fsin));
fclose(fadc);
fclose(fdac);
fclose(flpf);
fclose(fpha);
fclose(fsin);

dadc = typecast(dadc, "uint16");
ddac = typecast(ddac, "int16");
dsin = typecast(dsin, "int16");
dpha = typecast(dpha, "int16");
dadc2 = typecast(dadc2, "int16");
dsin2 = typecast(dsin2, "int16");
dlpf = reshape(dlpf, 20, []);
dlpf = dlpf(4:20,:);
dlpf = reshape(dlpf,1,[]);
dlpf = typecast(dlpf, "int16");

% figure(1)
% h = plot(dadc(1:2:end));
% hold on;
% h = plot(dadc(2:2:end));
% title "adc"
% figure(2)
% h = plot(ddac(1:2:end));
% hold on;
% h = plot(ddac(2:2:end));
% title "dac"
% figure(3)
% h = plot(dlpf(1:4:end));
% hold on;
% h = plot(dlpf(2:4:end));
% h = plot(dlpf(3:4:end));
% h = plot(dlpf(4:4:end));
% title "lpf"
% figure(4)
% h = plot(dpha(1:2:end));
% hold on;
% h = plot(dpha(2:2:end));
% title "phase"
% figure(5)
% h = plot(dsin(1:2:end));
% hold on;
% h = plot(dsin(2:2:end));
% title "sin"
% hold off;

% a1 = int32(dadc(1:2:end)) .* int32(dsin(1:2:end*2/3));
% b1 = int32(dadc(1:2:end)) .* int32(dsin(2:2:end*2/3));
% a2 = int32(dadc(2:2:end)) .* int32(dsin(1:2:end*2/3));
% b2 = int32(dadc(2:2:end)) .* int32(dsin(2:2:end*2/3));

fadc = fopen("adccapt.bin")
fsin = fopen("sincapt.bin")
dadc = double(fread(fadc));
dsin = uint8(fread(fsin));
dadc = double(dadc(2:2:end)*256+dadc(1:2:end))-2048;
dsin = typecast(dsin,'int16')/16;

s = 3*length(dadc)/4;

dadc = [dadc((s):end); dadc(1:(s-1))];
dsin = [dsin((s):end); dsin(1:(s-1))];
dsin = shiftdim(dsin,length(dsin)/4);
plot(dadc(1:2:end),'color','red');
hold on
plot(dadc(2:2:end),'color','green');
plot(dsin(1:2:end),'color','blue');
plot(dsin(2:2:end)),'color','purple';
hold off