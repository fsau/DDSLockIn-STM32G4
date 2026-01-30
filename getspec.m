pkg load instrument-control

ports = serialportlist();
s = serialport(ports{1},"timeout",0.2);

h = plot([1,1;1,1],[1,1;1,1]);
% axis([0 1024 0 4096]);

while !isempty(read(s,1024*4));
endwhile

fi = 32700;
fo = 32800;
fs = @(f) round(f*2^28/25e6);

tic
for fw = fs(fi):fs(fo)
    write(s,["ff" num2str(fw) "a\r\n"]);
    pause(0.0001);
    flush(s);
    write(s,"m");
    rawdata = read(s,1024*4);
    data = int16(rawdata(1:2:end)) + 256*int16(rawdata(2:2:end));
    ch0 = data(1:(end/2));
    ch1 = data((end/2+1):end);
    
    set(h(1),'xdata',1:length(ch0));
    set(h(1),'ydata',ch0);
    set(h(2),'xdata',1:length(ch1));
    set(h(2),'ydata',ch1);
    printf("fw=%dkHz S=%d\n",fw/2^28*25e3,length(data)/2);
    if(length(data)!=2048)
        break;
    endif
endfor
printf("N=%d\r\n", fs(fo)-fs(fi));
toc