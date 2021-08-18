global ser
global fc

fc.CTRL__SMA_HOST_CTRL(1);

buf = [hex2dec('AA'),hex2dec('55'),hex2dec('03'),hex2dec('00')];
nb_rx = 11;

% buf = [hex2dec('AA'),hex2dec('55'),hex2dec('05'),hex2dec('01'),hex2dec('03')];
% nb_rx = 8;

% buf = [hex2dec('AA'),hex2dec('55'),hex2dec('07'),hex2dec('01'),hex2dec('01')];
% nb_rx = 8;

buf = [hex2dec('00'), buf, crc8(buf)];
fwrite(ser,[10,length(buf),nb_rx,buf]);

r = fread(ser,nb_rx);
dec2hex(r)

crc_calc = dec2hex(crc8(r(4:end-1)))
