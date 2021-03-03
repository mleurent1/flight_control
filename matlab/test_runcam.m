global ser

ser.Timeout = 0.2;

% for x = 10:10:250
% for y = 10:10:250
% fprintf('%3d %3d\n',x,y)

buf = [hex2dec('CC'),hex2dec('00')];
% buf = [hex2dec('CC'),hex2dec('02'),hex2dec('01')];
% buf = [hex2dec('CC'),hex2dec('03'),hex2dec('01')];
% buf = [hex2dec('CC'),hex2dec('04'),hex2dec('02')];
% buf = [hex2dec('CC'),hex2dec('11'),hex2dec('01'),0];
% buf = [hex2dec('CC'),hex2dec('21'),x,y,str2double(sprintf('%d','A'))];
% buf = [hex2dec('CC'),hex2dec('22'),4,0,0,str2double(sprintf('%d','T')),str2double(sprintf('%d','O')),str2double(sprintf('%d','T')),str2double(sprintf('%d','O'))];

buf = [buf, crc8(buf)];
fwrite(ser,buf)

sleep(100)

fread(ser)

% end
% end