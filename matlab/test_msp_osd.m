global ser
global fc

fc.CTRL__MSP_HOST_CTRL(1);

% cmd = 84;
% data = [zeros(1,7), 2, zeros(1,2), 8,74, zeros(1,57*2),...
% 	0, zeros(1,24*2), 0, zeros(1,2*2), zeros(1,10)];
cmd = 110;
data = [111, 0,74, 0,0, 0,210, 4,76];
% cmd = 130;
% data = [3, 5,170, 110, 0,74, 0,210, 0, 4,76];

buf = [length(data), cmd, data];
buf = [double(uint8('$M>')), buf, checksum(buf)];

fwrite(ser,[8,length(buf),0,buf]);

% r = fread(ser,1)

%%
fc.CTRL__MSP_HOST_CTRL(1);

nbytes = 6;

if (ser.BytesAvailable)
	fread(ser,nbytes);
end

for n = 1:25
	
	if ~fc.STATUS__MSP_BUSY
		fwrite(ser,[8,0,nbytes]);
	else
		fprintf('MSP_BUSY\n')
	end
	r = fread(ser,nbytes);
	
	if ~isempty(r)
		fprintf(' %2X',r)
	end
	
	if length(r) == nbytes
		fprintf(' %s',r(1:3))
		fprintf(' %3d',r(4:nbytes-1))
	end
	fprintf('\n')
end
