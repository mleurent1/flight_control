global fc
global ser

fc.CTRL__RADIO_HOST_CTRL(1);

nbytes = 26;

if (ser.BytesAvailable)
	fread(ser,nbytes);
end

for n = 1:20
	
	if ~fc.STATUS__RADIO_BUSY
		fwrite(ser,[3,nbytes]);
	else
		fprintf('RADIO_BUSY\n')
	end
	r = fread(ser,nbytes);
	
	if ~isempty(r)
		fprintf(' %2X',r)
		fprintf('\n')
	end
	
	if length(r) == nbytes
		if (length(r) >= 26) && (r(1) == hex2dec('C8')) && (r(2) == 24) && (r(3) == hex2dec('16'))
			fprintf('rc chan: %d %d %d %d %d %d %d %d\n', ...
				floor(r(4)/2^0) + 2^8*mod(r(5),2^3), ...
				floor(r(5)/2^3) + 2^5*mod(r(6),2^6), ...
				floor(r(6)/2^6) + 2^2*mod(r(7),2^8) + 2^10*mod(r(8),2^1), ...
				floor(r(8)/2^1) + 2^7*mod(r(8),2^4), ...
				floor(r(8)/2^4) + 2^4*mod(r(9),2^7), ...
				floor(r(9)/2^7) + 2^1*mod(r(10),2^8) + 2^9*mod(r(11),2^2), ...
				floor(r(11)/2^2) + 2^6*mod(r(12),2^5), ...
				floor(r(12)/2^5) + 2^3*mod(r(13),2^8));
		elseif (length(r) >= 14) && (r(1) == hex2dec('C8')) && (r(2) == 12) && (r(3) == hex2dec('14'))
			fprintf('link stat:')
			fprintf(' %d', r(4:8))
			fprintf('\n')
		end
	end
end

%%
fc.CTRL__RADIO_HOST_CTRL(0);
if ~fc.STATUS__RADIO_BUSY
	fwrite(ser,[3,nbytes]);
else
	fprintf('RADIO_BUSY\n')
end