function debug_radio(nb_frames)

global fc
global ser

buf_len = 26;

while ser.BytesAvailable > 0
	fread(ser,ser.BytesAvailable);
end

n = 0;

fc.CTRL__DEBUG_RADIO(1);

while n < nb_frames
	
	if ser.BytesAvailable >= buf_len
		
		r = fread(ser,ser.BytesAvailable);
		r = reshape(r,buf_len,[]);
      
      n = n + size(r,2);
      
      for m = 1:size(r,2)
         fprintf(' %2X',r(:,m)')
			if (r(1,m) == hex2dec('C8')) && (r(2,m) == 24) && (r(3,m) == hex2dec('16'))
				fprintf(' rc chan: %d %d %d %d %d %d %d %d\n', ...
					floor(r(4,m)/2^0) + 2^8*mod(r(5,m),2^3), ...
					floor(r(5,m)/2^3) + 2^5*mod(r(6,m),2^6), ...
					floor(r(6,m)/2^6) + 2^2*mod(r(7,m),2^8) + 2^10*mod(r(8),2^1), ...
					floor(r(8,m)/2^1) + 2^7*mod(r(8,m),2^4), ...
					floor(r(8,m)/2^4) + 2^4*mod(r(9,m),2^7), ...
					floor(r(9,m)/2^7) + 2^1*mod(r(10,m),2^8) + 2^9*mod(r(11,m),2^2), ...
					floor(r(11,m)/2^2) + 2^6*mod(r(12,m),2^5), ...
					floor(r(12,m)/2^5) + 2^3*mod(r(13,m),2^8));
			end
			if (r(1,m) == hex2dec('C8')) && (r(2,m) == 12) && (r(3,m) == hex2dec('14'))
				fprintf(' link stat: %d %d %d %d\n', r(4:8))
			end
         fprintf('\n')
      end
      
	end
	
end

% fc.CTRL__DEBUG_RADIO(0);
fc.CTRL(0);
pause(0.5)

while ser.BytesAvailable > 0
	fread(ser,ser.BytesAvailable);
end

end
