function debug_radio(nb_frames)

global fc
global ser

buf_len = 32;

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
      
      for m = 1:size(r,2);
         fprintf(' %2X',r(:,m)')
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
