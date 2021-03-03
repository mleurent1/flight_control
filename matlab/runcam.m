function runcam(str)

global ser

if strcmp(str,'on')
   buf = [4,1];
elseif strcmp(str,'off')
   buf = [4,2];
elseif strcmp(str,'set')
   buf = [2,1];
elseif strcmp(str,'left')
   buf = [2,2];
elseif strcmp(str,'right')
   buf = [2,3];
elseif strcmp(str,'up')
   buf = [2,4];
elseif strcmp(str,'down')
   buf = [2,5];
elseif strcmp(str,'osd')
   buf = [2,4];
end

buf = [hex2dec('CC'),buf];
buf = [buf, crc8(buf)];
fwrite(ser,buf)

if strcmp(str,'set') || strcmp(str,'left') || strcmp(str,'right') || strcmp(str,'up') || strcmp(str,'down') || strcmp(str,'osd')
   if strcmp(str,'osd')
      sleep(3100)
   else
      sleep(100)
   end
   buf = [hex2dec('CC'),3];
   buf = [buf, crc8(buf)];
   fwrite(ser,buf)
end


end