function crc = crc8(x)

% sr = zeros(1,8);
% for n = 1:length(x)
%    bits = dec2bin(x(n),8)-'0';
%    for m = 1:8
%       b = mod(bits(m) + sr(8) + sr(7) + sr(6) + sr(4) + sr(2), 2);
%       sr = [b, sr(1:7)];
%    end
% end
% crc = bin2dec(num2str(sr));

crc = 0;
for n = 1:length(x)
   crc = bitxor(crc,x(n));
   for m = 1:8
      if bitget(crc,8)
         crc = bitxor(mod(bitshift(crc,1),2^8),hex2dec('D5'));
      else
         crc = mod(bitshift(crc,1),2^8);
      end
   end
end

end