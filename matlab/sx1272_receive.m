function packets = sx1272_receive(do_print)

if nargin < 1, do_print = 1; end

packets = [];

CrcEn = bitget(sx1272_read('0x1E'),3);

while bitget(sx1272_read('0x12'),7) %  Rx Done IRQ

   if bitget(sx1272_read('0x12'),6) % CRC error
      sx1272_write('0x12',2^5)
      p.status = -1;
   elseif CrcEn
      p.status = 1;
   else
      p.status = 0;
   end

   start_addr = sx1272_read('0x10');
   sx1272_write('0x0D',start_addr);

   p.size = sx1272_read('0x13');
   p.payload = sx1272_read(0,p.size);

   meta = sx1272_read('0x18',3);

   p.coderate = floor(meta(1)/2^5);
   p.snr = c2s(meta(2),8)/4;
   p.rssi = meta(3) - 153;

   packets = [packets, p];

   if do_print
      fprintf('STATUS:%2d ',p.status); 
      fprintf('CR:%d ',p.coderate);
      fprintf('PL:%3d ',p.size);
      fprintf('RSSI:%4d ',p.rssi);
      fprintf('SNR:%6.2f ',p.snr);
      fprintf('PAYLOAD:');
      fprintf(' %02X',p.payload);
      fprintf('\n');
   end

   sx1272_write('0x12',255); % Clear IRQ

end

end
