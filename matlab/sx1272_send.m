function sx1272_send(payload,timeout)

if nargin < 2
   timeout = 5;
end

% sets ANTENA swith to TX position
ftdi('GPIOH',0,[1,1]); 

% Write payload
sx1272_write('0x0D', 0); % sets SPI pointer to 0
sx1272_write(0, payload); % writes paylaod to Buffer in one single burst

% Clear IRQ
sx1272_write('0x12',0);

sx1272_write('0x01', 3 + 2^7);
sleep(0.01);

% wait for end of packet
tic
while ~bitget(sx1272_read('0x12'),4) && (toc < timeout)
   sleep(0.1)
end

% sets ANTENA switch to RX position
ftdi('GPIOH',0,[1,0]);

sx1272_write('0x01', 5 + 2^7);
sleep(0.01);


end

