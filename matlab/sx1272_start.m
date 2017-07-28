function sx1272_start

p.freq_hz = 865e6;      % center frequency
p.rf_power = 14;        % TX power, in dBm 
p.modulation = 'LORA';  % modulation to use for the packet 
p.bandwidth = 125e3;    % modulation bandwidth (LoRa only) 
p.datarate = 7;         % TX datarate (baudrate for FSK, SF for LoRa) 
p.coderate = 1;         % error-correcting code of the packet (LoRa only) 
p.invert_pol = 0;       % invert signal polarity, for orthogonal downlinks (LoRa only) 
p.f_dev = 25;           % frequency deviation, in kHz (FSK only) 
p.preamble = 8;         % set the preamble length, 0 for default 
p.no_crc = 0;           % if true, do not send a CRC in the packet 
p.no_header = 0;        % if true, enable implicit header mode (LoRa), fixed length (FSK) 
p.size = 16;            % payload size in bytes 
p.ppm_offset = 0;
p.sync_word = [1,2];

% Hard Reset
ftdi('GPIOL',5,[1,1]);
sleep(0.01)
ftdi('GPIOL',5,[1,0]);
sleep(0.01)

% Check version
if ~ismember(sx1272_read('0x42'), [hex2dec('21'),hex2dec('22')])
   error('SX1272: version not expected');
end

% PLL freq
f = round(p.freq_hz * 2^19 / 32e6);
PaRamp = 3; %3:500us, 9:40us
LowPnTxPllOff = 0;
LnaGain = 1;
TrimRxCrFo = 0;
LnaBoost = 2; % LNA current, 0:default, 2:150%
AgcAutoOn = 1; % LNA gain set by AGC
PaSelect = 1; %0:RFO, 1:PA boost, no RFO on ref board

% Output power
pwr = floor(p.rf_power);
if PaSelect
   if (pwr < 2) || (pwr > 20)
      warning('Output power must be between 2 and 20 dBm')
      OutputPower = 0;
   else
      if pwr > 17
         sx1272_write('0x5A','0x87');
         OutputPower = pwr - 5;
      else
         sx1272_write('0x5A','0x84');
         OutputPower = pwr - 2;
      end
   end
else
   if (pwr < -1) || (pwr > 14)
      warning('Output power must be between -1 and 14 dBm')
      OutputPower = 0;
   else
      OutputPower = pwr + 1;
   end
end

SymbTimeout = 100;
TxContinuousMode = 0;
PayloadMaxLength = 255;
bw = log2(p.bandwidth/125e3);
RfBypass = 0;
TxContChirp = 1;
AgcFreezeOnDetect = 0;
RxInvertIfFreq = 0;
DecDagcGain = 2;
FrameSyncPeak1Pos = p.sync_word(1);
FrameSyncPeak2Pos = p.sync_word(2);
PreambFineTimingGain = 1;
FreqToTimeDrift = hex2dec('25');
FreqToTimeInvert = hex2dec('1D');
TxChirpLowPass = 5;
SdEdgeSelect = 0;
SdMaxFreqDev = 0;
SdOrder = 1;

% Set chip in standby Lora mode
sx1272_write('0x01', 0);
sleep(0.01);
sx1272_write('0x01', 2^7);
sleep(0.01);
sx1272_write('0x01', 1 + 2^7);
sleep(0.01);

sx1272_write('0x06', bitand(f,hex2dec('FF0000'))/2^16);
sx1272_write('0x07', bitand(f,hex2dec('00FF00'))/2^8);
sx1272_write('0x08', bitand(f,hex2dec('0000FF')));
sx1272_write('0x09', OutputPower + 2^7*PaSelect);
sx1272_write('0x0A', PaRamp + 2^4*LowPnTxPllOff);
sx1272_write('0x0C', LnaBoost + 2^3*TrimRxCrFo + 2^5*LnaGain);
sx1272_write('0x0D', 0); % sets SPI pointer to 0
sx1272_write('0x0E', 0); % sets TX modem start addr to 0
sx1272_write('0x1D', p.ppm_offset + 2^1*~p.no_crc + 2^2*p.no_header + 2^3*p.coderate + 2^6*bw);
sx1272_write('0x1E', bitand(SymbTimeout,hex2dec('300'))/2^8 + 2^2*AgcAutoOn + 2^3*TxContinuousMode + 2^4*p.datarate);
sx1272_write('0x1F', bitand(SymbTimeout,hex2dec('0FF')));
sx1272_write('0x20', floor(p.preamble/2^8));
sx1272_write('0x21', mod(p.preamble,2^8));
sx1272_write('0x22', p.size);
sx1272_write('0x23', PayloadMaxLength);
Rx33 = bitand(sx1272_read('0x33'),bin2dec('10111111'));
sx1272_write('0x33', Rx33 + 2^6 * p.invert_pol);
sx1272_write('0x37', 10); % DetectMinSinglePeak
sx1272_write('0x38', DecDagcGain + 2^4*RxInvertIfFreq + 2^5*AgcFreezeOnDetect + 2^6*TxContChirp + 2^7*RfBypass);
sx1272_write('0x39', FrameSyncPeak2Pos + 2^4*FrameSyncPeak1Pos);
sx1272_write('0x3A', FreqToTimeDrift + 2^6*PreambFineTimingGain);
sx1272_write('0x3B', FreqToTimeInvert);
sx1272_write('0x3D', SdOrder + 2^1*SdMaxFreqDev + 2^4*SdEdgeSelect + 2^5*TxChirpLowPass);

% sets ANTENA switch to RX position
ftdi('GPIOH',0,[1,0]);

sx1272_write('0x01', 5 + 2^7);
sleep(0.01);
   
end


