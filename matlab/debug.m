function debug(DebugCase,NbSamples)

if nargin < 2, NbSamples = 512; end

global fc
global ser

%% To Customize
WindowSize = 256;
switch DebugCase
	case {1,2,5,6,7,8} % sensor rate (1kHz)
      SampleMask = hex2dec('001F');
	case 4 % radio rate (~100Hz)
		SampleMask = hex2dec('0003');
	case 3 % vbat rate (100Hz)
		SampleMask = hex2dec('0003');
end
c = 'brgcmkyb';

%%
figure(DebugCase);
clf
l = {};
a = {};

switch DebugCase
	case 1 % raw sensors
		dlen = 7;
		dtype = 'int16';
		for n = 1:3
			a{n} = subplot(3,1,n);
         a{n}.YLim = [-2^15,2^15];
		end
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(n));
			l{4+n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
		l{4} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{3},'Color',c(1));
	case 2 % scaled sensors
		dlen = 7;
		dtype = 'float';
		for n = 1:3
			a{n} = subplot(3,1,n);
		end
		a{1}.YLim = [-2000,2000];
		a{2}.YLim = [-16,16];
		a{3}.YLim = [-10,100];
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
			l{n+3} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(n));
		end
		l{7} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{3},'Color',c(1));
	case 3 % vbat
		dlen = 1;
		dtype = 'float';
		a{1} = axes;
		l{1} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(1));
	case 4 % raw commands
		dlen = 8;
		dtype = 'uint16';
		a{1} = axes;
		for n = 1:8
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 5 % commands
		dlen = 8;
		dtype = 'float';
		a{1} = subplot(211);
		a{2} = subplot(212);
		a{1}.YLim = [-1.05,1.05];
		a{2}.YLim = [-0.05,1.05];
		for n = [1,5,6,7,8]
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(n));
		end
		for n = 2:4
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 6 % pitch, roll, yaw
		dlen = 3;
		dtype = 'float';
		a{1} = axes;
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 7 % motors
		dlen = 4;
		dtype = 'float';
		for n = 1:4
			a{n} = subplot(4,1,n);
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{n},'Color',c(n));
		end
	case 8 % raw motors
		dlen = 4;
		dtype = 'int16';
		for n = 1:4
			a{n} = subplot(4,1,n);
			a{n}.YLim = [-10,2010];
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{n},'Color',c(n));
		end
end

for n = 1:length(a)
	a{n}.XLim = [1,WindowSize];
end

d = nan(dlen,WindowSize);
switch dtype
	case {'uint16','int16'}
		r1 = 2*dlen;
	case 'float'
		r1 = 4*dlen;
end
n = 0;

% Empty VCP buffer
fc.DEBUG(0);
sleep(10);
while (ser.BytesAvailable > 0)
	fread(ser,ser.BytesAvailable);
   sleep(10);
end

fc.DEBUG__MASK(SampleMask);
fc.DEBUG__CASE(DebugCase);

while (n < NbSamples)
	
	if (ser.BytesAvailable >= r1)

		r = fread(ser,r1);
		n = n + 1;

		for m = 1:dlen
			switch dtype
            case 'uint16'
					d(m,WindowSize) = 2^8*r((m-1)*2+2) + r((m-1)*2+1);
				case 'int16'
					d(m,WindowSize) = c2s(2^8*r((m-1)*2+2) + r((m-1)*2+1), 16);
				case 'float'
					d(m,WindowSize) = typecast(uint32(2^24*r((m-1)*4+4) + 2^16*r((m-1)*4+3) + 2^8*r((m-1)*4+2) + r((m-1)*4+1)), 'single');
				end
			set(l{m},'XData',1:WindowSize);
			set(l{m},'YData',d(m,:));
		end
		drawnow

		d(:,1:WindowSize-1) = d(:,2:WindowSize);

	end

end

% Empty VCP buffer
fc.DEBUG(0);
sleep(10);
while (ser.BytesAvailable > 0)
	fread(ser,ser.BytesAvailable);
   sleep(10);
end

end
