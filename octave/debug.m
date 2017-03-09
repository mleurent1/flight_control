function debug(DebugCase,NbSamples)

if nargin < 2, NbSamples = 1024; end

global fc
global ser

figure(DebugCase);
clf
l = {};
a = {};
c = 'brgcmk';

WindowSizeSensor = 256;
TimeWindowSizeSensor = 10000;
WindowSizeCommand = 256;
TimeWindowSizeCommand = 500;
WindowSizeVbat = 256;
TimeWindowSizeVbat = 500;

switch DebugCase
	case 1 % raw sensors
		dlen = 7;
		dtype = 'int16';
		WindowSize = WindowSizeSensor;
		TimeWindowSize = TimeWindowSizeSensor;
		for n = 1:3
			a{n} = subplot(3,1,n);
			set(a{n},'XLim',[0,TimeWindowSize],'YLim',[-2^15,2^15]);
		end
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
			l{3+n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(n));
		end
		l{7} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{3},'Color',c(1));
	case 2 % scaled sensors
		dlen = 7;
		dtype = 'float';
		WindowSize = WindowSizeSensor;
		TimeWindowSize = TimeWindowSizeSensor;
		for n = 1:3
			a{n} = subplot(3,1,n);
			set(a{n},'XLim',[0,TimeWindowSize]);
		end
		set(a{1},'YLim',[-2000,2000]);
		set(a{2},'YLim',[-16,16]);
		set(a{3},'YLim',[-10,100]);
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
			l{n+3} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(n));
		end
		l{7} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{3},'Color',c(1));
	case 3 % vbat
		dlen = 1;
		dtype = 'float';
		WindowSize = WindowSizeVbat;
		TimeWindowSize = TimeWindowSizeVbat;
		a{1} = axes;
		set(a{1},'XLim',[0,TimeWindowSize]);
		set(a{1},'YLim',[0,17]);
		l{1} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(1));
	case 4 % raw commands
		dlen = 6;
		dtype = 'int16';
		WindowSize = WindowSizeCommand;
		TimeWindowSize = TimeWindowSizeCommand;
		a{1} = axes;
		set(a{1},'XLim',[0,TimeWindowSize]);
		set(a{1},'YLim',[0,2048]);
		for n = 1:6
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 5 % commands
		dlen = 4;
		dtype = 'float';
		WindowSize = WindowSizeCommand;
		TimeWindowSize = TimeWindowSizeCommand;
		a{1} = subplot(211);
		a{2} = subplot(212);
		set(a{1},'XLim',[0,TimeWindowSize]);
		set(a{2},'XLim',[0,TimeWindowSize]);
		set(a{1},'YLim',[-1,1]);
		set(a{2},'YLim',[0,1]);
		l{1} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(1));
		for n = 2:4
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 6 % pitch, roll, yaw
		dlen = 3;
		dtype = 'float';
		WindowSize = WindowSizeSensor;
		TimeWindowSize = TimeWindowSizeSensor;
		a{1} = axes;
		set(a{1},'XLim',[0,TimeWindowSize]);
		set(a{1},'YLim',[-3000,3000]);
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 7 % motors
		dlen = 4;
		dtype = 'float';
		WindowSize = WindowSizeSensor;
		TimeWindowSize = TimeWindowSizeSensor;
		for n = 1:4
			a{n} = subplot(4,1,n);
			set(a{n},'XLim',[0,TimeWindowSize]);
			set(a{n},'YLim',[-3000,3000]);
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{n},'Color',c(n));
		end
	case 8 % raw motors
		dlen = 4;
		dtype = 'int16';
		WindowSize = WindowSizeSensor;
		TimeWindowSize = TimeWindowSizeSensor;
		for n = 1:4
			a{n} = subplot(4,1,n);
			set(a{n},'XLim',[0,TimeWindowSize]);
			set(a{n},'YLim',[-10,2010]);
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{n},'Color',c(n));
		end
end

t = nan(1,WindowSize);
d = nan(dlen,WindowSize);
switch dtype
	case 'int16'
		r1 = 2*dlen+2;
	case 'float'
		r1 = 4*dlen+2;
end
n = 0;
t0 = 0;

while n < NbSamples
	
	fc.DEBUG(DebugCase);
	r = double(srl_read(ser,r1));
	n = n + 1;
	
	t1 = r(1) + 2^8*r(2);
	if t1 < (t(WindowSize-1)-t0)
		t0 = t0 + 2^16;
	end
	if n == 1
		t = ones(1,WindowSize)*(t0 + t1);
	else
		t(WindowSize) = t0 + t1;
	end
	for m = 1:dlen
		switch dtype
			case 'int16'
				d(m,WindowSize) = c2s(2^8*r((m-1)*2+4) + r((m-1)*2+3), 16);
			case 'float'
				d(m,WindowSize) = typecast(uint32(2^24*r((m-1)*4+6) + 2^16*r((m-1)*4+5) + 2^8*r((m-1)*4+4) + r((m-1)*4+3)), 'single');
		end
		set(l{m},'XData',t-t(1));
		set(l{m},'YData',d(m,:));
	end
	drawnow
	
	t(1:WindowSize-1) = t(2:WindowSize);
	d(:,1:WindowSize-1) = d(:,2:WindowSize);
	
end

end

