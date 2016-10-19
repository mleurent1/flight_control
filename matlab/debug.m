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
TimeWindowSizeSensor = 5000;
WindowSizeCommand = 256;
TimeWindowSizeCommand = 500;

switch DebugCase
	case 1 % raw sensors
		dlen = 6;
		dtype = 'int16';
		WindowSize = WindowSizeSensor;
		TimeWindowSize = TimeWindowSizeSensor;
		for n = 1:2
			a{n} = subplot(2,1,n);
			a{n}.XLim = [0,TimeWindowSize];
			a{n}.YLim = [-2^15,2^15];
		end
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
			l{3+n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(n));
		end
	case 2 % scaled sensors
		dlen = 6;
		dtype = 'float';
		WindowSize = WindowSizeSensor;
		TimeWindowSize = TimeWindowSizeSensor;
		for n = 1:2
			a{n} = subplot(2,1,n);
			a{n}.XLim = [0,TimeWindowSize];
		end
		a{1}.YLim = [-2000,2000];
		a{2}.YLim = [-16,16];
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
			l{n+3} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(n));
		end
	case 4 % raw commands
		dlen = 6;
		dtype = 'int16';
		WindowSize = WindowSizeCommand;
		TimeWindowSize = TimeWindowSizeCommand;
		a{1} = axes;
		a{1}.XLim = [0,TimeWindowSize];
		a{1}.YLim = [0,2048];
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
		a{1}.XLim = [0,TimeWindowSize];
		a{2}.XLim = [0,TimeWindowSize];
		a{1}.YLim = [-1,1];
		a{2}.YLim = [0,1];
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
		a{1}.XLim = [0,TimeWindowSize];
		a{1}.YLim = [-2000,2000];
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
			a{n}.XLim = [0,TimeWindowSize];
			a{n}.YLim = [-2000,2000];
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{n},'Color',c(n));
		end
	case 8 % raw motors
		dlen = 4;
		dtype = 'int16';
		WindowSize = WindowSizeSensor;
		TimeWindowSize = TimeWindowSizeSensor;
		for n = 1:4
			a{n} = subplot(4,1,n);
			a{n}.XLim = [0,TimeWindowSize];
			a{n}.YLim = [990,2010];
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{n},'Color',c(n));
		end
end

t = nan(1,WindowSize);
d = nan(dlen,WindowSize);
switch dtype
	case 'int16'
		r1 = 2*dlen+1;
	case 'float'
		r1 = 4*dlen+1;
end
n = 0;
t0 = 0;

while n < NbSamples
	
	fc.DEBUG(DebugCase);
	r = fread(ser,r1);
	n = n + 1;
	
	if r(1) < (t(WindowSize-1)-t0)
		t0 = t0 + 256;
	end
	if n == 1
		t = ones(1,WindowSize)*(t0 + r(1));
	else
		t(WindowSize) = t0 + r(1);
	end
	for m = 1:dlen
		switch dtype
			case 'int16'
				d(m,WindowSize) = c2s(2^8*r((m-1)*2+3) + r((m-1)*2+2), 16);
			case 'float'
				d(m,WindowSize) = typecast(uint32(2^24*r((m-1)*4+5) + 2^16*r((m-1)*4+4) + 2^8*r((m-1)*4+3) + r((m-1)*4+2)), 'single');
		end
		l{m}.XData = t-t(1);
		l{m}.YData = d(m,:);
	end
	drawnow
	
	t(1:WindowSize-1) = t(2:WindowSize);
	d(:,1:WindowSize-1) = d(:,2:WindowSize);
	
end

end

