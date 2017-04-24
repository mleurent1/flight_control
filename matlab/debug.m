function debug(DebugCase,NbSamples)

if nargin < 2, NbSamples = 512; end

global fc
global ser

figure(DebugCase);
clf
l = {};
a = {};
c = 'brgcmk';

WindowSize = 512;

switch DebugCase
	case 1 % raw sensors
		dlen = 7;
		dtype = 'int16';
		for n = 1:3
			a{n} = subplot(3,1,n);
			set(a{n},'XLim',[1,WindowSize],'YLim',[-2^15,2^15]);
		end
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
			l{3+n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(n));
		end
		l{7} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{3},'Color',c(1));
	case 2 % scaled sensors
		dlen = 7;
		dtype = 'float';
		for n = 1:3
			a{n} = subplot(3,1,n);
			set(a{n},'XLim',[1,WindowSize]);
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
		a{1} = axes;
		set(a{1},'XLim',[1,WindowSize]);
		set(a{1},'YLim',[-1,20]);
		l{1} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(1));
	case 4 % raw commands
		dlen = 6;
		dtype = 'int16';
		a{1} = axes;
		set(a{1},'XLim',[1,WindowSize]);
		%set(a{1},'YLim',[0,2048]);
		for n = 1:6
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 5 % commands
		dlen = 4;
		dtype = 'float';
		a{1} = subplot(211);
		a{2} = subplot(212);
		set(a{1},'XLim',[1,WindowSize]);
		set(a{2},'XLim',[1,WindowSize]);
		set(a{1},'YLim',[-1,1]);
		set(a{2},'YLim',[0,1]);
		l{1} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{2},'Color',c(1));
		for n = 2:4
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 6 % pitch, roll, yaw
		dlen = 3;
		dtype = 'float';
		a{1} = axes;
		set(a{1},'XLim',[1,WindowSize]);
		set(a{1},'YLim',[-3000,3000]);
		for n = 1:3
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{1},'Color',c(n));
		end
	case 7 % motors
		dlen = 4;
		dtype = 'float';
		for n = 1:4
			a{n} = subplot(4,1,n);
			set(a{n},'XLim',[1,WindowSize]);
			set(a{n},'YLim',[-3000,3000]);
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{n},'Color',c(n));
		end
	case 8 % raw motors
		dlen = 4;
		dtype = 'int16';
		for n = 1:4
			a{n} = subplot(4,1,n);
			set(a{n},'XLim',[1,WindowSize]);
			set(a{n},'YLim',[-10,2010]);
			l{n} = line(nan(1,WindowSize),nan(1,WindowSize),'Parent',a{n},'Color',c(n));
		end
end

d = nan(dlen,WindowSize);
switch dtype
	case 'int16'
		r1 = 2*dlen+2;
	case 'float'
		r1 = 4*dlen+2;
end
n = 0;

while n < NbSamples
	
	fc.DEBUG(DebugCase);
	r = fread(ser,r1);
	n = n + 1;
	
	for m = 1:dlen
		switch dtype
			case 'int16'
				d(m,WindowSize) = c2s(2^8*r((m-1)*2+4) + r((m-1)*2+3), 16);
			case 'float'
				d(m,WindowSize) = typecast(uint32(2^24*r((m-1)*4+6) + 2^16*r((m-1)*4+5) + 2^8*r((m-1)*4+4) + r((m-1)*4+3)), 'single');
			end
		set(l{m},'XData',1:WindowSize);
		set(l{m},'YData',d(m,:));
	end
	drawnow
	
	d(:,1:WindowSize-1) = d(:,2:WindowSize);
	
end

end
