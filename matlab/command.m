n = 256;
t = 1000;

ts = zeros(n,1);
th = zeros(n,1);
ai = zeros(n,1);
el = zeros(n,1);
ru = zeros(n,1);

figure(4)
clf

a = subplot(211);
a.XLim = [0,t];
a.YLim = [-125,125];
lai = line(zeros(1,n),zeros(1,n));
lai.Color = 'b';
lel = line(zeros(1,n),zeros(1,n));
lel.Color = 'r';
lru = line(zeros(1,n),zeros(1,n));
lru.Color = 'g';

a = subplot(212);
a.XLim = [0,t];
a.YLim = [0,800];
lth = line(zeros(1,n),zeros(1,n));
lth.Color = 'b';

t0 = 0;
tic

while toc < 30
	
	ts(1:n-1) = ts(2:n);
	th(1:n-1) = th(2:n);
	ai(1:n-1) = ai(2:n);
	el(1:n-1) = el(2:n);
	ru(1:n-1) = ru(2:n);
	
	fc.DEBUG(4);
	sleep(25);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	th(n) = typecast(uint32(2^24*b(5) + 2^16*b(4) + 2^8*b(3) + b(2)), 'single');
	ai(n) = typecast(uint32(2^24*b(9) + 2^16*b(8) + 2^8*b(7) + b(6)), 'single');
	el(n) = typecast(uint32(2^24*b(13) + 2^16*b(12) + 2^8*b(11) + b(10)), 'single');
	ru(n) = typecast(uint32(2^24*b(17) + 2^16*b(16) + 2^8*b(15) + b(14)), 'single');
	
	lth.XData = ts-ts(1);
	lth.YData = th;
	lai.XData = ts-ts(1);
	lai.YData = ai;
	lel.XData = ts-ts(1);
	lel.YData = el;
	lru.XData = ts-ts(1);
	lru.YData = ru;
	
	drawnow
end

