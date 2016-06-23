n = 256;
t = 800;

ts = zeros(n,1);
th = zeros(n,1);
ai = zeros(n,1);
el = zeros(n,1);
ru = zeros(n,1);
ar = zeros(n,1);

figure(3)
clf

a = axes;
a.XLim = [0,t];
a.YLim = [0,2048];
lth = line(zeros(1,n),zeros(1,n));
lth.Color = 'b';
lai = line(zeros(1,n),zeros(1,n));
lai.Color = 'r';
lel = line(zeros(1,n),zeros(1,n));
lel.Color = 'g';
lru = line(zeros(1,n),zeros(1,n));
lru.Color = 'c';
lar = line(zeros(1,n),zeros(1,n));
lar.Color = 'm';

t0 = 0;
tic

while toc < 30
	
	ts(1:n-1) = ts(2:n);
	th(1:n-1) = th(2:n);
	ai(1:n-1) = ai(2:n);
	el(1:n-1) = el(2:n);
	ru(1:n-1) = ru(2:n);
	ar(1:n-1) = ar(2:n);
	
	fc.DEBUG(3);
	sleep(20);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	th(n) = 2^8 * b(3) + b(2);
	ai(n) = 2^8 * b(5) + b(4);
	el(n) = 2^8 * b(7) + b(6);
	ru(n) = 2^8 * b(9) + b(8);
	ar(n) = 2^8 * b(11) + b(10);

	lth.XData = ts-ts(1);
	lth.YData = th;
	lai.XData = ts-ts(1);
	lai.YData = ai;
	lel.XData = ts-ts(1);
	lel.YData = el;
	lru.XData = ts-ts(1);
	lru.YData = ru;
	lar.XData = ts-ts(1);
	lar.YData = ar;
	
	drawnow
end

