n = 256;
t = 3000;

ts = zeros(n,1);
gx = zeros(n,1);
gy = zeros(n,1);
gz = zeros(n,1);

figure(2)
clf

a = axes;
a.XLim = [0,t];
a.YLim = [-125,125];
lgx = line(zeros(1,n),zeros(1,n));
lgx.Color = 'b';
lgy = line(zeros(1,n),zeros(1,n));
lgy.Color = 'r';
lgz = line(zeros(1,n),zeros(1,n));
lgz.Color = 'g';

t0 = 0;
tic

while toc < 30
	
	ts(1:n-1) = ts(2:n);
	gx(1:n-1) = gx(2:n);
	gy(1:n-1) = gy(2:n);
	gz(1:n-1) = gz(2:n);
	
	fc.DEBUG(2);
	sleep(15);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	gx(n) = typecast(uint32(2^24*b(5) + 2^16*b(4) + 2^8*b(3) + b(2)), 'single');
	gy(n) = typecast(uint32(2^24*b(9) + 2^16*b(8) + 2^8*b(7) + b(6)), 'single');
	gz(n) = typecast(uint32(2^24*b(13) + 2^16*b(12) + 2^8*b(11) + b(10)), 'single');

	lgx.XData = ts-ts(1);
	lgx.YData = gx;
	lgy.XData = ts-ts(1);
	lgy.YData = gy;
	lgz.XData = ts-ts(1);
	lgz.YData = gz;
	
	drawnow
end

