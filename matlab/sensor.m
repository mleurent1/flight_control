n = 1024;
t = 25000;

ts = zeros(n,1);
gx = zeros(n,1);
gy = zeros(n,1);
gz = zeros(n,1);
ax = zeros(n,1);
ay = zeros(n,1);
az = zeros(n,1);

figure(1)
clf

a = subplot(211);
a.XLim = [0,t];
a.YLim = [-2^15,2^15];
lgx = line(zeros(1,n),zeros(1,n));
lgx.Color = 'b';
lgy = line(zeros(1,n),zeros(1,n));
lgy.Color = 'r';
lgz = line(zeros(1,n),zeros(1,n));
lgz.Color = 'g';

a = subplot(212);
a.XLim = [0,t];
a.YLim = [-2^15,2^15];
lax = line(zeros(1,n),zeros(1,n));
lax.Color = 'b';
lay = line(zeros(1,n),zeros(1,n));
lay.Color = 'r';
laz = line(zeros(1,n),zeros(1,n));
laz.Color = 'g';

fc.DEBUG_MUX(1);

t0 = 0;

while 1
	
	ts(1:n-1) = ts(2:n);
	gx(1:n-1) = gx(2:n);
	gy(1:n-1) = gy(2:n);
	gz(1:n-1) = gz(2:n);
	ax(1:n-1) = ax(2:n);
	ay(1:n-1) = ay(2:n);
	az(1:n-1) = az(2:n);
	
	ftdi('write',[4,0,0,0]);
	sleep(8);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	gx(n) = c2s(2^8 * b(2) + b(3), 16);
	gy(n) = c2s(2^8 * b(4) + b(5), 16);
	gz(n) = c2s(2^8 * b(6) + b(7), 16);
	ax(n) = c2s(2^8 * b(8) + b(9), 16);
	ay(n) = c2s(2^8 * b(10) + b(11), 16);
	az(n) = c2s(2^8 * b(12) + b(13), 16);
	
	lgx.XData = ts-ts(1);
	lgx.YData = gx;
	lgy.XData = ts-ts(1);
	lgy.YData = gy;
	lgz.XData = ts-ts(1);
	lgz.YData = gz;
	lax.XData = ts-ts(1);
	lax.YData = ax;
	lay.XData = ts-ts(1);
	lay.YData = ay;
	laz.XData = ts-ts(1);
	laz.YData = az;
	
	drawnow
end

% fc.DEBUG_MUX(0);

