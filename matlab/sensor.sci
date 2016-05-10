function sensor(t)

n = 1024;
tmax = 15000;

ts = zeros(n,1);
gx = zeros(n,1);
gy = zeros(n,1);
gz = zeros(n,1);
ax = zeros(n,1);
ay = zeros(n,1);
az = zeros(n,1);

figure(1);
clf()
subplot(211)
plot2d([],rect=[0,-2^15,tmax,2^15])
xpoly([],[]);
line_gx = gce();
line_gx.foreground = 2;
xpoly([],[]);
line_gy = gce();
line_gy.foreground = 3;
xpoly([],[]);
line_gz = gce();
line_gz.foreground = 5;
subplot(212)
plot2d([],rect=[0,-2^15,tmax,2^15])
xpoly([],[]);
line_ax = gce();
line_ax.foreground = 2;
xpoly([],[]);
line_ay = gce();
line_ay.foreground = 3;
xpoly([],[]);
line_az = gce();
line_az.foreground = 5;

reg(1,1);

t0 = 0;
tic;

while toc() < t
	
	ts(1:n-1) = ts(2:n);
	gx(1:n-1) = gx(2:n);
	gy(1:n-1) = gy(2:n);
	gz(1:n-1) = gz(2:n);
	ax(1:n-1) = ax(2:n);
	ay(1:n-1) = ay(2:n);
	az(1:n-1) = az(2:n);
	
	ftdi('write',[4,0,0,0]);
	sleep(10);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	ax(n) = c2s(2^8 * b(2) + b(3), 16);
	ay(n) = c2s(2^8 * b(4) + b(5), 16);
	az(n) = c2s(2^8 * b(6) + b(7), 16);
	gx(n) = c2s(2^8 * b(8) + b(9), 16);
	gy(n) = c2s(2^8 * b(10) + b(11), 16);
	gz(n) = c2s(2^8 * b(12) + b(13), 16);
	
	line_gx.data = [ts-ts(1),gx];
	line_gy.data = [ts-ts(1),gy];
	line_gz.data = [ts-ts(1),gz];
	line_ax.data = [ts-ts(1),ax];
	line_ay.data = [ts-ts(1),ay];
	line_az.data = [ts-ts(1),az];
	
end

reg(1,0);

endfunction
