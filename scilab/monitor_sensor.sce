n0 = 4096;
n1 = 256; 
gx_hist = zeros(1,n0);
gy_hist = zeros(1,n0);
gz_hist = zeros(1,n0);
ax_hist = zeros(1,n0);
ay_hist = zeros(1,n0);
az_hist = zeros(1,n0);

figure(1);
clf()
subplot(211)
plot2d([],rect=[1,-2^15,n0,2^15])
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
plot2d([],rect=[1,-2^15,n0,2^15])
xpoly([],[]);
line_ax = gce();
line_ax.foreground = 2;
xpoly([],[]);
line_ay = gce();
line_ay.foreground = 3;
xpoly([],[]);
line_az = gce();
line_az.foreground = 5;

for n = 1:100
	
	gx = get_sensor(reg.GYRO_X,n1);
	gx_hist(1:n0-n1) = gx_hist(n1+1:n0)
	gx_hist(n0-n1+1:n0) = gx;
	
	gy = get_sensor(reg.GYRO_Y,n1);
	gy_hist(1:n0-n1) = gy_hist(n1+1:n0)
	gy_hist(n0-n1+1:n0) = gy;
	
	gz = get_sensor(reg.GYRO_Z,n1);
	gz_hist(1:n0-n1) = gz_hist(n1+1:n0)
	gz_hist(n0-n1+1:n0) = gz;
	
	ax = get_sensor(reg.ACCEL_X,n1);
	ax_hist(1:n0-n1) = ax_hist(n1+1:n0)
	ax_hist(n0-n1+1:n0) = ax;
	
	ay = get_sensor(reg.ACCEL_Y,n1);
	ay_hist(1:n0-n1) = ay_hist(n1+1:n0)
	ay_hist(n0-n1+1:n0) = ay;
	
	az = get_sensor(reg.ACCEL_Z,n1);
	az_hist(1:n0-n1) = az_hist(n1+1:n0)
	az_hist(n0-n1+1:n0) = az;
		
	line_gx.data = [(1:n0)',gx_hist'];
	line_gy.data = [(1:n0)',gy_hist'];
	line_gz.data = [(1:n0)',gz_hist'];
	
	line_ax.data = [(1:n0)',ax_hist'];
	line_ay.data = [(1:n0)',ay_hist'];
	line_az.data = [(1:n0)',az_hist'];
	
	sleep(120)
end

