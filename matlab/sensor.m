n = 256;
t = 3000;

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

t0 = 0;
tic

while toc < 30
	
	ts(1:n-1) = ts(2:n);
	gx(1:n-1) = gx(2:n);
	gy(1:n-1) = gy(2:n);
	gz(1:n-1) = gz(2:n);
	ax(1:n-1) = ax(2:n);
	ay(1:n-1) = ay(2:n);
	az(1:n-1) = az(2:n);
	
	fc.DEBUG(1);
	sleep(15);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	gx(n) = c2s(2^8 * b(3) + b(2), 16);
	gy(n) = c2s(2^8 * b(5) + b(4), 16);
	gz(n) = c2s(2^8 * b(7) + b(6), 16);
	ax(n) = c2s(2^8 * b(9) + b(8), 16);
	ay(n) = c2s(2^8 * b(11) + b(10), 16);
	az(n) = c2s(2^8 * b(13) + b(12), 16);

% 	t0 = t0 + 1;
% 	ts(n) = t0;
% 	gx(n) = c2s(2^8 * mpu.GYRO_X_H + mpu.GYRO_X_L, 16);
% 	gy(n) = c2s(2^8 * mpu.GYRO_Y_H + mpu.GYRO_Y_L, 16);
% 	gz(n) = c2s(2^8 * mpu.GYRO_Z_H + mpu.GYRO_Z_L, 16);
% 	ax(n) = c2s(2^8 * mpu.ACCEL_X_H + mpu.ACCEL_X_L, 16);
% 	ay(n) = c2s(2^8 * mpu.ACCEL_Y_H + mpu.ACCEL_Y_L, 16);
% 	az(n) = c2s(2^8 * mpu.ACCEL_Z_H + mpu.ACCEL_Z_L, 16);

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

