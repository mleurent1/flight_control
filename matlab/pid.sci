function pid(t)

global R

reg(R.PITCH_P,100);
reg(R.PITCH_I,0);
reg(R.PITCH_D,0);

reg(R.ROLL_P,0);
reg(R.ROLL_I,0);
reg(R.ROLL_D,0);

reg(R.YAW_P,0);
reg(R.YAW_I,0);
reg(R.YAW_D,0);

n = 1024;
tmax = 12000;

ts = zeros(n,1);
pitch = zeros(n,1);
roll = zeros(n,1);
yaw = zeros(n,1);

figure(3);
clf()
plot2d([],rect=[0,-2^15,tmax,2^15])
xpoly([],[]);
line_pitch = gce();
line_pitch.foreground = 2;
xpoly([],[]);
line_roll = gce();
line_roll.foreground = 3;
xpoly([],[]);
line_yaw = gce();
line_yaw.foreground = 5;

reg(R.DEBUG_MUX,3);

t0 = 0;
tic;

while toc() < t
	
	ts(1:n-1) = ts(2:n);
	pitch(1:n-1) = pitch(2:n);
	roll(1:n-1) = roll(2:n);
	yaw(1:n-1) = yaw(2:n);
	
	ftdi('write',[4,0,0,0]);
	sleep(7);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	pitch(n) = c2s(2^8 * b(2) + b(3), 16);
	roll(n) = c2s(2^8 * b(4) + b(5), 16);
	yaw(n) = c2s(2^8 * b(6) + b(7), 16);
	
	line_pitch.data = [ts-ts(1),pitch];
	line_roll.data = [ts-ts(1),roll];
	line_yaw.data = [ts-ts(1),yaw];
	
end

reg(R.DEBUG_MUX,0);

endfunction
