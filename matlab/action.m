n = 256;
t = 3000;

fc.PITCH_P(double(typecast(single(100.0),'uint32')));

ts = zeros(n,1);
pi = zeros(n,1);
ro = zeros(n,1);
ya = zeros(n,1);

figure(5)
clf

a = axes;
a.XLim = [0,t];
%a.YLim = [-125,125];
lpi = line(zeros(1,n),zeros(1,n));
lpi.Color = 'b';
lro = line(zeros(1,n),zeros(1,n));
lro.Color = 'r';
lya = line(zeros(1,n),zeros(1,n));
lya.Color = 'g';

t0 = 0;
tic

while toc < 30
	
	ts(1:n-1) = ts(2:n);
	pi(1:n-1) = pi(2:n);
	ro(1:n-1) = ro(2:n);
	ya(1:n-1) = ya(2:n);
	
	fc.DEBUG(5);
	sleep(15);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	pi(n) = typecast(uint32(2^24*b(5) + 2^16*b(4) + 2^8*b(3) + b(2)), 'single');
	ro(n) = typecast(uint32(2^24*b(9) + 2^16*b(8) + 2^8*b(7) + b(6)), 'single');
	ya(n) = typecast(uint32(2^24*b(13) + 2^16*b(12) + 2^8*b(11) + b(10)), 'single');

	lpi.XData = ts-ts(1);
	lpi.YData = pi;
	lro.XData = ts-ts(1);
	lro.YData = ro;
	lya.XData = ts-ts(1);
	lya.YData = ya;
	
	drawnow
end

