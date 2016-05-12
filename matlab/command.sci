function command(t)

global R

n = 1024;
tmax = 1400;

ts = zeros(n,1);
th = zeros(n,1);
ai = zeros(n,1);
el = zeros(n,1);
ru = zeros(n,1);

figure(2);
clf()
plot2d([],rect=[0,0,tmax,2048])
xpoly([],[]);
line_th = gce();
line_th.foreground = 2;
xpoly([],[]);
line_ai = gce();
line_ai.foreground = 3;
xpoly([],[]);
line_el = gce();
line_el.foreground = 5;
xpoly([],[]);
line_ru = gce();
line_ru.foreground = 0;

reg(R.DEBUG_MUX,2);

t0 = 0;
tic;

while toc() < t
	
	ts(1:n-1) = ts(2:n);
	th(1:n-1) = th(2:n);
	ai(1:n-1) = ai(2:n);
	el(1:n-1) = el(2:n);
	ru(1:n-1) = ru(2:n);
	
	ftdi('write',[4,0,0,0]);
	sleep(11);
	b = ftdi('read');
	
	if b(1) < (ts(n-1)-t0)
		t0 = t0 + 256;
	end
	ts(n) = t0 + b(1);
	th(n) = 2^8 * b(5) + b(4);
	ai(n) = 2^8 * b(7) + b(6);
	el(n) = 2^8 * b(9) + b(8);
	ru(n) = 2^8 * b(11) + b(10);
	
	line_th.data = [ts-ts(1),th];
	line_ai.data = [ts-ts(1),ai];
	line_el.data = [ts-ts(1),el];
	line_ru.data = [ts-ts(1),ru];
	
end

reg(R.DEBUG_MUX,0);

endfunction
