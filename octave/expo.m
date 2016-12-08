% clear all

x = -1:(2/2047):1;

y1(x>0) = -sqrt(1 - x(x>=0)) + 1;
y1(x<0) =  sqrt(1 + x(x<0)) - 1;

e = 4;
y2(x>0) = ( e.^( x(x>=0)) - 1)/(e^1-1);
y2(x<0) = (-e.^(-x(x<0)) + 1)/(e^1-1);

b = 0.2;
x3p = x(x>=0) + b;
x3m = x(x< 0) - b;
y3p =  x3p.^2 - b^2;
y3m = -x3m.^2 + b^2;
s = (1+b)^2-b^2;
y3 = [y3m,y3p]/s;

figure(1)
plot(x,x,'b',x,y1,'r',x,y2,'g',x,y3,'m')

b_set = [0.15,0.2,0.4];

figure(2)
hold off
plot(x,x,'b--')
hold on
plot(x,x/2,'b--')
plot(x,x/3,'b--')
plot(x,x/4,'b--')

for n = 1:length(b_set)
	b = b_set(n);
	x3p = x(x>=0) + b;
	x3m = x(x< 0) - b;
	y3p =  x3p.^2 - b^2;
	y3m = -x3m.^2 + b^2;
	s = (1+b)^2-b^2;
	y3 = [y3m,y3p]/s;
	plot(x,y3)
end


%%
len = 100000;
alpha = 0.0002;

vacc = zeros(1,len);
vacc(1) = 12.6/alpha;
vin = [12.6*ones(1,len/4), 11.8*ones(1,len/4), 12.6*ones(1,len/4), 11.4*ones(1,len/4)];

for n = 2:len
   vacc(n) = vacc(n-1) * (1-alpha) + vin(n);
end

t = (0:len-1)*1e-3;
vout = vacc * alpha;

figure(3)
plot(t,vout)