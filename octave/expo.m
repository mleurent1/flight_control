% clear all

x = -1:(2/2047):1;

a = 3;


y1(x>=0) = (e.^( x(x>=0)*a) - 1)/(e^(a)-1);
y1(x<0) = (-e.^(-x(x<0)*a) + 1)/(e^(a)-1);

b = a + a^2/2 + a^3/6 + a^4/24 + a^5/120 + a^6/720;
y2(x>=0) = ((x(x>=0)*a) + (x(x>=0)*a).^(2)/2 + (x(x>=0)*a).^(3)/6 + (x(x>=0)*a).^(4)/24 + (x(x>=0)*a).^(5)/120 + (x(x>=0)*a).^(6)/720) / b;
y2(x<0) = -((-x(x<0)*a) + (-x(x<0)*a).^(2)/2 + (-x(x<0)*a).^(3)/6 + (-x(x<0)*a).^(4)/24 + (-x(x<0)*a).^(5)/120 + (-x(x<0)*a).^(6)/720) / b;

figure(1)
plot(x,x,'b--',x,x/8,'b--',x,y1,'r',x,y2,'g')


%%
len = 100000;
%alpha = 0.0002;
alpha = 0.0002;

vacc = zeros(1,len);
vacc(1) = 12.6/alpha;
vin = [12.6*ones(1,len/4), 11.8*ones(1,len/4), 12.6*ones(1,len/4), 11.4*ones(1,len/4)];

for n = 2:len
   vacc(n) = vacc(n-1) * (1-alpha) + vin(n);
end

t = (0:len-1)*1e-3;
vout = vacc * alpha;

figure(2)
plot(t,vout)

%%
len = 20;
alpha = 0.4;

cacc = zeros(1,len);
cin = ones(1,len);

for n = 2:len
   cacc(n) = cacc(n-1) * (1-alpha) + cin(n);
end

cout = cacc * alpha;

figure(3)
plot(0:len-1,cout)