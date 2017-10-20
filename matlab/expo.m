clear

x = -1:(2/2047):1;

a = 3;

y1(x>=0) = (exp(x(x>=0)*a) - 1)/(exp(a)-1);
y1(x<0) = (-exp(-x(x<0)*a) + 1)/(exp(a)-1);

b = a + a^2/2 + a^3/6 + a^4/24 + a^5/120 + a^6/720;
y2(x>=0) = ((x(x>=0)*a) + (x(x>=0)*a).^(2)/2 + (x(x>=0)*a).^(3)/6 + (x(x>=0)*a).^(4)/24 + (x(x>=0)*a).^(5)/120 + (x(x>=0)*a).^(6)/720) / b;
y2(x<0) = -((-x(x<0)*a) + (-x(x<0)*a).^(2)/2 + (-x(x<0)*a).^(3)/6 + (-x(x<0)*a).^(4)/24 + (-x(x<0)*a).^(5)/120 + (-x(x<0)*a).^(6)/720) / b;

figure(1)
plot(x,x,'b--',x,x/8,'b--',x,y1,'r',x,y2,'g')


%%
period_ms = 10;
t_const_ms = 5000;
alpha = 5*period_ms/t_const_ms;
t_ms = 0:period_ms:3*t_const_ms-1;
len = length(t_ms);
vout = zeros(1,len);
vout(1) = 15;
vin = [16.8*ones(1,len/3), 14.8*ones(1,len/3), 15.2*ones(1,len/3)];
for n = 2:len
   vout(n) = (1-alpha) * vout(n-1) + alpha * vin(n);
end
figure(2)
plot(t_ms,vout)

%%
period_ms = 1;
t_const_ms = 2000;
alpha = 5*period_ms/t_const_ms;
t_ms = 0:period_ms:2*t_const_ms-1;
len = length(t_ms);
cout = zeros(1,len);
cin = ones(1,len);
for n = 2:len
   cout(n) = (1-alpha) * cout(n-1)  + alpha * cin(n);
end
figure(3)
plot(t_ms,cout)