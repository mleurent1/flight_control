clear all

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

%%
vacc = zeros(1,5000);
vacc(1) = 12.6/0.001;

for n = 2:length(vacc)
   
   vacc(n) = vacc(n-1)*0.999 + 11.2;
   
end

figure(2)
plot(vacc*0.001)