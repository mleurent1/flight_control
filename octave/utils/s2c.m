function y = s2c(x,n)
	z = x;
	z(z<0) = z(z<0) + 2^n;
	y = z;
end