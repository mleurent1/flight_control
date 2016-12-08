function y = c2s(x,n)
	z = x;
	z(z>=2^(n-1)) = z(z>=2^(n-1)) - 2^n;
	y = z;
end