function s = c2s(c,n)
	s = c;
	s(s>=2^(n-1)) = s(s>=2^(n-1)) - 2^n;
endfunction
