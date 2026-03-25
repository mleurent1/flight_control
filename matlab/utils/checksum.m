function c = checksum(data)

c = 0;
for n = 1:length(data)
	c = bitxor(c, data(n));
end

end