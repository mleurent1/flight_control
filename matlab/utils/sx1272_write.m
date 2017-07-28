function sx1272_write(addr,data)

if ischar(addr)
    r=strfind(addr,'x'); % removes any 0x in front of address
    if ~isempty(r)
        addr = addr(r+1:end);
    end
    addrn = hex2dec(addr);
else
    addrn = addr;
end

if ischar(data)
    r=strfind(data,'x'); % removes any 0x in front of data
    if ~isempty(r)
        data = data(r+1:end);
    end
    datan = hex2dec(data);
else
    datan = data(:).';
end

ftdi('SPI',[2^7 + mod(addrn,2^7), datan]);

end
