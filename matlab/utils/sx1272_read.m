function data = sx1272_read(addr,nbbyte)

if nargin == 1
    nbbyte = 1;
end

if ischar(addr)
    r=strfind(addr,'x'); % removes any 0x in front of address
    if ~isempty(r)
        addr = addr(r+1:end);
    end
    addrn = hex2dec(addr);
else
    addrn = addr;
end

r = ftdi('SPI',[mod(addrn,2^7), zeros(1,nbbyte)]);
data = r(2:end);

end
