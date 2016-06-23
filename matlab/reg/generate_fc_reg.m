clear all

%% parse definitions

f = fopen('define_fc_reg.h','r');

n = 0;

l = fgetl(f);

while ischar(l)
	
	r = strtrim(l);

	if (length(r) >= 7) && strcmp(r(1:7),'#define')

		r = strtrim(r(8:end)); % Remove #define
		
		i = strfind(r,'__'); % Subfied test
		
		if isempty(i)
			
			n = n + 1;
			
			i = strfind(r,' ');
			reg(n).name = strtrim(r(1:i-1));
			r = r(i+1:end);

			reg(n).addr = n-1;
			reg(n).reset = str2double(strtrim(r));
			reg(n).description = [];
			reg(n).subfields = {};
			reg(n).mask = [];
			reg(n).subdescription = {};

		else
			
			name = strtrim(r(1:i-1));
			r = r(i+2:end);
			
			i = strfind(r,'(x)');
			
			if i
				
				subname = strtrim(r(1:i-1));
				r = r(i+3:end);
				
				i = strfind(r,'((x) & 0x');
				r = r(i+9:end);
				
				i = strfind(r,')');
				mask = hex2dec(strtrim(r(1:i-1)));
				r = r(i+1:end);

				i = strfind(r,'<<');
				r = r(i+2:end);

				i = strfind(r,')');
				expo = str2double(strtrim(r(1:i-1)));
				mask = mask * 2^expo;
				r = r(i+1:end);
							
			else
				
				i = strfind(r,'(1 <<');
				
				if i
					
					subname = strtrim(r(1:i-1));
					r = r(i+5:end);
					
					i = strfind(r,')');
					expo = str2double(strtrim(r(1:i-1)));
					mask = 2^expo;
					r = r(i+1:end);
					
				end
			end
			
			if ~isempty(name)
				reg_list = {reg(:).name};
				i = find(ismember(reg_list,name));
				reg(i).subfields = {reg(i).subfields{:}, subname};
				reg(i).mask = [reg(i).mask, mask];
			end
		end
	end
	
	l = fgetl(f);
end

fclose(f);

%% generate matlab register class and C database

f1 = fopen('fc_reg.m','w');
f2 = fopen('fc_reg.h','w');

fprintf(f1,'classdef fc_reg\n');
fprintf(f1,'	methods\n');
fprintf(f1,'		function data = read(obj,addr)\n');
fprintf(f1,'			%% WRAP YOUR READ FUNCTION HERE\n');
fprintf(f1,'		end\n');
fprintf(f1,'		function write(obj,addr,data)\n');
fprintf(f1,'			%% WRAP YOUR WRITE FUNCTION HERE\n');
fprintf(f1,'		end\n');

fprintf(f2,'#define REG_NB_ADDR %d\n\n', max([reg.addr])+1);
fprintf(f2,'volatile uint32_t reg[REG_NB_ADDR];\n\n');

for n = 1:length(reg)
	
	name = reg(n).name;
	addr = reg(n).addr;
	mask = reg(n).mask;
	subf = reg(n).subfields;
	
	fprintf(f1,'		function y = %s(obj,x)\n', name);
	fprintf(f1,'			if nargin < 2\n');
	fprintf(f1,'				y = obj.read(%d);\n', addr);
	fprintf(f1,'			else\n');
	fprintf(f1,'				obj.write(%d,x);\n', addr);
	fprintf(f1,'			end\n');
	fprintf(f1,'		end\n');
	
	fprintf(f2,'#define REG_%s reg[%d]\n', name, addr);
	
	if ~isempty(subf)
		for m = 1:length(subf)
			shift = find(fliplr(dec2bin(mask(m),8))=='1',1,'first')-1;
			fprintf(f1,'		function y = %s__%s(obj,x)\n', name, subf{m});
			fprintf(f1,'			r = obj.read(%d);\n', addr);
			fprintf(f1,'			if nargin <= 1\n');
			fprintf(f1,'				y = bitshift(bitand(r, %d), %d);\n', mask(m), -shift);
			fprintf(f1,'			else\n');
			fprintf(f1,'				w = bitand(bitshift(x, %d), %d) + bitand(r, %d);\n', shift, mask(m), (2^32-1)-mask(m));
			fprintf(f1,'				obj.write(%d,w);\n', addr);
			fprintf(f1,'			end\n');
			fprintf(f1,'		end\n');
			
			fprintf(f2,'#define REG_%s__%s ((reg[%d] & %dU) >> %d)\n', name, subf{m}, addr, mask(m), shift);
		end
	end
end

fprintf(f2,'\nconst uint32_t reg_init[REG_NB_ADDR] = \n{\n');

for n = 1:length(reg)
	
	name = reg(n).name;
	rset = reg(n).reset;
	
	if n < length(reg)
		fprintf(f2,'\t%d, // %s\n', rset, name);
	else
		fprintf(f2,'\t%d // %s\n', rset, name);
	end
end

fprintf(f1,'	end\n');
fprintf(f1,'	properties\n');
for n = 1:length(reg)
	fprintf(f1,'		%s_addr = %d;\n', reg(n).name, reg(n).addr);
end
fprintf(f1,'	end\n');
fprintf(f1,'end\n');

fprintf(f2,'};\n');

fclose(f1);
fclose(f2);

