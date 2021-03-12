clear

%% parse definitions

f = fopen('../../c/inc/mpu_reg.h','r');

n = 0;

l = fgetl(f);

while ischar(l)
	
	r = strtrim(l);

	if (length(r) >= 7) && strcmp(r(1:7),'#define')

		r = strtrim(r(8:end)); % Remove #define
		
      % Remove prefix MPU_
      if strcmp(r(1:4),'MPU_')
         r = strtrim(r(5:end));
      end
      
		i = strfind(r,'__'); % Subfied test
		
		if isempty(i)
			
			n = n + 1;
			
			i = strfind(r,' ');
			reg(n).name = strtrim(r(1:i-1));
			r = r(i+1:end);

			reg(n).addr = str2double(strtrim(r));
			reg(n).subf = {};
			reg(n).mask = [];

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
				reg(i).subf = {reg(i).subf{:}, subname};
				reg(i).mask = [reg(i).mask, mask];
			end
		end
	end
	
	l = fgetl(f);
end

fclose(f);

%% generate matlab register class

f = fopen('mpu_reg.m','w');

fprintf(f,'classdef mpu_reg\n');
fprintf(f,'\tmethods\n');
fprintf(f,'\t\tfunction data = read(obj,addr)\n');
fprintf(f,'\t\t\tglobal ser\n');
fprintf(f,'\t\t\tfwrite(ser,[2,addr]);\n');
fprintf(f,'\t\t\tdata = fread(ser,1);\n');
fprintf(f,'\t\tend\n');
fprintf(f,'\t\tfunction write(obj,addr,data)\n');
fprintf(f,'\t\t\tglobal ser\n');
fprintf(f,'\t\t\tfwrite(ser,[3,addr,0,0,0,data]);\n');
fprintf(f,'\t\tend\n');

for n = 1:length(reg)
	
	fprintf(f,'\t\tfunction y = %s(obj,x)\n', reg(n).name);
	fprintf(f,'\t\t\tif nargin < 2\n');
	fprintf(f,'\t\t\t\ty = obj.read(%d);\n', reg(n).addr);
	fprintf(f,'\t\t\telse\n');
	fprintf(f,'\t\t\t\tobj.write(%d,x);\n', reg(n).addr);
	fprintf(f,'\t\t\tend\n');
	fprintf(f,'\t\tend\n');
	
	if ~isempty(reg(n).subf)
		for m = 1:length(reg(n).subf)
			shift = find(fliplr(dec2bin(reg(n).mask(m),8))=='1',1,'first')-1;
			fprintf(f,'\t\tfunction y = %s__%s(obj,x)\n', reg(n).name, reg(n).subf{m});
			fprintf(f,'\t\t\tr = obj.read(%d);\n', reg(n).addr);
			fprintf(f,'\t\t\tif nargin < 2\n');
			fprintf(f,'\t\t\t\ty = bitshift(bitand(r, %d), %d);\n', reg(n).mask(m), -shift);
			fprintf(f,'\t\t\telse\n');
			fprintf(f,'\t\t\t\tw = bitand(bitshift(x, %d), %d) + bitand(r, %d);\n', shift, reg(n).mask(m), 2^8-1-reg(n).mask(m));
			fprintf(f,'\t\t\t\tobj.write(%d,w);\n', reg(n).addr);
			fprintf(f,'\t\t\tend\n');
			fprintf(f,'\t\tend\n');
		end
	end
end

fprintf(f,'\tend\n');
fprintf(f,'\tproperties\n');
for n = 1:length(reg)
	fprintf(f,'\t\t%s_addr = %d;\n', reg(n).name, reg(n).addr);
end
fprintf(f,'\tend\n');
fprintf(f,'end\n');

fclose(f);