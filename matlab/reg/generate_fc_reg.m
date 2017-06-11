%% generate register C database

f = fopen('../../sw/inc/fc_reg.h','w');

fprintf(f,'#include <stdint.h>\n\n');

fprintf(f,'#define NB_REG %d\n\n', length(reg));

for n = 1:length(reg)
	if length(reg(n).subf) == 1
		if strcmp(reg(n).subf{1}{4}, 'float')
			fprintf(f,'#define REG_%s regf[%d]\n', reg(n).subf{1}{1}, n-1);
		else
			fprintf(f,'#define REG_%s reg[%d]\n', reg(n).subf{1}{1}, n-1);
		end
	else
		fprintf(f,'#define REG_%s reg[%d]\n', reg(n).name, n-1);
		for m = 1:length(reg(n).subf)
			mask = sum(2.^(reg(n).subf{m}{3}:reg(n).subf{m}{2}));
			fprintf(f,'#define REG_%s__%s (%s_t)((reg[%d] & %dU) >> %d)\n', reg(n).name, reg(n).subf{m}{1}, reg(n).subf{m}{4}, n-1, mask, reg(n).subf{m}{3});
		end
	end
end

fprintf(f,'\n');

fprintf(f,'typedef struct\n{\n');
fprintf(f,'\t_Bool read_only;\n');
fprintf(f,'\t_Bool flash;\n');
fprintf(f,'\t_Bool is_float;\n');
fprintf(f,'\tuint32_t dflt;\n');
fprintf(f,'} reg_properties_t;\n\n');

fprintf(f,'extern uint32_t reg[NB_REG];\n');
fprintf(f,'extern float regf[NB_REG];\n');
fprintf(f,'extern reg_properties_t reg_properties[NB_REG];\n');

fclose(f);

f = fopen('../../sw/src/fc_reg.c','w');

fprintf(f,'#include "fc_reg.h"\n\n');

fprintf(f,'uint32_t reg[NB_REG];\n');
fprintf(f,'float regf[NB_REG];\n\n');

fprintf(f,'reg_properties_t reg_properties[NB_REG] = \n{\n');

% bools = {'false','true'};
bools = {'0','1'};

for n = 1:length(reg)
	if length(reg(n).subf) == 1
		if strcmp(reg(n).subf{1}{4}, 'float')
			default = typecast(single(reg(n).subf{1}{5}), 'uint32');
			float = 1;
		else
			default = reg(n).subf{1}{5};
			float = 0;
		end
	else
		default = 0;
		float = 0;
		for m = 1:length(reg(n).subf)
			default = default + bitshift(reg(n).subf{m}{5}, reg(n).subf{m}{3});
		end
	end
	if n < length(reg)
		fprintf(f,'\t{%s, %s, %s, %d}, // %s\n', bools{reg(n).read_only+1}, bools{reg(n).flash+1}, bools{float+1}, default, reg(n).name);
	else
		fprintf(f,'\t{%s, %s, %s, %d} // %s\n};\n', bools{reg(n).read_only+1}, bools{reg(n).flash+1}, bools{float+1}, default, reg(n).name);
	end
end

fclose(f);

%% generate matlab register class

f = fopen('fc_reg.m','w');

fprintf(f,'classdef fc_reg\n');
fprintf(f,'\tmethods\n');
fprintf(f,'\t\tfunction data = read(obj,addr)\n');
fprintf(f,'\t\t\tglobal ser\n');
fprintf(f,'\t\t\tfwrite(ser,[0,addr,0,0,0,0]);\n');
fprintf(f,'\t\t\tdata = sum(fread(ser,4) .* 2.^(0:8:24)'');\n');
fprintf(f,'\t\tend\n');
fprintf(f,'\t\tfunction write(obj,addr,data)\n');
fprintf(f,'\t\t\tglobal ser\n');
fprintf(f,'\t\t\tfwrite(ser,[1,addr,floor(mod(double(data) ./ 2.^(0:8:24),2^8))]);\n');
fprintf(f,'\t\tend\n');

for n = 1:length(reg)
	
	fprintf(f,'\t\tfunction y = %s(obj,x)\n', reg(n).name);
	fprintf(f,'\t\t\tif nargin < 2\n');
	if strcmp(reg(n).subf{1}{4}, 'float')
		fprintf(f,'\t\t\t\tz = obj.read(%d);\n', n-1);
		fprintf(f,'\t\t\t\ty = typecast(uint32(z), ''single'');\n');
	else
		fprintf(f,'\t\t\t\ty = obj.read(%d);\n', n-1);
	end
	fprintf(f,'\t\t\telse\n');
	if strcmp(reg(n).subf{1}{4}, 'float')
		fprintf(f,'\t\t\t\tz = typecast(single(x), ''uint32'');\n');
		fprintf(f,'\t\t\t\tobj.write(%d,z);\n', n-1);
	else
		fprintf(f,'\t\t\t\tobj.write(%d,x);\n', n-1);
	end
	fprintf(f,'\t\t\tend\n');
	fprintf(f,'\t\tend\n');
	
	if length(reg(n).subf) > 1
		for m = 1:length(reg(n).subf)
			mask = sum(2.^(reg(n).subf{m}{3}:reg(n).subf{m}{2}));
			fprintf(f,'\t\tfunction y = %s__%s(obj,x)\n', reg(n).name, reg(n).subf{m}{1});
			fprintf(f,'\t\t\tr = obj.read(%d);\n', n-1);
			fprintf(f,'\t\t\tif nargin < 2\n');
			fprintf(f,'\t\t\t\ty = bitshift(bitand(r, %d), %d);\n', mask, -reg(n).subf{m}{3});
			fprintf(f,'\t\t\telse\n');
			fprintf(f,'\t\t\t\tw = bitand(bitshift(x, %d), %d) + bitand(r, %d);\n', reg(n).subf{m}{3}, mask, 2^32-1-mask);
			fprintf(f,'\t\t\t\tobj.write(%d,w);\n', n-1);
			fprintf(f,'\t\t\tend\n');
			fprintf(f,'\t\tend\n');
		end
	end
end

fprintf(f,'\tend\n');
fprintf(f,'\tproperties\n');
for n = 1:length(reg)
	fprintf(f,'\t\t%s_addr = %d;\n', reg(n).name, n-1);
end
flash_list = find([reg.flash])-1;
fprintf(f,'\t\tflash_addr_list = %s;\n', mat2str(flash_list));
fprintf(f,'\t\tflash_float_list = [');
for n = 1:length(flash_list)
	if n == length(flash_list)
		fprintf(f,'%d];\n',strcmp(reg(flash_list(n)+1).subf{1}{4},'float'));
	else
		fprintf(f,'%d,',strcmp(reg(flash_list(n)+1).subf{1}{4},'float'));
	end
end
fprintf(f,'\t\tflash_name_list = {');
for n = 1:length(flash_list)
	if n == length(flash_list)
		fprintf(f,'''%s''};\n',reg(flash_list(n)+1).name);
	else
		fprintf(f,'''%s'',',reg(flash_list(n)+1).name);
	end
end

fprintf(f,'\tend\n');
fprintf(f,'end\n');

fclose(f);