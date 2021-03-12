clear
define_max7456_reg

%% generate register C database

f = fopen('../../c/inc/max7456_reg.h','w');

for n = 1:length(reg)
	fprintf(f,'#define MAX7456_%s %d\n', reg(n).name, reg(n).addr);
	if length(reg(n).subf) > 1
		fprintf(f,'\n');
		for m = 1:length(reg(n).subf)
         if reg(n).subf{m}{3} == reg(n).subf{m}{2}
            fprintf(f,'#define MAX7456_%s__%s (1 << %d)\n', reg(n).name, reg(n).subf{m}{1}, reg(n).subf{m}{3});
         else
            mask = sum(2.^(0:(reg(n).subf{m}{2}-reg(n).subf{m}{3})));
            fprintf(f,'#define MAX7456_%s__%s(x) (((x) & 0x%02X) << %d)\n', reg(n).name, reg(n).subf{m}{1}, mask, reg(n).subf{m}{3});
         end
		end
		fprintf(f,'\n');
	end
end

fclose(f);

%% generate matlab register class

f = fopen('max7456_reg.m','w');

fprintf(f,'classdef max7456_reg\n');
fprintf(f,'\tmethods\n');
fprintf(f,'\t\tfunction data = read(obj,addr)\n');
fprintf(f,'\t\t\tglobal ser\n');
fprintf(f,'\t\t\tfwrite(ser,[2,128+addr,0]);\n');
% fprintf(f,'\t\t\tsleep(100);\n');
fprintf(f,'\t\t\ttic;\n');
fprintf(f,'\t\t\twhile (ser.BytesAvailable < 2) && (toc < 1)\n');
fprintf(f,'\t\t\t\tsleep(1);\n');
fprintf(f,'\t\t\tend\n');
fprintf(f,'\t\t\tif ~ser.BytesAvailable\n');
fprintf(f,'\t\t\t\terror(''Timeout'')\n');
fprintf(f,'\t\t\tend\n');
fprintf(f,'\t\t\tr = fread(ser,2);\n');
fprintf(f,'\t\t\tdata = r(2);\n');
fprintf(f,'\t\tend\n');
fprintf(f,'\t\tfunction write(obj,addr,data)\n');
fprintf(f,'\t\t\tglobal ser\n');
fprintf(f,'\t\t\tfwrite(ser,[length(data)+1,addr,data]);\n');
% fprintf(f,'\t\t\tsleep(100);\n');
fprintf(f,'\t\t\ttic;\n');
fprintf(f,'\t\t\twhile (ser.BytesAvailable < (length(data)+1)) && (toc < 1)\n');
fprintf(f,'\t\t\t\tsleep(1);\n');
fprintf(f,'\t\t\tend\n');
fprintf(f,'\t\t\tif ~ser.BytesAvailable\n');
fprintf(f,'\t\t\t\terror(''Timeout'')\n');
fprintf(f,'\t\t\tend\n');
fprintf(f,'\t\t\tfread(ser,length(data)+1);\n');
fprintf(f,'\t\tend\n');

for n = 1:length(reg)
	
   fprintf(f,'\t\tfunction y = %s(obj,x)\n', reg(n).name);
   fprintf(f,'\t\t\tif nargin < 2\n');
   fprintf(f,'\t\t\t\ty = obj.read(%d);\n', n-1);
   fprintf(f,'\t\t\telse\n');
   fprintf(f,'\t\t\t\tobj.write(%d, uint32(x));\n', n-1);
   fprintf(f,'\t\t\tend\n');
	fprintf(f,'\t\tend\n');
	
	if length(reg(n).subf) > 1
		for m = 1:length(reg(n).subf)
			mask = sum(2.^(reg(n).subf{m}{3}:reg(n).subf{m}{2}));
			fprintf(f,'\t\tfunction y = %s__%s(obj,x)\n', reg(n).name, reg(n).subf{m}{1});
			fprintf(f,'\t\t\tr = double(obj.read(%d));\n', n-1);
			fprintf(f,'\t\t\tif nargin < 2\n');
         fprintf(f,'\t\t\t\tz = bitshift(bitand(r, %d), %d);\n', mask, -reg(n).subf{m}{3});
         fprintf(f,'\t\t\t\ty = z(1);\n');
         fprintf(f,'\t\t\telse\n');
         fprintf(f,'\t\t\t\tw = bitand(bitshift(double(x), %d), %d) + bitand(r, %d);\n', reg(n).subf{m}{3}, mask, 2^32-1-mask);
			fprintf(f,'\t\t\t\tobj.write(%d, w);\n', n-1);
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