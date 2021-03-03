clear
define_fc_reg

%% generate register C database

f = fopen('reg.h','w');

fprintf(f,'#define NB_REG %d\n\n', length(reg));

for n = 1:length(reg)
	if length(reg(n).subf) == 1
		if strcmp(reg(n).subf{1}{4}, 'single')
			fprintf(f,'#define REG_%s regf[%d]\n', reg(n).subf{1}{1}, n-1);
		else
			fprintf(f,'#define REG_%s reg[%d]\n', reg(n).subf{1}{1}, n-1);
		end
	else
		fprintf(f,'#define REG_%s reg[%d]\n', reg(n).name, n-1);
		for m = 1:length(reg(n).subf)
			mask = sum(2.^(reg(n).subf{m}{3}:reg(n).subf{m}{2}));
			fprintf(f,'#define REG_%s__%s (%s_t)((reg[%d] & %dU) >> %d)\n', reg(n).name, reg(n).subf{m}{1}, reg(n).subf{m}{4}, n-1, mask, reg(n).subf{m}{3});
			fprintf(f,'#define REG_%s__%s_Msk %dU\n', reg(n).name, reg(n).subf{m}{1}, mask);
			fprintf(f,'#define REG_%s__%s_Pos %dU\n', reg(n).name, reg(n).subf{m}{1}, reg(n).subf{m}{3});
		end
	end
	fprintf(f,'#define REG_%s_Addr %d\n', reg(n).name, n-1);
end

fclose(f);

f = fopen('reg.c','w');

fprintf(f,'reg_properties_t reg_properties[NB_REG] = \n{\n');

% bools = {'false','true'};
bools = {'0','1'};

for n = 1:length(reg)
	if length(reg(n).subf) == 1
      switch reg(n).subf{1}{4}
         case 'single'
            default = typecast(single(reg(n).subf{1}{5}), 'uint32');
            float = 1;   
         case {'int8','int16','int32'}
            default = typecast(int32(reg(n).subf{1}{5}), 'uint32');
            float = 0;
         otherwise
            default = reg(n).subf{1}{5};
            float = 0;
      end
	else
		default = 0;
		float = 0;
		for m = 1:length(reg(n).subf)
         switch reg(n).subf{1}{4}
            case {'int8','int16','int32'}
               default = default + bitshift(typecast(int32(reg(n).subf{m}{5}),'uint32'), reg(n).subf{m}{3});
            otherwise
               default = default + bitshift(reg(n).subf{m}{5}, reg(n).subf{m}{3});
         end
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
fprintf(f,'\t\t\tif obj.method\n');
fprintf(f,'\t\t\t\tr = sx1272_receive(0);\n');
fprintf(f,'\t\t\t\tsx1272_send([0,addr],1);\n');
fprintf(f,'\t\t\t\tsleep(300);\n');
fprintf(f,'\t\t\t\tr = sx1272_receive(0);\n');
fprintf(f,'\t\t\t\tdata = uint32(sum(r.payload(3:6) .* 2.^(0:8:24)));\n');
fprintf(f,'\t\t\telse\n');
fprintf(f,'\t\t\t\tfwrite(ser,[obj.target*4+0,addr]);\n');
fprintf(f,'\t\t\t\tdata = uint32(sum(fread(ser,4) .* 2.^(0:8:24)''));\n');
fprintf(f,'\t\t\tend\n');
fprintf(f,'\t\tend\n');
fprintf(f,'\t\tfunction write(obj,addr,data)\n');
fprintf(f,'\t\t\tglobal ser\n');
fprintf(f,'\t\t\tif obj.method\n');
fprintf(f,'\t\t\t\tsx1272_send([1,addr,floor(mod(double(data) ./ 2.^(0:8:24),2^8))],1);\n');
fprintf(f,'\t\t\telse\n');
fprintf(f,'\t\t\t\tfwrite(ser,[obj.target*4+1,addr,floor(mod(double(data) ./ 2.^(0:8:24),2^8))]);\n');
fprintf(f,'\t\t\tend\n');
fprintf(f,'\t\tend\n');

for n = 1:length(reg)
	
   if length(reg(n).subf) == 1
      type1 = reg(n).subf{1}{4};
   else
      type1 = 'uint32';
   end
   
   fprintf(f,'\t\tfunction y = %s(obj,x)\n', reg(n).name);
   fprintf(f,'\t\t\tif nargin < 2\n');
   switch type1
      case 'single'
         fprintf(f,'\t\t\t\ty = typecast(obj.read(%d), ''single'');\n', n-1);
      case {'int8','int16','int32'}
         fprintf(f,'\t\t\t\ty = typecast(obj.read(%d), ''int32'');\n', n-1);
      otherwise
         fprintf(f,'\t\t\t\ty = obj.read(%d);\n', n-1);
   end
   fprintf(f,'\t\t\telse\n');
   switch type1
      case 'single'
         fprintf(f,'\t\t\t\tobj.write(%d, typecast(single(x), ''uint32''));\n', n-1);
      case {'int8','int16','int32'}
         fprintf(f,'\t\t\t\tobj.write(%d, typecast(int32(x), ''uint32''));\n', n-1);
      otherwise
         fprintf(f,'\t\t\t\tobj.write(%d, uint32(x));\n', n-1);
   end
   fprintf(f,'\t\t\tend\n');
   
	fprintf(f,'\t\tend\n');
	
	if length(reg(n).subf) > 1
		for m = 1:length(reg(n).subf)
			mask = sum(2.^(reg(n).subf{m}{3}:reg(n).subf{m}{2}));
			fprintf(f,'\t\tfunction y = %s__%s(obj,x)\n', reg(n).name, reg(n).subf{m}{1});
			fprintf(f,'\t\t\tr = double(obj.read(%d));\n', n-1);
			fprintf(f,'\t\t\tif nargin < 2\n');
         fprintf(f,'\t\t\t\tz = typecast(uint32(bitshift(bitand(r, %d), %d)),''%s'');\n', mask, -reg(n).subf{m}{3}, reg(n).subf{m}{4});
         fprintf(f,'\t\t\t\ty = z(1);\n');
         fprintf(f,'\t\t\telse\n');
         switch reg(n).subf{m}{4}
            case {'int8','int16'}
               fprintf(f,'\t\t\t\tw = bitand(bitshift(double(typecast(int32(x),''uint32'')), %d), %d) + bitand(r, %d);\n', reg(n).subf{m}{3}, mask, 2^32-1-mask);
            otherwise
               fprintf(f,'\t\t\t\tw = bitand(bitshift(double(x), %d), %d) + bitand(r, %d);\n', reg(n).subf{m}{3}, mask, 2^32-1-mask);
         end
			fprintf(f,'\t\t\t\tobj.write(%d, uint32(w));\n', n-1);
			fprintf(f,'\t\t\tend\n');
			fprintf(f,'\t\tend\n');
		end
	end
end

fprintf(f,'\tend\n');
fprintf(f,'\tproperties\n');
fprintf(f,'\t\tmethod = 0;\n');
fprintf(f,'\t\ttarget = 0;\n');

fprintf(f,'\t\tinfo = struct(...\n');
for n = 1:length(reg)
   fprintf(f,'\t\t\t''%s'', [%d,%d,%d,%d],...\n', reg(n).name, n-1, reg(n).flash, strcmp(reg(n).subf{1}{4},'single'), length(reg(n).subf) > 1);
   if length(reg(n).subf) > 1
      for m = 1:length(reg(n).subf)
			mask = sum(2.^(reg(n).subf{m}{3}:reg(n).subf{m}{2}));
			fprintf(f,'\t\t\t''%s__%s'', [%d,%d,%d,2],...\n', reg(n).name, reg(n).subf{m}{1}, n-1, reg(n).flash, strcmp(reg(n).subf{m}{4},'single'));
      end
   end
end
fseek(f,-5,'cof');
fprintf(f,' );\n');

fprintf(f,'\tend\n');
fprintf(f,'end\n');

fclose(f);