function save_config

global fc
global ser

fwrite(ser,[6,0,0,0,0,0]);
sleep(3000)

reg = fieldnames(fc.info);

for n = 1:length(reg)
   i = eval(sprintf('fc.info.%s',reg{n}));
   if i(2) && (i(4) ~= 2)
      fc.target = 0;
      r = eval(sprintf('fc.%s',reg{n}));
      fc.target = 1;
      if i(3)
         eval(sprintf('fc.%s(%f);',reg{n},r));
      else
         eval(sprintf('fc.%s(%d);',reg{n},r));
      end
   end
end

fc.target = 0;

end