function read_config

global fc

reg = fieldnames(fc.info);

for n = 1:length(reg)
   i = eval(sprintf('fc.info.%s',reg{n}));
   if i(2) && (i(4) ~= 1)
      if i(3)
         fprintf('%s = %f\n', reg{n}, eval(sprintf('fc.%s',reg{n})));
      else
         fprintf('%s = %d\n', reg{n}, eval(sprintf('fc.%s',reg{n})));
      end
   end
end

end