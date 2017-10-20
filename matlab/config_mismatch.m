function config_mismatch

global fc

reg = fieldnames(fc.info);

for n = 1:length(reg)
   i = eval(sprintf('fc.info.%s',reg{n}));
   if i(2) && (i(4) ~= 1)
      fc.target = 0;
      r1 = eval(sprintf('fc.%s',reg{n}));
      fc.target = 1;
      r2 = eval(sprintf('fc.%s',reg{n}));
      if r1 ~= r2
         if i(3)
            fprintf('%s = %f, %f (flash)\n', reg{n}, r1, r2);
         else
            fprintf('%s = %d, %d (flash)\n', reg{n}, r1, r2);
         end
      end
   end
end

fc.target = 0;
   
end