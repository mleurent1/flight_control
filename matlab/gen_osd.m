% telem = '00.00V 00:00 -000dBm -00dB';
telem = '00.00V 000.0A 0000mAh 00:00  -000dBm -00dB';

telem_str = [osd_encode(telem), 255];

fprintf('uint16_t telem_str[%d] = {',length(telem_str));
fprintf('%2d, ',telem_str)
% fprintf('%2d, ',fliplr(telem_str))
fprintf('\b\b}; // %s\n', strtrim(telem))

%%
menu = [
   'P PITCH ROLL   '
   'I PITCH ROLL   '
   'D PITCH ROLL   '
   'P YAW          '
   'I YAW          '
   'RATE           '
   'EXPO           '
   'MOTOR START    '
   'MOTOR ARMED    '
   'MOTOR RANGE    '
   'VTX CHANNEL    '
   'VTX POWER      '
   'SAVE REG       '
   'SAVE VTX       '
   'RUNCAM         '];

menu_str = [osd_encode(menu), 255*ones(size(menu,1),1)];

fprintf('uint16_t menu_str[%d][%d] = {\n\t{',size(menu_str,1),size(menu_str,2));
for n = 1:size(menu_str,1)
   fprintf('%2d, ',menu_str(n,:))
   % fprintf('0x%2d, ',fliplr(menu_str(n,:)))
   if n == size(menu_str,1)
      fprintf('\b\b}  // %s\n};\n', strtrim(menu(n,:)))
   else
      fprintf('\b\b}, // %s\n\t{', strtrim(menu(n,:)))
   end
end

%%
vtx_band = [
   'BAND A  '
   'BAND B  '
   'BAND E  '
   'AIRWAVE '
   'RACE    '
   'LOW RACE'];
vtx_freq = [
   5865 5845 5825 5805 5785 5765 5745 5725
   5733 5752 5771 5790 5809 5828 5847 5866
   5705 5685 5665 5645 5885 5905 5925 5945
   5740 5760 5780 5800 5820 5840 5860 5880
   5658 5695 5732 5769 5806 5843 5880 5917
   5621 5584 5547 5510 5473 5436 5399 5362];

vtx = sprintf('%s %d %4d', vtx_band(1,:), 0, vtx_freq(1,1));

fprintf('uint16_t vtx_str[%d][%d] = {\n\t{', numel(vtx_freq),length(vtx)+1);
for n = 1:6
   for m = 1:8
      vtx = sprintf('%s %d %4d', vtx_band(n,:), m, vtx_freq(n,m));
		vtx_str = [osd_encode(vtx), 255];
		fprintf('%2d, ',vtx_str)
      % fprintf('%2d, ',fliplr(vtx_str))
      if (m == 8) && (n == 6)
         fprintf('\b\b}  // %s\n};\n', vtx)
      else
         fprintf('\b\b}, // %s\n\t{', vtx)
      end
   end
end

%%
saved = 'SAVED          ';

saved_str = [osd_encode(saved), 255];

fprintf('uint16_t saved_str[%d] = {',length(saved_str));
fprintf('%2d, ',saved_str)
% fprintf('%2d, ',fliplr(saved_str))
fprintf('\b\b}; // %s\n', strtrim(saved))

%%
reg_val = '0000.000       ';

reg_val_str = [osd_encode(reg_val), 255];

fprintf('uint16_t reg_val_str[%d] = {',length(reg_val_str));
fprintf('%2d, ',reg_val_str)
% fprintf('%2d, ',fliplr(reg_val_str))
fprintf('\b\b}; // %s\n', strtrim(reg_val))
