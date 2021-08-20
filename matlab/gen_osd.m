menu = [
   'P PITCH ROLL'
   'I PITCH ROLL'
   'D PITCH ROLL'
   'P YAW       '
   'I YAW       '
   'RATE        '
   'EXPO        '
   'MOTOR START '
   'MOTOR ARMED '
   'MOTOR RANGE '
   'I TRANSFER  '
   'VTX CHANNEL '
   'VTX POWER   '
   'SAVE REG    '
   'SAVE VTX    '
   'RUNCAM      '];

menu_str = [menu - 'A' + 11, zeros(size(menu,1),3), 255*ones(size(menu,1),1)];
menu_str(menu_str<0) = 0;

fprintf('const uint8_t menu_str[%d][%d] = {\n\t{',size(menu_str,1),size(menu_str,2));
for n = 1:size(menu_str,1)
   % fprintf('0x%02X, ',menu_str(n,:))
   fprintf('0x%02X, ',fliplr(menu_str(n,:)))
   if n == size(menu_str,1)
      fprintf('\b\b}  // %s\n};\n', strtrim(menu(n,:)))
   else
      fprintf('\b\b}, // %s\n\t{', strtrim(menu(n,:)))
   end
end

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
str = [vtx - 'A' + 11, 255];

fprintf('const uint8_t vtx_str[%d][%d] = {\n\t{', numel(vtx_freq),length(str));
for n = 1:6
   for m = 1:8
      vtx = sprintf('%s %d %4d', vtx_band(n,:), m, vtx_freq(n,m));
      str = [vtx - 'A' + 11, 255];
      str(str<0) = 0;
      fprintf('0x%02X, ',fliplr(str))
      if (m == 8) && (n == 6)
         fprintf('\b\b}  // %s\n};\n', vtx)
      else
         fprintf('\b\b}, // %s\n\t{', vtx)
      end
   end
end