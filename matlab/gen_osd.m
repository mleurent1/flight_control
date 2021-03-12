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
   'SAVE        '
   'RUNCAM      '];

menu_str = [menu - 'A' + 11, 255*ones(size(menu,1),1)];
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
