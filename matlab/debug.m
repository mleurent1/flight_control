function debug

global fc
global ser
global KEY_IS_PRESSED

t_max = 10;
status_period = 63e-3;
buf_len = 24*4;

KEY_IS_PRESSED = 0;

len = round(t_max/status_period);
t = (0:len-1)*status_period;

figure(1)
clf
set(gcf, 'KeyPressFcn', @myKeyPressFcn)

a = {};
l = {};

for n = 1:9
	a{n} = subplot(3,3,n);
end

set(a{1},'XLim',[t(1),t(end)],'YLabel',text('String','gyro (deg/s)'));
l{1,1} = line(t,nan(1,len),'Parent',a{1},'Color','b','DisplayName','pitch');
l{1,2} = line(t,nan(1,len),'Parent',a{1},'Color','r','DisplayName','roll');
l{1,3} = line(t,nan(1,len),'Parent',a{1},'Color','g','DisplayName','yaw');

set(a{2},'XLim',[t(1),t(end)],'YLabel',text('String','accel (G)'));
l{2,1} = line(t,nan(1,len),'Parent',a{2},'Color','b','DisplayName','pitch');
l{2,2} = line(t,nan(1,len),'Parent',a{2},'Color','r','DisplayName','roll');
l{2,3} = line(t,nan(1,len),'Parent',a{2},'Color','g','DisplayName','yaw');

set(a{3},'XLim',[t(1),t(end)],'YLabel',text('String','angle (deg)'),'YLim',[-95,95]);
l{3,1} = line(t,nan(1,len),'Parent',a{3},'Color','b','DisplayName','pitch');
l{3,2} = line(t,nan(1,len),'Parent',a{3},'Color','r','DisplayName','roll');

l{4,1} = bar(1:6,zeros(1,6),'Parent',a{4});
set(a{4},'XTickLabel',{'throttle','pitch','roll','yaw','arm','beep'},'YLim',[-1.05,1.05]);

set(a{5},'XLim',[t(1),t(end)],'YLabel',text('String','vbat (V)'));
l{5,1} = line(t,nan(1,len),'Parent',a{5},'Color','b');

set(a{6},'XLim',[t(1),t(end)],'YLabel',text('String','correction'),'YLim',[-605,605]);
l{6,1} = line(t,nan(1,len),'Parent',a{6},'Color','b','DisplayName','pitch');
l{6,2} = line(t,nan(1,len),'Parent',a{6},'Color','r','DisplayName','roll');
l{6,3} = line(t,nan(1,len),'Parent',a{6},'Color','g','DisplayName','yaw');

l{7,1} = bar(1:4,zeros(1,4),'Parent',a{7});
set(a{7},'XLabel',text('String','motor'),'YLim',[0,2010]);

set(a{8},'XLim',[t(1),t(end)],'YLabel',text('String','time sensor (us)'));%,'YLim',[-10,1010]);
l{8,1} = line(t,nan(1,len),'Parent',a{8},'Color','b','DisplayName','mean');
l{8,2} = line(t,nan(1,len),'Parent',a{8},'Color','r','DisplayName','max','LineStyle','--');
l{8,3} = line(t,nan(1,len),'Parent',a{8},'Color','r','DisplayName','min','LineStyle','--');

set(a{9},'XLim',[t(1),t(end)],'YLabel',text('String','time process (us)'));%,'YLim',[-10,1010]);
l{9,1} = line(t,nan(1,len),'Parent',a{9},'Color','b','DisplayName','mean');
l{9,2} = line(t,nan(1,len),'Parent',a{9},'Color','r','DisplayName','max','LineStyle','--');
l{9,3} = line(t,nan(1,len),'Parent',a{9},'Color','r','DisplayName','min','LineStyle','--');

for n = [1,2,3,6,8,9]
	set(a{n},'XLabel',text('String','time (s)'));
	%legend(a{n},'show','Location','NorthWest')
end
set(a{5},'XLabel',text('String','time (s)'));

sensor.gyro_x = nan(1,len);
sensor.gyro_y = nan(1,len);
sensor.gyro_z = nan(1,len);

sensor.accel_x = nan(1,len);
sensor.accel_y = nan(1,len);
sensor.accel_z = nan(1,len);

angle.pitch = nan(1,len);
angle.roll = nan(1,len);

radio.throttle = 0;
radio.pitch = 0;
radio.roll = 0;
radio.yaw = 0;
radio.aux = zeros(2,1);

vbat = nan(1,len);

pitch = nan(1,len);
roll = nan(1,len);
yaw = nan(1,len);

motor = zeros(4,1);

t_sensor = nan(3,len);
t_processing = nan(3,len);

while ser.BytesAvailable > 0
	fread(ser,ser.BytesAvailable);
end

fc.CTRL__DEBUG(1);

while ~KEY_IS_PRESSED
	
	if ser.BytesAvailable >= buf_len
		
		r = fread(ser,ser.BytesAvailable);
		r = reshape(r,buf_len,[]);
		d = size(r,2);
		len1 = len - d + 1;
		
		sensor.gyro_x(1:len-d) = sensor.gyro_x(d+1:len);
		sensor.gyro_y(1:len-d) = sensor.gyro_y(d+1:len);
		sensor.gyro_z(1:len-d) = sensor.gyro_z(d+1:len);
		sensor.accel_x(1:len-d) = sensor.accel_x(d+1:len);
		sensor.accel_y(1:len-d) = sensor.accel_y(d+1:len);
		sensor.accel_z(1:len-d) = sensor.accel_z(d+1:len);

		angle.pitch(1:len-d) = angle.pitch(d+1:len);
		angle.roll(1:len-d) = angle.roll(d+1:len);

		vbat(1:len-d) = vbat(d+1:len);

		pitch(1:len-d) = pitch(d+1:len);
		roll(1:len-d) = roll(d+1:len);
		yaw(1:len-d) = yaw(d+1:len);
		
		for m = 1:3
			t_sensor(m,1:len-d) = t_sensor(m,d+1:len);
			t_processing(m,1:len-d) = t_processing(m,d+1:len);
		end

		n = 0;

		sensor.gyro_x(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		sensor.gyro_y(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		sensor.gyro_z(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;

		sensor.accel_x(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		sensor.accel_y(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		sensor.accel_z(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;

		angle.pitch(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		angle.roll(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;

		radio.throttle = typecast(uint32(2^24*r(n+4,end) + 2^16*r(n+3,end) + 2^8*r(n+2,end) + r(n+1,end)), 'single');
		n = n + 4;
		radio.pitch = typecast(uint32(2^24*r(n+4,end) + 2^16*r(n+3,end) + 2^8*r(n+2,end) + r(n+1,end)), 'single');
		n = n + 4;
		radio.roll = typecast(uint32(2^24*r(n+4,end) + 2^16*r(n+3,end) + 2^8*r(n+2,end) + r(n+1,end)), 'single');
		n = n + 4;
		radio.yaw = typecast(uint32(2^24*r(n+4,end) + 2^16*r(n+3,end) + 2^8*r(n+2,end) + r(n+1,end)), 'single');
		n = n + 4;
		for m = 1:2
			radio.aux(m) = typecast(uint32(2^24*r(n+4,end) + 2^16*r(n+3,end) + 2^8*r(n+2,end) + r(n+1,end)), 'single');
			n = n + 4;
		end

		vbat(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;

		pitch(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		roll(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		yaw(len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;

		for m = 1:4
			motor(m) = 2^8*r(n+2,end) + r(n+1,end);
			n = n + 2;
		end

		t_sensor(1,len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		for m = 2:3
			t_sensor(m,len1:len) = 2^8*r(n+2,:) + r(n+1,:);
			n = n + 2;
		end
		t_processing(1,len1:len) = typecast(uint32(2^24*r(n+4,:) + 2^16*r(n+3,:) + 2^8*r(n+2,:) + r(n+1,:)), 'single');
		n = n + 4;
		for m = 2:3
			t_processing(m,len1:len) = 2^8*r(n+2,:) + r(n+1,:);
			n = n + 2;
		end

		set(l{1,1},'YData',sensor.gyro_x);
		set(l{1,2},'YData',sensor.gyro_y);
		set(l{1,3},'YData',sensor.gyro_z);
		
		set(l{2,1},'YData',sensor.accel_x);
		set(l{2,2},'YData',sensor.accel_y);
		set(l{2,3},'YData',sensor.accel_z);
		
		set(l{3,1},'YData',angle.pitch);
		set(l{3,2},'YData',angle.roll);

		set(l{4,1},'YData',[radio.throttle, radio.pitch, radio.roll, radio.yaw, radio.aux(1), radio.aux(2)]);

		set(l{5,1},'YData',vbat);
		
		set(l{6,1},'YData',pitch);
		set(l{6,2},'YData',roll);
		set(l{6,3},'YData',yaw);
		
		set(l{7,1},'YData',motor);
		
		for m = 1:3
			set(l{8,m},'YData',t_sensor(m,:));
			set(l{9,m},'YData',t_processing(m,:));
		end
		
		drawnow
		
	end
	
end

fc.CTRL__DEBUG(0);
pause(0.5)

while ser.BytesAvailable > 0
	fread(ser,ser.BytesAvailable);
end

end

function myKeyPressFcn(hObject, event)
	global KEY_IS_PRESSED
	KEY_IS_PRESSED  = 1;
end
