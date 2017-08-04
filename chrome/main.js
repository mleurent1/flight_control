var timeout = 0;
var timeoutIncr = 15;
var conHandle = [];
var conStatus = 0;
var txBuf = new ArrayBuffer(6);
var txBufUint8 = new Uint8Array(txBuf);
var rxBuf = new ArrayBuffer(4);
var rxBufUint8 = new Uint8Array(rxBuf);
var rxBufUint32 = new Uint32Array(rxBuf);
var rxBufFloat32 = new Float32Array(rxBuf);
var precisionFloat = 1e6;
var motorTestOn = false;
var currentPlot = 0;
var canvas;
var ctx;
var histLen = 0;
var sensorBuf = new ArrayBuffer(7*4);
var sensorBufPrev = new ArrayBuffer(7*4);
var sensorBufFloat32 = new Float32Array(sensorBuf);
var sensorBufPrevFloat32 = new Float32Array(sensorBufPrev);
var receiverBuf = new ArrayBuffer(8*4);
var receiverBufPrev = new ArrayBuffer(8*4);
var receiverBufFloat32 = new Float32Array(receiverBuf);
var receiverBufPrevFloat32 = new Float32Array(receiverBufPrev);
var motorBuf = new ArrayBuffer(4*4);
var motorBufPrev = new ArrayBuffer(4*4);
var motorBufInt32 = new Int32Array(motorBuf);
var motorBufPrevInt32 = new Int32Array(motorBufPrev);
var vbatBuf = new ArrayBuffer(1*4);
var vbatBufPrev = new ArrayBuffer(1*4);
var vbatBufFloat32 = new Float32Array(vbatBuf);
var vbatBufPrevFloat32 = new Float32Array(vbatBufPrev);
var axisWidth;
var axisHeight;

onload = function(){
	document.getElementById('readcfg').onclick = readConfig;
	document.getElementById('writecfg').onclick = writeConfig;
	
	document.getElementById('vbatmin').onchange = function(){ regFloat32Write(7,parseFloat(this.value)); log('Updated VBAT Min'); };
	document.getElementById('radiofilt').onchange = function(){ regFloat32Write(8,parseFloat(this.value)); log('Updated Radio Filter Alpha'); };
	document.getElementById('expopr').onchange = function(){ regFloat32Write(9,parseFloat(this.value)); log('Updated Expo of Pitch & Roll'); };
	document.getElementById('expoy').onchange = function(){ regFloat32Write(10,parseFloat(this.value)); log('Updated Expo of Yaw'); };
	document.getElementById('motorstart').onchange = function(){ regUint32Write(11,parseInt(this.value)); log('Updated Motor Start'); };
	document.getElementById('motorarmed').onchange = function(){ regUint32Write(12,parseInt(this.value)); log('Updated Motor Armed'); };
	document.getElementById('ratepr').onchange = function(){ regFloat32Write(13,parseFloat(this.value)); log('Updated Rate of Pitch & Roll'); };
	document.getElementById('ratey').onchange = function(){ regFloat32Write(14,parseFloat(this.value)); log('Updated Rate of Yaw'); };
	document.getElementById('throttle').onchange = function(){ regFloat32Write(15,parseFloat(this.value)); log('Updated Throttle Range'); };
	document.getElementById('pitchp').onchange = function(){ regFloat32Write(16,parseFloat(this.value)); log('Updated Pitch P'); };
	document.getElementById('pitchi').onchange = function(){ regFloat32Write(17,parseFloat(this.value)); log('Updated Pitch I'); };
	document.getElementById('pitchd').onchange = function(){ regFloat32Write(18,parseFloat(this.value)); log('Updated Pitch D'); };
	document.getElementById('rollp').onchange = function(){ regFloat32Write(19,parseFloat(this.value)); log('Updated Roll P'); };
	document.getElementById('rolli').onchange = function(){ regFloat32Write(20,parseFloat(this.value)); log('Updated Roll I'); };
	document.getElementById('rolld').onchange = function(){ regFloat32Write(21,parseFloat(this.value)); log('Updated Roll D'); };
	document.getElementById('yawp').onchange = function(){ regFloat32Write(22,parseFloat(this.value)); log('Updated Yaw P'); };
	document.getElementById('yawi').onchange = function(){ regFloat32Write(23,parseFloat(this.value)); log('Updated Yaw I'); };
	document.getElementById('yawd').onchange = function(){ regFloat32Write(24,parseFloat(this.value)); log('Updated Yaw D'); };
	document.getElementById('gyrofilt').onchange = function(){ regUint32Write(25,parseInt(this.value)); log('Updated Gyro/Accel Filter'); };
	document.getElementById('motorsel1').onclick = motorTest;
	document.getElementById('motorsel2').onclick = motorTest;
	document.getElementById('motorsel3').onclick = motorTest;
	document.getElementById('motorsel4').onclick = motorTest;
	document.getElementById('motortest').value = 0;
	document.getElementById('motortest').onchange = motorTest;
	
	chrome.serial.getDevices(openPort);
	
	document.getElementById('plotsel').innerHTML = '';
	var sel = document.createElement('select');
	sel.id = 'plotselin';
	var opt = document.createElement('option');
	opt.value = 'Gyros';
	opt.innerHTML = 'Gyros';
	sel.appendChild(opt);
	opt = document.createElement('option');
	opt.value = 'Receiver';
	opt.innerHTML = 'Receiver';
	sel.appendChild(opt);
	opt = document.createElement('option');
	opt.value = 'Motors';
	opt.innerHTML = 'Motors';
	sel.appendChild(opt);
	opt = document.createElement('option');
	opt.value = 'Vbat';
	opt.innerHTML = 'Vbat';
	sel.appendChild(opt);
	document.getElementById('plotsel').appendChild(sel);
	document.getElementById('plotstart').value = 'Plot';
	document.getElementById('plotstart').onclick = startPlot;
	
	canvas = document.getElementById("plot");
	ctx = canvas.getContext("2d");
	ctx.font = "9px arial";
}

onclose = function(){
	chrome.serial.disconnect(conStatus.connectionId, function(){});
};

function openPort(ports){
	document.getElementById('comsel').innerHTML = '';
	
	var sel = document.createElement('select');
	sel.onchange = function(){ openSer(this.value); };
	sel.id = 'comselin';
	
	var opt = document.createElement('option');
	opt.value = '';
	opt.innerHTML = 'Select COM port';
	sel.appendChild(opt);
	
	for(var i in ports){
		var opt = document.createElement('option');
		opt.value = ports[i].path;
		opt.innerHTML = ports[i].path;
		sel.appendChild(opt);
	}
	
	document.getElementById('comsel').appendChild(sel);
	
	document.getElementById('refresh').value = 'Refresh';
	document.getElementById('refresh').onclick = function(){ chrome.serial.getDevices(openPort); };
}

function openSer(port){	
	chrome.serial.onReceive.addListener(serRead);
	chrome.serial.connect(port, {bitrate: 115200}, serOpen);
	
	log('Connected to port '+port);
	document.getElementById('comsel').removeChild(document.getElementById('comselin'));
	document.getElementById('comsel').innerHTML = port;
	
	document.getElementById('refresh').value = 'Close';
	document.getElementById('refresh').onclick = function(){
		chrome.serial.disconnect(conHandle.connectionId, function(){
			conStatus = 0;
			log('COM port closed');
			clearElements();
			chrome.serial.getDevices(openPort);
		});
	}
	
	conStatus = 1;
}

function serOpen(conInfo){
	conHandle = conInfo;
}

function serRead(info){
	//changeEndianness(info.data, rxBuf);
	var x = new Uint8Array(info.data);
	rxBufUint8[0] = x[0];
	rxBufUint8[1] = x[1];
	rxBufUint8[2] = x[2];
	rxBufUint8[3] = x[3];
}

function startPlot(){
	document.getElementById('plotstart').value = 'Stop';
	document.getElementById('plotstart').onclick = stopPlot;
	
	ctx.clearRect(0,0,canvas.width,canvas.height);
	
	histLen = 0;
	chrome.serial.onReceive.removeListener(serRead);
	if (document.getElementById('plotselin').value == 'Gyros'){
		axisWidth = 26;
		ctx.beginPath();
		ctx.fillText(" 1000",0,canvas.height/4);
		ctx.fillText("    0",0,canvas.height/2);
		ctx.fillText("-1000",0,canvas.height*3/4);
		ctx.strokeStyle = "rgb(192,192,192)";
		ctx.moveTo(axisWidth,      canvas.height/4);
		ctx.lineTo(canvas.width-1, canvas.height/4);
		ctx.moveTo(axisWidth,      canvas.height/2);
		ctx.lineTo(canvas.width-1, canvas.height/2);
		ctx.moveTo(axisWidth,      canvas.height*3/4);
		ctx.lineTo(canvas.width-1, canvas.height*3/4);
		ctx.stroke();
		chrome.serial.onReceive.addListener(updatePlotSensor);
		regUint32Write(3,2+(31<<8));
	} else if (document.getElementById('plotselin').value == 'Receiver'){
		axisWidth = 12;
		ctx.beginPath();
		ctx.fillText(" 1",0, (0.1/2.2)*canvas.height);
		ctx.fillText(" 0",0, canvas.height/2);
		ctx.fillText("-1",0, (2.1/2.2)*canvas.height);
		ctx.strokeStyle = "rgb(192,192,192)";
		ctx.moveTo(axisWidth,      (0.1/2.2)*canvas.height);
		ctx.lineTo(canvas.width-1, (0.1/2.2)*canvas.height);
		ctx.moveTo(axisWidth,      canvas.height/2);
		ctx.lineTo(canvas.width-1, canvas.height/2);
		ctx.moveTo(axisWidth,      (2.1/2.2)*canvas.height);
		ctx.lineTo(canvas.width-1, (2.1/2.2)*canvas.height);
		ctx.stroke();
		chrome.serial.onReceive.addListener(updatePlotReceiver);
		regUint32Write(3,5+(31<<8));
	} else if (document.getElementById('plotselin').value == 'Motors'){
		axisWidth = 26;
		ctx.beginPath();
		ctx.fillText("2000",0, ( 250/10000)*canvas.height);
		ctx.fillText("   0",0, (2250/10000)*canvas.height);
		ctx.fillText("2000",0, (2750/10000)*canvas.height);
		ctx.fillText("   0",0, (4750/10000)*canvas.height);
		ctx.fillText("2000",0, (5250/10000)*canvas.height);
		ctx.fillText("   0",0, (7250/10000)*canvas.height);
		ctx.fillText("2000",0, (7750/10000)*canvas.height);
		ctx.fillText("   0",0, (9750/10000)*canvas.height);
		ctx.strokeStyle = "rgb(192,192,192)";
		ctx.moveTo(axisWidth,      ( 250/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, ( 250/10000)*canvas.height);
		ctx.moveTo(axisWidth,      (2250/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (2250/10000)*canvas.height);
		ctx.moveTo(axisWidth,      (2750/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (2750/10000)*canvas.height);
		ctx.moveTo(axisWidth,      (4750/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (4750/10000)*canvas.height);
		ctx.moveTo(axisWidth,      (5250/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (5250/10000)*canvas.height);
		ctx.moveTo(axisWidth,      (7250/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (7250/10000)*canvas.height);
		ctx.moveTo(axisWidth,      (7750/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (7750/10000)*canvas.height);
		ctx.moveTo(axisWidth,      (9750/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (9750/10000)*canvas.height);
		ctx.stroke();
		chrome.serial.onReceive.addListener(updatePlotMotor);
		regUint32Write(3,8+(31<<8));
	} else if (document.getElementById('plotselin').value == 'Vbat'){
		axisWidth = 20;
		ctx.beginPath();
		ctx.fillText("16.8",0, (1.2/8)*canvas.height);
		ctx.fillText("15  ",0, (3  /8)*canvas.height);
		ctx.fillText("12.6",0, (5.4/8)*canvas.height);
		ctx.fillText("11  ",0, (7  /8)*canvas.height);
		ctx.strokeStyle = "rgb(192,192,192)";
		ctx.moveTo(axisWidth,      (1.2/8)*canvas.height);
		ctx.lineTo(canvas.width-1, (1.2/8)*canvas.height);
		ctx.moveTo(axisWidth,      (3  /8)*canvas.height);
		ctx.lineTo(canvas.width-1, (3  /8)*canvas.height);
		ctx.moveTo(axisWidth,      (5.4/8)*canvas.height);
		ctx.lineTo(canvas.width-1, (5.4/8)*canvas.height);
		ctx.moveTo(axisWidth,      (7  /8)*canvas.height);
		ctx.lineTo(canvas.width-1, (7  /8)*canvas.height);
		ctx.stroke();
		chrome.serial.onReceive.addListener(updatePlotVbat);
		regUint32Write(3,3+(3<<8));
	}
}

function stopPlot(){
	regUint32Write(3,0);
	chrome.serial.onReceive.removeListener(updatePlotSensor);
	chrome.serial.onReceive.removeListener(updatePlotReceiver);
	chrome.serial.onReceive.removeListener(updatePlotMotor);
	chrome.serial.onReceive.removeListener(updatePlotVbat);
	chrome.serial.onReceive.addListener(serRead);
	
	document.getElementById('plotstart').value = 'Plot';
	document.getElementById('plotstart').onclick = startPlot;
}

function readConfig(){
	txBufUint8[0] = 0;
	txBufUint8[1] = 0;
	chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	timeout = timeoutIncr;
	setTimeout(function(){ document.getElementById('version').innerHTML = rxBufUint32[0]; }, timeout);
	timeout += timeoutIncr;
	
	regFloat32Read(7,'vbatmin');
	regFloat32Read(8,'radiofilt');
	regFloat32Read(9,'expopr');
	regFloat32Read(10,'expoy');
	regUint32Read(11,'motorstart');
	regUint32Read(12,'motorarmed');
	regFloat32Read(13,'ratepr');
	regFloat32Read(14,'ratey');
	regFloat32Read(15,'throttle');
	regFloat32Read(16,'pitchp');
	regFloat32Read(17,'pitchi');
	regFloat32Read(18,'pitchd');
	regFloat32Read(19,'rollp');
	regFloat32Read(20,'rolli');
	regFloat32Read(21,'rolld');
	regFloat32Read(22,'yawp');
	regFloat32Read(23,'yawi');
	regFloat32Read(24,'yawd');
	regUint32Read(25,'gyrofilt');
	
	setTimeout(function(){ log('Read config DONE');}, timeout);
}

function writeConfig(){
	// Erase Flash
	txBufUint8[0] = 6;
	chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	timeout = 2000;
	
	regUint32Flash(0);
	regFloat32Flash(7);
	regFloat32Flash(8);
	regFloat32Flash(9);
	regFloat32Flash(10);
	regUint32Flash(11);
	regUint32Flash(12);
	regFloat32Flash(13);
	regFloat32Flash(14);
	regFloat32Flash(15);
	regFloat32Flash(16);
	regFloat32Flash(17);
	regFloat32Flash(18);
	regFloat32Flash(19);
	regFloat32Flash(20);
	regFloat32Flash(21);
	regFloat32Flash(22);
	regFloat32Flash(23);
	regFloat32Flash(24);
	regUint32Flash(25);
	
	setTimeout(function(){ log('Write config DONE');}, timeout);
}

function motorTest(){
	var sel = ((document.getElementById('motorsel1').checked) ? 1 : 0)
	        + ((document.getElementById('motorsel2').checked) ? 2 : 0)
			+ ((document.getElementById('motorsel3').checked) ? 4 : 0)
			+ ((document.getElementById('motorsel4').checked) ? 8 : 0);
	regUint32Write(2, parseInt(document.getElementById('motortest').value) + (sel << 16));
	if ((sel > 0) && (motorTestOn == false)) {
		motorTestOn = true;
		log('Motor Test ON');
	}
	else if ((sel == 0) && motorTestOn) {
		motorTestOn = false;
		log('Motor Test OFF');
	}
}

function updatePlotSensor(info){
	var i;
	var x = new Float32Array(info.data);
	var c = ["rgb(255,0,0)", "rgb(0,255,0)", "rgb(0,0,255)"]
	
	for (i=0; i<3; i++){
		sensorBufPrevFloat32[i] = (histLen > 0) ? sensorBufFloat32[i] : 0;
		sensorBufFloat32[i] = -(x[i] / 4000) * canvas.height;
	}
	
	if (histLen < (canvas.width-axisWidth)){
		
		for (i=0; i<3; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(histLen+axisWidth, sensorBufPrevFloat32[i] + canvas.height/2);
			ctx.lineTo(histLen+1+axisWidth, sensorBufFloat32[i] + canvas.height/2);
			ctx.stroke();
		}
		
		histLen = histLen + 1;
		
	} else {
		
		var imgData = ctx.getImageData(1+axisWidth,0,canvas.width-1-axisWidth,canvas.height);
		ctx.clearRect(canvas.width-1,0,1,canvas.height);
		ctx.putImageData(imgData,axisWidth,0);
		
		ctx.beginPath();
		ctx.strokeStyle = "rgb(192,192,192)";
		ctx.moveTo(canvas.width-2,canvas.height/4);
		ctx.lineTo(canvas.width-1,canvas.height/4);
		ctx.moveTo(canvas.width-2,canvas.height/2);
		ctx.lineTo(canvas.width-1,canvas.height/2);
		ctx.moveTo(canvas.width-2,canvas.height*3/4);
		ctx.lineTo(canvas.width-1,canvas.height*3/4);
		ctx.stroke();
		
		for (i=0; i<3; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(canvas.width-2, sensorBufPrevFloat32[i] + canvas.height/2);
			ctx.lineTo(canvas.width-1, sensorBufFloat32[i] + canvas.height/2);
			ctx.stroke();
		}
	}
}

function updatePlotReceiver(info){
	var i;
	var x = new Float32Array(info.data);
	var c = ["rgb(255,0,0)", "rgb(0,255,0)", "rgb(0,0,255)", "rgb(255,255,0)", "rgb(0,255,255)", "rgb(255,0,255)", "rgb(0,0,0)", "rgb(128,128,128)"];
	
	for (i=0; i<8; i++){
		receiverBufPrevFloat32[i] = (histLen > 0) ? receiverBufFloat32[i] : 0;
		receiverBufFloat32[i] = -(x[i] / 2.2) * canvas.height;
	}
	
	if (histLen < (canvas.width-axisWidth)){
		
		for (i=0; i<8; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(histLen+axisWidth, receiverBufPrevFloat32[i] + canvas.height/2);
			ctx.lineTo(histLen+1+axisWidth, receiverBufFloat32[i] + canvas.height/2);
			ctx.stroke();
		}
		
		histLen = histLen + 1;
		
	} else {
		
		var imgData = ctx.getImageData(1+axisWidth,0,canvas.width-1-axisWidth,canvas.height);
		ctx.clearRect(canvas.width-1,0,1,canvas.height);
		ctx.putImageData(imgData,axisWidth,0);
		
		ctx.beginPath();
		ctx.strokeStyle = "rgb(192,192,192)";
		ctx.moveTo(canvas.width-2, (0.1/2.2)*canvas.height);
		ctx.lineTo(canvas.width-1, (0.1/2.2)*canvas.height);
		ctx.moveTo(canvas.width-2, canvas.height/2);
		ctx.lineTo(canvas.width-1, canvas.height/2);
		ctx.moveTo(canvas.width-2, (2.1/2.2)*canvas.height);
		ctx.lineTo(canvas.width-1, (2.1/2.2)*canvas.height);
		ctx.stroke();
		
		for (i=0; i<8; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(canvas.width-2, receiverBufPrevFloat32[i] + canvas.height/2);
			ctx.lineTo(canvas.width-1, receiverBufFloat32[i] + canvas.height/2);
			ctx.stroke();
		}
	}
}

function updatePlotMotor(info){
	var i;
	var x = new Int32Array(info.data);
	var c = ["rgb(255,0,0)", "rgb(0,255,0)", "rgb(0,0,255)", "rgb(0,255,255)"];
	
	for (i=0; i<4; i++){
		motorBufPrevInt32[i] = (histLen > 0) ? motorBufInt32[i] : 0;
		motorBufInt32[i] = -(x[i] / 10000) * canvas.height;
	}
	
	if (histLen < (canvas.width-axisWidth)){
		
		for (i=0; i<4; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(histLen+axisWidth, motorBufPrevInt32[i] + (((2500*(i+1)-250)/10000)*canvas.height));
			ctx.lineTo(histLen+1+axisWidth, motorBufInt32[i] + (((2500*(i+1)-250)/10000)*canvas.height));
			ctx.stroke();
		}
		
		histLen = histLen + 1;
		
	} else {
		
		var imgData = ctx.getImageData(1+axisWidth,0,canvas.width-1-axisWidth,canvas.height);
		ctx.clearRect(canvas.width-1,0,1,canvas.height);
		ctx.putImageData(imgData,axisWidth,0);
		
		ctx.beginPath();
		ctx.strokeStyle = "rgb(192,192,192)";
		ctx.moveTo(canvas.width-2, ( 250/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, ( 250/10000)*canvas.height);
		ctx.moveTo(canvas.width-2, (2250/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (2250/10000)*canvas.height);
		ctx.moveTo(canvas.width-2, (2750/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (2750/10000)*canvas.height);
		ctx.moveTo(canvas.width-2, (4750/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (4750/10000)*canvas.height);
		ctx.moveTo(canvas.width-2, (5250/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (5250/10000)*canvas.height);
		ctx.moveTo(canvas.width-2, (7250/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (7250/10000)*canvas.height);
		ctx.moveTo(canvas.width-2, (7750/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (7750/10000)*canvas.height);
		ctx.moveTo(canvas.width-2, (9750/10000)*canvas.height);
		ctx.lineTo(canvas.width-1, (9750/10000)*canvas.height);
		ctx.stroke();
		
		for (i=0; i<4; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(canvas.width-2, motorBufPrevInt32[i] + (((2500*(i+1)-250)/10000)*canvas.height));
			ctx.lineTo(canvas.width-1, motorBufInt32[i] + (((2500*(i+1)-250)/10000)*canvas.height));
			ctx.stroke();
		}
	}
}

function updatePlotVbat(info){
	var x = new Float32Array(info.data);
	
	vbatBufPrevFloat32[0] = (histLen > 0) ? vbatBufFloat32[0] : 0;
	vbatBufFloat32[0] = ((18 - x[0]) / 8) * canvas.height;
	
	if (histLen < (canvas.width-axisWidth)){
		
		ctx.beginPath();
		ctx.strokeStyle = "rgb(255,0,0)";
		ctx.moveTo(histLen+axisWidth, vbatBufPrevFloat32[0]);
		ctx.lineTo(histLen+1+axisWidth, vbatBufFloat32[0]);
		ctx.stroke();
		
		histLen = histLen + 1;
		
	} else {
		
		var imgData = ctx.getImageData(1+axisWidth,0,canvas.width-1-axisWidth,canvas.height);
		ctx.clearRect(canvas.width-1,0,1,canvas.height);
		ctx.putImageData(imgData,axisWidth,0);
		
		ctx.beginPath();
		ctx.strokeStyle = "rgb(192,192,192)";
		ctx.moveTo(canvas.width-2, (1.2/8)*canvas.height);
		ctx.lineTo(canvas.width-1, (1.2/8)*canvas.height);
		ctx.moveTo(canvas.width-2, (3  /8)*canvas.height);
		ctx.lineTo(canvas.width-1, (3  /8)*canvas.height);
		ctx.moveTo(canvas.width-2, (5.4/8)*canvas.height);
		ctx.lineTo(canvas.width-1, (5.4/8)*canvas.height);
		ctx.moveTo(canvas.width-2, (7  /8)*canvas.height);
		ctx.lineTo(canvas.width-1, (7  /8)*canvas.height);
		ctx.stroke();
		
		ctx.beginPath();
		ctx.strokeStyle = "rgb(255,0,0)";
		ctx.moveTo(canvas.width-2, vbatBufPrevFloat32[0]);
		ctx.lineTo(canvas.width-1, vbatBufFloat32[0]);
		ctx.stroke();
	}
}

function clearElements(){
	document.getElementById('version').innerHTML = [];
	document.getElementById('vbatmin').value = [];
	document.getElementById('radiofilt').value = [];
	document.getElementById('expopr').value = [];
	document.getElementById('expoy').value = [];
	document.getElementById('motorstart').value = [];
	document.getElementById('motorarmed').value = [];
	document.getElementById('ratepr').value = [];
	document.getElementById('ratey').value = [];
	document.getElementById('throttle').value = [];
	document.getElementById('pitchp').value = [];
	document.getElementById('pitchi').value = [];
	document.getElementById('pitchd').value = [];
	document.getElementById('rollp').value = [];
	document.getElementById('rolli').value = [];
	document.getElementById('rolld').value = [];
	document.getElementById('yawp').value = [];
	document.getElementById('yawi').value = [];
	document.getElementById('yawd').value = [];
	document.getElementById('gyrofilt').value = [];
	document.getElementById('motorsel1').checked = 0;
	document.getElementById('motorsel2').checked = 0;
	document.getElementById('motorsel3').checked = 0;
	document.getElementById('motorsel4').checked = 0;
	document.getElementById('motortest').value = 0;
}

function regUint32Read(addr, name){
	setTimeout(function(){
		txBufUint8[0] = 0;
		txBufUint8[1] = addr;
		chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	}, timeout);
	timeout += timeoutIncr;
	setTimeout(function(){
		document.getElementById(name).value = rxBufUint32[0];
	}, timeout);
	timeout += timeoutIncr;
}

function regFloat32Read(addr, name){
	setTimeout(function(){
		txBufUint8[0] = 0;
		txBufUint8[1] = addr;
		chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	}, timeout);
	timeout += timeoutIncr;
	setTimeout(function(){
		document.getElementById(name).value = Math.round(rxBufFloat32[0]*precisionFloat)/precisionFloat;
	}, timeout);
	timeout += timeoutIncr;
}

function regUint32Write(addr, data){
	txBufUint8[0] = 1;
	txBufUint8[1] = addr;
	setTxBufValUint32(data);
	chrome.serial.send(conHandle.connectionId, txBuf, function(){});
}

function regFloat32Write(addr, data){
	txBufUint8[0] = 1;
	txBufUint8[1] = addr;
	setTxBufValFloat32(data);
	chrome.serial.send(conHandle.connectionId, txBuf, function(){});
}

function regUint32Flash(addr){
	setTimeout(function(){
		txBufUint8[0] = 0;
		txBufUint8[1] = addr;
		chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	}, timeout);
	timeout += timeoutIncr;
	setTimeout(function(){
		txBufUint8[0] = 5;
		txBufUint8[1] = addr;
		setTxBufValUint32(rxBufUint32[0]);
		chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	}, timeout);
	timeout += timeoutIncr;
}

function regFloat32Flash(addr){
	setTimeout(function(){
		txBufUint8[0] = 0;
		txBufUint8[1] = addr;
		chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	}, timeout);
	timeout += timeoutIncr;
	setTimeout(function(){
		txBufUint8[0] = 5;
		txBufUint8[1] = addr;
		setTxBufValFloat32(rxBufFloat32[0]);
		chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	}, timeout);
	timeout += timeoutIncr;
}

function setTxBufValUint32(data){
	var txBufVal = new ArrayBuffer(4);
	var txBufValUint8 = new Uint8Array(txBufVal);
	var txBufValUint32 = new Uint32Array(txBufVal);
	txBufValUint32[0] = data;
	txBufUint8[2] = txBufValUint8[0];
	txBufUint8[3] = txBufValUint8[1];
	txBufUint8[4] = txBufValUint8[2];
	txBufUint8[5] = txBufValUint8[3];
}

function setTxBufValFloat32(data){
	var txBufVal = new ArrayBuffer(4);
	var txBufValUint8 = new Uint8Array(txBufVal);
	var txBufValFloat32 = new Float32Array(txBufVal);
	txBufValFloat32[0] = data;
	txBufUint8[2] = txBufValUint8[0];
	txBufUint8[3] = txBufValUint8[1];
	txBufUint8[4] = txBufValUint8[2];
	txBufUint8[5] = txBufValUint8[3];
}

function changeEndianness(bufIn, bufOut) {
	var reorderView = new Uint8Array(bufOut);
	var orderView = new Uint8Array(bufIn);
	reorderView[3] = orderView[0];
	reorderView[2] = orderView[1];
	reorderView[1] = orderView[2];
	reorderView[0] = orderView[3];
}

function log(line){
	document.getElementById('console').innerHTML+= line+'<br>';
	document.getElementById('console').scrollTop = document.getElementById('console').scrollHeight;
}
