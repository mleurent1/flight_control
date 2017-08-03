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
	
	canvas = document.getElementById("plot");
	ctx = canvas.getContext("2d");
	
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
	
	document.getElementById('plotsel').appendChild(sel);
	
	document.getElementById('plotstart').value = 'Plot';
	document.getElementById('plotstart').onclick = startPlot;
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

function updatePlotSensor(info){
	var i;
	var x = new Float32Array(info.data);
	var c = ["rgb(255,0,0)", "rgb(0,255,0)", "rgb(0,0,255)"]
	
	for (i=0; i<3; i++){
		sensorBufPrevFloat32[i] = sensorBufFloat32[i];
		sensorBufFloat32[i] = -(x[i] / 4000) * canvas.height;
	}
	
	if (histLen < canvas.width){
		
		for (i=0; i<3; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(histLen, sensorBufPrevFloat32[i] + canvas.height/2);
			ctx.lineTo(histLen+1, sensorBufFloat32[i] + canvas.height/2);
			ctx.stroke();
		}
		
		histLen = histLen + 1;
		
	} else {
		
		var imgData = ctx.getImageData(1,0,canvas.width-1,canvas.height);
		ctx.clearRect(canvas.width-1,0,1,canvas.height);
		ctx.putImageData(imgData,0,0);
		
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
		receiverBufPrevFloat32[i] = receiverBufFloat32[i];
		receiverBufFloat32[i] = -x[i] * (canvas.height/2);
	}
	
	if (histLen < canvas.width){
		
		for (i=0; i<8; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(histLen, receiverBufPrevFloat32[i] + canvas.height/2);
			ctx.lineTo(histLen+1, receiverBufFloat32[i] + canvas.height/2);
			ctx.stroke();
		}
		
		histLen = histLen + 1;
		
	} else {
		
		var imgData = ctx.getImageData(1,0,canvas.width-1,canvas.height);
		ctx.clearRect(canvas.width-1,0,1,canvas.height);
		ctx.putImageData(imgData,0,0);
		
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
		motorBufPrevInt32[i] = motorBufInt32[i];
		motorBufInt32[i] = -(x[i] / 2000) * (canvas.height/4);
	}
	
	if (histLen < canvas.width){
		
		for (i=0; i<4; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(histLen, motorBufPrevInt32[i] + (canvas.height*(i+1)/4));
			ctx.lineTo(histLen+1, motorBufInt32[i] + (canvas.height*(i+1)/4));
			ctx.stroke();
		}
		
		histLen = histLen + 1;
		
	} else {
		
		var imgData = ctx.getImageData(1,0,canvas.width-1,canvas.height);
		ctx.clearRect(canvas.width-1,0,1,canvas.height);
		ctx.putImageData(imgData,0,0);
		
		for (i=0; i<4; i++){
			ctx.beginPath();
			ctx.strokeStyle = c[i];
			ctx.moveTo(canvas.width-2, motorBufPrevInt32[i] + (canvas.height*(i+1)/4));
			ctx.lineTo(canvas.width-1, motorBufInt32[i] + (canvas.height*(i+1)/4));
			ctx.stroke();
		}
		
	}
}

function startPlot(){
	document.getElementById('plotstart').value = 'Stop';
	document.getElementById('plotstart').onclick = stopPlot;
	
	ctx.clearRect(0,0,canvas.width,canvas.height);
	
	histLen = 0;
	chrome.serial.onReceive.removeListener(serRead);
	if (document.getElementById('plotselin').value == 'Gyros'){
		chrome.serial.onReceive.addListener(updatePlotSensor);
		regUint32Write(3,2+(31<<8));
	} else if (document.getElementById('plotselin').value == 'Receiver'){
		chrome.serial.onReceive.addListener(updatePlotReceiver);
		regUint32Write(3,5+(31<<8));
	}
	else if (document.getElementById('plotselin').value == 'Motors'){
		chrome.serial.onReceive.addListener(updatePlotMotor);
		regUint32Write(3,8+(31<<8));
	}
}

function stopPlot(){
	regUint32Write(3,0);
	chrome.serial.onReceive.removeListener(updatePlotSensor);
	chrome.serial.onReceive.removeListener(updatePlotReceiver);
	chrome.serial.onReceive.removeListener(updatePlotMotor);
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

/*
canvas.beginPath();
canvas.strokeStyle = "rgb(128,128,128)"; "rgb(11,153,11)"; "rgb(66,44,255)"
canvas.moveTo(x,y);
canvas.lineTo(w,y);
canvas.stroke();
*/
