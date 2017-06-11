var timeout = 0;
var timeoutIncr = 15;
var conHandle = [];
var conStatus = 0;
var txBuf = new ArrayBuffer(6);
var txBufUint8 = new Uint8Array(txBuf);
var rxBuf = new ArrayBuffer(4);
var rxBufUint8 = new Uint8Array(rxBuf);
var rxBufUint32 = new Uint32Array(rxBuf,0,1);
var rxBufFloat32 = new Float32Array(rxBuf,0,1);
var precisionFloat = 1e6;
var motorTestOn = false;
var currentPlot = 0;
var canvas;

onload = function(){
	document.getElementById('readcfg').onclick = readConfig;
	document.getElementById('writecfg').onclick = writeConfig;
	document.getElementById('bind').onchange = function(){
		txBufUint8[0] = 1;
		txBufUint8[1] = 8;
		setTxBufValUint32((document.getElementById('bind').checked) ? 1 : 0);
		chrome.serial.send(conHandle.connectionId, txBuf, function(){});
		log('Updated Receiver Bind');
	};
	document.getElementById('motorstart').onchange = function(){ regUint32Write(11,parseInt(this.value)); log('Updated Motor Start'); };
	document.getElementById('motorarmed').onchange = function(){ regUint32Write(12,parseInt(this.value)); log('Updated Motor Armed'); };
	document.getElementById('throttle').onchange = function(){ regFloat32Write(15,parseFloat(this.value)); log('Updated Throttle Range'); };
	document.getElementById('vbatmin').onchange = function(){ regFloat32Write(7,parseFloat(this.value)); log('Updated VBAT Min'); };
	document.getElementById('ratepr').onchange = function(){ regFloat32Write(13,parseFloat(this.value)); log('Updated Expo'); };
	document.getElementById('expopr').onchange = function(){ regFloat32Write(9,parseFloat(this.value)); log('Updated Rate'); };
	document.getElementById('ratey').onchange = function(){ regFloat32Write(14,parseFloat(this.value)); log('Updated Expo'); };
	document.getElementById('expoy').onchange = function(){ regFloat32Write(10,parseFloat(this.value)); log('Updated Rate'); };
	document.getElementById('pitchp').onchange = function(){ regFloat32Write(16,parseFloat(this.value)); log('Updated Pitch P'); };
	document.getElementById('pitchi').onchange = function(){ regFloat32Write(17,parseFloat(this.value)); log('Updated Pitch I'); };
	document.getElementById('pitchd').onchange = function(){ regFloat32Write(18,parseFloat(this.value)); log('Updated Pitch D'); };
	document.getElementById('rollp').onchange = function(){ regFloat32Write(19,parseFloat(this.value)); log('Updated Roll P'); };
	document.getElementById('rolli').onchange = function(){ regFloat32Write(20,parseFloat(this.value)); log('Updated Roll I'); };
	document.getElementById('rolld').onchange = function(){ regFloat32Write(21,parseFloat(this.value)); log('Updated Roll D'); };
	document.getElementById('yawp').onchange = function(){ regFloat32Write(22,parseFloat(this.value)); log('Updated Yaw P'); };
	document.getElementById('yawi').onchange = function(){ regFloat32Write(23,parseFloat(this.value)); log('Updated Yaw I'); };
	document.getElementById('yawd').onchange = function(){ regFloat32Write(24,parseFloat(this.value)); log('Updated Yaw D'); };
	document.getElementById('atten').onchange = function(){ regFloat32Write(25,parseFloat(this.value)); log('Updated Throttle Atten'); };
	document.getElementById('motorsel1').onclick = motorTest;
	document.getElementById('motorsel2').onclick = motorTest;
	document.getElementById('motorsel3').onclick = motorTest;
	document.getElementById('motorsel4').onclick = motorTest;
	document.getElementById('motortest').value = 0;
	document.getElementById('motortest').onchange = motorTest;
	canvas = document.getElementById("plot").getContext("2d");
	chrome.serial.getDevices(openPort);
	openPlot();
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

function openPlot(){
	document.getElementById('plotsel').innerHTML = '';
	
	var sel = document.createElement('select');
	sel.onchange = function(){
		currentPlot = this.value;
		
		document.getElementById('plotsel').removeChild(document.getElementById('plotselin'));
		document.getElementById('plotsel').innerHTML = currentPlot;
		
		document.getElementById('plotstart').value = 'Stop';
		document.getElementById('plotstart').onclick = openPlot;
	};
	sel.id = 'plotselin';
	
	var opt = document.createElement('option');
	opt.value = '';
	opt.innerHTML = 'Select Plot';
	sel.appendChild(opt);
	opt = document.createElement('option');
	opt.value = 'Gyros';
	opt.innerHTML = 'Gyros';
	sel.appendChild(opt);
	opt = document.createElement('option');
	opt.value = 'Accel';
	opt.innerHTML = 'Accel';
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
	document.getElementById('plotstart').onclick = function(){  };
}

function readConfig(){
	txBufUint8[0] = 0;
	txBufUint8[1] = 0;
	chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	timeout = timeoutIncr;
	setTimeout(function(){ document.getElementById('version').innerHTML = rxBufUint32[0]; }, timeout);
	timeout += timeoutIncr;
	
	setTimeout(function(){ txBufUint8[1] = 8; chrome.serial.send(conHandle.connectionId, txBuf, function(){}); }, timeout);
	timeout += timeoutIncr;
	setTimeout(function(){ document.getElementById('bind').checked = (rxBufUint32[0]) ? true : false; }, timeout);
	timeout += timeoutIncr;
	
	regUint32Read(11,'motorstart');
	regUint32Read(12,'motorarmed');
	
	regFloat32Read(7,'vbatmin');
	regFloat32Read(9,'expopr');
	regFloat32Read(13,'ratepr');
	regFloat32Read(10,'expoy');
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
	regFloat32Read(25,'atten');
	
	setTimeout(function(){ log('Read config DONE');}, timeout);
}

function writeConfig(){
	// Erase Flash
	txBufUint8[0] = 6;
	chrome.serial.send(conHandle.connectionId, txBuf, function(){});
	timeout = 3000;
	
	regUint32Flash(0);
	regUint32Flash(8);
	regUint32Flash(11);
	regUint32Flash(12);
	
	regFloat32Flash(7);
	regFloat32Flash(9);
	regFloat32Flash(13);
	regFloat32Flash(10);
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
	regFloat32Flash(25);
	
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
	document.getElementById('bind').checked = 0;
	document.getElementById('motorstart').value = [];
	document.getElementById('motorarmed').value = [];
	document.getElementById('throttle').value = [];
	document.getElementById('vbatmin').value = [];
	document.getElementById('ratepr').value = [];
	document.getElementById('expopr').value = [];
	document.getElementById('ratey').value = [];
	document.getElementById('expoy').value = [];
	document.getElementById('pitchp').value = [];
	document.getElementById('pitchi').value = [];
	document.getElementById('pitchd').value = [];
	document.getElementById('rollp').value = [];
	document.getElementById('rolli').value = [];
	document.getElementById('rolld').value = [];
	document.getElementById('yawp').value = [];
	document.getElementById('yawi').value = [];
	document.getElementById('yawd').value = [];
	document.getElementById('atten').value = [];
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
