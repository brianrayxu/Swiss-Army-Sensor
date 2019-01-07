

// Modules
var SerialPort = require('serialport');
var Readline = require('@serialport/parser-readline')
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);


// Open serial port
var port = new SerialPort('/dev/cu.SLAB_USBtoUART', {baudRate: 115200});

// Read data from serial
var read_data;
var msg;
const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', function (data) {
  read_data = data;
  console.log('Read:', read_data);
  msg = parseFloat(read_data);  // Convert to float
  io.emit('message', msg);
});




/*
 // Test stuff --> no serial port
 var i = 0;
 setInterval( function() {
   console.log('Read:', i);
   io.emit('message', i);
   i++;
   if (i > 10) {i=0;}
 }, 1000);

*/

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});
