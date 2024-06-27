importScripts("https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.5.0/socket.io.js");
var socket;
function mysocket(ip) {

  // sending a connect request to the server.
  console.log('connection to', ip);
    socket = io.connect('http://192.168.0'+ip+':5000');
//  socket = new WebSocket('ws://192.168.20.100:5000');

  // An event handler for a change of value 
  

  socket.on('after connect', function(msg) {
    postMessage(msg);
    console.log('After connect on socket', msg);
    socket.emit('joiner');
  });

  socket.on('update value', function(msg) {
      console.log('Messafe recieved from server',msg);
      
  });

  socket.on('logs', function(msg) {
      console.log('logs :', msg);
  });

 

}
function clicker(){
      console.log('button is clicked at client');
      socket.emit('joiner','drone1');
    // socket.emit('checker', {
    //       who: $(this).attr('id'),
    //       data: 'hey'
    //   });
      return false;
}

// self.port.start();

// var parameters = {}
// location.search.slice(1).split("&").forEach( function(key_value) { var kv = key_value.split("="); parameters[kv[0]] = kv[1]; })

// var ip_addr = parameters['ip_addr'];
// console.log('got here'+ip_addr);
// mysocket(ip_addr);

// self.onconnect = function(e) {
//   var port = e.ports[0];  // get the port

//   port.onmessage = function(e) {
//       console.log('Worker revceived arguemnts:', e.data);
//       port.postMessage(e.data[0] + e.data[1]);
//   }
// }

var ports = [];
self.addEventListener('connect', function(eventC){
  'use strict';
  var parameters = {}
  location.search.slice(1).split("&").forEach( function(key_value) { var kv = key_value.split("="); parameters[kv[0]] = kv[1]; })

  var ip_addr = parameters['ip_addr'];
  socket = io.connect('http://192.168.0.'+ip_addr+':5000');
  ports = eventC.ports;
  var port = ports[0];

  port.addEventListener('message', function(eventM){
    var data = eventM.data;
    console.log('o************ OnMessage ************o\n\n'
      , '\t data:', data, '\n'
    );
    // port.postMessage('from "clientPort": with love :)');
  }, false);

  port.start();


  socket.on('after connect', function(msg) {
    port.postMessage(msg);
    console.log('After connect on socket', msg);
    socket.emit('joiner');
  });

  socket.on('logs', function(msg) {
      port.postMessage(msg);
      console.log('Messafe recieved from server',msg);
      
  });

  

}, false);