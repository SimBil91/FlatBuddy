var express = require('express');
var app=express();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var mysql = require('mysql');

app.use('/js', express.static('/home/buddy/FlatBuddy/src/budspeech/server/js'));

app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

var con = mysql.createConnection({
  host: "127.0.0.1",
  user: "buddy",
  password: "travel",
  database : 'FB'
});


function get_temperature(socket) {
  var temp={};
  temp.val=[];
  temp.time=[];
  con.query('SELECT value,time FROM data WHERE type=1', function (error, results, fields) {
    if (error) throw error;
    for (i=0;i<results.length;i++) {
      temp.val[i]=results[i].value;
      temp.time[i]=results[i].time;
    }
    socket.emit('temp_data',temp);
  });
}

function get_altitude(socket) {
  var alti={};
  alti.val=[];
  alti.time=[];
  //con.connect();
  con.query('SELECT value,time FROM data WHERE type=2', function (error, results, fields) {
    if (error) throw error;
    for (i=0;i<results.length;i++) {
      alti.val[i]=results[i].value;
      alti.time[i]=results[i].time;
    }
    socket.emit('alti_data',alti);
  });
}

io.on('connection', function(socket){
  get_temperature(socket);
  get_altitude(socket);
  console.log('a user connected');
   socket.on('disconnect', function(){
    console.log('user disconnected');
  });
    socket.on('it_works', function(data){
    console.log('it works! '+ data);
  });
    socket.on('resp', function(data){
    io.emit('buddy_resp',data);
  });
  socket.on('chat_message', function(msg){
      console.log(msg)
    io.emit('user_demand',msg);
  });
});

http.listen(3000, function(){
  console.log('listening on *:3000');
});
