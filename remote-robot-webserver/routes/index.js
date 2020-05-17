var express = require('express');
var router = express.Router();
var template = require('../views/index');
var url = require('url');
var path = require('path');
var awsIot = require('aws-iot-device-sdk');
var bodyParser = require('body-parser');

router.use(bodyParser.json()); // support json encoded bodies
router.use(bodyParser.urlencoded({ extended: true })); // support encoded bodies

var sensorData = {
  accel_y: String,
  flex_avg: String,
  pitch_forward: String,
  pitch_backward: String
};

//Getting the home page
router.get('/', function(req, res, next) {
  res.render('index');
});

// Subscribing to AWS IoT Core MQTT broker using certificates
var device = awsIot.device({
  keyPath: "C:\\nodejs\\MiniProject\\certs\\c45fc8bd81-private.pem.key",
  certPath: "C:\\nodejs\\MiniProject\\certs\\c45fc8bd81-certificate.pem.crt",
  caPath: "C:\\nodejs\\MiniProject\\certs\\amazonRootCA1.pem",
  clientId: "nodejsClient",
  host: "a2ot8lh5ttyzzl-ats.iot.us-east-1.amazonaws.com"
});

device
  .on('connect', function() {
    console.log('connect');
    device.subscribe('myESP32/esp32topic');
  });

// Parsing incoming data
device
  .on('message', function(topic, payload) {
    incomingData = payload.toString();
    sensorData.accel_y = incomingData[0].concat(incomingData[1]).concat(incomingData[2]).concat(incomingData[3]);
    sensorData.flex_avg = incomingData[6].concat(incomingData[7]).concat(incomingData[8]).concat(incomingData[9]);
  });

  router.get('/getAccelY', function(req, res, next) {
    res.send(sensorData.accel_y);
  });

  router.get('/getFlexAvg', function(req, res, next) {
    res.send(sensorData.flex_avg);
  });

module.exports = router;
