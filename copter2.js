// Run this to receive a png image stream from your drone.

var arDrone = require('ar-drone');
var cv = require('opencv');
var http    = require('http');
var fs = require('fs');

console.log('Connecting png stream ...');

//var stream  = arDrone.createUdpNavdataStream();
var client = arDrone.createClient();
var pngStream = client.getPngStream();
var window = new cv.NamedWindow('Drone Video', 0);
var windowGray = new cv.NamedWindow('Drone Video Black', 0);
var windowThres = new cv.NamedWindow('threshold video', 0);
var windowObjectDet = new cv.NamedWindow('Target find ?', 0);
var maxArea = 6000;
var alreadyFlying = false; 
var j = 1; 

var newCentroid_X =0;  // X coordinate of the target. by defaut I  ;
var newCentroid_Y = 0; // assumed it is also located at the screen. by doing this, 
                       // the drone doesn't turn too much the first time he finds the target   ;

//var videoStream = client.getVideoStream(); 
var lowThresh = 0; // used in canny edge detector
var highThresh = 100; // used in canny edge detector
var GREEN = [0, 255, 0]; // B, G, R
var WHITE = [255, 255, 255]; // B, G, R
var RED   = [0, 0, 255]; // B, G, R
var bufferHSV, bufferGray; 
var matriceHSV= new cv.Matrix(); 
var Blue_lowerThreshold = [110, 50, 50]; // HSV value for blue color lower_red_hue_range
var Blue_upperThreshold = [130, 255, 255]; //
var bufferThreshed // put the image buffer after image has been threshold using the inRange fct. with desired color
var processingImage = false;
var lastPng;
var navData;
var flying = false;
var startTime = new Date().getTime();
var log = function(s){
var time = ( ( new Date().getTime() - startTime ) / 1000 ).toFixed(2);

  console.log(time+" \t"+s);
}

// display color and b/w video of drone (small delay observed)
  pngStream
    .on('error', console.log)
    .on('data', function(pngBuffer) {
      console.log("got drone images");
      lastPng = pngBuffer;

        if(alreadyFlying==false) {
          console.log('not flying');
          return;
        } 

      cv.readImage(lastPng, function(err, im) {
        if (err) throw err;
        //console.log(im.size())
        if (im.size()[0] > 0 && im.size()[1] > 0){

          window.show(im);
          im_gray = im.copy();
          im_gray.convertGrayscale();
          windowGray.show(im_gray);

          var width = im.width();
          var height = im.height();

          var newCentroid_X = im.width() * 0.5;   // X coordinate of the target. by defaut I  ;
          var newCentroid_Y = im.height() * 0.5;  // assumed it is also located at the screen. by doing this, 
                                                  // the drone doesn't turn too much the first time he finds the target   ;


          if (width < 1 || height < 1) throw new Error('Image has no size');

          var big = new cv.Matrix(height, width);
          var all = new cv.Matrix(height, width);

          //im_canny=im.copy();
          //im_canny.canny(lowThresh, highThresh);
          //im_threshed = im.copy();
          im.convertHSVscale();
          //im_thres = im.copy();
          im.inRange(Blue_lowerThreshold,Blue_upperThreshold);
          im.save('./image_thresholded.jpg');
          windowThres.show(im);

          //contours = im_canny.findContours(); 
          contours = im.findContours();
          for(i = 0; i < contours.size(); i++) {
            if(contours.area(i) > maxArea) {
              console.log('contour of target found: contour area ='+contours.area(i));
              big.drawContour(contours, i, GREEN);
              var arcLength = contours.arcLength(i, true);
              contours.approxPolyDP(i, 0.01 * arcLength, true);
              if(contours.cornerCount(i)==4){
                all.drawContour(contours, i, RED);
                windowObjectDet.show(all);
              }


              try{
                targetFound();
                var moments = contours.moments(i);
              console.log('moments =' +moments);
              var cgx = Math.round(moments.m10 / moments.m00); //centroid x coordinate of detected contour
              var cgy = Math.round(moments.m01 / moments.m00); // centroid y coordinate of contour
              console.log('centroid_X = '+cgx ,'centroid_Y = '+cgy);

              

                    if(newCentroid_X != cgx || newCentroid_Y != cgy){
                      console.log('target has moved');
                      newCentroid_X =cgx;
                      newCentroid_Y = cgy; 

                    var centerX = im.width() * 0.5; // X coordinate of the center of the image 
                    var centerY = im.height() * 0.5; // Y coord...........

                    /*
                    now we compute the values (speed) to be passed to the droneClient to 
                    turn (clockwise, counterclockwise ) or to change its altitude(up,down)
                    */
                    var turnAmount = -(cgx - centerX ) / centerX; 
                    var heightAmount = -(cgy - centerY ) / centerY; 


                    turnAmount = Math.min( 1, turnAmount );
                    turnAmount = Math.max( -1, turnAmount );
                    console.log('turnAmount = ' +turnAmount, 'heightAmount =' +heightAmount);

                    heightAmount = 0;
                    moveAndTrack(turnAmount, heightAmount);
                    } 

                    else if (newCentroid_X ==cgx || newCentroid_Y ==cgy){
                      console.log('target did not move '); 
                    }

              
                
              }
              catch(error){
                console.log("what is the problem? : "+ error);
              }
              
            
            }
            else{
              targetNotFound();
            }
            
          }

  big.save('./big.png');
  all.save('./all.png');
  //console.log('Image saved to ./big.png && ./all.png');
        };
          

      }); 
   
    });


  client.takeoff();
   client.after(2000,function(){ 
    //log("going up");
   //this.up(1);
   this.clockwise(0);
  alreadyFlying =true;
  })
   .after(30000, function() {
    this.stop();
    this.land();
    alreadyFlying = false;
  });



  // client.on('navdata', function(navdata) {
  //   navData = navdata;
  // })
//clearInterval(faceInterval); 

function targetFound(){
  //console.log("target found");
  client.up(0);
  //client.on('navdata', function(navdata) {
    // if(navdata.demo){
    //   console.log('where is the target?:  Altitude= '+ navdata.demo.altitude + ' ,  altitude meter =' + navdata.demo.altitudeMeters);
    //   }
    // });
}

function targetNotFound(){
  console.log("target not yet found");

}

function moveAndTrack(turnAmount, heightAmount){

  if( Math.abs( turnAmount ) > Math.abs( heightAmount ) ){
                
                if( turnAmount < 0 ) {
                  log( "turning clockwise  "+turnAmount );
                  client.clockwise( Math.abs( turnAmount ) );
                  console.log('drone turning to right'); 

                }
                else {
                  client.counterClockwise( turnAmount );
                log( "turning counterClockwise"+turnAmount );
                console.log('drone turning to left'); 

                }
                setTimeout(function(){
                    log("stopping turn");
                    client.clockwise(0);
                    //this.stop();
                },100);
              }
  else {
    log( "going vertical "+heightAmount );
    if(  heightAmount < 0 ) client.down( heightAmount );
    else client.up( heightAmount );
    setTimeout(function(){
      log("stopping altitude change");
      
      client.up(0);

    },50);

  }
}

// Server created to display and process drone images (display in browser)

    var server = http.createServer(function(req, res) {
      if (!lastPng) {
        res.writeHead(503);
        res.end('Did not receive any png data yet.');
        return;
      }
          // cv.readImage( lastPng, function(err, im) {
          //   im.convertGrayscale();
          //   im.save('./black_whi.png');
          //   lastPng = im.toBuffer();
          // });

       pngStream.on('data', sendPng);
       //pngStream.on('data', sendSomething);

        res.writeHead(200, { 'Content-Type': 'multipart/x-mixed-replace; boundary=--daboundary' });

        function sendPng(buffer) {
          console.log(buffer.length);
          cv.readImage(buffer, function(err, im) {
              // im.convertGrayscale();
              // buffer=im.toBuffer(); 
            im.convertHSVscale();
            bufferHSV=im.toBuffer();
            im.inRange(Blue_lowerThreshold,Blue_upperThreshold);
            im.save('./threshedImageHTTP.jpg'); 
            bufferThreshed=im.toBuffer();
          });
          res.write('--daboundary\nContent-Type: image/png\nContent-length: ' + buffer.length + '\n\n');
          //res.write(buffer);
          res.write(bufferThreshed);

        }

    });

    server.listen(8080, function() {
      console.log('Serving latest png on port 8080 ...');
    });

