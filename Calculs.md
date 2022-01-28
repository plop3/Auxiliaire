https://gist.github.com/shoebahmedadeel/0d8ca4eaa65664492cf1db2ab3a9e572

roll  =  (atan2(-Accel_Y, Accel_Z)*180.0)/M_PI;
pitch =  (atan2(Accel_X sqrt(Accel_Y*Accel_Y + Accel_Y*Accel_Y))*180.0)/M_PI;

https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/

   accelerationX = (signed int)(((signed int)rawData_X) * 3.9);
   accelerationY = (signed int)(((signed int)rawData_Y) * 3.9);
   accelerationZ = (signed int)(((signed int)rawData_Z) * 3.9);
   pitch = 180 * atan (accelerationX/sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
   roll = 180 * atan (accelerationY/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
   yaw = 180 * atan (accelerationZ/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;

https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data

  Accroll = atan2( accy / (accy^2 + accz^2) )
  Accpitch = atan2( accx / (accx^2 + accz^2) )

http://franciscoraulortega.com/pubs/Algo3DFusionsMems.pdf 
