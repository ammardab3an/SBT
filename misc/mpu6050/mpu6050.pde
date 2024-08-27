
import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from https://github.com/postspectacular/toxiclibs/releases
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];
float[] xzy = new float[3];
float[] zyx = new float[3];

void setup() {
  
    // 300px square viewport using OpenGL rendering
    size(300, 300, OPENGL);
    gfx = new ToxiclibsSupport(this);
  
    // setup lights and antialiasing
    lights();
    smooth();
  
    String portName = "COM6"; 
  
    // open the serial port
    port = new Serial(this, portName, 115200);
    
    // send single character to trigger DMP init/start
    // (expected by MPU6050_DMP6 example Arduino sketch)
    port.write('r');
}

void draw() {
  
    if (millis() - interval > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        port.write('r');
        interval = millis();
    }
    
    // black background
    background(0);
    
    // translate everything to the middle of the viewport
    pushMatrix();
    translate(width / 2, height / 2);
      
    if(teapotPacket[11]==1){
      rotateZ(-ypr[0]*PI/180);
      rotateX(-ypr[1]*PI/180);
      rotateY(ypr[2]*PI/180);
    }
    else{
      if(false){
        float[] axis = quat.toAxisAngle();
        rotate(axis[0], -axis[2], axis[3], -axis[1]);        
      }
      else if(false){
        rotateY(ypr[0]);
        rotateX(-ypr[1]);
        rotateZ(-ypr[2]);
      }
      else if(false){
        rotateZ(-xzy[0]);
        rotateY(xzy[1]);
        rotateX(-xzy[2]);
      }
      else if(true){
        rotateY(zyx[0]);
        rotateX(-zyx[1]);
        rotateZ(-zyx[2]);
      }
    }

    // draw main body in red
    fill(255, 0, 0, 200);
    box(10, 10, 200);
    
    // draw front-facing tip in blue
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();
    
    // draw wings and tail fin in green
    fill(0, 255, 0, 200);
    beginShape(TRIANGLES);
    vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);  // wing top layer
    vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);  // wing bottom layer
    vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);  // tail left layer
    vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);  // tail right layer
    endShape();
    beginShape(QUADS);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    endShape();
    
    popMatrix();
}


void serialEvent(Serial port) {
  
    interval = millis();
    while (port.available() > 0) {
      
        int ch = port.read();
        if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
        
        synced = 1;
        print((char)ch);

        if ((serialCount == 1 && ch != 2)
            || (serialCount == 12 && ch != '\r')
            || (serialCount == 13 && ch != '\n'))  {
            serialCount = 0;
            synced = 0;
            return;
        }

        if (serialCount > 0 || ch == '$') {
            teapotPacket[serialCount++] = (char)ch;
        }
        
        if(serialCount != 14){
          continue;
        }
         
        serialCount = 0; // restart packet byte position
        
        // get quaternion from data packet
        
        if(teapotPacket[11]==1){
          ypr[0] = (float)((long)((teapotPacket[2] << 24) | (teapotPacket[3] << 16) | (teapotPacket[4] << 8)))/(1<<16);
          ypr[1] = (float)((long)((teapotPacket[5] << 24) | (teapotPacket[6] << 16) | (teapotPacket[7] << 8)))/(1<<16);
          ypr[2] = (float)((long)((teapotPacket[8] << 24) | (teapotPacket[9] << 16) | (teapotPacket[10] << 8)))/(1<<16);
          println("ypr:\t" + ypr[0] + "\t" + ypr[1] + "\t" + ypr[2]);
        }
        else{
          
          q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
          q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
          q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
          q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
          for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
          quat.set(q[0], q[1], q[2], q[3]);
          //println("q:\t" + round(q[0]*1000.0f)/1000.0f + "\t" + round(q[1]*1000.0f)/1000.0f + "\t" + round(q[2]*1000.0f)/1000.0f + "\t" + round(q[3]*1000.0f)/1000.0f);
          
          //ypr[0] = atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
          //ypr[1] = 2*atan2(sqrt(1+2*(q[0]*q[2]-q[1]*q[3])), 1-2*(q[0]*q[2]-q[1]*q[3]))-PI/2;
          //ypr[2] = atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
          //println("ypr:\t" + ypr[0]*180/PI + "\t" + ypr[1]*180/PI + "\t" + ypr[2]*180/PI);
          
          float s = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
          //println(s);
          float x0 = q[0]/s;
          float x1 = q[1]/s;
          float x2 = q[2]/s;
          float x3 = q[3]/s;
          //float u0 = -2 * (x1*x2-x0*x3);
          //if (u0 > 1.0) {
          //  u0 = 1.0;
          //} else {
          //  if (u0 < -1.0) {
          //    u0 = -1.0;
          //  }
          //}
          //xzy[0] = atan2(2*(x2*x3+x0*x1), ((x0*x0-x1*x1)+x2*x2)-x3*x3);
          //xzy[1] = asin(u0);
          //xzy[2] = atan2(2*(x1*x3+x0*x2), ((x0*x0+x1*x1)-x2*x2)-x3*x3);
          //println("xzy:\t" + xzy[0]*180/PI + "\t" + xzy[1]*180/PI + "\t" + xzy[2]*180/PI);
          
          float u0 = -2 * (x1*x3-x0*x2);
          if (u0 > 1.0) {
            u0 = 1.0;
          } else {
            if (u0 < -1.0) {
              u0 = -1.0;
            }
          }
          zyx[0] = atan2(2*(x1*x2+x0*x3), ((x0*x0+x1*x1)-x2*x2)-x3*x3);
          zyx[1] = asin(u0);
          zyx[2] = atan2(2*(x2*x3+x0*x1), ((x0*x0-x1*x1)-x2*x2)+x3*x3);
          println("xzy:\t" + zyx[0]*180/PI + "\t" + zyx[1]*180/PI + "\t" + zyx[2]*180/PI);
        }
    }
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
    
        // Center point
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
}
