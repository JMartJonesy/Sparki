#include <Sparki.h>
#include "PositionTracker.h"

PositionTracker p = PositionTracker();
float sensorLX = 0.0;
float sensorLY = 0.0;
float sensorRX = 0.0;
float sensorRY = 0.0;
float c1x = 0.0;
float c1y = 0.0;
float c2x = 0.0;
float c2y = 0.0;
float radius = 7.65;
int lThreshold = 600;
int rThreshold = 750;
//lineRight, lineCenter, lineLeft, edgeLeft, edgeLRight
void setup()
{
  sparki.clearLCD();
  sparki.servo(SERVO_CENTER);
  sparki.speed(60);
}

void loop()
{
  /*while(true)
   {
   sparki.clearLCD();
   sparki.println(sparki.edgeRight());
   sparki.updateLCD();
   //sparki.println(sparki.edgeLeft());
   //sparki.updateLCD();
   delay(100);
  }*/
  boolean right = false;
  boolean left = false;
  p.update();
  sparki.moveForward();
  while(!right || !left)
  {
    int pingLeft = sparki.edgeLeft();
    int pingRight = sparki.edgeRight();
    p.update();
    if(pingLeft < lThreshold && !left)
    {
      left = true;
      p.update();
      sparki.moveStop();
      setLSensorXY(p.getCenter().x, p.getCenter().y, p.getAngle());
      sparki.moveForward();
      sparki.beep();
      sparki.updateLCD();
    }
    else if(pingRight < rThreshold && !right)
    {
      right = true;
      p.update();
      sparki.moveStop();
      setRSensorXY(p.getCenter().x, p.getCenter().y, p.getAngle());
      sparki.moveForward();
      sparki.beep();
      sparki.updateLCD();
    }
    p.update();
  }
  sparki.moveStop();
  p.update();
  findCenter(sensorLX, sensorLY, sensorRX, sensorRY);
  
  float c1a = getToDegrees(c1x,c1y,p.getCenter().x, p.getCenter().y);
  float c2a = getToDegrees(c2x,c2y,p.getCenter().x, p.getCenter().y);
  float turnDegree = 0;
  float moveDist = 0;
  if(abs(c1a) > abs(c2a))
  {
    turnDegree = c2a;
    moveDist = findDist(c2x,c2y,p.getCenter().x, p.getCenter().y);
  }
  else
  {
    turnDegree = c1a;
    moveDist = findDist(c1x,c1y,p.getCenter().x, p.getCenter().y);
  }
  sparki.moveLeft(turnDegree);
  p.update();
  sparki.moveForward(moveDist);
  p.update();
  
  do
  {
    sparki.moveRight();
  }
  while(sparki.lineCenter() > lThreshold || sparki.lineLeft() > lThreshold || sparki.lineRight() > lThreshold);
  sparki.moveStop();
  sparki.println(sparki.lineCenter());
  sparki.println(sparki.lineLeft());
  sparki.println(sparki.lineRight());
  sparki.updateLCD();
  sparki.beep();
  
  delay(1000000000);
}

float findDist(float toX, float toY, float currentX, float currentY)
{
  return sqrt(((toX-currentX)*(toX-currentX)) + ((toY-currentY)*(toY-currentY)));
}

float getToDegrees(float toX, float toY, float currentX, float currentY)
{
  float xdiff = toX - currentX;
  float ydiff = toY - currentY;
  float phi = atan2(ydiff, xdiff);
  return ((atan2(sin(phi - p .getAngle()), cos(phi - p.getAngle()))) * 180 / PI); 
}

void findCenter(float x1, float y1, float x2, float y2)
{
  float q = sqrt(((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)));
  float x3 = (x1+x2) / 2;
  float y3 = (y1+y2) / 2;
  c1x = x3 + sqrt((radius*radius) - ((q/2)*(q/2))) * ((y1-y2)/q);
  c1y = y3 + sqrt((radius*radius) - ((q/2)*(q/2))) * ((x2-x1)/q);
  c2x = x3 - sqrt((radius*radius) - ((q/2)*(q/2))) * ((y1-y2)/q);
  c2y = y3 - sqrt((radius*radius) - ((q/2)*(q/2))) * ((x2-x1)/q);
  sparki.print(c1x);
  sparki.print(",");
  sparki.println(c1y);
  sparki.print(c2x);
  sparki.print(",");
  sparki.println(c2y);
  sparki.updateLCD();
}

void setLSensorXY(float x, float y, float angle)
{
  sensorLX = x + cos(angle + .749269314) * 5.872818744;
  sensorLY = y + sin(angle + .749269314) * 5.872818744;
}

void setRSensorXY(float x, float y, float angle)
{
  sensorRX = x + cos(angle - .749269314) * 5.872818744;
  sensorRY = y + sin(angle - .749269314) * 5.872818744;
}

