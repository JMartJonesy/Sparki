#include <Sparki.h>
#include "PositionTracker.h"

int ping = 0;
int positionIndex = 0;
boolean obstacle = false;
double pingMax = 8;
double locs[3][2] = {{20, 0}, {0, 0}};
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

//lineRight, lineCenter, lineLeft, edgeLeft, edgeLRight
void setup()
{
  sparki.servo(SERVO_CENTER);
  sparki.speed(60);
  sparki.clearLCD();
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
  if(positionIndex < sizeof(locs) / sizeof(locs[0]))
  {
     p.update();
     sparki.println(p.getCenter().x);
     sparki.println(p.getCenter().y);
     sparki.println(p.getAngle());
     sparki.updateLCD();
     delay(10000);
     obstacle = false;
     double turnDegrees = getToDegrees(locs[positionIndex][0],  p.getCenter().x, locs[positionIndex][1], p.getCenter().y);
     sparki.moveLeft(turnDegrees);
     sparki.moveStop();
     p.update();
     sparki.moveForward();
     boolean left = false;
     boolean right = false;
     int lThreshold = 600;
     int rThreshold = 750;
     while(!atGoal() && !obstacle)
     {
        p.update();
        ping = sparki.ping();
        if(ping <= pingMax)
          obstacle = true;

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
        }
        else if(pingRight < rThreshold && !right)
        {
          right = true;
          p.update();
          sparki.moveStop();
          setRSensorXY(p.getCenter().x, p.getCenter().y, p.getAngle());
          sparki.moveForward();
        }
        if(left && right)
          localize();
     }
     
     sparki.moveStop();
     if(obstacle)
       wallFollow();
     else
       positionIndex++;
  }
}

boolean atGoal()
{
    if(fabs(locs[positionIndex][0] - p.getCenter().x) <= 1 && fabs(locs[positionIndex][1] - p.getCenter().y) <=1)
      return true;
    return false;
}

void wallFollow()
{
  /*
    double a;
    double b = 1;
    double c;
    if(positionIndex != 0)
    {
      if (locs[positionIndex][0] == locs[positionIndex - 1][0]) 
      { // If prevous and goal X are the same, to avoid division by 0
        a = 1;
        b = 0;
        c = -locs[positionIndex][0];
      } 
      else 
      {
        a = -(locs[positionIndex][1] - locs[positionIndex - 1][1]) / 
             (locs[positionIndex][0] - locs[positionIndex - 1][0]);
        c = (((locs[positionIndex][1] - locs[positionIndex - 1][1]) / 
              (locs[positionIndex][0] - locs[positionIndex - 1][0])) * locs[positionIndex - 1][0]) - locs[positionIndex - 1][1];
      }
    }
    
    while(true)
    {
      if(obstacle)
      {
        sparki.moveLeft(90);
        p.update();
        obstacle = false;
      }
      sparki.servo(70);
      sparki.moveForward();
      delay(1200);
      p.update();
      sparki.moveStop();
      ping = sparki.ping();
      if(ping > pingMax)
        turnMe(ping, false);
      else if(ping < pingMax)
        turnMe(ping, true);
 //     sparki.moveStop();
 //     p.update();
      double d = mLineDistance(a,b,c);
      
      sparki.servo(SERVO_CENTER);
      delay(400);
      
      ping = sparki.ping();
      if(ping < pingMax)
        obstacle = true;
        
      if(d <= 6) {
        sparki.servo(SERVO_CENTER);
        delay(1000);
        break;
      }
    }
*/
}

void turnMe(int obstacleDistance, boolean left)
{
   double degreesToTurn;
   obstacleDistance = fabs(obstacleDistance - pingMax);
   if(obstacleDistance > 70)
     degreesToTurn = 0.261;
   else if(obstacleDistance > 50)
     degreesToTurn = 0.174;
   else if(obstacleDistance > 25)
     degreesToTurn = 0.087;
   else
     degreesToTurn = 0.052;
   
   if(left)
     sparki.moveLeft(-1*degreesToTurn*180/PI);
   else
     sparki.moveRight(degreesToTurn*180/PI);

   sparki.moveStop();
   p.update();
}

double mLineDistance(double a, double b, double c)
{
   return fabs((a * p.getCenter().x) + (b * p.getCenter().y) + c) / sqrt((a * a) + (b * b));
}

void localize()
{
  /*boolean right = false;
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
  }*/
  int lThreshold = 600;
  int rThreshold = 750;
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
  
  do
  {
    sparki.moveRight();
  }
  while(sparki.lineCenter() > lThreshold || sparki.lineLeft() > lThreshold || sparki.lineRight() > lThreshold);
  sparki.moveStop();
  p.update();
  
  p.setCenter((point){locs[positionIndex][0], locs[positionIndex][1]});
  p.setAngle(PI/2);
  
  sparki.print("X: ");
  sparki.println(p.getCenter().x);
  sparki.print("Y: ");
  sparki.println(p.getCenter().y);
  sparki.print("A: ");
  sparki.println(p.getAngle());
  sparki.updateLCD();
  p.update();
  
  sparki.print("X: ");
  sparki.println(p.getCenter().x);
  sparki.print("Y: ");
  sparki.println(p.getCenter().y);
  sparki.print("A: ");
  sparki.println(p.getAngle());
  sparki.updateLCD();
}

float findDist(float toX, float toY, float currentX, float currentY)
{
  return sqrt(((toX-currentX)*(toX-currentX)) + ((toY-currentY)*(toY-currentY)));
}

float getToDegrees(float toX, float toY, float currentX, float currentY)
{
  float phi = atan2(toY - currentY, toX - currentX);
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

