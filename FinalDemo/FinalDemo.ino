#include <Sparki.h>
#include "PositionTracker.h"

int ping = 0;
int positionIndex = 0;
boolean obstacle = false;
double pingMax = 8;
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

char readByties[4];
double goal[2] = {0, 0};
double prevGoal[2] = {0,0};
boolean firstGoal = true;
boolean notGoal = true;
boolean wallFollowed = false;
boolean objectGrabbed = false;

void setup()
{
  sparki.servo(SERVO_CENTER);
  sparki.clearLCD();
  sparki.gripperOpen();
  delay(5000);
  sparki.gripperStop();
  Serial1.begin(9600);
  while(Serial1.available() < 4);
  Serial1.readBytes(readByties, 4);
  positionIndex = *(int *)&readByties;
  sendOK();
  firstGoal = true;
  notGoal = true;
  wallFollowed = false;
  objectGrabbed = false;
}

//Recieves the location to move to and starts moving there obstacle avoiding and localize if necessary
//Picking up the object if the goal location is reached
void loop()
{
    if(positionIndex > 0)
    {
      if(!wallFollowed)
      {
        prevGoal[0] = goal[0];
        prevGoal[1] = goal[1];
        while(Serial1.available() < 4);
        Serial1.readBytes(readByties, 4);
        goal[0] = *(float *)&readByties;
        while(Serial1.available() < 4);
        Serial1.readBytes(readByties, 4);
        goal[1] = *(float *)&readByties;
        while(Serial1.available() < 4);
        Serial1.readBytes(readByties, 4);
        if(*(float *)&readByties)
        {
          notGoal = false;
        }
      }
      p.update();
      obstacle = false;
      double turnDegrees = getToDegrees(goal[0], goal[1], p.getCenter().x, p.getCenter().y);
      sparki.moveLeft(turnDegrees);
      p.update();
      sparki.clearLCD();
      sparki.moveStop();
      //if(!firstGoal)
      //{
        sparki.moveForward(4);
        p.update();
      //}
      sparki.moveForward();
      boolean left = false;
      boolean right = false;
      int lThreshold = 600;
      int rThreshold = 750;
      while(!atGoal() && !obstacle)
      {
        p.update();
        ping = sparki.ping();
        double dist = findDist(goal[0], goal[1], p.getCenter().x, p.getCenter().y);
        if(!objectGrabbed)
        {
          if(ping <= pingMax && !notGoal && dist > (pingMax + 10))
            obstacle = true;   
          else if(ping <= pingMax && notGoal)
            obstacle = true;
        }
          
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
        }
        else if(pingRight < rThreshold && !right)
        {
          right = true;
          p.update();
          sparki.moveStop();
          setRSensorXY(p.getCenter().x, p.getCenter().y, p.getAngle());
          sparki.moveForward();
          sparki.beep();
        }
        if(left && right)
        {
        localize();
        break;
        }
     }
     p.update();
     
     sparki.moveStop();
     if(obstacle)
     {
       wallFollow();
       wallFollowed = true;
     }
     else
     {
       if(!notGoal && !objectGrabbed)
       {
         sparki.gripperClose();
         delay(5000);
         sparki.gripperStop();
         sparki.moveForward(10);
         sparki.moveBackward(10);
         p.update();
         objectGrabbed = true;
       }
       firstGoal = false;
       wallFollowed = false;
       positionIndex--;
       delay(10000);
       sendOK();
     }
   }
   else
   {
     sparki.moveForward(10);
     sparki.gripperOpen();
     delay(5000);
     sparki.gripperStop();
     sparki.moveBackward(10);
     p.update();
     sparki.moveRight(90);
     p.update();
     delay(5000);
     setup();
   }
}

//Sends a 1 to the python script to request the next location to move to
void sendOK()
{
  Serial1.write(1);
  Serial1.flush(); 
}

//Checks to see if you are at the goal
boolean atGoal()
{
    if(findDist(goal[0], goal[1], p.getCenter().x, p.getCenter().y) <= .5)
      return true;
    return false;
}

//Wall follows an obsatcle until reaching the m-line on the other side
void wallFollow()
{    
    while(true)
    {
      if(obstacle)
      {
        sparki.moveLeft(90);
        p.update();
        obstacle = false;
      }
      sparki.servo(75);
      sparki.moveForward();
      delay(1200);
      p.update();
      sparki.moveStop();
      do
      {
        ping = sparki.ping();
      }
      while(ping == -1);
      if(ping > pingMax)
        turnMe(ping, false);
      else if(ping < pingMax)
        turnMe(ping, true);
      
      double d = mLineDistance();
      sparki.servo(SERVO_CENTER);
      delay(500);
      ping = sparki.ping();
      
      if(d <= 2) 
      {
        sparki.servo(SERVO_CENTER);
        break;
      }
    }
}

//Calculates how much to turn when wall following
void turnMe(int obstacleDistance, boolean left)
{
   double degreesToTurn;
   obstacleDistance = fabs(obstacleDistance - pingMax);
   if(obstacleDistance > 70)
     degreesToTurn = 0.361;
   else if(obstacleDistance > 50)
     degreesToTurn = 0.261;
   else if(obstacleDistance > 25)
     degreesToTurn = 0.161;
   else
     degreesToTurn = 0.061;
   
   if(left)
     sparki.moveLeft(degreesToTurn*180/PI);
   else
     sparki.moveRight(degreesToTurn*180/PI);

   sparki.moveStop();
   p.update();
}

//Find the distance from Sparki to the m-line
double mLineDistance()
{
   float top = fabs(((goal[1] - prevGoal[1])*p.getCenter().x) - 
               ((goal[0] - prevGoal[0])*p.getCenter().y) + (goal[0]*prevGoal[1]) - (goal[1]*prevGoal[0]));
   float bot = sqrt(((goal[1] - prevGoal[1]) * (goal[1] - prevGoal[1])) + 
                   ((goal[0] - prevGoal[0]) * (goal[0] - prevGoal[0])));
   return top/bot;
}

//Localize on a landmark and update Sparkis current angle and location
void localize()
{
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
  
  sparki.beep();
  sparki.moveStop();
  p.setCenter((point){goal[0], goal[1]});
  p.setAngle(PI/2);
}

//Returns the distance between two points
float findDist(float toX, float toY, float currentX, float currentY)
{
  return sqrt(((toX-currentX)*(toX-currentX)) + ((toY-currentY)*(toY-currentY)));
}

//Find the angle necessary for sparki to look directly at the next goal/landmark
float getToDegrees(float toX, float toY, float currentX, float currentY)
{
  float phi = atan2(toY - currentY, toX - currentX);
  return ((atan2(sin(phi - p .getAngle()), cos(phi - p.getAngle()))) * 180 / PI); 
}

//Finds the two centers of the circles given two points on the circle
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

//Sets the left edge sensors (x,y) position after seeing a landmark
void setLSensorXY(float x, float y, float angle)
{
  sensorLX = x + cos(angle + .749269314) * 5.872818744;
  sensorLY = y + sin(angle + .749269314) * 5.872818744;
}

//Sets the right edge sensors (x,y) position after seeing a landmark
void setRSensorXY(float x, float y, float angle)
{
  sensorRX = x + cos(angle - .749269314) * 5.872818744;
  sensorRY = y + sin(angle - .749269314) * 5.872818744;
}
