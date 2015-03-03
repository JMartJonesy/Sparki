/*******************************************
Algorithm 2 Demo 2
Jesse Martinez
Ivan Brčić
********************************************/
#include <Sparki.h> // include the sparki library

int ping = 0;
int positionIndex = 0;
boolean obstacle = false;
double distance = 0;
double pingMax = 14;
double locs[5][2] = {{0,0}, {60,60}, {-20,40}, {-30,-50}, {0,0}};
double sparkiBot[4] = {0,0,0,0}; //x, y, degree, steps

void setup() // code inside these brackets runs first, and only once
{
  sparki.servo(SERVO_CENTER);
}

void loop() // code inside these brackets runs over and over forever
{
  if(positionIndex < sizeof(locs)/sizeof(locs[0])) {
    obstacle = false;
    double ydiff = locs[positionIndex][1] - sparkiBot[1];
    double xdiff = locs[positionIndex][0] - sparkiBot[0];
    double gamma = atan2(ydiff, xdiff) * 180 / PI;
    double turnDegrees = gamma - sparkiBot[2];
    
    //sparki.print("X");
    //sparki.print(sparkiBot[0]);
    //sparki.print(",Y");
    //sparki.println(sparkiBot[1]);
    //sparki.print("GAMMA");
    //sparki.println(gamma);
    //sparki.updateLCD();
    
    if(turnDegrees > 180)
      turnDegrees -= 360;
    else if(turnDegrees < -180)
      turnDegrees += 360;
      
    if(turnDegrees < 0) 
      sparki.moveRight();
    else 
      sparki.moveLeft();
     
    double turned = 0;
    while(turned < abs(turnDegrees)) 
      turned = abs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_DEGREE;
    sparki.moveStop();
    sparkiBot[2] = gamma;
    sparkiBot[3] = sparki.totalTravel(0);
   
    sparki.moveForward();
    while(!atGoal() && !obstacle) 
    {
      moveMe(); 
      ping = sparki.ping();
      if(ping <= pingMax)
        obstacle = true;
    }
    
    sparki.moveStop();
    if(obstacle)
      wallFollow();
    else if(atGoal())
    {
      //delay(10000);
      positionIndex++;
    }
    
    //sparki.println("Bottom");
  }
}

boolean atGoal()
{
  //sparki.print("X,Y:");
  //sparki.print(sparkiBot[0]);
  //sparki.print(",");
  //sparki.println(sparkiBot[1]);
  //sparki.print("GX,GY:");
  //sparki.print(locs[positionIndex][0]);
  //sparki.print(",");
  //sparki.println(locs[positionIndex][1]);
  //sparki.updateLCD();
  
  double xThreshold = abs(locs[positionIndex][0] - sparkiBot[0]);
  double yThreshold = abs(locs[positionIndex][1] - sparkiBot[1]);
  if(xThreshold <= 1 && yThreshold <=1)
    return true;
  return false;
}

void wallFollow()
{
  double a;
  double b = 1;
  double c;
  if(positionIndex != 0)
  {
    a = (-locs[positionIndex][1] - locs[positionIndex - 1][1]) / 
      (locs[positionIndex][0] - locs[positionIndex - 1][0]);
    c = (((locs[positionIndex][1] - locs[positionIndex - 1][1]) / 
      (locs[positionIndex][0] - locs[positionIndex - 1][0])) * locs[positionIndex - 1][0]) - locs[positionIndex - 1][1];
  }
  sparki.print("a:");
  sparki.println(a);
  sparki.print("c:");
  sparki.println(c);
  sparki.println("Wall Follow");
  sparki.updateLCD();
  while(true)
  {
    if(obstacle)
    {
      sparki.moveLeft(90);
      sparkiBot[2] += 90;
      obstacle = false;
    }
    sparki.servo(SERVO_RIGHT);
    sparki.moveForward();
    delay(500);
    moveMe();
    sparki.moveStop();
    ping = sparki.ping();
    if(ping > pingMax)
      turnMe(ping, false);
    else if(ping < pingMax)
      turnMe(ping, true);
    sparki.moveStop();
    sparkiBot[3] = sparki.totalTravel(0);
    sparki.print("Angle");
    sparki.println(sparkiBot[2]);
    if(mLineDistance(a,b,c) <= 3)
    {
      sparki.servo(SERVO_CENTER);
      sparki.print("HERE");
      sparki.println(sparkiBot[2]);
      sparki.updateLCD();
      delay(1000);
      break;
    }
  }
}

void moveMe()
{
  distance = abs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_CM;
  sparkiBot[0] = sparkiBot[0] + (cos(sparkiBot[2] * (PI/180)) * distance);
  sparkiBot[1] = sparkiBot[1] + (sin(sparkiBot[2] * (PI/180)) * distance);
  sparkiBot[3] = sparki.totalTravel(0);
}

void turnMe(int obstacleDistance, boolean left)
{
 double turned = 0;
 double degreesToTurn = (obstacleDistance - pingMax) / 20;
 if(left)
   sparki.moveLeft();
 else
   sparki.moveRight();
 do
 {
   turned = abs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_DEGREE;
 }
 while(turned < degreesToTurn);
 sparki.moveStop();
 if(left)
   sparkiBot[2] += turned;
 else
   sparkiBot[2] -= turned;
 sparkiBot[3] = sparki.totalTravel(0);
}

double mLineDistance(double a, double b, double c)
{
  return abs((a * sparkiBot[0]) + (b * sparkiBot[1]) + c) / sqrt((a * a) + (b * b));
}
