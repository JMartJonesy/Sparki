/*******************************************
Demo 3
Jesse Martinez
Ivan Brčić
********************************************/
#include <Sparki.h> // include the sparki library

int ping = 0;
int positionIndex = 0;
boolean obstacle = false;
double distance = 0;
double pingMax = 8;
double locs[5][2] = {{0,0}, {60, 60}, {-60, 50}, {-60, -40}, {0, 0}};
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
    
    if(turnDegrees > 180)
      turnDegrees -= 360;
    else if(turnDegrees < -180)
      turnDegrees += 360;
      
    if(turnDegrees < 0) 
      sparki.moveRight();
    else 
      sparki.moveLeft();
     
    double turned = 0;
    while(turned < fabs(turnDegrees)) 
      turned = fabs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_DEGREE;
    sparki.moveStop();
    
    sparki.clearLCD();
    sparki.println(turned);
    sparki.updateLCD();
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
      delay(1000);
      positionIndex++;
    }
    
//    sparki.println("Bottom");
  }
}

boolean atGoal()
{

  double xThreshold = fabs(locs[positionIndex][0] - sparkiBot[0]);
  double yThreshold = fabs(locs[positionIndex][1] - sparkiBot[1]);
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
    if (locs[positionIndex][0] == locs[positionIndex - 1][0]) { // If prevous and goal X are the same, to avoid division by 0
      a = 1;
      b = 0;
      c = -locs[positionIndex][0];
    } else {
      a = -(locs[positionIndex][1] - locs[positionIndex - 1][1]) / 
           (locs[positionIndex][0] - locs[positionIndex - 1][0]);
      c = (((locs[positionIndex][1] - locs[positionIndex - 1][1]) / 
            (locs[positionIndex][0] - locs[positionIndex - 1][0])) * locs[positionIndex - 1][0]) - locs[positionIndex - 1][1];
    }

    sparki.clearLCD();
    sparki.print("a: ");
    sparki.println(a);
    sparki.print("c: ");
    sparki.println(c);
    sparki.updateLCD();
  }

  while(true)
  {
    if(obstacle)
    {
      sparki.moveLeft(90);
      sparkiBot[2] += 90;
      obstacle = false;
    }
    sparki.servo(70);
    sparki.moveForward();
    delay(1200);
    moveMe();
    sparki.moveStop();
    ping = sparki.ping();
    if(ping > pingMax)
      turnMe(ping, false);
    else if(ping < pingMax)
      turnMe(ping, true);
    sparki.moveStop();
    sparkiBot[3] = sparki.totalTravel(0);

    double d = mLineDistance(a,b,c);
      
    sqrt((locs[positionIndex][1] * locs[positionIndex][1] ) + (locs[positionIndex][0] * locs[positionIndex][0]));
    sparki.print("d:.");
    sparki.println(d);
    sparki.updateLCD();
    
     sparki.servo(SERVO_CENTER);
    delay(400);
    ping = sparki.ping();
    if(ping < pingMax)
      obstacle = true;
    if(d <= 6)
    
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
  distance = fabs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_CM;
  sparkiBot[0] = sparkiBot[0] + (cos(sparkiBot[2] * (PI/180)) * distance);
  sparkiBot[1] = sparkiBot[1] + (sin(sparkiBot[2] * (PI/180)) * distance);
  sparkiBot[3] = sparki.totalTravel(0);
}

void turnMe(int obstacleDistance, boolean left)
{
 double turned = 0;
 double degreesToTurn;
 obstacleDistance = fabs(obstacleDistance - pingMax);
 if(obstacleDistance > 70)
   degreesToTurn = 15;
 else if(obstacleDistance > 50)
   degreesToTurn = 10;
 else if(obstacleDistance > 25)
   degreesToTurn = 5;
 else
   degreesToTurn = 3;
 
 if(left)
   sparki.moveLeft();
 else
   sparki.moveRight();
 do
 {
   turned = fabs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_DEGREE;
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
  return fabs((a * sparkiBot[0]) + (b * sparkiBot[1]) + c) / sqrt((a * a) + (b * b));
}
