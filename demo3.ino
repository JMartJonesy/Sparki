/*******************************************
Algorithm 2 Demo 2
Jesse Martinez
Ivan Brčić
********************************************/
#include <Sparki.h> // include the sparki library

int ping = 0;
int positionIndex = 0;
boolean obstacle = false;
boolean passed = false;
double distance = 0;
double pingMax = 16;
double locs[1][2] = {{100,100}};
double sparkiBot[5] = {0,0,0,0,0}; //x, y, degree, steps, goal Degree

void setup() // code inside these brackets runs first, and only once
{
  sparki.servo(SERVO_CENTER);
}

void loop() // code inside these brackets runs over and over forever
{
  if (positionIndex < sizeof(locs)/sizeof(locs[0])) {
    obstacle = false;
    double ydiff = locs[positionIndex][1] - sparkiBot[1];
    double xdiff = locs[positionIndex][0] - sparkiBot[0];
    double gamma = atan2(ydiff, xdiff) * 180 / PI;
    double turnDegrees = gamma - sparkiBot[2];
    
    sparkiBot[4] = gamma;
    
    sparki.print("X");
    sparki.print(sparkiBot[0]);
    sparki.print(",Y");
    sparki.println(sparkiBot[1]);
    sparki.print("GAMMA");
    sparki.println(gamma);
    sparki.updateLCD();
    
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
      if(passed)
      {
        passed = false;
        break;
      }
    }
    
    sparki.moveStop();
    if(obstacle)
      wallFollow();
    else if(atGoal())
      positionIndex++;
    
    sparki.println("Bottom");
  }
}

boolean atGoal()
{
  sparki.print("X,Y:");
  sparki.print(sparkiBot[0]);
  sparki.print(",");
  sparki.println(sparkiBot[1]);
  sparki.print("GX,GY:");
  sparki.print(locs[positionIndex][0]);
  sparki.print(",");
  sparki.println(locs[positionIndex][1]);
  sparki.updateLCD();
  
  double xThreshold = abs(locs[positionIndex][0] - sparkiBot[0]);
  double yThreshold = abs(locs[positionIndex][1] - sparkiBot[1]);
  if(xThreshold <= 2 && yThreshold <=2)
    return true;
  if(locs[positionIndex][0] - sparkiBot[0] > 2 && locs[positionIndex][0] - sparkiBot[0] > 2)
    passed = true;
  return false;
}

void wallFollow()
{
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
      moveMeRight(ping);
    sparki.servo(SERVO_CENTER);
    delay(400);
    sparki.moveStop();
    sparkiBot[3] = sparki.totalTravel(0);
    sparki.print("Angle");
    sparki.println(sparkiBot[2]);
    if(abs(sparkiBot[2] - sparkiBot[4]) <= 15)
    {
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

void moveMeRight(int obstacleDistance)
{
 double turned = 0;
 double degreesToTurn = (obstacleDistance - pingMax) / 20;
 sparki.moveRight();
 do
 {
   turned = abs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_DEGREE;
 }
 while(turned < degreesToTurn);
 sparki.moveStop();
 sparkiBot[2] -= turned;
 sparkiBot[3] = sparki.totalTravel(0);
}
