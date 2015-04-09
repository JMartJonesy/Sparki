/*******************************************
Algorithm 2 Demo 2
Jesse Martinez
Ivan Brčić
********************************************/
#include <Sparki.h> // include the sparki library

double locs[4][2] = {{30,-21}, {21,30}, {-30,-21}, {0,0}};
double sparkiBot[5] = {0,0,0,0};
int positionIndex = 0;

void setup() // code inside these brackets runs first, and only once
{
  sparki.servo(SERVO_CENTER);
}

void loop() // code inside these brackets runs over and over forever
{
  if (positionIndex < sizeof(locs)/sizeof(locs[0])) {
    double ydiff = locs[positionIndex][1] - sparkiBot[1];
    double xdiff = locs[positionIndex][0] - sparkiBot[0];
    double gamma = atan2(ydiff, xdiff) * 180 / PI;
    double turnDegrees = gamma - sparkiBot[2];
    
    if(turnDegrees > 180)
      turnDegrees -= 360;
    else if(turnDegrees < -180)
      turnDegrees += 360;
      
    if(turnDegrees < 0) {
      sparki.moveRight();
    }
     else {
      sparki.moveLeft();
     }

    double turned = 0;
    while(turned < abs(turnDegrees)) {
      turned = abs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_DEGREE;
    }
    sparki.moveStop();
    sparkiBot[2] = gamma;
    sparkiBot[3] = sparki.totalTravel(0);
    
    double moveDistance = sqrt(pow(ydiff, 2) + pow(xdiff, 2));
    double moved = 0;
    sparki.moveForward();
    while(moved < moveDistance) {
      moved = abs(sparki.totalTravel(0) - sparkiBot[3]) / STEPS_PER_CM;
    }
    
    sparki.moveStop();
    sparkiBot[0] = locs[positionIndex][0];
    sparkiBot[1] = locs[positionIndex][1];
    sparkiBot[3] = sparki.totalTravel(0);
    positionIndex++;
  }
  
}
