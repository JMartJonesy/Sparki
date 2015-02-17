/*******************************************
 Basic Sparki Code
 
This is the most basic code you can use to
run Sparki. 
********************************************/
#include <Sparki.h> // include the sparki library

void setup() // code inside these brackets runs first, and only once
{
    double positions[4][2] = {{2,2}, {1, 1}, {-1, -1}, {0, 0}};
    int numOfPositions = sizeof(positions)/sizeof(positions[0]); // Number of coordinates given to sparki.
    double currentPosition[2] = {0, 0};
    double currentAngle = 0;
    sparki.clearLCD();

    for (int i = 0; i < numOfPositions; i++){
      
      double gama = atan2(positions[i][1] - currentPosition[1], positions[i][0] - currentPosition[0]) * 180.0 / PI;
      double turnTo = gama - currentAngle;
      currentAngle = gama;
      
      if (turnTo > 180){
        turnTo -= 360;
      } else if (turnTo < - 180) {
        turnTo += 360;
      }
      sparki.moveLeft(turnTo);
      
      double distanceToMove = sqrt( pow(positions[i][1]-currentPosition[1], 2) + pow(positions[i][0]-currentPosition[0], 2));
      sparki.moveForward(distanceToMove * 10);
      
      currentPosition[0] = positions[i][0];
      currentPosition[1] = positions[i][1];
      
      sparki.println(turnTo);
      sparki.updateLCD();
      
    }
    
}

void loop() // code inside these brackets runs over and over forever
{

}
