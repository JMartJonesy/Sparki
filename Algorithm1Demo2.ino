/*******************************************
Algorithm 1 Demo 2
Jesse Martinez
Ivan Brčić
********************************************/
#include <Sparki.h> // include the sparki library

void setup()
{
  double sparkiBot[3] = {0,0,0}; //X,Y,Angle
  double locs[5][2] = {{3,3}, {2,2}, {-1,-1}, {-1,2}, {0,0}};
  
  sparki.clearLCD();
  
  for( int i = 0; i < (sizeof(locs)/sizeof(locs[0])); ++i )
  {
    double ydiff = locs[i][1] - sparkiBot[1];
    double xdiff = locs[i][0] - sparkiBot[0];
    double gamma = atan2(ydiff, xdiff) * 180 / PI;
    double turnDegrees = gamma - sparkiBot[2];
    
    sparkiBot[2] = gamma;
    
    if(turnDegrees > 180)
      turnDegrees -= 360;
    else if(turnDegrees < -180)
      turnDegrees += 360;
      
    sparki.print("Turning ");
    sparki.print(turnDegrees);
    sparki.println(" Degrees");
    sparki.moveLeft(turnDegrees);
    
    double moveDistance = sqrt(pow(ydiff, 2) + pow(xdiff, 2));
    sparki.print("Moving ");
    sparki.print(moveDistance * 10);
    sparki.print(" to (");
    sparki.print(locs[i][0]);
    sparki.print(", ");
    sparki.print(locs[i][1]);
    sparki.println(")");
    
    sparki.moveForward(moveDistance);
    sparkiBot[0] = locs[i][0];
    sparkiBot[1] = locs[i][1];
    
    sparki.updateLCD();
    delay(5000);
  } 
}

void loop(){}
