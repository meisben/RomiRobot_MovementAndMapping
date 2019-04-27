#ifndef _Mapping_h
#define _Mapping_h
#include <EEPROM.h>

const byte MAP_RESOLUTION = 25;
const int MAP_X=1800;
const int MAP_Y=1800;

class Mapper
{
    public:
        void resetMap();
        void printMap();
        void updateMapFeature(byte feature, int y, int x);
        void updateMapFeature(byte feature, float y, float x);
        void returnClosestUnexploredCoOrdinates(float myPosey, float myPosex);
        void checkIfRomiIsInsideMap(int, int);
        
        int  indexToPose(int i, int map_size, int resolution);
        int  poseToIndex(int x, int map_size, int resolution);

        byte returnMapFeature(int x_index, int y_index);

        bool isTileExplored(int x_index, int y_index);

        float targetXCoordinate = 0;
        float targetYCoordinate = 0;
    
    private:
        int X_size;
        int Y_size;
        const byte MAP_DEFAULT_FEATURE = '#';
};

void Mapper::resetMap()
{

    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for (int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            
            if (eeprom_address > 1023)
            {
                Serial.println(F("Error: EEPROM Address greater than 1023"));
            }
            else
            {
                EEPROM.update(eeprom_address, MAP_DEFAULT_FEATURE );
                
            }
        }
    }

}

void Mapper::printMap()
{

    Serial.println("Map");
    Serial.println("---");
    Serial.println("KEY --->> # = unexplored | O = object | L = line | . = empty space");
    Serial.println("");
    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            Serial.print( (char)value );
            Serial.print(" ");
        }
        Serial.println("");
    }
  
}

int Mapper::poseToIndex(int x, int map_size, int resolution)
{
    return x / (map_size / resolution);
}

int Mapper::indexToPose(int i, int map_size, int resolution)
{
    return i* (map_size / resolution);
}


void Mapper::updateMapFeature(byte feature, float y, float x) {
  updateMapFeature( feature, (int)y, (int)x );  
}

void Mapper::updateMapFeature(byte feature, int y, int x)
{
    if (x > MAP_X || x < 0 || y > MAP_Y || y < 0)
    {
//      Serial.print(F("Error:Invalid co-ordinate mssg1"));
//      Serial.print("  |  eX, eY = ");
//      Serial.print(x);
//      Serial.print(", ");
//      Serial.println(y);
      return;
    }

    int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
    int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);  

    int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;  

    if (eeprom_address > 1023)
    {
        Serial.println(F("Error: EEPROM Address greater than 1023"));
    }
    else
    {
        EEPROM.update(eeprom_address, feature);
    }
        

}


byte Mapper::returnMapFeature(int x_index, int y_index)
{ 
  int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;  
  byte value;
  value = EEPROM.read(eeprom_address);//, value);
//  Serial.print("Map feature in current position = ");
//  Serial.print( (char)value );
//  Serial.println(" ");
  return value;
}

// check if the tile with passed x and y co-ordinates is Explored
bool Mapper::isTileExplored(int x_index, int y_index)
{
  bool tileExplored;
  byte value;
  value = returnMapFeature(x_index, y_index);
  if(value == '#'){
    tileExplored = false;
  }
  else{
    tileExplored = true;
  }
  return tileExplored;  
}


void Mapper::returnClosestUnexploredCoOrdinates(float myPoseX, float myPoseY)
{
    int x = int(myPoseX);
    int y = int(myPoseY); //not used yet!

    
    if (x > MAP_X || x < 0 || y > MAP_Y || y < 0)
    {
      Serial.println(F("Error:Invalid co-ordinate mssg2"));
      return;
    }

    int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
    int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);  

    int orig_eeprom_address = (x_index * MAP_RESOLUTION) + y_index;

    //Some variables which are used searching for unexplored tiles
    bool foundIt = 0;
    int unexploredGridCellAddress = 0;
    
    // while loop used to find next unexplored tile to travel to

    byte i = random(0,MAP_RESOLUTION+1);
    byte j = random(0,MAP_RESOLUTION+1);
        
    while(i<MAP_RESOLUTION && foundIt == 0){
      while(j<MAP_RESOLUTION && foundIt == 0){

        int eeprom_address = (i*MAP_RESOLUTION)+j;
        byte value;
        value = EEPROM.read(eeprom_address);//, value);

        if(value == '#'){
              unexploredGridCellAddress = eeprom_address;
              foundIt = 1;              
            }

        j++;
      }
      i++;
    }
    

    int new_x_index = unexploredGridCellAddress / 25;
    int new_y_index = unexploredGridCellAddress % 25;

    targetXCoordinate = indexToPose(new_x_index, MAP_X, MAP_RESOLUTION);
    targetYCoordinate = indexToPose(new_y_index, MAP_Y, MAP_RESOLUTION);

    //Some debugging print statements (can leave these commented out!)

    Serial.println("");
    Serial.println(" ------ returnClosestUnexploredCoOrdinates ----- ");
    Serial.print("new_x_index = ");
    Serial.print(new_x_index);
    Serial.print(" | new_y_index = ");
    Serial.println(new_y_index);
    Serial.print("targetXCoordinate = ");
    Serial.print(targetXCoordinate);
    Serial.print(" | targetYCoordinate = ");
    Serial.println(targetYCoordinate);
    Serial.println(" ------ ------  ------ ----- ");
    Serial.println("");

}



#endif
