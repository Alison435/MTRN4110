// Added in direction actuation commands for robot 
// from Liam's original planning/floodfill code

//////////////////////////////////

//PUT SERIAL MONITOR IN NEWLINE MODE

//MUST PRESS ENTER AFTER EACH INPUT

//////////////////////////////////

#define ROWS_wall 5
#define COLS_wall 9
#define H_size (ROWS_wall+1)*COLS_wall  //54
#define V_size ROWS_wall*(COLS_wall+1) //50
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

//MAZE STUFF
/////////////

int horizontalWalls[ROWS_wall+1][COLS_wall];
int verticalWalls[ROWS_wall][COLS_wall+1];


//// vertical walls array
//  int verticalWalls[ROWS_wall][COLS_wall + 1] = { {1,0,0,0,1,0,0,0,0,1}, 
//                                                  {1,1,1,0,0,1,0,1,1,1}, 
//                                                  {1,0,0,1,2,2,1,1,1,1}, 
//                                                  {1,0,0,0,1,2,2,2,2,1}, 
//                                                  {1,0,1,0,0,0,1,0,1,1}, 
//                                                                };
//                                     //verticalWalls[5][10]
//                                                                                  
//  // horizontal walls array                                                 
//  int horizontalWalls[ROWS_wall + 1][COLS_wall]  = { {1,1,1,1,1,1,1,1,1}, 
//                                                     {0,0,1,1,0,1,1,2,2}, 
//                                                     {0,1,0,1,1,1,2,2,1},
//                                                     {1,0,1,1,0,1,2,2,2},
//                                                     {0,1,0,1,0,1,0,1,0},
//                                                     {1,1,1,1,1,1,1,1,1}
//                                                    
//                                                     };            //horizontalWalls[6][9]
                                                     

//// vertical walls array
//  int verticalWalls[ROWS_wall][COLS_wall + 1] = { {1,0,0,0,0,0,0,0,0,1}, 
//                                                  {1,0,0,0,0,0,0,0,0,1}, 
//                                                  {1,0,0,0,0,0,0,0,0,1}, 
//                                                  {1,0,0,0,0,0,0,0,0,1}, 
//                                                  {1,0,0,0,0,0,0,0,0,1}, 
//                                                                };
//                                     //verticalWalls[5][10]
//                                     
//                                                                                  
//  // horizontal walls array                                                 
//  int horizontalWalls[ROWS_wall + 1][COLS_wall]  = { {1,1,1,1,1,1,1,1,1}, 
//                                                     {0,0,0,0,0,0,0,0,0}, 
//                                                     {0,0,0,0,0,0,0,0,0},
//                                                     {0,0,0,0,0,0,0,0,0},
//                                                     {0,0,0,0,0,0,0,0,0},
//                                                     {1,1,1,1,1,1,1,1,1}
//                                                    
//                                                     };            //horizontalWalls[6][9]

//// vertical walls array
//  int verticalWalls[ROWS_wall][COLS_wall + 1] = { {1,0,0,0,0,0,0,0,0,1}, 
//                                                  {1,0,0,0,0,0,0,0,0,1}, 
//                                                  {1,0,0,0,1,1,0,0,0,1}, 
//                                                  {1,0,0,1,1,1,0,0,0,1}, 
//                                                  {1,1,1,0,0,1,0,0,0,1}, 
//                                                                };
//                                     //verticalWalls[5][10]
//                                     
//                                                                                  
//  // horizontal walls array                                                 
//  int horizontalWalls[ROWS_wall + 1][COLS_wall]  = { {1,1,1,1,1,1,1,1,1}, 
//                                                     {0,0,0,0,0,0,0,0,0}, 
//                                                     {0,0,0,0,1,0,0,0,0},
//                                                     {1,1,1,0,0,0,0,0,0},
//                                                     {0,1,0,1,0,0,0,0,0},
//                                                     {1,1,1,1,1,1,1,1,1}
//                                                    
//                                                     };            //horizontalWalls[6][9]
                       

//Mouse Stuff  
int mouseSRow;
int mouseSColumn;
int mouseHeading;

//Store values from flood fill
int values[ROWS_wall][COLS_wall];
int path[ROWS_wall][COLS_wall];

//ONLY FOR 3X3 MAZE FOR NOW

int goalRow = 2;
int goalCol = 4;

//Maximum value is the goal value
int Max_Value_Goal;

char commandArray[(ROWS_wall*ROWS_wall)]; //used to intepret results
int command_i = 0;
int x = 0;

///STORING VALUES STUFF FROM SERIAL INPUT
//////////////
//arrays to store stuff in
int array_H[H_size];
int array_V[V_size];

//var from serial
unsigned int integerValue=0;  // Max value is 65535
char incomingByte;

int h = 0;
int v = 0;


////////////////////////////////////
//Function Declarations
void Print_Maze();
void Get_Maze_Layout();
void Get_Robot_Pos();
void Get_Start();
void Get_Dir();
void flood_fill();
void print_value();
/////////////////////////////

                                                   
void setup() {


  //Setup Serial
  Serial.begin(9600);
  Serial3.begin(9600);

  //GOOD
  //get maze layout
  Get_Maze_Layout();
  
  //GOOD
  //get intial mouse direction
  Get_Dir();
  
  //GOOD
  //get initial starting position
  Get_Start();

  //GOOD
  //carry out flood fill on maze
  flood_fill();

  //print maze
  Print_Maze();

  digitalWrite(13,LOW);
}

///////////////////////////////////
//Function Definitions
///////////////////////////////////

//get maze layout
void Get_Maze_Layout(){

  // Parsing entire line through serial (testing horz and vert maze input)
  // and filling in array_H and array_V (used for Floodfill)
  
  #define MAX_INPUT_HORZ 54
  #define MAX_INPUT_VERT 50

  // Counters for horizontal and vertical input arrays
  int ihorz = 0;
  int ivert = 0;

  // Flags for when input string is completed
  int allHorzWalls = 0;
  int allVertWalls = 0;    

  int integerValue = 0;
  //result * 10 + ( num[i] - '0' );

  while (allVertWalls == 0)
  {
    while (Serial3.available () > 0)
    {
      char detectedWallV = Serial3.read();
      integerValue = 0;
      switch (detectedWallV)
      {  
        case('\n'):         
          //array_V[ivert] = 0; //null terminator
          allVertWalls = 1;
          break;

        case '\r':   // discard carriage return
          break;
                    
        default:
          if (ivert < MAX_INPUT_VERT)
            integerValue *= 10;
            integerValue = ((detectedWallV - '0') + integerValue);
            array_V[ivert++] = integerValue; //fill in array_V            
          break;
       }
    }  
  }
  Serial.println("Vertical DONE");

  while (allHorzWalls == 0)
  {
    while (Serial3.available () > 0)
    {
      char detectedWallH = Serial3.read();
      integerValue = 0;
      switch (detectedWallH)
      {  
        case('\n'):         
          //array_H[ihorz] = 0; //null terminator
          allHorzWalls = 1;
          break;

        case '\r':   // discard carriage return
          break;
                             
        default:
          if (ihorz < MAX_INPUT_HORZ)
            integerValue *= 10;
            integerValue = ((detectedWallH - '0') + integerValue);
            array_H[ihorz++] = integerValue; //fill in array_H           
          break;
       }
    }  
  }
  
  Serial.println("Horizontal DONE");
  
/////////////////////////////////////

  //print array_V
  v = 0 ;
  while(v<V_size){
      v++;
  }


  //Print array contents
  //print array_H
  h = 0 ;
  while(h< H_size){
      h++;
  }
 

/////////////////////////////////////


////Convert array to suitable size array
////store values

      //horizontal
      for(int row = 0; row < ROWS_wall+1; row++){
        for(int col = 0; col < COLS_wall; col++){
          if(x != H_size){
            horizontalWalls[row][col] = array_H[x++];
          }
        }
      }
      x=0;
      //vertical
      for(int row = 0; row < ROWS_wall; row++){
        for(int col = 0; col < COLS_wall + 1; col++){
          if(x != V_size){
            verticalWalls[row][col] = array_V[x++];
          }
        }
      }

}

//Printing the Maze
void Print_Maze(){

bool cellMoved;
int value = 0;
int CurrRowCheck = 2;
int CurrColCheck = 4;
//
////keep track of the column and row of every two cells 
int prevRow = 2;
int prevCol = 4;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
//FINDING PATH
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

while (CurrRowCheck != mouseSRow || CurrColCheck != mouseSColumn)
{
           for(int i = 0;i < 2*ROWS_wall+1;i++)
           {          
            for(int j = 0;j < 2*COLS_wall+1;j++)
            {

             //Add Flood Fill Values
                if(i%2 == 1 && j%2== 1){
                    if((i-1)/2 == mouseSRow && (j-1)/2 == mouseSColumn)
                    {

                    } else
                    {
                        //represents the current cell the for loops are checking
                        value = values[(i-1)/2][(j-1)/2]; 

                        // in flood fill reverse path from goal
                        if(value <= Max_Value_Goal){
 
                            if (cellMoved == true); cellMoved = false;

                            if(((i-1)/2) == CurrRowCheck && ((j-1)/2) == CurrColCheck && (CurrRowCheck != mouseSRow || CurrColCheck != mouseSColumn))
                            {
                                // if a valid curr row and curr col. (tracker for the Cells)
                                values[CurrRowCheck][CurrColCheck] = 'p';  

                                
                                // ----------------CHECK WEST------------
                                if(verticalWalls[CurrRowCheck][CurrColCheck] == 0){ 
                                                    //Serial.print("!"); //check if direction was valid
                                    if(values[CurrRowCheck][CurrColCheck - 1] == (value - 1))
                                    {

                                        path[CurrRowCheck][CurrColCheck] = value;
                                        CurrRowCheck = CurrRowCheck;
                                        CurrColCheck = CurrColCheck - 1; //move to west cell                                                             
                                        //Serial.print(CurrRowCheck); Serial.print(CurrColCheck);
    
                                          //Serial.print(CurrRowCheck); Serial.print(CurrColCheck);
                                        if (cellMoved == false) {
                                          cellMoved = true; //Serial.print("_W");
                                          commandArray[command_i++] = 'W';
                                        }
                                                     
                                    }
                                }

                              // ----------------CHECK NORTH------------
                                //check NORTH
                                //only check if the walls or not there
                                if(horizontalWalls[CurrRowCheck][CurrColCheck] == 0){
                                 // Serial.print("#"); //check if direction was valid
                                    //check if the north
                                    if(values[CurrRowCheck-1][CurrColCheck] == value - 1 ){                                        
                                            path[CurrRowCheck][CurrColCheck] = value;
                                            CurrRowCheck = CurrRowCheck - 1;
                                            CurrColCheck = CurrColCheck;
                                            if (cellMoved == false)
                                            {
                                              cellMoved = true;// Serial.print("_N");
                                              commandArray[command_i++] = 'N' ;
                                            }  
                                      }                              
                                  }

                               // ----------------CHECK  EAST------------
                                if(verticalWalls[CurrRowCheck][CurrColCheck+1] == 0){
                                       //Serial.print("E"); //check if direction was valid

                                    if(values[CurrRowCheck][CurrColCheck + 1] == (value - 1) ){ //right of current robot position
                                            path[CurrRowCheck][CurrColCheck] = value;
                                            CurrRowCheck = CurrRowCheck;
                                            CurrColCheck = CurrColCheck + 1;  //move to east cell    
                                            if (cellMoved == false){
                                              cellMoved = true;// Serial.print("_E");
                                              commandArray[command_i++] = 'E';
                                            }
                                    }
                                }

                                
                                 // ----------------CHECK SOUTH------------
                                if(horizontalWalls[CurrRowCheck + 1][CurrColCheck] == 0){
                                             //Serial.print("S"); //check if direction was valid
                                    if(values[CurrRowCheck + 1][CurrColCheck] == value - 1 ){                                      
                                          path[CurrRowCheck][CurrColCheck] = value;
                                          CurrRowCheck = CurrRowCheck + 1;
                                          CurrColCheck = CurrColCheck;
                                            if (cellMoved == false)
                                            {
                                              cellMoved = true;// Serial.print("_S");
                                              commandArray[command_i++] = 'S';
                                            }
                                    }
                                }                                                                                                                      
                            }
                        }  
                    }          
                                                          
                }
                             
            }
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
//PRINTING
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

           for(int i = 0;i < 2*ROWS_wall+1;i++)
           {
           
            for(int j = 0;j < 2*COLS_wall+1;j++)
            {
                //Add Horizontal Walls
                if(i%2 == 0 && j%2 == 1)
                {
                    if(horizontalWalls[i/2][j/2] == true)
                    {
                        Serial.print(" -----");
                    } else if(horizontalWalls[i/2][j/2] == 2){
                         Serial.print(" *****");
                    }
                    else
                    {
                        Serial.print("      ");
                    }
                }

                //Add Vertical Walls
                if(i%2 == 1 && j%2 == 0)
                {
                    if(verticalWalls[i/2][j/2] == true)
                    {      
                            Serial.print("|  ");
                        }else if(verticalWalls[i/2][j/2] == 2){
                                Serial.print("*  ");
                        }
                        else
                        {
                            Serial.print("   ");                    
                    }
            }

             //Add Flood Fill Values
                if(i%2 == 1 && j%2== 1){
                    if((i-1)/2 == mouseSRow && (j-1)/2 == mouseSColumn)
                    {
                        if(mouseHeading == NORTH)
                        {
                            Serial.print("^");
                            Serial.print("  "); 
                        }
                        else if(mouseHeading == EAST)
                        {
                            Serial.print(">");
                            Serial.print("  ");
                        }
                        else if(mouseHeading == SOUTH)
                        {
                            Serial.print("v");
                            Serial.print("  ");
                        }
                        else if(mouseHeading == WEST)
                        {
                            Serial.print("<");
                            Serial.print("  ");
                        }
                    }
                    else
                    {
                        if((i-1)/2 == goalRow && (j-1)/2 == goalCol){
                            if(path[(i-1)/2][(j-1)/2] < 10){
                                Serial.print("X");
                                Serial.print("  "); 
                            } else {
                                Serial.print("X");
                                Serial.print("  ");                                  
                            }                                                        
                        } else {
                            if (path[(i-1)/2][(j-1)/2] != 0)
                            {
                                //print path values
                                if(path[(i-1)/2][(j-1)/2] < 10){
                                    Serial.print(path[(i-1)/2][(j-1)/2]);
                                    Serial.print("  "); 
                                } else {
                                    Serial.print(path[(i-1)/2][(j-1)/2]);
                                    Serial.print(" ");                                  
                                }                                             
                            } else {
                                  if(path[((i-1)/2)][((j-1)/2)] < 10){
                                      Serial.print(" ");
                                      Serial.print("  ");                                  
                                  } else {
                                      Serial.print("  ");
                                      Serial.print(" ");                                  
                                  }                                     
                            }                          
                        }
                    }
                }                                                                                                
            }
             Serial.print("\n");           
        }
        Serial.print("\n");       
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

void Get_Dir(){

  int direction;
  h=0;
  while(h<1){
      if (Serial3.available() > 0) {   // something came across serial
        integerValue = 0;         // throw away previous integerValue
        while(1) {            // force into a loop until 'n' is received
          incomingByte = Serial3.read();
          if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
          if (incomingByte == '\r') break;   // exit the while(1), we're done receiving
          if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
          integerValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          integerValue = ((incomingByte - 48) + integerValue);
        }
        direction = integerValue;  
        h++;
    }
    if(direction == 0){
        mouseHeading = NORTH;
    } else if(direction == 1){
        mouseHeading = EAST;
    } else if(direction == 2){
        mouseHeading = SOUTH;
    } else if(direction == 3){
        mouseHeading = WEST;
    }
  }

    Serial.print("Direction DONE \n");

}

void Get_Start(){
  
//    //Enter the row position of mouse
  h=0;
  while(h<1){
      if (Serial3.available() > 0) {   // something came across serial
        integerValue = 0;         // throw away previous integerValue
        while(1) {            // force into a loop until 'n' is received
          incomingByte = Serial3.read();
          if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
          if (incomingByte == '\r') break;   // exit the while(1), we're done receiving
          if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
          integerValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          integerValue = ((incomingByte - 48) + integerValue);
        }
        mouseSRow = integerValue;  
        h++;
    }
  }
  Serial.print("StartRow DONE \n");

    //Enter the column position of mouse
  h=0;
  while(h<1){
      if (Serial3.available() > 0) {   // something came across serial
        integerValue = 0;         // throw away previous integerValue
        while(1) {            // force into a loop until 'n' is received
          incomingByte = Serial3.read();
          if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
          if (incomingByte == '\r') break;   // exit the while(1), we're done receiving
          if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
          integerValue *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
          integerValue = ((incomingByte - 48) + integerValue);
        }
        mouseSColumn = integerValue;  
        h++;
    }
  }
  //Serial.print(mouseSRow);
  //Serial.print(mouseSColumn);
  Serial.print("StartCol DONE \n");

}

void flood_fill(){
  //start from mouse start postion then flood
  int row = 0 ;
  int col = 0;
  int mazeValueChanged = 1;
  int CurrentExploredValue = 0;
  int N = (ROWS_wall*COLS_wall)-1;
  
  //set all cell values to 8 (unexplored)
  //Set start cell to 0
  for(int row = 0;row < ROWS_wall; row++){
    for(int col = 0; col < COLS_wall; col++){
       if(row == mouseSRow && col == mouseSColumn){
         values[mouseSRow][mouseSColumn] = 0;
       }else {
         values[row][col] = N;
       }
    }
  }
 
//  //Do floodfill
  while(mazeValueChanged != 0){
    mazeValueChanged = 0;
    for(int FloodR = 0; FloodR < ROWS_wall; FloodR++){
        for(int FloodC = 0; FloodC < COLS_wall; FloodC++){
            if(values[FloodR][FloodC] == CurrentExploredValue){
                //check NORTH
                if(horizontalWalls[FloodR][FloodC] == 0){
                    if(values[FloodR-1][FloodC] == N){
                        values[FloodR-1][FloodC] = values[FloodR][FloodC] + 1;
                        mazeValueChanged = 1;
                    }
                }
                
                if(verticalWalls[FloodR][FloodC+1] == 0){ //check EAST
                    if(values[FloodR][FloodC+1] == N){
                        values[FloodR][FloodC+1] = values[FloodR][FloodC]+1;
                        mazeValueChanged = 1;
                    }
                }
                
                if(horizontalWalls[FloodR+1][FloodC] == 0){ //checl SOUTH
                    if(values[FloodR+1][FloodC] == N){
                        values[FloodR+1][FloodC] = values[FloodR][FloodC]+1;
                        mazeValueChanged = 1;
                    }
                }
                
                if(verticalWalls[FloodR][FloodC] == 0){ //check WEST
                    if(values[FloodR][FloodC-1] == N){
                        values[FloodR][FloodC-1] = values[FloodR][FloodC]+1;
                        mazeValueChanged = 1;                 
                    }
                }   
            }          
        }
    }
    CurrentExploredValue += 1;
  }

  //Store value of goal
  Max_Value_Goal = values[goalRow][goalCol];


//////////////////////////
//testing
//print contents of values array

 for(int row = 0;row < ROWS_wall; row++){
    for(int col = 0; col < COLS_wall; col++){
         Serial.print(values[row][col]);
    }
 } 

//Print CurrentExploredValue at end
    Serial.print("\n");
//  Serial.print( CurrentExploredValue);
//  Serial.print("\n");


//Print value of goal
//  Serial.print(Max_Value_Goal);
//  Serial.print("\n"); 
///////////////////////////////

}

// Parameters for Robot to act on Floodfill algorithm results
#define MAX_INPUT 45
char robotMovements[MAX_INPUT];
char moveChar;
int move_i = 0;

void loop()
{

// Serial.println("COORDINATES OF VALID PATH:");
// for(int row = 0;row < ROWS_wall; row++){
//    for(int col = 0; col < COLS_wall; col++){
//         if (values[row][col] == 'p')
//         {
//          Serial.print("{");
//          Serial.print(row); Serial.print(","); Serial.print(col);
//          Serial.print("} ");
//         }
//    }
// }


     for (int k = Max_Value_Goal; k >= 0; k--)
     {      
          if(commandArray[k] == 'N'){
              if(commandArray[k-1] == 'W' ){
                  //Serial.print("forward"); Serial.print("   ");  
                  robotMovements[move_i++] = '^';                   
                  //Serial.print("Turn Left."); Serial.print("   ");
                  robotMovements[move_i++] = '<';                   
              
              } else if(commandArray[k-1] == 'E'){
                  //Serial.print("forward"); Serial.print("   ");
                  robotMovements[move_i++] = '^';                   
                  //Serial.print("Turn Right."); Serial.print("   "); 
                  robotMovements[move_i++] = '>';                    
                                                    
              } else if(commandArray[k-1] == 'N'){
                  //Serial.print("forward."); Serial.print("   ");                 
                  robotMovements[move_i++] = '^';                   
              }else {
                  //Serial.print("forward."); Serial.print("   ");
                  robotMovements[move_i++] = '^';                   
              }   
//////////////////////////////////////////////    
          } else if(commandArray[k] == 'S'){
              if(commandArray[k-1] == 'W' ){
                  
                  //Serial.print("forward"); Serial.print("   ");   
                  robotMovements[move_i++] = '^';                   
                  //Serial.print("Turn Right."); Serial.print("   ");
                  robotMovements[move_i++] = '>';                   
           
              } else if(commandArray[k-1] == 'E'){
                  //Serial.print("forward"); Serial.print("   ");                   
                  robotMovements[move_i++] = '^';                   
                  //Serial.print("Turn Left."); Serial.print("   ");  
                  robotMovements[move_i++] = '<';                   
              
              } else if(commandArray[k-1] == 'S'){
                  //Serial.print("forward."); Serial.print("   ");                 
                  robotMovements[move_i++] = '^';                   
              }else {
                  //Serial.print("forward."); Serial.print("   ");                 
                  robotMovements[move_i++] = '^';                   
              }   
//////////////////////////////////////////////               
          } else if(commandArray[k] == 'E'){
              if(commandArray[k-1] == 'N' ){
                  //Serial.print("forward"); Serial.print("   ");
                  robotMovements[move_i++] = '^';                   
                  //Serial.print("Turn Left."); Serial.print("   ");  
                  robotMovements[move_i++] = '<';                   
               
              } else if(commandArray[k-1] == 'S'){
                  //Serial.print("forward"); Serial.print("   ");
                  robotMovements[move_i++] = '^';                   
                  //Serial.print("Turn Right."); Serial.print("   "); 
                  robotMovements[move_i++] = '>';                    
                
              } else if(commandArray[k-1] == 'E'){
                  //Serial.print("forward."); Serial.print("   ");                                
                  robotMovements[move_i++] = '^';                   
              }else {
                  //Serial.print("forward."); Serial.print("   ");    
                  robotMovements[move_i++] = '^';                                
              }  
//////////////////////////////////////////////                     
          } else if(commandArray[k] == 'W'){
              if(commandArray[k-1] == 'N' ){
                  //Serial.print("forward"); Serial.print("   ");
                  robotMovements[move_i++] = '^';                   
                  //Serial.print("Turn Right."); Serial.print("   "); 
                  robotMovements[move_i++] = '>';                    
              
              } else if(commandArray[k-1] == 'S'){
                  //Serial.print("forward"); Serial.print("   ");
                  robotMovements[move_i++] = '^';                   
                  //Serial.print("Turn Left."); Serial.print("   ");  
                  robotMovements[move_i++] = '<';                   
                 
              } else if(commandArray[k-1] == 'W'){
                  //Serial.print("forward."); Serial.print("   ");                                
                  robotMovements[move_i++] = '^';                   
              }else {
                  //Serial.print("forward."); Serial.print("   ");    
                  robotMovements[move_i++] = '^';                                
              }      
          }  
     }
     
  // Interpret commands/direction sequence (for Robot actuation)  
  
  Serial.println("--COMMANDS READY--");
          
  // Iterate through command Array
  for (int i = 0; i < MAX_INPUT; i++)
  {
    char j = robotMovements[i];
      
    switch (j)
    {  
      case('^'):
        Serial.print("^");
        //resetAllEncoders();
        //robotForward(STRAIGHT_DISTANCE,30.0);
        //resetAllEncoders();
        delay(15);        
        break;
    
      case('>'):
        Serial.print(">");
        //resetAllEncoders();
        //robotTurn(1);
        //resetAllEncoders();
        delay(15);        
        break;
    
      case('<'):        
        Serial.print("<");
        //resetAllEncoders();
        //robotTurn(0);
        //resetAllEncoders();
        delay(15);        
        break;
                
     default:
       break;
    }
    delay(50);             
  }

  Serial.println("");
  while (true) {};
  
}
