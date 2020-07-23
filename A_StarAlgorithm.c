  // Author  Jishnu Mohan P R
  // P2RAU17010 
  // for mobile robotics project 
  

  #include<SoftwareSerial.h>
  
  #define L1 30     // Left motor
  #define L2 22
  #define R1 32     // Right motor
  #define R2 24
  
  #define E1 11
  #define E2 10
          
  int X = 0;     // Starting X and Y coordinates of Robot
  int Y = 0;
  int ROW = 7;    // size of Grid
  int COL = 7;    // size of Grid
  int dx = 7;   // Destination X Coordinate
  int dy = 7;   // Destination Y Coordinate
  int roface = 4; // 1-East, 2-West, 3-North, 4-South
                 // Columns  0,1,2,3,4,5,6,7    Rows
  int Environment [8][8] = {{1,1,0,0,0,0,0,0},//0           // 1 --> no obstacle    0 --> obstacle
                            {1,1,0,0,0,0,0,0},//1
                            {1,1,1,1,1,1,1,1},//2
                            {1,0,1,0,0,0,0,1},//3
                            {1,1,1,1,1,0,0,1},//4
                            {0,0,0,0,1,0,0,1},//5
                            {1,0,0,0,1,1,1,1},//6
                            {1,1,1,1,1,1,1,1}};//7
                           //0,1,2,3,4,5,6,7
  int for_delay = 500;
  int rev_delay = 500;
  int turn_delay = 500;;
//  int lowestf = 9999;     //starting with the lowest being highest
 struct NODE{
  bool onopen;
  bool onclosed;
  int g;
  double h;
  double f;
  int parentx;
  int parenty;
} node[8][8];
struct nod{              // to store the lowest f value node 
  int x;
  int y;
  double f;
  }lowfnode;

  struct sol{             // to store solution
    int sx;
    int sy;}pathsol[20];
    
  bool IsValid(int row, int col){         // if the cell is valid return true else return false
    return ((row <= ROW)&&(row>= 0)&&(col<=COL)&&(col>=0));
    }
  
  bool IsUnblocked(int row, int col){       // if the cell is not blocked return true else return false
    if(Environment[row][col] == 1){
      return(true);}
    else{return(false);}
      }
  bool IsDestination(int row, int col){   // if the cell is destination return true else return false
    if((row == dx)&&(col == dy)){
      return(true);}
    else{return(false);}
    }
  double CalH(int row, int col){              // return the 
   // return(sqrt((dx-row)*(dx-row)+(dy-col)*(dy-col)));}
   int abx,aby,H;
   abx = dx-row;
   aby = dy-col;
    //return(abs(dx-row)+abs(dy-col));}
    H= abs(abx) +abs(aby);
    return(H);}
    
  void TracePath(){
    Serial.println("Trace path");
      int x,y,i=0;
      int tempx,tempy;
      x = dx;
      y = dy;
      while(!((node[x][y].parentx == X)&&(node[x][y].parenty == Y))){
        pathsol[i].sx = x;// Serial.print("Inside fisrst while i = ");
        pathsol[i].sy = y;
        tempx = node[x][y].parentx;
        tempy = node[x][y].parenty;
        i++;
        x = tempx;
        y = tempy;
       // Serial.println(i);
        }
        pathsol[i].sx = x;
        pathsol[i].sy = y;     
        x = X;
        y = Y;
                  //while(!(pathsol[i].sx == X )&& (pathsol[i].sy == Y))
                  while(i>=0){
                 //Serial.print("Inside second while ");
                 Serial.print(pathsol[i].sx);
                 Serial.print("\t");
                  Serial.println(pathsol[i].sy); 
                    if(roface == 1){  // Facing East
              /////// To traverse in the path/////////////////
              if(pathsol[i].sx == x){
                if(pathsol[i].sy == y-1){     //// West
                  LeftTurn();Serial.println("Reverse");
                  delay(rev_delay);
                  roface = 2;
                  Forward();Serial.println("Forward");
                  delay(for_delay);
                  Stop();    
                  }
                  if(pathsol[i].sy == y+1){           /////East
                    Forward();Serial.println("forward");
                    delay(for_delay);
                    Stop();
                    }
                }

                if(pathsol[i].sy == y){
                  if(pathsol[i].sx == x-1){     /////North
                    LeftTurn();Serial.println("Left");
                    delay(turn_delay);
                    roface = 3;
                    Forward();Serial.println("forward");
                    delay(for_delay);
                    Stop();
                    }
                    if(pathsol[i].sx == x+1){         /////South
                      RightTurn();Serial.println("Right");
                      delay(turn_delay);
                      roface = 4;
                      Forward();Serial.println("forward");
                      delay(for_delay);
                      Stop();
                      
                      }
                  }   
                                   x = pathsol[i].sx;
                 y = pathsol[i].sy; 
                i--;
               // Serial.println(i);
                  }
                                  if(roface == 2){  // Facing West
              /////// To traverse in the path/////////////////
              if(pathsol[i].sx == x){
                if(pathsol[i].sy == y-1){     //// West
                  Forward();Serial.println("forward");
                  delay(for_delay);
                  Stop();    
                  }
                  if(pathsol[i].sy == y+1){           /////East
                    LeftTurn();Serial.println("Reverse");
                    delay(rev_delay);
                    roface = 1;
                    Forward();Serial.println("forward");
                    delay(for_delay);
                    Stop();
                    }
                }

                if(pathsol[i].sy == y){
                  if(pathsol[i].sx == x-1){     /////North
                    RightTurn();Serial.println("Right");
                    delay(turn_delay);
                    roface = 3;
                    Forward();Serial.println("forward");
                    delay(for_delay);
                    Stop();
                    }
                    if(pathsol[i].sx == x+1){         /////South
                      LeftTurn();Serial.println("Left");
                      delay(turn_delay);
                      roface = 4;
                      Forward();Serial.println("forward");
                      delay(for_delay);
                      Stop();
                      
                      }
                  }    
                   x = pathsol[i].sx;
                 y = pathsol[i].sy;
                i--;
               // Serial.println(i);
                  }
                   if(roface == 3){//facing North
              /////// To traverse in the path/////////////////
              if(pathsol[i].sx == x){
                if(pathsol[i].sy == y-1){     //// West
                  LeftTurn();Serial.println("Left");
                  delay(turn_delay);
                  roface = 2;
                  Forward();Serial.println("forward");
                  delay(for_delay);
                  Stop();    
                  }
                  if(pathsol[i].sy == y+1){           /////East
                    RightTurn();Serial.println("Right");
                    delay(turn_delay);
                    roface = 1;
                    Forward();Serial.println("forward");
                    delay(for_delay);
                    Stop();
                    }
                }

                if(pathsol[i].sy == y){
                  if(pathsol[i].sx == x-1){     /////North
                    Forward();Serial.println("forward");
                    delay(for_delay);
                    Stop();
                    }
                    if(pathsol[i].sx == x+1){         /////South
                      LeftTurn();Serial.println("Reverse");
                      delay(rev_delay);
                      roface = 4;
                      Forward();Serial.println("forward");
                      delay(for_delay);
                      Stop();
                      
                      }
                  }    
                 x = pathsol[i].sx;
                 y = pathsol[i].sy;
                i--;
               // Serial.println(i);
                  }
            if(roface == 4){//Facing South
              /////// To traverse in the path/////////////////
              if(pathsol[i].sx == x){
                if(pathsol[i].sy == y-1){     //// West
                  RightTurn();     Serial.println("right turn");
                  delay(turn_delay);
                  roface = 2;
                  Forward();        Serial.println("forward");    
                  delay(for_delay);
                  Stop();    
                  }
                  if(pathsol[i].sy == y+1){           /////East
                    LeftTurn();     Serial.println("left turn");
                    delay(turn_delay);
                    roface = 1;
                    Forward();      Serial.println("forward");
                    delay(for_delay);
                    Stop();
                    }
                }

                if(pathsol[i].sy == y){
                  if(pathsol[i].sx == x-1){     /////North
                    LeftTurn();     Serial.println("reverse");
                    delay(rev_delay);
                    roface = 3;
                    Forward();    Serial.println("forward");
                    delay(for_delay);
                    Stop();
                    }
                    if(pathsol[i].sx == x+1){         /////South
                      Forward();      Serial.println("forward");
                      delay(for_delay);
                      Stop();
                      
                      }
                  } 
                 x = pathsol[i].sx;
                 y = pathsol[i].sy;
                i--;
               // Serial.println(i);
            }
                  }
                  
                  }

void initnodes(){
  Serial.println("init nodes");
  int x,y;
  for(x=0;x<8;x++){
    for(y=0;y<8;y++){    
      node[x][y].onopen = false;
      node[x][y].onclosed = false;
      node[x][y].g = 0;
      node[x][y].h = 0;
      node[x][y].f = 0;
      node[x][y].parentx = -1;
      node[x][y].parenty = -1;
    }
  }
  // the parameters of starting node
  node[X][Y].g = 0;
  node[X][Y].h = 0;
  node[X][Y].f = 0;
  node[X][Y].parentx = X;
  node[X][Y].parenty = Y; 
}

  void FindPath(){
    int i,j;
    i = X;
    j = Y;
    lowfnode.f = 9999;
   // to break the loop
    //node[X][Y].onopen = true;
    while(lowfnode.f !=0){       
      Serial.println("find path");               
      node[i][j].onclosed = true;
      node[i][j].onopen = false;
      lowfnode.f = 9999;

                    // FIRST successor WEST
        if(IsValid(i,j-1) == true){         Serial.print("Inside i, j-1 \t"); Serial.print(i);Serial.println(j-1);
        if(IsDestination(i,j-1) == true){
          node[i][j-1].parentx = i;
          node[i][j-1].parenty = j;
          lowfnode.f = 0;
          }
          //else if (IsUnblocked(i,j-1) == true){                   
          else if ((node[i][j-1].onclosed == false) && (IsUnblocked(1,j-1) == true)){
          //node[i][j-1].g = node[i][j-1].g + 1;
          node[i][j-1].h = CalH(i,j-1);
         // node[i][j-1].f = node[i][j-1].g + node[i][j-1].h;Serial.print("f = ");Serial.println(node[i][j-1].f);
           node[i][j-1].f = node[i][j-1].h;Serial.print("f = ");Serial.println(node[i][j-1].f); 
          node[i][j-1].onclosed = true;  
          node[i][j-1].onopen = true ;   Serial.print("lowfnode f=  ");Serial.println(lowfnode.f);
          if(node[i][j-1].f <= lowfnode.f){
            node[i][j-1].parentx = i;
            node[i][j-1].parenty = j;
            lowfnode.x = i;
            lowfnode.y = j-1;
            lowfnode.f = node[i][j-1].f;Serial.print("after condition");Serial.println(lowfnode.f);
            }
          }
        }
                  // SECOND successor NORTH
      if(IsValid(i-1,j) == true){Serial.print("Inside i-1, j \t");Serial.print(i-1); Serial.println(j);
        if(IsDestination(i-1,j) == true){
          node[i-1][j].parentx = i;
          node[i-1][j].parenty = j;
          lowfnode.f = 0;
          }
          //else if (IsUnblocked(i-1,j) == true){                   
         else if ((node[i-1][j].onclosed == false) && (IsUnblocked(i-1,j) == true)){
          //node[i-1][j].g = node[i-1][j].g + 1;
          node[i-1][j].h = CalH(i-1,j);
         // node[i-1][j].f = node[i-1][j].g + node[i-1][j].h;Serial.print("f = ");Serial.println(node[i-1][j].f); 
          node[i-1][j].f =  node[i-1][j].h;Serial.print("f = ");Serial.println(node[i-1][j].f); 
          node[i-1][j].onclosed = true;        
          node[i-1][j].onopen = true ;   Serial.print("lowfnode f=  ");Serial.println(lowfnode.f);
          if(node[i-1][j].f <= lowfnode.f){
            node[i-1][j].parentx = i;
            node[i-1][j].parenty = j;
            lowfnode.x = i-1;
            lowfnode.y = j;
            lowfnode.f = node[i-1][j].f;Serial.print("after condition");Serial.println(lowfnode.f);
            }
          }
        } 
                       // THIRD succesor EAST
        if(IsValid(i,j+1) == true){   Serial.print("Inside i, j+1\t"); Serial.print(i);Serial.println(j+1);
        if(IsDestination(i,j+1) == true){
          node[i][j+1].parentx = i;
          node[i][j+1].parenty = j;
          lowfnode.f = 0;
          }
              // else if (IsUnblocked(i,j+1) == true){                  
         else if ((node[i][j+1].onclosed == false) && (IsUnblocked(i,j+1) == true)){
         // node[i][j+1].g = node[i][j+1].g + 1;
          node[i][j+1].h = CalH(i,j+1);
          //node[i][j+1].f = node[i][j+1].g + node[i][j+1].h;Serial.print("f = ");Serial.println(node[i][j+1].f); 
          node[i][j+1].f =  node[i][j+1].h;Serial.print("f = ");Serial.println(node[i][j+1].f);
          node[i][j+1].onclosed = true;  
          node[i][j+1].onopen = true ;   Serial.print("lowfnode f=  ");Serial.println(lowfnode.f);
          if(node[i][j+1].f <= lowfnode.f){ 
            node[i][j+1].parentx = i;
            node[i][j+1].parenty = j;
            lowfnode.x = i;
            lowfnode.y = j+1;
            lowfnode.f = node[i][j+1].f;Serial.print("after condition");Serial.println(lowfnode.f);
            }
          }
        }   
                  // FOURTH successor SOUTH
        if(IsValid(i+1,j) == true){Serial.print("Inside i+1, j \t"); Serial.print(i+1); Serial.println(j);
        if(IsDestination(i+1,j) == true){
          node[i+1][j].parentx = i;
          node[i+1][j].parenty = j;
          lowfnode.f = 0;
          }
          //else if (IsUnblocked(i+1,j) == true){                   
          else if ((node[i+1][j].onclosed == false) && (IsUnblocked(i+1,j) == true)){
         // node[i+1][j].g = node[i+1][j].g + 1;
          node[i+1][j].h = CalH(i+1,j);
         // node[i+1][j].f = node[i+1][j].g + node[i+1][j].h;Serial.print("f = ");Serial.println(node[i+1][j].f); 
         node[i+1][j].f = node[i+1][j].h;Serial.print("f = ");Serial.println(node[i+1][j].f); 
          node[i+1][j].onclosed = true;  
          node[i+1][j].onopen = true ;   Serial.print("lowfnode f=  ");Serial.println(lowfnode.f);
          if(node[i+1][j].f <= lowfnode.f){
            node[i+1][j].parentx = i;
            node[i+1][j].parenty = j;
            lowfnode.x = i+1;
            lowfnode.y = j;
            lowfnode.f = node[i+1][j].f;Serial.print("after condition");Serial.println(lowfnode.f);
            }
          }
        }
      i = lowfnode.x;
      j = lowfnode.y;
      Serial.print("low i = ");
      Serial.println(i);
      Serial.print("low j = ");
      Serial.println(j);     
            Serial.print("low f = ");
      Serial.println(lowfnode.f);  
      }
    }
  
  void Forward(){
    digitalWrite(L1, HIGH);       // left motor   ON
    digitalWrite(L2, LOW);  
    digitalWrite(R1, HIGH);        // right motor  ON
    digitalWrite(R2, LOW);    
  }
  
  void RightTurn(){
    digitalWrite(L1, HIGH);        //left motor   ON
    digitalWrite(L2, LOW);  
    digitalWrite(R1, LOW);         //right motor   OFF
    digitalWrite(R2, LOW);    
  }
  
  void LeftTurn(){
    digitalWrite(L1, LOW);        //left motor    OFF
    digitalWrite(L2, LOW);  
    digitalWrite(R1, HIGH);        //right motor   ON
    digitalWrite(R2, LOW);    
  }
  
  void Stop(){
    digitalWrite(L1, LOW);       // left motor   OFF
    digitalWrite(L2, LOW);  
    digitalWrite(R1, LOW);        // right motor  OFF
    digitalWrite(R2, LOW);    
  }
  
  void Reverse(){
    digitalWrite(L1, LOW);       // left motor   ON (reverse)
    digitalWrite(L2, HIGH);  
    digitalWrite(R1, LOW);        // right motor  ON (reverse)
    digitalWrite(R2, HIGH);    
  }
  
  void loop() {
  }
    void setup() {
    Serial.begin(9600);  
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(R1, OUTPUT);  
    pinMode(R2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);
    analogWrite(E1, 70);
    initnodes();
    FindPath();
    TracePath();
  }
  