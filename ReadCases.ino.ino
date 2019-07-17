 void setup() {
  Serial.begin(9600);
  pinMode(13,OUTPUT);
      
 }

 void loop(){
    if (Serial.available() > 0)   
      {
        char command = Serial.read();  
        
  
        // Physical motion after each decision is made
        // TODO: synthesise actuation with logic autonomously
        switch (command)
        {
              
          case '1':
             //heading sent
            digitalWrite(13,HIGH);
            delay(1000);
            digitalWrite(13,LOW);
            break;
    
          case '2':
            digitalWrite(13,HIGH);
            delay(2000);
            digitalWrite(13,LOW);
            break;
                
          case '3':
              digitalWrite(13,HIGH);
              delay(3000);
              digitalWrite(13,LOW);
              break;
             
          default:
            break;           
         }
      }
 }
