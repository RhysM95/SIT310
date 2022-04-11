int motorright = 9;                                                              
int motorrightdir  = 7;
int motorleft = 10;
int motorleftdir  = 8;

//For serial receive.
const byte numChars = 11;
char receivedChars[numChars]; // an array to store the received data
String received; //The data as a string
boolean newData = false;

void setup() {
  pinMode(motorright, OUTPUT);                                                      
  pinMode(motorleft, OUTPUT);     
  pinMode(motorrightdir, OUTPUT);  
  pinMode(motorleftdir, OUTPUT);  

  Serial.begin(9600);
}

void loop() 
{
    recvWithEndMarker();
    processCommand();              
}

void processCommand() {
 if (newData == true)
 {
  Serial.print("Data Received");
  Serial.println(receivedChars);
  String instruction = received.substring(0,5);
  String data = received.substring(6,10);
  Serial.print(instruction);
  Serial.print(data);
  if(instruction == "MOVEF") forward(data.toInt());
  if(instruction == "MOVEB") backward(data.toInt());
  newData = false;
 }
}

void recvWithEndMarker() 
{
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial.available() > 0 && newData == false) 
 {
  rc = Serial.read();

  if (rc != endMarker) 
  {
    receivedChars[ndx] = rc;
    ndx++;
    if (ndx >= numChars) 
    {
      ndx = numChars - 1;
    }
 }
 else 
  {
  receivedChars[ndx] = '\0'; // terminate the string
  received = String(receivedChars);
  ndx = 0;
  newData = true;
  }
 }
}

//direction is controlled by the digital pin 7 and 8.
// HIGH is backward, LOW is forward
// Pins 9 and 10 control speed.
// Length of time controls the distance

void forward(int time)
{
digitalWrite(motorrightdir, LOW);
analogWrite(motorright,180); 
digitalWrite(motorleftdir, LOW);
analogWrite(motorleft, 180); 
delay(time);
stop();
}

void backward(int time)
{
digitalWrite(motorrightdir, HIGH);
analogWrite(motorright,180); 
digitalWrite(motorleftdir, HIGH);
analogWrite(motorleft, 180);
delay(time);
stop();
}

void stop()
{
analogWrite(motorright, 0); 
analogWrite(motorleft, 0);   
}
