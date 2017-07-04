#include <SPI.h>

#define READY 'Y'
#define MSG_START '<'
#define MSG_STOP '>'

// Define pins for PWM motor controller
#define pwmA 3
#define dirA 2

#define pwmB 5
#define dirB 9

#define pwmC 6
#define dirC 4

#define ENC_A 7
#define ENC_B 8
#define ENC_C 10

#define R .184
#define Rw .05
#define ENC_REV 2000
// ENC_DIST = Rw*2*pi/ENC_REV
#define ENC_DIST 1.570796326794897e-04
#define sqrt3 1.732050807568877
#define CURRENT_SENSE 1.05e-03
#define I_MOTOR_MAX   3
#define MOTOR_TO_SENSE_RESISTANCE 4.7e03

union msg_t{
  float f[3];
  char  c[12];
};
// function for easy echoing of variables
void echo(char* name, double val)
{
  Serial.print(name);
  Serial.print(" = ");
  Serial.println(val, DEC);
}

float integrate(float x,float dx, float lim){
  if(x+dx > lim){
    return lim;
  }
  else if(x+dx < -lim){
    return -lim;
  }
  else{
    return x+dx;
  }
}

float clip(float x,float lim){
  if(x > lim){
    return lim;
  }
  else if(x < -lim){
    return -lim;
  }
  else{
    return x;
  }
}

void reset_vector(float *X,int N){
  for(int i = 0; i < N; ++i){
    X[i] = 0.0;
  }
}
void copy_vector(float *src,float *trg, int N){
  for(int i = 0; i < N; ++i){
    trg[i] = src[i];
  }
}

void F2U(float* F, float* U) {
  U[0] = -2./3.*F[0] - 1./3./R*F[2];
  U[1] = 1./3.*F[0] + 1/sqrt3*F[1] - 1./3./R*F[2];
  U[2] = 1./3.*F[0] - 1/sqrt3*F[1] - 1./3./R*F[2];
}

void U2F(float* U, float* F) {
  F[0] = -U[0] + 1./2.*U[1] + 1./2.*U[2];
  F[1] = sqrt3/2.*U[1] - sqrt3/2.*U[2];
  F[2] = -R*U[0] + -R*U[1] + -R*U[2];
}

void e2x(signed long* e, float* x) {
  x[0] = 2./3.*e[0] - 1./3.*e[1] - 1./3.*e[2];
  x[1] = -1./sqrt3*e[1] + 1./sqrt3*e[2];
  x[2] = 1./3./R*e[0] + 1./3./R*e[1] + 1./3./R*e[2];
  
  x[0] *= ENC_DIST;
  x[1] *= ENC_DIST;
  x[2] *= ENC_DIST;
}

void setPwm(int motorDir, int motorSpeed, int val) {
  if(val > 0) {
    digitalWrite( motorDir, HIGH );
  } else {
    digitalWrite( motorDir, LOW );
  }
  if(abs(val) > 255) {
    val = 255;
  }
  analogWrite( motorSpeed, abs(val) );
}

// Stops the Robot
void allStop() {
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
  analogWrite(pwmC, 0);
}

void initEncoder(int pinNr) {
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(pinNr, LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(pinNr, HIGH);       // Terminate SPI conversation 
}

void initBase() {
  // Set motor pins
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(dirC, OUTPUT);
  
  // Initialize all PWMs to zero
  allStop();

  // Set slave selects as outputs
  pinMode(ENC_A, OUTPUT);
  pinMode(ENC_B, OUTPUT);
  pinMode(ENC_C, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC_A, HIGH);
  digitalWrite(ENC_B, HIGH);
  digitalWrite(ENC_C, HIGH);
  
  SPI.begin();
  
  // Initialize encoders
  initEncoder(ENC_A);
  initEncoder(ENC_B);
  initEncoder(ENC_C);
}

long readEncoder(int encoder) {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(ENC_A,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(ENC_A,HIGH);     // Terminate SPI conversation 
  }
  
  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(ENC_B,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(ENC_B,HIGH);     // Terminate SPI conversation 
  }
   // Read encoder 3
  else if (encoder == 3) {
    digitalWrite(ENC_C,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(ENC_C,HIGH);     // Terminate SPI conversation 
  }
    
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(ENC_A,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC_A,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ENC_A,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC_A,HIGH);     // Terminate SPI conversation   
  
  // Set encoder2's data register to 0
  digitalWrite(ENC_B,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC_B,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder2's current data register to center
  digitalWrite(ENC_B,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC_B,HIGH);     // Terminate SPI conversation 
  
   // Set encoder3's data register to 0
  digitalWrite(ENC_C,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC_C,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder2's current data register to center
  digitalWrite(ENC_C,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC_C,HIGH);     // Terminate SPI conversation 
  
}

// controller
float Kp = 8000.0;
float Kp_theta = 300.0;
float Ki = 4000.0;
float Ki_theta = 100.0;
float Kd = 0.0;
float Kd_theta = 0.0;
float Klp = 0.1; 
float Ik[3] = {0.0,0.0,0.0};
float Dk[3] = {0.0,0.0,0.0};
float F[3] = {0.0,0.0,0.0};
float U[3] = {0.0,0.0,0.0};


unsigned long time = 0;
int rate = 200;
float Ts = 1./rate;
int slow_wait = 0;

// system states
 signed long e[3];
 signed long e_old[3];
 float ek[3] = {0.0,0.0,0.0};
 float xk0[3] = {0.0,0.0,0.0};
 float xk1[3] = {0.0,0.0,0.0};
 float xk2[3] = {0.0,0.0,0.0};
 float v[3] = {0.0,0.0,0.0};
 float v_set[3] = {0.0,0.0,0.0};
 float v_ref[3] = {0.0,0.0,0.0};
 int cmd_hist = 0;
 msg_t ref;




void trim() {
  char next;
  while(Serial.available()) {
    next = Serial.peek();
    if(next == ' ') {
      Serial.read();
    } else {
      return;
    }
  }
}
 
void getRef(float* reference) {
  char data;
  if(Serial.available()) {
    trim();
    data = Serial.read();
    if(data == MSG_START) {
      //Serial.println("Laeze mer..");
      reference[1] = Serial.parseFloat();
      reference[0] = -Serial.parseFloat();
      reference[2] = Serial.parseFloat();
      //Serial.println("Done");
    }
    trim();
    data = Serial.read();
    if(data == MSG_STOP) {
      /*Serial.println("Successful read:");
      echo("x", reference[0]);
      echo("y", reference[1]);
      echo("t", reference[2]);
      */
    }
    
    cmd_hist = 0;
  }
  // if no command in last second (rate iterations), set reference to zero
  else if(++cmd_hist > rate){
    reset_vector(reference,3);
  }
}

void getRef2(char* ref_c, float* ref_f) {
  char data;
  int i;
  if(Serial.available() >=6) {
    for(i = 0; i < 6; ++i){
      ref_c[i] = Serial.read();
    }
    Serial.flush();
    cmd_hist = 0;
  }
  else if(++cmd_hist > rate){
    reset_vector(ref_f,3);
  }
}



void setup() {
 Serial.begin(115200);      // Serial com for data output
 //reset_vector(ref.f,3);
 
 initBase();       //Serial.println("Encoders Initialized...");
 clearEncoderCount();  //Serial.println("Encoders Cleared...");
}


void loop() {
 
  copy_vector(xk1,xk2,3);
  copy_vector(xk0,xk1,3);
  
  e[0] = readEncoder(1); 
  e[1] = readEncoder(2);
  e[2] = readEncoder(3);
  e2x(e, xk0);
  
  if(slow_wait > 0) {
    slow_wait--;
  }
  else{  
    v[0] = 0.2*rate*(xk0[0]-xk1[0]) + 0.8*v[0];
    v[1] = 0.2*rate*(xk0[1]-xk1[1]) + 0.8*v[1];
    v[2] = 0.2*rate*(xk0[2]-xk1[2]) + 0.8*v[2];
    
    int eBreak = analogRead(A0);
  
    if(eBreak < 1000) {
      allStop();
      reset_vector(Ik,3);
      reset_vector(F,3);
      reset_vector(v_set,3);
      reset_vector(v_ref,3);
      //reset_vector(ref.f,3);
  
    } 
    else {
      // limit acc to 0.3 m/s2 / 1.0 rad/s2
      v_set[0] = v_set[0] + clip(v_ref[0]-v_set[0],0.8*Ts);
      v_set[1] = v_set[1] + clip(v_ref[1]-v_set[1],0.8*Ts);
      v_set[2] = v_set[2] + clip(v_ref[2]-v_set[2],4.0*Ts);

      //v_set[0] = v_set[0] + clip(ref.f[0]-v_set[0],0.3*Ts);    
      //v_set[1] = v_set[1] + clip(ref.f[1]-v_set[1],0.3*Ts);    
      //v_set[2] = v_set[2] + clip(ref.f[2]-v_set[2],1.0*Ts);   
   
      ek[0] = v_set[0] - v[0];
      ek[1] = v_set[1] - v[1];
      ek[2] = v_set[2] - v[2];
  
      Ik[0] = integrate(0.97*Ik[0],Ts*ek[0],0.2);
      Ik[1] = integrate(0.97*Ik[1],Ts*ek[1],0.2);
      Ik[2] = integrate(0.97*Ik[2],Ts*ek[2],2.0);
      
      Dk[0] = rate*rate*(xk0[0]-2*xk1[0]+xk2[0]);
      Dk[1] = rate*rate*(xk0[1]-2*xk1[1]+xk2[1]);
      Dk[2] = rate*rate*(xk0[2]-2*xk1[2]+xk2[2]);
     
      F[0] = Klp*(Kp*ek[0]+Ki*Ik[0]+Kd*Dk[0]) + (1-Klp)*F[0];
      F[1] = Klp*(Kp*ek[1]+Ki*Ik[1]+Kd*Dk[1]) + (1-Klp)*F[1];  
      F[2] = 0.4*(Kp_theta*ek[2]+Ki_theta*Ik[2]+Kd_theta*Dk[2]) + 0.6*F[2];
  
      F2U(F, U);
  
      int maxU = 0;
      int i;
      for(i = 0; i < 3; ++i) {
          if(abs(U[i]) > maxU) {
              maxU = abs(U[i]);
          }
      }
      if(maxU > 255) {
          for(i = 0; i < 3; ++i) {
              U[i] *= 255./maxU;
          }
      }
      U2F(U, F);
   
      setPwm(dirA, pwmA, U[0]);
      setPwm(dirB, pwmB, U[1]);
      setPwm(dirC, pwmC, U[2]);
      
      /*Serial.print("set ");
      Serial.print(U[0]);
      Serial.print(" ");
      Serial.print(U[1]);
      Serial.print(" ");
      Serial.println(U[2]);*/
    }
  }

    float V_measured_M1 = analogRead(A1)*5.0/1023;
    float V_measured_M2 = analogRead(A2)*5.0/1023;
    float V_measured_M3 = analogRead(A3)*5.0/1023;

    float I_M1 = V_measured_M1/MOTOR_TO_SENSE_RESISTANCE/CURRENT_SENSE*I_MOTOR_MAX*1000; // A to mA
    float I_M2 = V_measured_M2/MOTOR_TO_SENSE_RESISTANCE/CURRENT_SENSE*I_MOTOR_MAX*1000;
    float I_M3 = V_measured_M3/MOTOR_TO_SENSE_RESISTANCE/CURRENT_SENSE*I_MOTOR_MAX*1000;

    // M1 rightfront
    // M2 back
    // M3 leftfrint
  
  getRef(v_ref);
  Serial.print(xk0[1]);
  Serial.print(" ");
  Serial.print(-xk0[0]);
  Serial.print(" ");
  Serial.print(xk0[2]);
    Serial.print(" ");
  Serial.print(I_M1);
    Serial.print(" ");
  Serial.print(I_M2);
    Serial.print(" ");
  Serial.println(I_M3);
  //getRef2(ref.c,ref.f);
  
  signed long sleep = 1000000/rate - (micros() - time);
  if(sleep > 0) {
    delayMicroseconds(sleep);
  } else {
    Serial.println("Too slow!");
    slow_wait = 3;
  }
  time = micros();
}

