
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3
#include <Servo.h>

long duration; 
Servo s1;
Servo s2;
char buffin[64];
const int NMAX = 64;
char buff[NMAX];

int ve1, ve2;


void setup() {
 
pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
pinMode(echoPin, INPUT); 
Serial.begin(9600);
s1.attach(5);
s2.attach(7);
ve1=0;
ve2=0;
}

void loop() {


duration = 0.0025;

if(Serial.available()>0){

digitalWrite(trigPin, LOW);
delayMicroseconds(1);//2 Orig
digitalWrite(trigPin, HIGH);
delayMicroseconds(1); //10 orig
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);


light_sense();
motors();

}
}


void motors(){
  

int len;

len=Serial.readBytes(buffin,4);
  
int *v, v1,v2, *vt;
char *p, *pb, *ch, c1;

pb=buffin;
p=pb;

v=(int*)p;
v1=*v;
p += sizeof(int);

vt=(int *)p;
v2=*vt;



s1.write(v1);
s2.write(v2);

  }

void light_sense(){

 unsigned int light, distance;

  for(int i=0; i<2;i++){
  light=light+analogRead(A0);
    }
  light=light/2;

distance = duration * 0.034 / 2;
char *p, *p_b;
unsigned int *d, *I_L;
int *v1,*v2;
p_b=buff;
p=p_b;

  I_L=(unsigned int*)p;
  *I_L=light;
  p+=sizeof(unsigned int);
  d=(unsigned int*)p;
  *d=distance;

  p+=sizeof(unsigned int);
  v1=(int*)p;
  *v1=ve1;
  p+=sizeof(int);
  v2=(int*)p;
  *v2=ve1;
  
Serial.write(buff,4);

  }
