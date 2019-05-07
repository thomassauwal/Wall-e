#include <Wire.h>
#include <math.h>
#include <LIDARLite.h>
#include <Servo.h>

///Variables pour la detection
int TabMesDis[180]; //distances des objets détectés 
int TabMesPos[180]; //Position (angle) du Lidar
int TabMesInd[180];
boolean TabMesEtat[180];
int TabInstMes[5];
int MesDis;
int MesDisPrec=2000;
int i=0;
int j=0;
int k=0;
int iMin=0;
int imax;
int t1;
int AngleDepart=80;
int AngleRecherche=120;
int ElimBord=5;
Servo myservo;  // create servo object to control a servo
int val;    // variable to read the value from the analog pin
int DisMin=0;

///Variables pour le déplacement
int DistATrouver = 0; 
int speed = 200; 


boolean detection = true; 

LIDARLite myLidarLite;

void setup() {

  Serial.begin(9600);
  myLidarLite.begin();
  delay (20);
 myservo.attach(2);  // attaches the servo on pin 9 to the servo object

              
 // Motor_1 controll pin initiate;
 pinMode(4, OUTPUT);     
 pinMode(5, OUTPUT);    
 pinMode(11, OUTPUT); // Speed control
 
 // Motor_2 controll pin initiate;
 pinMode(7, OUTPUT);     
 pinMode(8, OUTPUT);    
 pinMode(12, OUTPUT);  // Speed control
 
 //Enable the Motor Shield output;  
 pinMode(6, OUTPUT); 
 digitalWrite(6, HIGH);  
}


void loop() 
{
//////////////////////////////////////
///        Détection                //
//////////////////////////////////////

  if (detection == true)
  {
    i=0;
     for (t1=700 ; t1<2300; t1+=10)
    {
      i++;
      val = map(i, 0, AngleRecherche, AngleDepart, AngleDepart+AngleRecherche);     // scale it to use it with the servo (value between 0 and 180)
      myservo.writeMicroseconds(t1);                  // sets the servo position according to the scaled value
      TabMesPos[i]=t1;
      TabMesInd[i]=i;
      //delay(100);
      MesDis=0;
      k=0;
      
      for (j=0;j<3;j++)
        {
          TabInstMes[j]=myLidarLite.distance();
          //delay(2);
            if (TabInstMes[j] > 4000 || TabInstMes[j] < 2 )
            {
              /*
              Serial.print("mesure hors limites :");
              Serial.print(k);
              Serial.print("  ");
              Serial.println(TabInstMes[j]);
              */ 
            }
            else
            {
              MesDis+=TabInstMes[j];
              k+=1;
            }
          }
          if (k==0)
          {
            TabMesEtat[i]=LOW; 
            TabMesDis[i]=0;  
            //Serial.println("k=0  ");
          }
          else
          {
            TabMesEtat[i]=HIGH;
            TabMesDis[i]=MesDis/k;
          }
      /*
      Serial.print(TabInstMes[0]);
      Serial.print(" ");
      Serial.print(TabInstMes[1]);
      Serial.print(" ");
      Serial.print(TabInstMes[2]);
      Serial.print(" - ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.print(TabMesDis[i]);
      Serial.println(" ");
      */
      delay(5);
    }
    imax=i;
    Serial.println("Fin mesure");
  
    Serial.println("Debut lecture tableau des mesures");
     DisMin=4000;
    for (i=1 ; i<imax; i++)
    {
      if(TabMesDis[i]<DisMin && TabMesEtat[i]==HIGH) // Recherche objet le plus proche
      {
        DisMin=TabMesDis[i];
        iMin=i;
      }
      
    }
    /*
    Serial.print("Objet le plus proche : iMin : ");
    Serial.print(iMin);
    Serial.print(" Dis : ");
    Serial.print(TabMesDis[iMin]);
    Serial.print(" Pos deg : ");
    Serial.println(TabMesPos[iMin]);
    */
    myservo.writeMicroseconds(TabMesPos[iMin]);//se positionne sur l'objet 
    delay(1000);     
    
    myservo.writeMicroseconds(1500); //se positionne droit
    for (i=1 ; i<imax; i++)
    {
      if(TabMesEtat[i]==HIGH)
      {
        //Serial.print(" Dis : ");
       /*
        Serial.println(TabMesDis[i]);
        Serial.print(" Ind : ");
        Serial.print(TabMesInd[i]);
        Serial.print(" Etat : ");
        Serial.print(TabMesEtat[i]);
        Serial.print(" Pos :");
        Serial.println(TabMesPos[i]); 
        */
      }
    }
  }
   
  delay(1000);               // wait for a 2 seconds 

//////////////////////////////////////
///        Déplacement              //
//////////////////////////////////////

detection = false; //Permet d'arrêter la rotation du Lidar

///Detection a droite
    if (TabMesPos > 700 && TabMesPos[iMin]<1500) //détecte à droite  
    {
      do {
         analogWrite(11,speed);    // set the motor_1 speed ;
         digitalWrite(4, HIGH);   
         digitalWrite(5, LOW);  // Set the rotation of motor_1
         
         analogWrite(12,speed - 20);    // set the motor_2 speed ;
         digitalWrite(7, HIGH);  
         digitalWrite(8, LOW);  // Set the rotation of motor_1
  
         DistATrouver = myLidarLite.distance(); //Mesure par le lidar 
      }    while(TabMesDis[iMin] -5 >= DistATrouver || TabMesDis[iMin] +5 <= DistATrouver ); //marge d'erreur de +- 5
  
        //Arret des roues quand positionné sur le Lidar 
        analogWrite(11,0);    // set the motor_1 speed ;
        analogWrite(12,0);    // set the motor_2 speed ;
    }  


///Detection a gauche
    if (TabMesPos > 700 && TabMesPos[iMin]<1500) 
    {
      do {
         analogWrite(11,speed);    // set the motor_1 speed ;
         digitalWrite(4, LOW);   
         digitalWrite(5, HIGH);  // Set the rotation of motor_1
         
         analogWrite(12,speed - 20);    // set the motor_2 speed ;
         digitalWrite(7, LOW);  
         digitalWrite(8, HIGH);  // Set the rotation of motor_1
  
         DistATrouver = myLidarLite.distance(); 
      }    while(TabMesDis[iMin] -5 >= DistATrouver || TabMesDis[iMin] +5 <= DistATrouver ); 
  
        analogWrite(11,0);    // set the motor_1 speed ;
        analogWrite(12,0);    // set the motor_2 speed ;
    }  

///Pour avancer 
    do 
    {
       analogWrite(11,speed);    // set the motor_1 speed ; gauche
       digitalWrite(4, HIGH);   
       digitalWrite(5, LOW);  // Set the rotation of motor_1
       
       analogWrite(12,speed - 20);    // set the motor_2 speed ; droite 
       digitalWrite(7, LOW);  
       digitalWrite(8, HIGH);  // Set the rotation of motor_1
    
       DistATrouver = myLidarLite.distance(); 
    }
    while (DistATrouver > 3);
    
       analogWrite(11,0);    // set the motor_1 speed ;
       analogWrite(12,0);    // set the motor_2 speed ;
    }
