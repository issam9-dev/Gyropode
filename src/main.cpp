#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Encoder.h>          // AJOUT


Adafruit_MPU6050 mpu;



ESP32Encoder encoderGauche;                                
ESP32Encoder encoderDroit;                                

#define CLK_G  26   // CLK encodeur gauche                // A
#define DT_G   27   // DT  encodeur gauche                // A
#define CLK_D  18   // CLK encodeur droit                 // A
#define DT_D   19   // DT  encodeur droit                 // A

// Tops encodeurs (impulsions / tour)                      
const float top_gauche = 750.0;                            
const float top_droit  = 748.8;                            

// Variables asservissement vitesse                        
float Kpvitesse = 0.0;                                     
float vcons     = 0.0;   // consigne vitesse (tours/s)    
float vobs      = 0.0;   // vitesse observée  (tours/s)   
long  countG_prev = 0, countD_prev = 0;                   


char FlagCalcul = 0;
float Ve, Vs = 0;
float Te = 10;
float Tau = 500, tot;
float angle;
float theta_filtre, thetaAccel, theta_W, theta_final, C0=0.0, d_theta_gyroscope, theta_equilibre = 0.0;


sensors_event_t a, g, temp;
float A, B;


// Variables Asservissement ----
float Kp = 0.0;
float Kd = 0.0;
float theta_cons = 0.0;  // Consigne d'angle
float dutycycle1, dutycycle2;


// Broches de commande (in1/in2 = moteur gauche, in3/in4 = moteur droit)
int in1 = 32, in2 = 33;
int in3 = 17, in4 = 16;


// Canaux PWM 
int canal1 = 0; // in1
int canal2 = 1; // in2
int canal3 = 2; // in3
int canal4 = 3; // in4


int frequence  = 20000; //Fréquence PWM en Hz, inaudible bon moteur
int resolution = 10; //Résolution PWM : 2^10 = 1023 niveaux (0 à 1023)
float ec; // calculée avant saturation et conversion PWM
float erreur;    // Erreur angulaire = theta_cons - theta_final


void controle(void *parameters)
{
  TickType_t xLastWakeTime; 
  xLastWakeTime = xTaskGetTickCount();  // Initialise le timer
  while (1)
  {
    mpu.getEvent(&a, &g, &temp);  // Lecture accéléromètre + gyroscope

    // ---- Calcul vitesse observée ----                   
    long cG = encoderGauche.getCount() ;               
    long cD = encoderDroit.getCount()  ;                  // Mise à jour mémoire
    long deltaG = cG - countG_prev;                        // tops écoulés depuis le dernier cycle
    long deltaD = cD - countD_prev;               
    countG_prev = cG;                                  
    countD_prev = cD;                                 
    float vG = (float)deltaG / top_gauche / (Te / 1000.0); // Conversion tops  tours/s :  tops / (tops/tour) / (Te en secondes)
    float vD = (float)deltaD / top_droit  / (Te / 1000.0);
    vobs = (vG + vD) / 2.0;                                // Vitesse moyenne des deux roues

    // Mesure d'angle
    thetaAccel = atan2(a.acceleration.y, a.acceleration.x) * 180 / PI; //  angle donné par l'accéléromètre 
    theta_filtre = A * thetaAccel + B * theta_filtre;  //lissage de thetaAccel (passe-bas, coeff A)
    d_theta_gyroscope = -(180 / PI) * g.gyro.z;  // vitesse angulaire gyro convertie en °/s
    theta_W = A * (Tau / 1000) * d_theta_gyroscope + B * theta_W; //intégration du gyro lissée (passe-haut, coeff B)
    theta_final = theta_W + theta_filtre + theta_equilibre;  //fusion des deux + offset d'équilibre mécanique

    // ---- Consigne angle issue de la boucle vitesse ---- 
    theta_cons = Kpvitesse * (vcons - vobs);   // Boucle vitesse
    if (theta_cons > 20) theta_cons += 20;  // saturation à 20°
    if (theta_cons < -20) theta_cons -= -20; 

    // Calcul de l'asservissement
    erreur = theta_cons - theta_final; 

    ec = (Kp * erreur) - (Kd * d_theta_gyroscope);

    if (ec > 0) ec += C0; // Compensation frottement 
    if (ec < 0) ec -= C0;

    if (ec >  0.45) ec =  0.45;  // Saturation haute
    if (ec < -0.45) ec = -0.45;

    dutycycle1 = (0.5 + ec) * 1023; 
    if (ec < -0.45) ec = -0.45;                      
    dutycycle2 = (0.5 - ec) * 1023;

    ledcWrite(canal1, dutycycle1);
    ledcWrite(canal2, dutycycle2);
    ledcWrite(canal3, dutycycle2);
    ledcWrite(canal4, dutycycle1);

    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}


void setup()
{
  Wire.begin(22, 21);
  Serial.begin(115200);
  Serial.printf("Bonjour \n\r");

  // ---- Initialisation encodeurs ----                    // AJOUT
  encoderGauche.attachFullQuad(CLK_G,DT_G);              // AJOUT
  encoderDroit.attachFullQuad(CLK_D,DT_D);               // AJOUT
  encoderGauche.setCount(0);                               // AJOUT
  encoderDroit.setCount(0);                                // AJOUT

  ledcSetup(canal1, frequence, resolution);
  ledcSetup(canal2, frequence, resolution);
  ledcSetup(canal3, frequence, resolution);
  ledcSetup(canal4, frequence, resolution);

  ledcAttachPin(in1, canal1);
  ledcAttachPin(in2, canal2);
  ledcAttachPin(in3, canal3);
  ledcAttachPin(in4, canal4);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
// A = poids de l'accéléromètre (court terme), B = poids du gyroscope (long terme)
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;

  xTaskCreate(controle, "controle", 10000, NULL, 10, NULL);
}
void reception(char ch)
{
  static String chaine = "";
  String commande, valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index  = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1) { commande = chaine; valeur = ""; }
    else { commande = chaine.substring(0, index); valeur = chaine.substring(index + 1, length); }

    if (commande == "Tau") { Tau = valeur.toFloat(); A = 1 / (1 + Tau / Te); B = Tau / Te * A; }
    if (commande == "Te")  { Te  = valeur.toInt();   A = 1 / (1 + Tau / Te); B = Tau / Te * A; }
    if (commande == "Kp")  Kp  = valeur.toFloat();
    if (commande == "Kd")  Kd  = valeur.toFloat();
    if (commande == "C0")  C0  = valeur.toFloat();// Compensation frottement
    if (commande == "T0")  theta_equilibre = valeur.toFloat();// Offset équilibre mécanique
    if (commande == "Kpv") Kpvitesse = valeur.toFloat(); 
    if (commande == "Vc")  vcons      = valeur.toFloat(); // Consigne vitesse (tours/s)

    chaine = "";   // Remise à zéro du buffer
  }
  else { chaine += ch; } // Accumulation caractère par caractère
}


void loop()
{
  if (FlagCalcul == 1)
  {
    Serial.printf("%f %f %f %f \n", theta_filtre, d_theta_gyroscope, theta_final, vobs); // Affiche
//   theta_filtre      angle accéléromètre filtré (°)
//   d_theta_gyroscope vitesse angulaire gyro (°/s)
//   theta_final       angle fusionné (°)
//   vobs              vitesse observée (tours/s)
    FlagCalcul = 0;
  }
}


void serialEvent()
{
  while (Serial.available() > 0) { reception(Serial.read()); }
}

