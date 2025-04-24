/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                                                 Micro-EMS Avionicsduino V 4.06
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Micro-EMS Avionicsduino V 4.06 is free software    
  MIT License (MIT)
  
  Copyright (c) 2025 AvionicsDuino - benjamin.fremond@avionicsduino.com
  https://avionicsduino.com/index.php/en/teensy-micro-ems/?preview=true
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
    
 *****************************************************************************************************************************/ 
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                Connexions physiques des différents composants avec la carte Teensy 4.0
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// Signal compte tours sur pin 22 + bibliothèque FreqMeasure
// Signal fuel flow sur pin 9 + bibliothèque FreqCount
// Signal fuel level sur pin 16/A2
// GrayHill optical rotary encoder sur pins 2,3,4
// Tension batterie sur pin 23/A9
// Red LED sur pin 19
// Green LED sur pin 20
// Ecran OLED 128x128 Adafruit 4741 sur SPI (GND->GND, Vin->3.3v, Cs->14, A0/DC->15, CLK->SCK/13, Data->MOSI/11)
// Transceiver CAN sur CAN2 pins 0 et 1
// Module Bluetooth HC-05 sur Serial2 pins 7 et 8

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  Inclusions des bibliothèques et fichiers externes
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <QuadEncoder.h>
#include <Adafruit_SSD1327.h>
#include <FreqCount.h>
#include <FreqMeasure.h>
#include <EEPROM.h> 
#include <FlexCAN_T4.h> 
#include <SPI.h> 
#include <TimeLib.h> 

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Création des objets
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN_Micro_EMS;  // Crée l'objet Flexan_T4 "CAN_Micro_EMS" en mode CAN 2.0.
QuadEncoder encoder(1, 4, 3, 1);                         // Crée l'objet QuadEncoder "encodeur" sur le canal 1. Phase A (pin 4), PhaseB(pin 3), pullups activées(1). 
Adafruit_SSD1327 oled(128, 128, &SPI, 15, -1, 14);      // Crée l'objet écran "oled"

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Déclarations des variables et constantes globales
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// ****************************************************************** Affectation des différentes broches **********************************************************************************************
#define pinFuelLevel      A5          // Jauge connectée sur pin 19/A5
#define pinVBat           A9          // Mesure tension batterie
#define pinRedLED         20          // Voyant vert
#define pinGreenLED       21          // Voyant rouge
#define encoderButtonPin   2          // Switch de l'encodeur rotatif

//************************** Variables utilisées pour la mesure et le calcul du débit carburant, de la quantité de carburant consommée et de la quantité restante, grâce au Red Cube ******************************************************************
uint32_t NbTotalPulses = 0;      // nombre total d'impulsions du Red Cube enregistrées depuis la mise en route
uint16_t freq = 0;                // stocke la fréquence du Red Cube à un moment donné
float calcFuelLevel;
float startFuelLevel;
float fuelUsed=0.0;              // Quantité d'essence consommée en litres depuis la mise en route
float fuelFlow;                  // Débit instantané du carburant
float prevFiltrFuelFlowVal=0.0;  // Valeur filtrée précédente du débit carburant en L/h
#define fuelTankCapacity 79      // Capacité maximale du réservoir du F-PBCF (en litres)
uint16_t kFactor; 
bool flagOKwriteKfactorEeprom = false;
uint16_t kFactorAdjustTimeOut=2000; // Si on modifie le K factor avec l'encodeur, la nouvelle valeur ne sera enregistrée en EEPROM que 2 secondes après la modification, si le flag ci-dessus est vrai
uint32_t kFactorAdjustStartTime=0;
bool flagOKwriteCalcFuelLevelEeprom = false;
uint16_t calcFuelLevelAdjustTimeOut=3000; // Si la variable calcFuelLevel change, la nouvelle valeur ne sera enregistrée en EEPROM que 3 secondes après la modification, si le flag ci-dessus est vrai
uint32_t calcFuelLevelAdjustStartTime = 0;
bool flagOKwriteCalcFuelLevelEepromEndOfFlight = true;

//******************************************************* Variables utilisées pour la gestion de l'EEPROM **********************************************
uint16_t fuelLevelEepromAddress;           // Adresse de l'EEPROM à laquelle est mémorisée périodiquement la variable float calcFuelLevel (quantité d'essence présente dans le réservoir). 
                                           // Cette adresse change à chaque setup pour répartir l'usure de l'EEPROM (wear levelling)
                                           // Les 1025 premiers octets de l'EEPROM (octets 0 à 1024) lui sont réservés, voir setup()
#define kFactorEepromAdress 1034           // adresse de stockage fixe du facteur K (variable kFactor). Cette variable change rarement, wear levelling inutile.

//*********************************** Variables utilisées pour la mesure de la quantité de carburant dans le réservoir grâce à la jauge ******************************************************************
#define fuelSenderNbSwitches 29     // La jauge Wema de F-PBCF comporte 29 reed switches (https://i.imgur.com/wIbZ90K.jpg)
float valA5 =0.0;                  // Valeur lue sur pin 19/A5, à convertir en niveaux de carburant dans le réservoir (bornes haute et basse) grâce au tableau d'étalonnage ci-dessous
float prevFiltrvalA5;            // valeur filtrée précédente de valA5
typedef struct                   // Structure définissant les champs de chaque ligne du tableau refTableFuelSender initialisé ci-dessous : à chaque pas de la jauge Wema correspond une résistance,
                                 // donc une tension en sortie du pont diviseur de mesure, donc une valeur numérisée de 0 à 1023 (numVal) pour cette tension. Et à chaque pas correspondent des valeurs haute et basse
                                 // qui donnent la fourchette dans laquelle se situe la quantité d'essence dans le réservoir.
{
  uint16_t numVal;
  uint8_t upperLimit;
  uint8_t lowerLimit;
} ReferenceTable;

ReferenceTable refTableFuelSender[30] =   // Les données de ce tableau ont été obtenues par un étalonnage précis du réservoir, en le remplissant, puis en le vidangeant, litre par litre.
{
  {5,11,0}, {84,13,9}, {157,15,11}, {222,17,14}, {281,19,16}, {301,21,18}, {321,23,20}, {340,25,22}, {358,27,24}, {376,30,26}, {393,33,29}, {410,35,31}, {427,39,34}, {445,41,37}, {461,44,40},
  {478,46,42}, {493,50,45}, {508,53,48}, {524,56,51}, {538,59,55}, {552,62,57}, {565,65,61}, {579,67,63}, {606,70,66}, {631,73,69}, {655,75,71}, {678,77,74}, {700,79,76}, {720,80,78}, {10000,80,80}
};
uint8_t  upperLimit = 0, lowerLimit = 0;  // variables contenant les limites de la fourchette d'incertitude dans laquelle se situe le niveau mesuré de carburant dans le réservoir
uint8_t  IndexTable = 0;                  // variable utilisée lors des recherches dans le tableau refTableFuelSender

//***************************************************** Variables utilisées pour la mesure, le calcul et l'affichage du RPM ******************************************************************
uint32_t zeroHandlingStartTime;  // Variable permettant de mesurer le nombre de millisecondes entre deux requêtes FreqMeasure.available infructueuses, pour permettre la mesure d'une fréquence compte tours nulle
                                 // La fonction FreqMeasure ne peut pas mesurer une fréquence nulle (en fait une absence de fréquence)
uint16_t zeroHandlingTimeOut = 500;          
float rpm=0;                   // Nombre de tours moteur par minute
float prevFiltrRpmVal = 0;     // Valeur filtrée précédente du RPM
float nonFilteredRpmVal = 0.0; // Valeur non filtrée ni arrondie du RPM
uint16_t rpmInt;

//******************************************************** Variables et constantes utilisées pour la gestion de la tension batterie et des voyants ********************************************************
float vBat=0.0, prevFiltrVBatVal = 12.7;
boolean lowVoltage=false;                                              //Indicateur de tension basse si la variable est vraie
uint16_t rpmThreshold = 2500;                                          // Seuil de RPM au dessus du quel la LED rouge clignote rapidement en cas de tension basse, sinon elle s'allume en continu (situation normale au ralenti)
float vBatThreshold = 13.25;                                           // En dessous de ce seuil, on agit sur les LED
uint32_t blinkRedLedStartTime;                                           // Variable utilisée pour le clignotement de la LED rouge
uint16_t blinkRedLedTimeOut = 300;

// ******************************************************************* Variables utilisées par l'encodeur rotatif et le menu **************************************************************************
volatile uint32_t currentEncoderPosition = 0;
volatile uint32_t previousEncoderPosition = 0;
volatile char menuOption;
const uint16_t debounceTimeOut = 200;
volatile unsigned long debounceStartTime=0;

// ************************************************************************************* Variables messages CAN **************************************************************************************
CAN_message_t msgFuelsenderRPMvBat;
CAN_message_t msgFuelFlowAndLevel;
CAN_message_t msgTotalNbSecFuelUsed;
CAN_message_t msgrpm;

// ************************************************************ Variables utilisées pour le chronométrage de la boucle principale **********************************************************
uint32_t nombreBoucleLoop = 0;
uint32_t dureeNdbBoucles = 0;
uint32_t Ndb = 4000000;
float dureeLoop;
uint32_t topHoraire;

// ************************************************************ Variables utilisées pour rafraichir l'affichage toutes les 100 ms (10 FPS) ***********************************************************
uint32_t oledUpdateStartTime; // Pour espacer les affichages à l'écran et les envois de données sur le CAN bus
uint8_t  oledUpdatePeriod = 100;

// ************************************************** Variables utilisées pour la gestion de l'heure UTC et de la date reçues de l'EFIS via le CAN bus ************************************
uint8_t nbHour, nbMin, nbSec;
uint16_t anneeGNSS=0;
uint8_t moisGNSS=0, jourGNSS=0, heureGNSS=0, minuteGNSS=0, secondeGNSS=0;

uint32_t BTSendUpdateStartTime; // Pour espacer les envois de données par Bluetooth
uint8_t  BTSendUpdatePeriod = 200;

//************************************************************************* Variables diverses **************************************************************************************************************
uint32_t vBatFuelLevelMeasureStartTime;          // Pour espacer les mesures de la tension batterie et du niveau d'essence
uint8_t vBatFuelLevelMeasureUpdatePeriod = 100;
uint16_t totalNbSec = 0; // Compte le nombre total de secondes écoulées depuis la mise en route du moteur

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             SETUP
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
// ********************************************************************************* Initialisations des bibliothèques *******************************************************************************************  
  Serial.begin(115200);
  Serial2.begin(115200);
  FreqCount.begin(1000000);    // FreqCount compte le nombre de période pendant 1000000 µsec. Le résultat est donc une fréquence en Hz
  FreqMeasure.begin();

// ******************************************************************************* Initialisation de l'écran OLED *******************************************************************************************
  oled.begin(); 
  oled.setRotation(1);
  oled.setTextColor(SSD1327_WHITE,SSD1327_BLACK); 
  oled.setTextSize(2);
  oled.setCursor (10,45);
  oled.println("Micro-EMS"); 
  oled.setCursor (25,75); 
  oled.print  ("V 4.06"); 
  oled.display();
  delay(1000);
  oled.clearDisplay();
  
// ***************************************************************** Affichages fixes *****************************************************************************
  oled.setTextSize(2);
  oled.setCursor (64,3);
  oled.print("V");
  oled.drawLine(81,0,81,17,4);
  oled.drawLine(66,40,66,65,4);
  oled.drawLine(0, 18, 128, 18, 4);
  oled.setCursor (65,21);
  oled.setTextSize(2);
  oled.print(" L/h");
  oled.drawLine(0, 38, 128, 38, 4);
  //oled.setCursor(60,48);
  //oled.print(" L");
  oled.setTextSize(1);
  oled.setCursor (0,40);
  oled.print("Level calc.");
  oled.setCursor (71,40);
  oled.print("Fuel used");
  oled.drawLine(0, 68, 128, 68, 4);
  oled.setCursor (10,71);
  oled.print("Level probe between");
  oled.setCursor (55,86);
  oled.print("and");
  oled.drawLine(0, 100, 128, 100, 4);
  oled.drawLine(78,102, 78, 128,4);
  oled.display();
  
//****************************************************************** Initialisation du CAN Bus ********************************************************************
  CAN_Micro_EMS.begin();
  CAN_Micro_EMS.setBaudRate(500000);
  CAN_Micro_EMS.setMaxMB(16);
  CAN_Micro_EMS.enableFIFO();
  CAN_Micro_EMS.enableFIFOInterrupt();
//  CAN_Micro_EMS.onReceive(FIFO,canSniff); // Le micro-EMS ne reçoit aucun message
  CAN_Micro_EMS.mailboxStatus();

  msgFuelsenderRPMvBat.id =  26;
  msgFuelsenderRPMvBat.len =  8;

  msgFuelFlowAndLevel.id = 32;
  msgFuelFlowAndLevel.len = 8;

  msgTotalNbSecFuelUsed.id = 33;
  msgTotalNbSecFuelUsed.len = 6;

  msgrpm.id = 51;
  msgrpm.len = 4;

//**************************************************************** Initialisation de certaines broches ***********************************************************
  pinMode (pinFuelLevel, INPUT_DISABLE);
  pinMode (pinVBat, INPUT_DISABLE);
  pinMode (pinRedLED, OUTPUT);
  pinMode (pinGreenLED,OUTPUT);
  pinMode (encoderButtonPin, INPUT_PULLUP);

// ****************************************************************** Initialisation des 2 variables stockées en EEPROM *************************************************************  
// Ces variables sont le facteur K et le niveau d'essence dans le réservoir (uint16_t kFactor sur 2 octets et float calcFuelLevel sur 4 octets).
// Le niveau d'essence est mis à jour par le programme, à chaque fois qu'il varie d'un litre, avec une écriture dans l'EEPROM à chaque fois. La mémoire flash est donc beaucoup sollicitée en écriture.
// Pour éviter "l'usure" rapide d'un unique emplacement constant, on change cet emplacement à chaque setup, pour "répartir" dans le temps l'usage de la mémoire flash sur les 1025 premiers octets (0 à 1024 inclus) de l'EEPROM.
// Le stockage de calcFuelLevel s'effectue sur 5 octets : un octet marqueur de valeur 100, puis les 4 octets du float

  bool flag = false;                                // flag temporaire permettant de savoir si un octet égal à 100 a été trouvé ou non
  for(int16_t i=0;i<1021;i+=5)                      // On va chercher un octet égal à 100, en partant du début de l'EEPROM, en testant uniquement tous les 5 octets.
  {
    uint8_t j = EEPROM.read(i);                     // On lit la valeur de l'octet stocké à l'adresse i.
    if (j==100)                                     // S'il est égal à 100, c'est qu'on a trouvé l'emplacement où est stockée la valeur calcFuelLevel, à savoir les 4 octets suivants, qui commencent à l'adresse i+1.
    {
      flag = true;
      EEPROM.get(i+1,calcFuelLevel);                // On attribue donc la valeur stockée à l'adresse i+1 à la variable calcFuelLevel,
      startFuelLevel = calcFuelLevel;               // et par défaut, avant un éventuel ajustement par le pilote en cas d'avitaillement, la même valeur est attribuée à startFuelLevel
      EEPROM.write(i,0);                            // puis on remet à zéro l'octet d'adresse i où était préalablement stockée la valeur 100.
      if (i==1020) i=-5;                            // Si on était au dernier emplacement attribué au stockage de calcFuelLevel, alors on doit repartir au début de l'EEPROM.
      EEPROM.write(i+5,100);                        // Maintenant, pour la suite du programme, on va stocker la variable calcFuelLevel 5 octets plus loin qu'avant, en la faisant précéder d'un octet 100,
      EEPROM.put(i+6, calcFuelLevel);               // puis on stocke calcFuelLevel à sa nouvelle adresse,
      fuelLevelEepromAddress = i+6;                 // on mémorise cette nouvelle adresse pour la suite du programme,
      break;                                        // et on sort immédiatement de la boucle for.
    }
  }
  if (!flag)                                           // Si l'emplacement de stockage du niveau d'essence n'a pas été trouvé (à la première utilisation du programme sur une carte Teensy à l'EEPROM vierge)
  {                                                    // (avec une carte Teensy dont l'EEPROM contient déjà des données inconnues, il est préférable de remettre préalablement tous les octets de l'EEPROM à zéro)
    EEPROM.write(0,100);                               // alors on stocke un octet de valeur 100 à l'adresse 0,
    calcFuelLevel = 39.3;                              // on attribue la valeur 42.7 à la variable calcFuelLevel,
    fuelLevelEepromAddress = 1;                        // on va utiliser l'adresse 1 pour la stocker,
    EEPROM.put(fuelLevelEepromAddress, calcFuelLevel); // on stocke cette valeur à l'adresse mémorisée à la ligne précédente.
    kFactor = 200;                                     // Puis on attribue la valeur 177 à la variable kFactor
    EEPROM.put(kFactorEepromAdress, kFactor);          // et on stocke le kFactor à son adresse dédiée.
  }
  EEPROM.get(kFactorEepromAdress, kFactor);            // On récupère le facteur K dans l'EEPROM.
  
// *************************************************************************** Initialisation de l'encodeur rotatif ********************************************************************************************
  attachInterrupt(encoderButtonPin, Bouton_ISR, RISING);
  encoder.setInitConfig();
  encoder.EncConfig.IndexTrigger = ENABLE;  //enable to  use index counter
  encoder.EncConfig.INDEXTriggerMode = RISING_EDGE;
  encoder.init();
  delay(200);
  
// ********************************************************************* Dernières initialisations diverses avant loop() **************************************************************** 
  encoder.write(0);
  currentEncoderPosition = 0;
  previousEncoderPosition = 0;
  menuOption = ' ';
  digitalWrite(pinRedLED, LOW); 
  digitalWrite(pinGreenLED, LOW);   
  zeroHandlingStartTime = millis();
  blinkRedLedStartTime = millis();
  vBatFuelLevelMeasureStartTime = millis();
  oledUpdateStartTime = millis();
  topHoraire = millis();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                             LOOP
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{

// ***************************************** Exploitation du Red Cube : calcul du débit carburant, de la quantité consommée, et de la quantité restante ****************************************************************  
if (FreqCount.available())  // Cette condition est vraie toutes les secondes, qu'il y ait eu ou non des impulsions au cours de la dernière seconde écoulée.
                            // Le code ci-dessous est donc exécuté très précisément toutes les secondes.                         
  { 
    totalNbSec ++;
    freq = FreqCount.read();
    NbTotalPulses = NbTotalPulses + freq;

    if (freq>=80) // Des mesures expérimentales sur le Red Cube ont montré que sa réponse est linéaire au-delà de 80 Hz. En dessous, la pente de la courbe s'infléchit, une petite correction est nécessaire
    {
      fuelFlow = (float)freq*3600.0/(kFactor*100.0);
      fuelUsed = fuelUsed + (float)freq/(kFactor*100.0);
    }
    else
    {
      fuelFlow = (float)freq*3600.0/(kFactor*100.0 * (0.0014 * (float)freq + 0.8912)); 
      fuelUsed = fuelUsed + (float)freq/(kFactor*100.0 * (0.0014 * (float)freq + 0.8912));
    }
    fuelFlow = filtrageRII (prevFiltrFuelFlowVal, fuelFlow, 0.3); // La périodicité de l'application du filtre étant la seconde, le coefficient de filtrage doit être modeste.
                                                                  // Mais on a déjà un bon filtrage en amont puisqu'on a échantilloné sur une seconde, 
                                                                  // on a ainsi déjà une fréquence moyenne des impulsions survenues durant cette seconde
    prevFiltrFuelFlowVal = fuelFlow;
    calcFuelLevel = startFuelLevel-fuelUsed;
    if ((fuelUsed > 1.0) &&(rpm == 0) && (flagOKwriteCalcFuelLevelEepromEndOfFlight==true))    // Si du carburant a été consommé et si le RPM est à zéro => vol terminé => on enregistre en EEPROM le niveau restant calculé.
    {
      EEPROM.put(fuelLevelEepromAddress, calcFuelLevel); // alors calcFuelLevel est sauvegardé en EEPROM
      flagOKwriteCalcFuelLevelEepromEndOfFlight = false; // Et ce flag est mis sur false pour ne pas faire plus d'un enregistrement après l'arêt du moteur
      Serial.println("calcFuelLevel sauvegardé en EEPROM");
    }
    if ((!flagOKwriteCalcFuelLevelEepromEndOfFlight) && (rpm>2000)) flagOKwriteCalcFuelLevelEepromEndOfFlight=true; //Mais si le moteur est remis en marche, un nouvel enregistrement sera possible
  }  

// ********************************************************* Mesure de la vitesse de rotation du moteur ****************************************************************  
  
  if (FreqMeasure.available())   // Cette condition est vraie dès qu'une impulsion a été détectée et que sa période a été mesurée
                                 // Le code ci-dessous est donc exécuté très souvent, à chaque tour complet du moteur (soit 30 à 90 fois par seconde environ)
  {
    rpm = FreqMeasure.countToFrequency(FreqMeasure.read())*60;  // La fréquence en Hz est calculée d'après la période
    nonFilteredRpmVal = rpm;                                    // On sauvegarde le rpm avant filtrage pour envoi ultérieur au CAN bus
    rpm = filtrageRII (prevFiltrRpmVal, rpm, 0.03);             // Comme le filtre est exécuté à chaque période du signal, on peut utiliser un coefficient élevé.
    prevFiltrRpmVal = rpm;
    zeroHandlingStartTime = millis();                           // On remet zeroHandlingStartTime sur le timing actuel
  }
  else                                                          // Si aucune impulsion n'a été détectée, on va vérifier le temps écoulé depuis la dernière impulsion détectée
  {
    if ((millis()- zeroHandlingStartTime)> zeroHandlingTimeOut) // Si ce temps dépasse 500 ms, c'est que le moteur est à l'arrêt
    {
      rpm = 0;
      nonFilteredRpmVal = 0;
    }
  }

//********************************************************** Mesure de la tension batterie, puis du niveau d'essence (jauge) *****************************************
  if((millis()- vBatFuelLevelMeasureStartTime) >= vBatFuelLevelMeasureUpdatePeriod)
  {
    vBatFuelLevelMeasureStartTime = millis();
    
    // -------------------------------------------------------- Mesure de la tension de la batterie -----------------------------------------------------------------
    uint16_t digitalVal = analogRead(pinVBat);
    vBat = (digitalVal*3.316/1023.0)*4.71;             // Résistances du pont diviseur : 8.2k et 2.21k (d'ou 8200+2210)/2210=4.71)
    vBat = vBat -0.01; // Compensation de l'erreur par rapport à une mesure faite avec un multimètre Fluke 115 aux bornes de la batterie0, pour une tension de 13.9 volts.
    vBat = filtrageRII (prevFiltrVBatVal, vBat, 0.05); 
    prevFiltrVBatVal = vBat; 
    if(vBat > vBatThreshold)                                    // Si la tension batterie est supérieure au seuil, LED verte allumée, LED rouge éteinte
    {
      lowVoltage = false;
      digitalWrite(pinGreenLED, HIGH);
      digitalWrite(pinRedLED, LOW); 
    }
    else
    {
      lowVoltage = true;
      if (rpm<rpmThreshold)                                   // Si la tension est inférieure au seuil, moteur au ralenti, LED rouge allumée fixe, LED verte éteinte
      {
        digitalWrite(pinGreenLED, LOW);
        digitalWrite(pinRedLED, HIGH); 
      }
      else                                                    // Si la tension est inférieure au seuil, et RPM > 2500, LED rouge clignote, LED verte éteinte
      {
        if(millis()-blinkRedLedStartTime > blinkRedLedTimeOut)
        {
          digitalWrite (pinRedLED, !digitalRead(pinRedLED));
          digitalWrite(pinGreenLED, LOW);
          blinkRedLedStartTime = millis();
        }
      }
    }

  //--------------------------------------------------- Puis mesure du niveau d'essence -------------------------------------------------------------------
  valA5 = (float)analogRead(pinFuelLevel);            // Acquisitions sur la broche A5
  valA5 = filtrageRII (prevFiltrvalA5, valA5, 0.05);  // filtrage
  prevFiltrvalA5 = valA5;  
  // puis on détermine l'index correspondant dans le tableau refTableFuelSender pour aller ensuite chercher les valeurs des bornes haute et basse
  while( IndexTable<fuelSenderNbSwitches+2) // on va parcourir toute la table depuis l'index 0
   {
     if (refTableFuelSender[IndexTable].numVal > valA5) // on recherche dans la table TabRefValA0 la première ligne dont la valeur numVal est supérieure à valA5.
                                                        //(d'où la nécessité d'avoir rajouté une valeur fictive numVal très élevée en dernière position de refTableFuelSender).
        {
          // et dès qu'on l'a trouvée, on peut chercher dans la table la valeur la plus proche, entre la première supérieure à valA5 et la précédente
          if ((refTableFuelSender[IndexTable].numVal-valA5)>=(valA5-refTableFuelSender[IndexTable-1].numVal))
             {
               upperLimit = refTableFuelSender[IndexTable-1].upperLimit;
               lowerLimit = refTableFuelSender[IndexTable-1].lowerLimit;
             }
          else
             {
               upperLimit = refTableFuelSender[IndexTable].upperLimit;
               lowerLimit = refTableFuelSender[IndexTable].lowerLimit; 
             }
          IndexTable = 0; // on remet l'index à 0 
          break; // et on sort de la boucle while
        }
      IndexTable++ ; // Si on n'a pas encore trouvé, on va tester l'élément suivant dans la table
   }
  IndexTable = 0; // Quand on a trouvé, on remet l'index à zéro pour la prochaine recherche 
  }

// *************************************************** Lecture de la position de l'encodeur et traitement éventuel de la modification d'un paramètre ********************************
// (L'encodeur ne doit évidemment être manipulé qu'immédiatement après la mise en marche du système, et avant toute consommation significative de carburant)
  currentEncoderPosition = encoder.read();
  if(currentEncoderPosition != previousEncoderPosition)
  {
    switch(menuOption)
    {
      case ' ':
        break;
      case 'U':
        calcFuelLevel = calcFuelLevel+(int16_t)encoder.getHoldDifference();  
        if (calcFuelLevel>fuelTankCapacity || calcFuelLevel<1) calcFuelLevel = 79.0;
        startFuelLevel = calcFuelLevel;
        fuelUsed = 0.0;
        flagOKwriteCalcFuelLevelEeprom = true;
        calcFuelLevelAdjustStartTime = millis();
        break;
      case 'D':
        calcFuelLevel = calcFuelLevel+(float)((int16_t)encoder.getHoldDifference())/10;
        if (calcFuelLevel>fuelTankCapacity || calcFuelLevel<1) calcFuelLevel = 79.0;
        startFuelLevel = calcFuelLevel;
        fuelUsed = 0.0;
        flagOKwriteCalcFuelLevelEeprom = true;
        calcFuelLevelAdjustStartTime = millis();
        break;
      case 'K':
        kFactor = kFactor+encoder.getHoldDifference();
        flagOKwriteKfactorEeprom = true;
        kFactorAdjustStartTime = millis();
        break;       
    }    
  }
  previousEncoderPosition = currentEncoderPosition;

// ****************************************************************************** Mise à jour des informations affichées à l'écran puis envoi au CAN bus **********************************************************
// variables à afficher : vBat, fuelFlow, calcFuelLevel, upperLimit, lowerLimit, rpm, et temps moteur
  if ((millis()-oledUpdateStartTime)>=oledUpdatePeriod)
  {
    oledUpdateStartTime=millis();
    uint32_t top = micros();
    //------------------------------- Affichage temps moteur -------------------------------
    nbHour = totalNbSec/3600;
    nbMin = (totalNbSec % 3600)/60;
    nbSec = (totalNbSec % 60);
    oled.setCursor (85,4);
    oled.setTextSize(1);
    oled.printf("%01u:%02u:%02u",nbHour,nbMin,nbSec);
    
    //---------------------------- Affichage de la tension batterie -----------------------
    oled.setTextSize(2);
    oled.setCursor (0,0);
    oled.printf("%5.2f",vBat);

    //----------------------------- Affichage du débit carburant ----------------------------
    oled.setCursor (17,21);
    oled.printf("%4.1f",fuelFlow);

    //----------------------------- Affichage du carburant consommé ----------------------------
    oled.setCursor (77,51);
    oled.printf("%4.1f",fuelUsed);

    // ---------------------------- Affichage du niveau carburant calculé -------------------
    oled.setCursor (8,51);
    oled.printf("%4.1f",calcFuelLevel);

    // --------------------------- Affichage limites haute et basse ------------------------
    oled.setCursor (85,82);
    oled.printf("%2u",upperLimit);
    oled.setCursor (20,82);
    oled.printf("%2u",lowerLimit);
    
    // ------------------------------ Affichage du RPM ------------------------------------
    rpmInt = (uint16_t)(rpm+0.5);
    rpmInt = 10*((rpmInt+5)/10);
    if(rpmInt<10000)
    {
      oled.setTextSize(3);
      oled.setCursor (3,107);
      oled.printf("%4u",rpmInt);
    }

    //--------------------------- Affichage éventuel du menu ------------------
    oled.setTextSize(1);
    switch(menuOption)
    {
      case ' ':
        oled.setCursor(85,103);
        oled.print("Menu :");
        oled.setCursor(85,111);
        oled.print("Press  ");
        oled.setCursor(85,119);
        oled.print("button ");
        break;
      case 'U':
        oled.setCursor(85,103);
        oled.print("Adjust ");
        oled.setCursor(85,111);
        oled.print("F. lev.");
        oled.setCursor(85,119);
        oled.print("U. dig.");
        break;
      case 'D':
        oled.setCursor(85,103);
        oled.print("Adjust ");
        oled.setCursor(85,111);
        oled.print("F. lev.");
        oled.setCursor(85,119);
        oled.print("D. dig.");        
        break;
      case 'K':
        oled.setCursor(85,103);
        oled.print("Adjust ");
        oled.setCursor(85,111);
        oled.print("K Fact");
        oled.setCursor(85,119);
        oled.printf("  %3u %s", kFactor, "   ");        
        break;      
    }
    
    //--------------------------- Mise à jour de l'écran -----------------------    
    oled.display();
    
    //-------------------------- Envoi des données au CAN bus ------------------
  
    //......................................Message msgFuelFlowAndLevel
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgFuelFlowAndLevel.buf[i] = ((byte*) &fuelFlow)[i];
      msgFuelFlowAndLevel.buf[i + 4] = ((byte*) &calcFuelLevel)[i];
    }
    CAN_Micro_EMS.write(msgFuelFlowAndLevel);

    //......................................Message msgFuelsenderRPMvBat
    msgFuelsenderRPMvBat.buf[0]= lowerLimit;
    msgFuelsenderRPMvBat.buf[1]= upperLimit;
    for (uint8_t i = 0; i < 2; i++ )
    {
      msgFuelsenderRPMvBat.buf[i + 2] = ((byte*) &rpmInt)[i];
    }
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgFuelsenderRPMvBat.buf[i + 4] = ((byte*) &vBat)[i];
    }
    CAN_Micro_EMS.write(msgFuelsenderRPMvBat);

    //......................................Message msgTotalNbSecFuelUsed
    for (uint8_t i = 0; i < 2; i++ )
    {
      msgTotalNbSecFuelUsed.buf[i] = ((byte*) &totalNbSec)[i];
    }
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgTotalNbSecFuelUsed.buf[i + 2] = ((byte*) &fuelUsed)[i];
    }
    CAN_Micro_EMS.write(msgTotalNbSecFuelUsed);

    //................................... Message msgrpm
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgrpm.buf[i] = ((byte*) &nonFilteredRpmVal)[i];
    }
    CAN_Micro_EMS.write(msgrpm);
  }

// ********************************************************************* Envoir des données sur Bluetooth ******************************************************************************************
  if ((millis()-BTSendUpdateStartTime)>=BTSendUpdatePeriod)
  {
    BTSendUpdateStartTime = millis();
    Serial2.print(';'); // Pour séparer d'avec l'horodatage fourni par le programme Android Serial Bluetooth Terminal

    Serial2.printf("%01u:%02u:%02u",nbHour,nbMin,nbSec);
    Serial2.print(';');
    Serial2.print (vBat,1);           Serial2.print(';');
    Serial2.print (fuelFlow,1);       Serial2.print(';');
    Serial2.print (calcFuelLevel,1);  Serial2.print(';');
    Serial2.print (fuelUsed,1);       Serial2.print(';');
    Serial2.print (lowerLimit);       Serial2.print(';');
    Serial2.print (upperLimit);       Serial2.print(';');
    Serial2.print (rpmInt);           Serial2.println(';');
  }

// ************************************************************************* Mise à jour éventuelle des informations stockées en EEPROM **********************************************************************

if (flagOKwriteKfactorEeprom &&((millis()-kFactorAdjustStartTime)> kFactorAdjustTimeOut))
{
  flagOKwriteKfactorEeprom = false;
  EEPROM.put(kFactorEepromAdress, kFactor); 
}

if (flagOKwriteCalcFuelLevelEeprom &&((millis()-calcFuelLevelAdjustStartTime)> calcFuelLevelAdjustTimeOut))
{
  flagOKwriteCalcFuelLevelEeprom = false;
  EEPROM.put(fuelLevelEepromAddress, calcFuelLevel); 
}

// ************************************************************************* Mesure de la durée de la boucle principale Loop ************************************************************************************

  if (nombreBoucleLoop >= Ndb)
  {
    dureeNdbBoucles = millis() - topHoraire;
    dureeLoop = (float)dureeNdbBoucles / Ndb;
    topHoraire = millis();
    nombreBoucleLoop = 0;
    //Serial.println (dureeLoop, 6);
  }
  else
  {
    nombreBoucleLoop++;
  }
  
}
//***************************************************************************************************** Fin de la fonction loop() ***********************************************************************************************

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                            Fonction utilisée pour le filtrage des données brutes issues des capteurs
////-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Fonction de filtre à Réponse Impulsionnelle Infinie (RII)
  float filtrageRII (float valeurFiltreePrecedente, float valeurCourante , float coeffFiltrageRII)
  {
    return valeurFiltreePrecedente  * (1 - coeffFiltrageRII) + valeurCourante * coeffFiltrageRII ;
  }

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                  Routine d'interruption liée au traitement d'une pression sur le bouton 
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Bouton_ISR() 
{
  if ((millis() - debounceStartTime) >= debounceTimeOut)
  {
    debounceStartTime = millis ();
    encoder.write(0);
    currentEncoderPosition = 0;
    previousEncoderPosition = 0;
    switch(menuOption)
    {
      case ' ':
        menuOption = 'U';
        break;
      case 'U':
        menuOption = 'D';
        break;
      case 'D':
        menuOption = 'K';
        break;
      case 'K':
        menuOption = ' ';
        break;      
    }
  }
}
