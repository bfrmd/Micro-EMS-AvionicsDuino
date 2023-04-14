
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                Connexions physiques des différents composants avec la carte Teensy 4.0
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// Signal compte tours sur pin 22 + bibliothèque FreqMeasure
// Signal fuel flow sur pin 9 + bibliothèque FreqCount
// Signal fuel level sur pin 16/A2
// Bouton incrémentation niveau essence sur pin 15/A1
// Bouton décrémentation niveau essence sur pin 14/A0
// Tension batterie sur pin 23/A9
// Red LED sur pin 19
// Green LED sur pin 20
// Ecran ILI9341 sur pins 4, 5, 10, 11, 12, 13
// Transceiver CAN sur CAN2 pins 0 et 1

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  Inclusions des bibliothèques et fichiers externes
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <FreqCount.h>
#include <FreqMeasure.h>
#include <SPI.h>
#include <ILI9341_t3n.h>
#include <ili9341_t3n_font_ComicSansMS.h>
#include "font_DroidSans.h"
#include "font_DroidSans_Bold.h"
#include "ili9341_t3n_font_ArialBold.h"
#include <EEPROM.h> 
#include <FlexCAN_T4.h> 
#include "intro.c"

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Création des objets
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TFT_CS              10
#define TFT_DC              5
#define TFT_MOSI            11
#define TFT_CLK             13
#define TFT_RST             4
#define TFT_MISO            12
#define TFT_BACKLIGHT       3           
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_CLK, TFT_MISO);   // Crée l'objet écran "tft"
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN_Micro_EMS;                              // Crée l'objet Flexan_T4 "CAN_Micro_EMS" en mode CAN 2.0.

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Déclarations des variables et constantes globales
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// ****************************************************************** Affectation des différentes broches **********************************************************************************************
#define pinFuelLevel A2          // Jauge connectée sur pin 16/A2
#define pinIncButton 15          // Bouton d'incrémentation sur la pin 15 (pour ajuster en début de vol la quantité d'essence présente dans le réservoir)
#define pinDecButton 14          // Décrémentation sur pin 14
#define pinVBat      A9          // Mesure tension batterie
#define pinRedLED    19          // Voyant vert
#define pinGreenLED  20          // Voyant rouge

//************************** Variables utilisées pour la mesure et le calcul du débit carburant, de la quantité de carburant consommée et de la quantité restante, grâce au Red Cube ******************************************************************
uint32_t NbTotalPulses = 0;      // nombre total d'impulsions du Red Cube enregistrées depuis la mise en route
uint16_t freq = 0.0;                // stocke la fréquence du Red Cube à un moment donné
uint32_t Kb;                     // facteur K de base pris en compte pour les calculs (en modifiant un peu le programme, cette variable pourra être ajusté via le menu de l'EMS et le CAN Bus)
float fuelUsed=0.0;              // Quantité d'essence consommée en litres depuis la mise en route
float memFuelUsed=0.0;           // Variable permettant de mémoriser la valeur de fuelUsed à un instant donné
float fuelFlow;                  // Débit instantané du carburant
float prevFiltrFuelFlowVal=0.0;  // Valeur filtrée précédente du débit carburant en L/h
#define fuelTankCapacity 79      // Capacité maximale du réservoir du F-PBCF (en litres)
float calcFuelLevel;             // stocke le niveau en litres calculé par le fuel flow   
float startFuelLevel;            // stocke le niveau de carburant ajusté par le pilote avec les boutons en début de vol
#define debounceTime 200         // Constante utilisée pour la gestion des rebonds des boutons d'ajustement du niveau de carburant
volatile uint32_t lastButtPress; // Mémorisation du timing de la dernière pression d'un bouton

//******************************************************* Déclaration des variables utilisées pour la gestion de l'EEPROM **********************************************
uint16_t fuelLevelEepromAddress;           // adresse de l'EEPROM à laquelle est mémorisée calcFuelLevel, la quantité d'essence présente dans le réservoir. Cette adresse change à chaque mise sous tension (en fait, à chaque setup)
                                           // Les 1024 premiers octets de l'EEPROM lui sont réservés, voir setup
uint32_t testEeprom;                       // Cette variable de test est utilisée uniquement pour déterminer si l'eeprom a (ou non) été déjà utilisée. Elle est stockée à l'adresse eeprom 1030.
#define testEepromAdress 1030
#define kFactorEepromAdress 1034           // adresse de stockage du facteur K de base Kb

//*********************************** Variables utilisées pour la mesure de la quantité de carburant dans le réservoir grâce à la jauge ******************************************************************
#define fuelSenderNbSteps 29     // La jauge Wema de F-PBCF comporte 29 reed switches, donc 29 pas (https://i.imgur.com/wIbZ90K.jpg)
float valA2 =0;                  // Valeur lue sur pin 16/A2, à convertir en niveaux de carburant dans le réservoir (bornes haute et basse) grâce au tableau d'étalonnage ci-dessous
float prevFiltrValA2;            // valeur filtrée précédente de valA2

typedef struct                   // Structure définissant les champs de chaque ligne du tableau refTableFuelSender initialisé ci-dessous : à chaque pas de la jauge Wema correspond une résistance,
                                 // donc une tension en sortie du pont de mesure, donc une valeur numérisée de 0 à 1023 (numVal) pour cette tension. Et à chaque pas correspondent des valeurs haute et basse
                                 // qui donnent la fourchette dans laquelle se situe la quantité d'essence restante dans le réservoir.
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
uint32_t zeroHandlingTimeOut;  // Variable permettant de mesurer le nombre de millisecondes entre deux requêtes FreqMeasure.available infructueuses, pour permettre la mesure d'une fréquence compte tours nulle
                               // Nativement, la fonction FreqMeasure est incapable de mesurer une fréquence nulle (en fait une absence de fréquence)
float rpm=0;                   // Nombre de tours moteur par minute
float prevFiltrRpmVal = 0;     // Valeur filtrée précédente du RPM
bool okDisplayData = false;    // Flag fixé sur true par timer toutes les 200 millisecondes
int16_t tachoNeedleAngle=90;   // Ancienne position de l'aiguille du compte tours (pour effacement avant affichage nouvelle position)
#define xTacho 120
#define yTacho 278
#define lengthNeedle 94

//******************************************************** Variables et constantes utilisées pour la gestion de la tension batterie et des voyants ********************************************************
float vBat=0.0, prevFiltrVBatVal = 12.7;
boolean lowVoltage=false;                                              //Indicateur de tension basse si la variable est vraie
uint16_t rpmThreshold = 2500;                                          // Seuil de RPM au dessus du quel la LED rouge clignote rapidement en cas de tension basse, sinon elle s'allume en continu (situation normale au ralenti)
float vBatThreshold = 13.25;                                           // En dessous de ce seuil, on agit sur les LED
uint32_t blinkRedLedTimeOut;                                           // Variable utilisée pour le clignotement de la LED rouge

//************************************************************************* Variables diverses **************************************************************************************************************
char buf[6];                         // buffer pour la fonction sprintf
uint32_t vBatFLevelTimeOut;          // Pour espacer les mesures de la tension batterie et du niveau d'essence 
uint32_t displayTimeOut;              // Pour espacer les affichages à l'écran (et les envois de données sur le CAN bus) 
uint16_t ILI9341_DARK_ORANGE;
CAN_message_t msg;

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             SETUP
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() 
{
//***************************************************************** Initialisation des bibliothèques ************************************************************ 
  Serial.begin(115200);
  FreqCount.begin(1000000);    // FreqCount compte le nombre de période pendant 1000000 µsec. Le résultat est donc une fréquence en Hz
  FreqMeasure.begin();

//*********************************************************** Initialisation de l'écran couleur ******************************************************************

  tft.begin(); 
  tft.setRotation(2); // Tourne l'affichage de 180°, l'écran étant utilisé "tête en bas" pour des raisons pratiques liées au montage dans le boîtier. 
  ILI9341_DARK_ORANGE =  tft.color565(255, 64, 0);
  displaySetup();    //  Ecran d'accueil

//************************************************************************** Traçage du compte tours **********************************************************
  tft.fillScreen(ILI9341_BLACK);
  for (int16_t x=-105; x<=+105; x+=7)
  {
    drawlineAngle(xTacho,yTacho, 110, x,ILI9341_WHITE);
  }
  tft.fillCircle(xTacho,yTacho,103,ILI9341_BLACK);
  
  for (int16_t x=-105; x<=+105; x+=35)
  {
    drawlineAngle(xTacho,yTacho, 110, x,ILI9341_WHITE);
  }
  tft.fillCircle(xTacho,yTacho,95,ILI9341_BLACK);
  drawArc(xTacho,yTacho,110,10, -106, -57, ILI9341_DARK_ORANGE);
  drawArc(xTacho,yTacho,110,10, -55, 17,ILI9341_YELLOW);
  drawArc(xTacho,yTacho,110,10, 19, 86,ILI9341_GREEN);
  drawArc(xTacho,yTacho,110,10, 88, 98,ILI9341_YELLOW);
  drawArc(xTacho,yTacho,110,10, 100, 106,ILI9341_DARK_ORANGE);
  displayTachoLabels();

// ***************************************************************** Affichages fixes *****************************************************************************
    tft.drawFastHLine(0,0,240,ILI9341_DARKGREY);  
    tft.setFont(Arial_20_Bold);
    tft.setCursor(35, 10);
    tft.print("Volts");
    tft.setFont(Arial_20_Bold);
    tft.setCursor(2, 48);
    tft.print("Fuel Flow");
    tft.setFont(Arial_12_Bold);
    tft.setCursor(2,82);
    tft.println("Fuel"); tft.print("used");
    tft.setCursor(2,122);
    tft.print("Low");

//****************************************************************** Initialisation du CAN Bus ********************************************************************
  CAN_Micro_EMS.begin();
  CAN_Micro_EMS.setBaudRate(500000);
  CAN_Micro_EMS.setMaxMB(16);
  CAN_Micro_EMS.enableFIFO();
  CAN_Micro_EMS.enableFIFOInterrupt();
  CAN_Micro_EMS.onReceive(FIFO,canSniff); // Le micro-EMS emet des données mais n'en attend aucune pour l'instant
  CAN_Micro_EMS.mailboxStatus();

//**************************************************************** Initialisation de certaines broches ***********************************************************

  pinMode(pinIncButton, INPUT_PULLUP);                  // On initialise les broches connectées aux boutons d'incrémentation ou décrémentation du niveau d'essence du réservoir.
  pinMode(pinDecButton, INPUT_PULLUP);                  // On active volontairement le pullup interne, le filtre RC anti rebond "hard" pour les boutons ne comporte ainsi qu'une seule résistance externe et un condensateur.
  attachInterrupt(pinIncButton, incButtonISR, FALLING); // Puis on attache les ISR à ces broches
  attachInterrupt(pinDecButton, decButtonISR, FALLING); 
  pinMode (pinFuelLevel, INPUT_DISABLE);
  pinMode (pinVBat, INPUT_DISABLE);
  pinMode (pinRedLED, OUTPUT);
  pinMode (pinGreenLED,OUTPUT);

// ****************************************************************** Initialisation des variables stockées en EEPROM *************************************************************  
// Ces variables sont le facteur K et le niveau d'essence dans le réservoir.
// Le niveau d'essence est mis à jour par le programme, à chaque fois qu'il varie d'un litre, avec une écriture dans l'EEPROM (émulée en FLASH) à chaque fois. La mémoire flash est donc beaucoup sollicitée en écriture.
// Pour éviter "l'usure" rapide d'un unique emplacement constant, on change cet emplacement à chaque setup, pour "répartir" dans le temps l'usage de la mémoire flash sur les 1025 premiers octets (0 à 1024 inclu) de l'EEPROM.

 EEPROM.get(testEepromAdress, testEeprom);          // On commence par tester la valeur entière 32 bits stockée à l'adresse testEepromAdress (1030)
 if (testEeprom != 12345678)                       // Si la valeur uint32_t stockée à l'adresse testEepromAdress n'est pas égale à 12345678, c'est que l'EEPROM n'a encore jamais été utilisée.
                                                    // Il faut donc initialiser toutes les variables, puis les inscrire ensuite dans l'EEPROM
 {
  testEeprom = 12345678;
  EEPROM.put(testEepromAdress, testEeprom);
  Kb = 17700;
  EEPROM.put(kFactorEepromAdress, Kb);
  calcFuelLevel = 40.0;                             // On attribue une valeur intermédiaire à calcFuelLevel
  EEPROM.write(0, 100);                             // On stocke un octet de valeur 100 à l'adresse 0. Un octet égal à 100 dans l'EEPROM indique que calcFuelLevel est stocké dans les 4 octets suivants.
  EEPROM.put (1, calcFuelLevel);                    // Puis on stocke la variable float calcFuelLevel dans les 4 octets suivants.
 } 
 else
 {                                                                                                
  EEPROM.get(kFactorEepromAdress, Kb);              // On récupère le facteur K dans l'EEPROM.
  for(int16_t i=0;i<1021;i+=5)                     // Et maintenant, on va chercher un octet égal à 100, en partant du début de l'EEPROM, en testant uniquement tous les 5 octets.
  {
    uint8_t j = EEPROM.read(i);                     // On lit la valeur de l'octet stocké à l'adresse i.
    if (j==100)                                     // S'il est égal à 100, c'est qu'on a trouvé l'emplacement où est stockée la valeur calcFuelLevel, à savoir les 4 octets suivants, qui commencent à l'adresse i+1.
    {
      Serial.println(i);
      EEPROM.get(i+1,calcFuelLevel);                // On attribue donc la valeur stockée à l'adresse i+1 à la variable calcFuelLevel,
      calcFuelLevel = int(calcFuelLevel+0.5);       // on arrondi au chiffre rond le plus proche, pour simplifier un éventuel ajustement manuel
      startFuelLevel = calcFuelLevel;               // et par défaut, avant un éventuel ajustement par le pilote en cas d'avitaillement, la même valeur est attribuée à startFuelLevel
      EEPROM.write(i,0);                            // puis on remet à zéro l'octet d'adresse i où était préalablement stockée la valeur 100.
      if (i==1020) i=-5;                            // Si on était au dernier emplacement attribué au stockage de calcFuelLevel, alors on doit repartir au début de l'EEPROM.
      EEPROM.write(i+5,100);                        // Maintenant, pour la suite du programme, on va stocker la variable calcFuelLevel 5 octets plus loin qu'avant, en la faisant précéder d'un octet 100,
      EEPROM.put(i+6, calcFuelLevel);               // puis on stocke calcFuelLevel à sa nouvelle adresse,
      fuelLevelEepromAddress = i+6;                 // on mémorise cette nouvelle adresse pour la suite du programme,
      break;                                        // et on sort immédiatement de la boucle for.
    }
  }
 }
 //Kb = 17700;                                        // En attendant d'implémenter une fonction d'ajustement du facteur K via les menus de l'EMS et le CAN bus
 //Kb=23200;
 Kb=21500;
//**************************************************************** Dernières initialisations diverses avant loop() ********************************************************************* 
  digitalWrite(pinRedLED, LOW); 
  digitalWrite(pinGreenLED, LOW);   
  zeroHandlingTimeOut = millis();
  blinkRedLedTimeOut = millis();
  vBatFLevelTimeOut = millis();
  displayTimeOut = millis();
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             LOOP
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() 
{
// ***************************************** Exploitation du Red Cube : calcul du débit carburant, de la quantité consommée, et de la quantité restante ****************************************************************  
  
  if (FreqCount.available())  // Cette condition est vraie s'il y a eu une ou des impulsions au cours de la dernière seconde écoulée, ce qui est toujours le cas après le lancement du moteur
                              // Le code ci-dessous est donc exécuté très précisément toutes les secondes.                         
  {
    freq = FreqCount.read();
    
    NbTotalPulses = NbTotalPulses + freq;
//    Serial.print("freq : "); Serial.print(freq); Serial.print("\t\t NbTotalPulses : "); Serial.println(NbTotalPulses);
    
    if (freq>=80)
    {
      fuelFlow = freq*3600.0/Kb;
      fuelUsed = fuelUsed + float(freq)/Kb;
    }
    else
    {
      fuelFlow = freq*3600.0/(Kb * (0.0014 * freq + 0.8912)); 
      fuelUsed = fuelUsed + float(freq)/(Kb * (0.0014 * freq + 0.8912));
    }
    if (fuelFlow>99.0) fuelFlow=99.0; // Pour prévenir un bug d'affichage (décimale à la ligne suivante) en cas de gros "hoquet" du Red Cube au lancement du système
    fuelFlow = filtrageRII (prevFiltrFuelFlowVal, fuelFlow, 0.3); // La périodicité de l'application du filtre étant la seconde, le coefficient de filtrage doit être modeste.
                                                                  // Mais on a déjà un bon filtrage en amont puisqu'on a échantilloné sur une seconde, 
                                                                  // on a ainsi déjà une fréquence moyenne des impulsions survenues durant cette seconde
    prevFiltrFuelFlowVal = fuelFlow;
    
    calcFuelLevel = startFuelLevel-fuelUsed;
    if ((fuelUsed-memFuelUsed)>=1)                       // Si un nouveau litre entier vient juste d'être consommé
    {
      EEPROM.put(fuelLevelEepromAddress, calcFuelLevel); // alors calcFuelLevel est sauvegardé en EEPROM
      memFuelUsed = fuelUsed;                            // et on repart pour un nouveau litre avant d'écrire en EEPROM
    } 
  }

//******************************************************* Vérification du niveau d'essence en cas d'ajustement par le pilote avant le début du vol *****************************************************
// il faut empêcher que les limites ne soient dépassées. 
// Une incrémentation manuelle ne doit pas dépasser la capacité du réservoir, et seule une décrémentation manuelle par le pilote permet d'arriver à un niveau inférieur à 1 litre.
// Le pilote peut, en décrémentant manuellement en dessous de 1 litre, repartir de la valeur constante fuelTankCapacity, après un remplissage du réservoir.
  if ((startFuelLevel>fuelTankCapacity) || (startFuelLevel<=1))
   {
    startFuelLevel = fuelTankCapacity; 
    calcFuelLevel = startFuelLevel;
   }

// ********************************************************* Mesure de la vitesse de rotation du moteur ****************************************************************  
  
  if (FreqMeasure.available())   // Cette condition est vraie dès qu'une impulsion a été détectée et que sa période a été mesurée
                                 // Le code ci-dessous est donc exécuté très souvent, à chaque tour complet du moteur (soit 30 à 90 fois par seconde environ)
                                 // Attention au piège que peut représenter le gros ralentissement de loop() lorsqu'un rafraichissement d'affichage a lieu !!!!!!!!
                                 // Il y a peut-être un risque que plusieurs mesures s'accumulent... A vérifier et tester +++++
  {
    rpm = FreqMeasure.countToFrequency(FreqMeasure.read())*60; // La fréquence en Hz est calculée d'après la période
    rpm = filtrageRII (prevFiltrRpmVal, rpm, 0.03);         // Comme le filtre est exécuté à chaque période du signal, on peut utiliser un coefficient élevé.
    prevFiltrRpmVal = rpm;
    zeroHandlingTimeOut = millis();                         // On remet zeroHandlingTimeOut sur le timing actuel
  }
  else                                                     // Si aucune impulsion n'a été détectée, on va vérifier le temps écoulé depuis la dernière impulsion détectée
  {
    if ((millis()- zeroHandlingTimeOut)> 500)              // Si ce temps dépasse 500 ms, c'est que le moteur est à l'arrêt
    {
      rpm = 0;
    }
  }
  if (rpm>6500) rpm = 6500; // Prévention d'un bug d'affichage en cas de "hoquet" du NCV1124 lors de la phase de tests et d'adaptation du circuit à la forme d'onde du capteur à reluctance variable du Rotax
//************************************** Toutes les 50 ms (20 Hz) : mesure de la tension batterie, puis du niveau d'essence (jauge) *****************************************
  if((millis()- vBatFLevelTimeOut)>=50)
  {
    vBatFLevelTimeOut = millis();
    // *********** Mesure de la tension de la batterie *******************
    uint16_t digitalVal = analogRead(pinVBat);
    vBat = (digitalVal*3.3/1023.0)*4.71;             // Résistances du pont diviseur : 8.2k et 2.21k (d'ou 8200+2210)/2210=4.71)
    vBat = filtrageRII (prevFiltrVBatVal, vBat, 0.05); 
    prevFiltrVBatVal = vBat; 
    
    if(vBat>vBatThreshold)                                    // Si la tension batterie est supérieure au seuil, LED verte allumée, LED rouge éteinte
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
        if(millis()-blinkRedLedTimeOut>300)
        {
          digitalWrite (pinRedLED, !digitalRead(pinRedLED));
          digitalWrite(pinGreenLED, LOW);
          blinkRedLedTimeOut = millis();
        }
      }
    }

  //*********** Puis mesure du niveau d'essence **********************************

  valA2 = float(analogRead(pinFuelLevel));            // Acquisitions sur la broche A2
  valA2 = filtrageRII (prevFiltrValA2, valA2, 0.05);  // filtrage
  prevFiltrValA2 = valA2;  
  // puis on détermine l'index correspondant dans le tableau refTableFuelSender pour aller ensuite chercher les valeurs des bornes haute et basse
  while( IndexTable<fuelSenderNbSteps+2) // on va parcourir toute la table depuis l'index 0
   {
     if (refTableFuelSender[IndexTable].numVal > valA2) // on recherche dans la table TabRefValA0 la première ligne dont la valeur numVal est supérieure à ValA2.
                                                        //(d'où la nécessité d'avoir rajouté une valeur fictive numVal très élevée en dernière position de refTableFuelSender).
        {
          // et dès qu'on l'a trouvée, on peut chercher dans la table la valeur la plus proche, entre la première supérieure à ValA2 et la précédente
          if ((refTableFuelSender[IndexTable].numVal-valA2)>=(valA2-refTableFuelSender[IndexTable-1].numVal))
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
      IndexTable++ ; // Si on n'a pas encore trouvé, on va tester l'élément suivant dans la tabe
   }
  IndexTable = 0; // Quand on a trouvé, on remet l'index à zéro pour la prochaine recherche 
  }
//********************************************************* Rafraichissement de l'écran et envoi de données au CAN Bus toutes les 200 ms ****************************************************************************

  if((millis()- displayTimeOut)>=200)
  {
    displayTimeOut = millis();
    
    // ---------------- Affichage du RPM -----------------------------
    uint16_t rpmInt = uint16_t(rpm+0.5);
    rpmInt = 10*((rpmInt+5)/10);
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);  
    tft.setFont(Arial_32_Bold);
    tft.setCursor(70,yTacho+10);
    sprintf(buf, "%4d", rpmInt);
    tft.print(buf);
    drawlineAngle(xTacho,yTacho, lengthNeedle, tachoNeedleAngle,ILI9341_BLACK); // tachoNeedleAngle indique encore l'ancien angle de l'aiguille
    tft.fillCircle(xTacho,yTacho,5,ILI9341_BLACK);
    tachoNeedleAngle = rpmInt*0.035-105;                                        // tachoNeedleAngle indique maintenant l'actuel angle de l'aiguille
    displayTachoLabels();
    tft.fillCircle(xTacho,yTacho,5,ILI9341_WHITE);
    drawlineAngle(xTacho,yTacho, lengthNeedle, tachoNeedleAngle,ILI9341_WHITE);

    // --------------- Affichage de la tension --------------------------
    tft.setFont(Arial_32_Bold);
    tft.setCursor(115, 3);
    if(lowVoltage)
    {
      tft.setTextColor(ILI9341_DARK_ORANGE,ILI9341_BLACK);
      tft.print(vBat,1);
      tft.print("  ");
      tft.fillRect(3,3,25,33,ILI9341_DARK_ORANGE);
      tft.fillRect(212,3,25,33,ILI9341_DARK_ORANGE);  
    }
    else
    {
      tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK); 
      tft.print(vBat,1);
      tft.print("  ");
      tft.fillRect(3,3,25,33,ILI9341_GREEN);
      tft.fillRect(212,3,25,33,ILI9341_GREEN);
    }
    tft.drawFastHLine(0,38,240,ILI9341_DARKGREY);
    
    // ----------------- Affichage du fuel flow ----------------------------
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK); 
    tft.setFont(Arial_32_Bold);
    tft.fillRect(143,42,96,50,ILI9341_BLACK);
    tft.setCursor(144, 43);
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK); 
    tft.print(fuelFlow,1);
    tft.drawFastHLine(0,78,240,ILI9341_DARKGREY);   

    // ----------------- Affichage des niveaux de carburant ----------------
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);     
    tft.setFont(Arial_12_Bold);
    tft.setCursor(145,83);
    tft.print("Calc.");
    tft.setCursor(145,100);
    tft.print("level");
    tft.setFont(Arial_32_Bold);
    tft.setCursor(190,83);
    tft.print(calcFuelLevel,0);
    tft.setCursor(45, 82);
    tft.print(fuelUsed,1); tft.print(" ");
    tft.drawFastHLine(0,119,240,ILI9341_DARKGREY); 
    tft.setFont(Arial_12_Bold);
    tft.setCursor(85,123);
    tft.println("Measured");  
    tft.setCursor(104,140);
    tft.print("level");  
    tft.setCursor(202,123);
    tft.print("High");
    tft.fillRect(0,143,44,28,ILI9341_BLACK);
    tft.setCursor(0, 143);
    tft.setFont(Arial_28_Bold);
    tft.print(lowerLimit);   
    tft.setCursor(196, 143);
    tft.print(upperLimit);      

    // ----------------- Envoi des données au CAN Bus à destination de l'EMS ---------------- 
    msg.id = 61; 
    uint8_t b;
    msg.buf[0]= uint8_t(calcFuelLevel+0.5);                 // Conversion du float calcFuelLevel en un entier non signé sur 8 bits, arrondi au litre le plus proche
    msg.buf[1]= lowerLimit;
    msg.buf[2]= upperLimit;
    uint16_t fuelFlowInt = uint16_t((fuelFlow*10)+0.5); // On converti le float fuelFlow exprimé en L/h en un uint16_t exprimé en décilitres/h, arrondi au décilitre le plus proche
    b=fuelFlowInt & 0xFF;                               // fuelFlowInt est de type uint16_t, donc sur 2 octets à passer l'un après l'autre sur le CAN Bus. Ici on extrait l'octet faible
    msg.buf[3]= b;
    b=fuelFlowInt>>8;                                   // puis l'octet fort
    msg.buf[4]= b;
    b = uint8_t((vBat*10)+0.5);                         // On converti le float vBat exprimé en volts en un uint8_t exprimé en décivolts, arrondi au décivolt le plus proche
    msg.buf[5] = b;
    b=rpmInt & 0xFF;                                    // rpmInt est de type uint16_t, donc sur 2 octets à passer l'un après l'autre sur le CAN Bus. Ici on extrait l'octet faible
    msg.buf[6]= b;
    b=rpmInt>>8;                                        // puis l'octet fort
    msg.buf[7]= b;  
    CAN_Micro_EMS.write(msg);                           // envoi du message
  }
  
}
//***************************************************************************************************** Fin de la boucle infinie Loop ***********************************************************************************************

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                            Fonction utilisée pour le filtrage des données brutes issues des capteurs
////-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Fonction de filtre à Réponse Impulsionnelle Infinie (RII)
  float filtrageRII (float valeurFiltreePrecedente, float valeurCourante , float coeffFiltrageRII)
  {
    return valeurFiltreePrecedente  * (1 - coeffFiltrageRII) + valeurCourante * coeffFiltrageRII ;
  }

//------------------------------------------------------------------------------------------------------------------------------
// Deux routines d'interruptions sur les broches 14 et 15 (boutons d'ajustage de la quantité d'essence dans le réservoir. Au parking, AVANT DE COMMENCER LE VOL !!)
//------------------------------------------------------------------------------------------------------------------------------

  void incButtonISR()
  {
    if ((millis() - lastButtPress) > debounceTime)
    {
      startFuelLevel=startFuelLevel+1.0;
      calcFuelLevel=calcFuelLevel+1.0;
      fuelUsed = 0.0;
      memFuelUsed = 0.0;
      lastButtPress = millis ();
    }  
  }
   
  void decButtonISR()
  {
    if ((millis() - lastButtPress) > debounceTime)
    {
      startFuelLevel=startFuelLevel-1.0;
      calcFuelLevel=calcFuelLevel-1.0;
      fuelUsed = 0.0;
      memFuelUsed = 0.0;
      lastButtPress = millis ();
    }  
  }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   ISR de réception du CAN Bus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  void canSniff(const CAN_message_t &msg) 
  {
    
  }

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                       Fonctions graphiques
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void drawlineAngle(uint16_t x,uint16_t y, float dist, float angle,uint16_t color)
{
  tft.drawLine (x,y,x+cos(angle*0.0174532925-PI/2)*dist, y+sin(angle*0.0174532925-PI/2)*dist,color);
}

void drawArc(uint16_t x, uint16_t y, uint16_t radius, uint16_t thickness, float startAngle, float endAngle, uint16_t color)
{
  for (int i = startAngle; i < endAngle; i += 1) 
  {
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (radius + thickness) + x;
    uint16_t y0 = sy * (radius + thickness) + y;
    uint16_t x1 = sx * radius + x;
    uint16_t y1 = sy * radius + y;
    float sx2 = cos((i + 1 - 90) * 0.0174532925);
    float sy2 = sin((i + 1 - 90) * 0.0174532925);
    int x2 = sx2 * (radius + thickness) + x;
    int y2 = sy2 * (radius + thickness) + y;
    int x3 = sx2 * radius + x;
    int y3 = sy2 * radius + y;
    tft.fillTriangle(x0, y0, x1, y1, x2, y2, color);
    tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
  }   
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                       Quelques fonctions utilitaires
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void displaySetup () // Gère l'affichage bref du nom et de la version du programme lors de la mise sous tension du système
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE); 
  tft.setTextSize(2);
  tft.setFont(DroidSans_20_Bold);
  tft.setCursor(0,10);
  tft.print ("Micro-EMS - V 3.2");
  // Pause de qq secondes, avec une photo défilante
  for (int16_t i=-240; i<240; i+=2)
  {
  tft.writeRect(i, 70, photo.width, photo.height, (uint16_t*)(photo.pixel_data));
  }                                     
}

void displayTachoLabels()
{
  tft.setTextColor(ILI9341_WHITE);  
  tft.setFont(DroidSans_14_Bold);
  tft.setCursor(100, yTacho-40);
  tft.print("RPM");
  tft.setCursor(30,yTacho+17);
  tft.print("0");
  tft.setCursor(31,yTacho-35);
  tft.print("10");
  tft.setCursor(58,yTacho-76);
  tft.print("20");
  tft.setCursor(110,yTacho-93);
  tft.print("30");
  tft.setCursor(162,yTacho-76);
  tft.print("40");
  tft.setCursor(190,yTacho-35);
  tft.print("50");
  tft.setCursor(193,yTacho+17);
  tft.print("60");
} 
