/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
///////////////////////Code-Info/////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/*
Dies ist der fertige Coderelease von unserem Roboter 2012/2013.
 
 In ihm sind alle Grundfunktionen enthalten. Bei der Ballverfolgung
 wurden teile vereinfacht, damit nicht jeder die selbe Verfolgung hat.
 Man muss sich selber Gedanken machen. Ziel ist es den Einstiegsaufwand
 möglichst gering zu halten.
 
 Fahreigenschaften: Der Roboter fährt immer mit der Ballcapturingzone
 in Richtung gegnerisches Tor und hat dabei eine kleine Toleranz.
 Während der Fahrt wird immer nachkorregiert, sodass er möglichst immer
 zum gegnerischen Tor schaut. Diese Implimentierung haben wir rausgenommen,
 da sich jedes Team eigene Gedanken machen soll, wie sie das realisieren.
 Der Grundgedanke ist beschrieben.
 
 Beide Roboter sind identisch aufgebaut und fahren mit der selben Software.
 Eigenschaften werden über Jumper geregelt, derzeit nur Bluetoothmaster
 oder Bluetoothslave.
 
 Einige Umsetzungen mögen nicht schön sein, aber sie funktionieren.
 Viele Sachen sind global deklariert, was die Wiedreverwandbarkeit
 erschwert. Wir empfehlen so wenig wie möglich globale Variablen zu
 benutzen oder ganz genau darauf achtet das man Sie nicht unabsichtlich
 überschreibt.
 
 Die Software ist auf die auch veröffentlichte Hardware angepasst.
 
 Einbinden der Wire Bibliothek:
 http://www.arduino.cc/en/Hacking/Libraries
 
 Bei Verbesserungen/Anregungen und Verständnisproblemen schriebt uns an:
 frt.robotik@gmail.com
 
 Bitte teilt uns mit wenn ihr Code von hier übernehmt. Wenn mehr als 50%
 dieses Codes verwendet werden sollten, schreibt uns eine Mail.
 
 Wir verbieten es nicht den Code zu nehmen, wir wollen ledglich wissen,
 ob die Offenlegung etwas bringt.
 
 Die Software wurde geschrieben von:
 Tobias Hübner - Low Level Ebene(Grundfunktionen)
 Duc Pham - High Level Ebene(Taktik)
 
 Hardware:
 Jan Phillip Foermer(Desgin, Konstruktion, Platine)
 
 Hinweise: Kommunikation - wird nicht verwendet in der Taktik 
 
 *////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
///////////////////////Include///////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
#include <Wire.h>//I2C Bibliothek
#include "EEPROM.h"//EEPROM Bibliothek zum schrieben auf den festen Speicher
#include <SoftwareSerial.h>//Simuliert Serielleschnittstelle für das Display

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////Pin-Mapping//////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#define SofwareReset TRUE                                  //True, der fix wir düber Wire.begin() gemacht
//False, der Reset vom Ardunioboard wird genutzt

//Leds
#define StatusLed                    22                    //Led auf dem Tower der I2C-Sensoren

//Taster für die 6 Schalter
#define s0i                          35
#define s1i                          33
#define s2i                          37
#define s3i                          27 
#define s4i                          29
#define s5i                          39

//Jumper auf dem Board, zur Detektierung ob die Module Aktiv sind
#define DisplayJumper                25
#define BluetoothJumper              49

//Ports für die Lichtschranke
#define ReciverStrom                 26
#define EmitterStrom                 36 
#define LichtschrankeRead            14

//Strom für die TSOP's
#define cballStrom                   24

//Motorpins für die Richtung
#define DirectionPort11              50
#define DirectionPort12              52
#define DirectionPort21              46
#define DirectionPort22              48
#define DirectionPort31              42
#define DirectionPort32              44

//Motorpins für die Geschwindigkeit
#define SpeedPort1                    2
#define SpeedPort2                    3
#define SpeedPort3                    4

//Arduinoresetpin
#define PINRESET                     23

//Displaypin
#define txPinDisplay                 19

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////I2C//////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
//keine Gleichenardessen benutzen!
//Kompassadresse
#define CmpsAddr                   0x60

//Ultraschalladressen
#define xUS1                       0x73
#define xUS2                       0x70
#define yUS1                       0x72
#define yUS2                       0x71

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////Konstaten////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//Kompass
#define Richtwert                   360                                             //Kompasswert soll maximal 360 sein

//Ultraschall
// x
#define XMittelinks                 36                                              //diverse Angaben zur Mitte
#define XMitterechts                42
#define XMitte                      39

#define XMnahlinks                  20                                              //Nah an der Wand angaben
#define XMnahrechts                 63
#define XMnahnahlinks               30                                             
#define XMnahnahrechts              53


// y
#define YMittevorn                  28                                              // diverse y angaben
#define YMittehinten                22
#define YMitte                      25
#define YBalleckwert                33
#define YMittevornball              21                                            


#define YMnahvorn                   55                                            
#define YMnahnahvorn                40                                             
#define YMax                       160
#define anderwand                   12                                              // an der linken/ rechten/ vorderen Wand

//cball
#define ballcapwert                160                                              // Lichtwertgrenze wenn jemand in der Lichtschranke ist
#define max_background             400                                              // Mindestlichtwert; über diesem Wert reagiert der Roboter nicht auf Licht
#define max_lichtschranke          250                                              // Maximalwert der Lichtschranke, drüber kaputt

#define shifting                    28                                              //Verschiebung um die RAW Werte der Dioden zu lesen

//Timer
#define delayUS                    15
#define delayLS                     1
#define delayCS                     2
#define LCDdelay                   10

//EEPROM Angaben der Positionen im Speicher
#define KompassNordEEPROM          10
#define KompassSudEEPROM           20
#define KompassNahNahNordLinks     30
#define KompassNahNahNordRechts    40


byte debugSensorEEPROM[15] = {
  50, 55, 60, 65, 70, 75, 80,
  85, 90, 95 , 100, 105, 110, 115, 120};


//Bereich in dem die Sensoren funktionstüchtig scheinen 
#define TsopLow                    18
#define TsopHigh                 1000

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////GlobaleVariablen/////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//Kompassmanipulationswerte
int NullKorrektur; //Gegnerischestor(Norden)
int SUD; //Eigenestor(Süden)
int nahnahNORDlinks; //maximale Ungenauigkeit nach Westen
int nahnahNORDrechts; //maximale Ungenauigkeit nach Osten


//Roboterpositionsangaben
int dRob;//manipulierte Richtung des Roboters
int x1, x2;//x Position des Roboters, links und rechts
int y1, y2;//y Position des Roboters, vorne und hinten

//cball
int CballR[42];//Werte der einzelnen Sensoren
//0-13 Sortierte RAW
//14-27 Sortierte Dioden
//28-41 unsortierte RAW
byte Dioden[14];                                                                       //Ports der Dioden
boolean debugSensorFail[15];                                                           //Sensorfunktioniertarray
byte cReset = 0;                                                                       //Zählvariable für das entladen der Kodensatoren der TSOP's
boolean ballcap = false;                                                               //Angabe zum Ball in den Capturingzone
int max_detector;                                                                      //Diode die am stärksten angestrahlt wird
boolean habeball = false;                                                              //Angabe zum sehen des Balls auf dem SPielfeld
boolean Defekt = false;                                                                //Angabe ob mind ein Sensor defekt ist
int drill = 0;                                                                         //Hilfsvariable am Start für die jeweilige Richtung in die gestartet wird

//Taster
boolean s[8];                                                                          //Angabe ob Taster/Jumper betätigt ist

//Timer
unsigned long Timer[11];                                                               //Speicherung der Zeit des Timers

//Status
boolean r = false;                                                                     //Aufenthalt auf der rechten Spielfeldhälfte
boolean l = false;                                                                     //Aufenthalt auf der linken Spielfeldhälfte
boolean started = true;                                                                //Roboter am fahren?

char myStat, getStat;                                                                  //Status von mir, vom anderen Bot

//LCD
SoftwareSerial LCD = SoftwareSerial(0, txPinDisplay);                                  //Instanz von LCD
int ebene = 0;                                                                         //Angabe zum Menüpunkt in dem man sich befindet
int LSensor = 0;                                                                       //Angabe welcher Sensor angeschaut wird

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////LEDS////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//Initialisiert die Led auf dem Tower
void initLed(){
  pinMode(StatusLed,OUTPUT);
}

//Led auf dem Tower an bzw ausschalten
void led(boolean on){
  if(on)digitalWrite(StatusLed, HIGH);//An, true
  else digitalWrite(StatusLed, LOW);//Aus, false
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////Taster///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
//setzen aller Schalter,Jumperports als Input
void initSchalter(){
  pinMode(s0i,INPUT);
  pinMode(s1i,INPUT);
  pinMode(s2i,INPUT);
  pinMode(s3i,INPUT);
  pinMode(s4i,INPUT);
  pinMode(s5i,INPUT);
  pinMode(BluetoothJumper,INPUT);
  pinMode(DisplayJumper,INPUT);

}

//Liest alle Schalter,Jumperports aus
void ReadSchalter(){
  s[0] = digitalRead(s0i);
  s[1] = digitalRead(s1i);
  s[2] = digitalRead(s2i);
  s[3] = digitalRead(s3i);
  s[4] = digitalRead(s4i);
  s[5] = digitalRead(s5i);
  s[6] = digitalRead(BluetoothJumper);
  s[7] = digitalRead(DisplayJumper);
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////Bluetooth////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void initMaster(){
  Serial3.begin(9600);
  Serial3.print("$$$");//Internen Bereich betreten
  delay(50);
  Serial3.println("SM,1");//Mastermode einschalten
  delay(50);
  Serial3.println("C,00066645B71F");//connect to Slave
  delay(50);
  Serial3.println("---");//Internen Bereich verlassen
}

void initSlave(){
  Serial3.begin(9600);
  Serial3.print("$$$");//Internen Bereich betreten
  delay(50);
  Serial3.println("SM,0");//Slavemode einschalten
  delay(50);
  Serial3.println("---");//Internen Bereich verlassen
}


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
///////////////////////////RESETTING/////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

/*
Auf dem Arduinoboard gibt es den Port Reset
 Wenn dieser auf low gezogen wird, dann wird der
 Arduino neu gestartet
 pinMode(INPUT) bewirkt ein high, setzen wir pinMode(OUTPUT),
 so wird der Arduino resettet
 
 Grund für diesen Fehler ist das zu schnelle Auslesen der Sensoren,
 dadurch kommt es zu Kolisionen auf dem Bus und er stützt ab.
 Er hängt in einer Schleife fest aus der er nicht rauskommt.
 Dieser Bug wurde gepatcht und kann in der twi.c nachvollzogen werden.
 In einer der ersten Versionen konnte man nicht über twi_init den Bus
 neustarten, mittlerweile geht dies, daher empfehlen wir die
 Softwarereset Variante. Beides ist aber möglich, solang auf dem Board
 der Jumper dafür aufgesteckt ist.
 */
void initReset(){
  pinMode(PINRESET, INPUT);
}

void fuaRESET(){
#if SoftwareReset
  Wire.begin();
#else
  pinMode(PINRESET, OUTPUT);
  delay(50);//an diese Stelle kommt das Programm gar nicht, aber zur Verdeutlichung was 
  pinMode(PINRESET, INPUT);//danach passiert ist es hier ausgeschrieben 
#endif
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////Ultraschall/////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
//die Ultraschallwerte x1,x2,y1,y2 sind global
/*
Die Init wird zu jedem Start aufgerufen. Eigentlich muss dies nur einmal
 gemacht werden, da der SRF10 die gestzeten Reichweiten und Empfindlichkeiten
 speichert. Sicher ist sicher, daher wird es jedesmal neugeschrieben.
 Weiter Infos zum Sensor: http://www.robot-electronics.co.uk/htm/srf10tech.htm
 */
int initUS(){
  int x = 0x00;

  Wire.beginTransmission(xUS1);//Verbindung herstellen
  Wire.write(0x01); // SRF10 Register 1
  Wire.write(x); // Empfindlichkeit, Wert 0 bis 16; 0 geanu; 16 ungenau
  Wire.endTransmission();//Verbindung beenden

    Wire.beginTransmission(xUS1);//Verbindung herstellen
  Wire.write(0x2); // SRF10 Register 2
  Wire.write(100); // Reichweite, Wert (43mm*Register2)+43mm; z.B. 43*64+43=2795mm
  Wire.endTransmission();//Verbindung beenden 

    Wire.beginTransmission(xUS2);//Verbindung herstellen
  Wire.write(0x01); // SRF10 Register 1
  Wire.write(x); // Empfindlichkeit, Wert 0 bis 16
  Wire.endTransmission();//Verbindung beenden

    Wire.beginTransmission(xUS2);//Verbindung herstellen
  Wire.write(0x2); // SRF10 Register 2
  Wire.write(100); // Reichweite, Wert (43mm*Register2)+43mm; z.B. 43*64+43=2795mm
  Wire.endTransmission();//Verbindung beenden 

    Wire.beginTransmission(yUS1);//Verbindung herstellen
  Wire.write(0x01); // SRF10 Register 1
  Wire.write(x); // Empfindlichkeit, Wert 0 bis 16
  Wire.endTransmission();//Verbindung beenden

    Wire.beginTransmission(yUS1);//Verbindung herstellen
  Wire.write(0x2); // SRF10 Register 2
  Wire.write(100); // Reichweite, Wert (43mm*Register2)+43mm; z.B. 43*64+43=2795mm
  Wire.endTransmission();//Verbindung beenden 

    Wire.beginTransmission(yUS2);//Verbindung herstellen
  Wire.write(0x01); // SRF10 Register 1
  Wire.write(x); // Empfindlichkeit, Wert 0 bis 16
  Wire.endTransmission();//Verbindung beenden

    Wire.beginTransmission(yUS2);//Verbindung herstellen
  Wire.write(0x2); // SRF10 Register 2
  Wire.write(100); // Reichweite, Wert (43mm*Register2)+43mm; z.B. 43*64+43=2795mm
  Wire.endTransmission();//Verbindung beenden
}

/*
Diese Funktion liest die Werte aus den Ultraschallsensoren aus und
 speichert sie in den globalen Variablen ab.
 
 Zunächst werden Messbefehler an jeden Sensor (fast) gleichzeitig geschickt.
 Falls die Ultraschallsensoren sehr schlecht Werte zurückliefern, sollten diese
 in einem gewissen Zeit Abstand angesprochen werden. Es kan zu Überlagerungen
 der Ultraschallwellen kommen, bei uns war dies nicht der Fall, kann aber auftreten.
 
 Danach werden 15ms gewartet, da der Sensor diese Zeit zum Errechnen der Werte brauch.
 Diese Zeit muss je nach Entfernung angepasst werden. Bei kurzen Distanzen kann die Zeit geringer
 werden. Bei großen Distanzen muss sie zwangsläufig größer werden.
 */
int readUS(){
  int i = 0x00;

  Wire.beginTransmission(xUS1);//Verbindung herstellen
  Wire.write(i); // Sende Befehl zur Messung
  Wire.write(0x51); // Messergebnis in Millimetern
  Wire.endTransmission();//Verbindung beenden

    Wire.beginTransmission(xUS2);//Verbindung herstellen
  Wire.write(i); // Sende Befehl zur Messung
  Wire.write(0x51); // Messergebnis in Millimetern
  Wire.endTransmission();//Verbindung beenden

    Wire.beginTransmission(yUS1);//Verbindung herstellen
  Wire.write(i); // Sende Befehl zur Messung
  Wire.write(0x51); // Messergebnis in Millimetern
  Wire.endTransmission();//Verbindung beenden 

    Wire.beginTransmission(yUS2);//Verbindung herstellen
  Wire.write(i); // Sende Befehl zur Messung
  Wire.write(0x51); // Messergebnis in Millimetern
  Wire.endTransmission();//Verbindung beenden 

    delay(10);  

  Wire.beginTransmission(xUS1);//Verbindung herstellen
  Wire.write(0x02);// Messung anfordern 
  Wire.endTransmission();//Verbindung beenden       
  if (Wire.requestFrom(xUS1,2) == 255){//Daten empfangen
    return 0;
  }
  byte x1a = Wire.read();//Byte lesen
  byte x1b = Wire.read();//Byte lesen
  x1 = (x1a << 8) + x1b;//Bytes zusammensetzen

  Wire.beginTransmission(xUS2);//Verbindung herstellen
  Wire.write(0x02);// Messung anfordern 
  Wire.endTransmission();//Verbindung beenden

    if (Wire.requestFrom(xUS2,2) == 255){//Daten empfangen
    return 0;
  }
  byte x2a = Wire.read();//Byte lesen
  byte x2b = Wire.read();//Byte lesen
  x2 = (x2a << 8) + x2b;//Bytes zusammensetzen

  Wire.beginTransmission(yUS1);//Verbindung herstellen
  Wire.write(0x02);// Messung anfordern 
  Wire.endTransmission();//Verbindung beenden      

    if (Wire.requestFrom(yUS1,2) == 255){//Daten empfangen
    return 0;
  }

  byte y1a = Wire.read();//Byte lesen
  byte y1b = Wire.read();//Byte lesen
  y1 = (y1a << 8) + y1b;//Bytes zusammensetzen

  Wire.beginTransmission(yUS2);//Verbindung herstellen
  Wire.write(0x02);// Messung anfordern
  Wire.endTransmission();//Verbindung beenden      

    if (Wire.requestFrom(yUS2,2) == 255){//Daten empfangen
    return 0;
  }
  byte y2a = Wire.read();//Byte lesen
  byte y2b = Wire.read();//Byte lesen
  y2 = (y2a << 8) + y2b;//Bytes zusammensetzen

  return 1;
}

/*
In der Timer Version werden zunächst die Messbefehle gegeben.
 Danach läuft das Prorgamm ganz normal weiter und andere Sensoren
 könne ausgelsen werden und Motoren angesteuert werden.
 Nachdem der Timer abgelaufen ist, wird der Sensor ausgelesen.
 Dies bringt einen Vorteil für die Ballverfolgung, da dort sehr
 schnell reagiert werden muss.
 */
int sendUSTimer(){
  int i = 0x00;

  Wire.beginTransmission(xUS1);//Verbindung herstellen
  Wire.write(i); // Sende Befehl zur Messung
  Wire.write(0x51); // Messergebnis in Millimetern
  Wire.endTransmission();//Verbindung beenden 


    Wire.beginTransmission(xUS2);//Verbindung herstellen
  Wire.write(i); // Sende Befehl zur Messung
  Wire.write(0x51); // Messergebnis in Millimetern
  Wire.endTransmission();//Verbindung beenden

    Wire.beginTransmission(yUS1);//Verbindung herstellen
  Wire.write(i); // Sende Befehl zur Messung
  Wire.write(0x51); // Messergebnis in Millimetern
  Wire.endTransmission();//Verbindung beenden 

    Wire.beginTransmission(yUS2);//Verbindung herstellen
  Wire.write(i); // Sende Befehl zur Messung
  Wire.write(0x51); // Messergebnis in Millimetern
  Wire.endTransmission();//Verbindung beenden 

    return 1;
}

int readUSTimer(){
  Wire.beginTransmission(xUS1);//Verbindung herstellen
  Wire.write(0x02);// Messung anfordern 
  Wire.endTransmission();//Verbindung beenden

    if (Wire.requestFrom(xUS1,2) == 255){//Daten empfangen
    return 0;
  }
  byte x1a = Wire.read();//Byte lesen
  byte x1b = Wire.read();//Byte lesen
  x1 = (x1a << 8) + x1b;//Bytes zusammensetzen

  Wire.beginTransmission(xUS2);//Verbindung herstellen
  Wire.write(0x02);// Messung anfordern 
  Wire.endTransmission();//Verbindung beenden

    if (Wire.requestFrom(xUS2,2) == 255){//Daten empfangen
    return 0;
  }
  byte x2a = Wire.read();//Byte lesen
  byte x2b = Wire.read();//Byte lesen
  x2 = (x2a << 8) + x2b;//Bytes zusammensetzen

  Wire.beginTransmission(yUS1);//Verbindung herstellen
  Wire.write(0x02);// Messung anfordern 
  Wire.endTransmission();//Verbindung beenden       

    if (Wire.requestFrom(yUS1,2) == 255){//Daten empfangen
    return 0;
  }
  byte y1a = Wire.read();//Byte lesen
  byte y1b = Wire.read();//Byte lesen
  y1 = (y1a << 8) + y1b;//Bytes zusammensetzen

  Wire.beginTransmission(yUS2);//Verbindung herstellen
  Wire.write(0x02);// Messung anfordern 
  Wire.endTransmission();//Verbindung beenden

    if (Wire.requestFrom(yUS2,2) == 255){//Daten empfangen
    return 0;
  }
  byte y2a = Wire.read();//Byte lesen
  byte y2b = Wire.read();//Byte lesen
  y2 = (y2a << 8) + y2b;//Bytes zusammensetzen

  return 1;
}


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////Cball////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/*
Der Cball ist ein Sammelbegriff für unser TSOP's.
 Initialisierung des Cball. Setzen aller Ausgänge
 und füllen der Arrays mit der Position des TSOP's
 */
void initCball(){
  //Array füllen
  Dioden[0]  = 0;
  Dioden[1]  = 1;
  Dioden[2]  = 2;
  Dioden[3]  = 3;
  Dioden[4]  = 4;
  Dioden[5]  = 5;
  Dioden[6]  = 6;
  Dioden[7]  = 7;
  Dioden[8]  = 8;
  Dioden[9]  = 9;
  Dioden[10] = 10;
  Dioden[11] = 11;
  Dioden[12] = 12;
  Dioden[13] = 13;

  //Ausgänge stzen
  pinMode(EmitterStrom,OUTPUT);
  pinMode(ReciverStrom,OUTPUT);
  pinMode(cballStrom, OUTPUT);

  //einschalten
  digitalWrite(EmitterStrom, HIGH);
  digitalWrite(ReciverStrom,HIGH); 
  digitalWrite(cballStrom,HIGH);  

}

/*
Auslesen aller Lichtsensoren des Roboters
 Die Idee zum auschalten der Sensoren kommt
 vom Team C-Palb. http://www.c-palb.de/cms/q_a.html
 
 Diese Idee haben wir ein wenig modifiziert und auf unseren
 Roboter angepasst.
 */
void readCball(){
  if (cReset > 5){//nach jedem 6. mal Sensoren resetten
    digitalWrite(cballStrom,LOW);//Sensoren ausschalten, Kondensatoren entladen
    delay(2);
    digitalWrite(cballStrom,HIGH);//Sensoren anschalten zum Messen
    cReset = 0;
  }
  cReset++;

  int i; //laufvariable
  for(i=0;i<=13;i++){
    if (debugSensorFail[i] == true){//Sensor wurde ausgeschaltet, Fehler im Sensor
      CballR[i]    = 1500;//beliebiger Wert über TSOPHigh
      CballR[i+14] = i;//Sensornummer eintragen
      CballR[i+28] = CballR[i];//Rawwert eintragen   
    }
    else{//Sensor funktioniert ordnugsgemäß, nicht deaktiviert
      CballR[i]    = analogRead(Dioden[i]);//Wert des Sensor lesen
      CballR[i+14] = i;//Sensornummer eintragen
      CballR[i+28] = CballR[i];//Rawwert übernehmen
    }  
  }
  /*
  CballR-Array Erklärung:
   Aufteilung in 3 Bereiche
   0-13 Rawwerte die sortiert werden
   14-27 Sensornummer die mit sortiert werden 
   28-41 Rawwerte die nicht sortiert werden
   
   */
  isort_c(CballR,14);//angepasstes Insertionsort für das Array
  max_detector = CballR[14];//die größte angestrahlte Diode wird fest übernommen

  int lichtschrankenwert = analogRead(LichtschrankeRead);//Lichtschranke auslesen

  if (lichtschrankenwert < max_lichtschranke && CballR[34] < ballcapwert && CballR[35] < ballcapwert){//Der Ball liegt in der capturingzone und der Ball liegt vor ihm
    ballcap = true;
  }  
  else if (lichtschrankenwert >= max_lichtschranke || CballR[34] >= ballcapwert || CballR[35] >= ballcapwert){//etwas ist in der lichtschranke, aber Ball liegt zu weit weg oder daneben
    ballcap = false;
  }  


  if (CballR[max_detector + shifting] > max_background){//überprüfen ob höchst angestrahlte diode den ball sieht  
    habeball = false;//kein Ball in Reichweite
  } 
  else {
    habeball = true;//Ball ist im sichtfeld
  } 

  //habeball ist eine Statusvariable in unserer groben Logik

}

//angepasstes Insertionsort
//http://de.wikipedia.org/wiki/Insertionsort
void isort_c(int *a, int n) {
  int k;
  int shift = n;
  for (k = 0; k <= n-1; ++k) {
    int key = a[k];
    int keyb = a[k + shift];
    int i = k - 1;
    while ((i >= 0) && (key <= a[i])) {
      a[i + 1] = a[i];
      a[i + 1 + shift] = a[i + shift];
      --i;
    }
    a[i + 1] = key;
    a[i + 1 + shift] = keyb; 
  }

}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////Kompass//////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

/*
Modfiziertes Auslesen des Kompassensors, da die einfache Funktion
 zu lang dauerte. Diese dauerte 640ms oder länger, da dort die Werte
 geglättet werden und die Beschleunigung eingerechnet wird.
 Die Rawwerte werden alle 14ms aktualisiert, daher benutzt diese
 Funktion nur die Rawwerte und rechnet diese hier um.
 http://pishrobot.com/files/products/datasheets/cmps10.pdf
 */
int readCmps10TobsC(){
  int calc = 1000.0;

  Wire.beginTransmission(0x60);//Verbindung herstellen           
  Wire.write(10);//high byte X anfordern                              
  Wire.endTransmission();//Verbindung beenden 
  if (Wire.requestFrom(0x60,1) == 255){
    return -1;
  }        
  byte x1 = Wire.read();//byte lesen                     

  Wire.beginTransmission(0x60);//Verbindung herstellen           
  Wire.write(11);//low byte X anfordern                              
  Wire.endTransmission();//Verbindung beenden 

    if (Wire.requestFrom(0x60,1) == 255){
    return -1;
  }                    
  byte x2 = Wire.read();//byte lesen 

  int calcX = (x1<<8)+x2;//high und low byte zusammenstzen

  Wire.beginTransmission(0x60);//Verbindung herstellen          
  Wire.write(12);//high byte Y anfordern                              
  Wire.endTransmission();//Verbindung beenden  

    if (Wire.requestFrom(0x60,1) == 255){
    return -1;
  }                 
  byte y1 = Wire.read();//byte lesen                     

  Wire.beginTransmission(0x60);//Verbindung herstellen          
  Wire.write(13);//low Byte Y anfordern                              
  Wire.endTransmission();//Verbindung beenden  

    if (Wire.requestFrom(0x60,1) == 255){
    return -1;
  }                        
  byte y2 = Wire.read();//byte lesen 

  int calcY = (y1<<8)+y2;//high und low byte zusammenstzen

  //Z Achse wird nicht ausgelesen, da der Sensor fest verbaut ist

  //aus X und Y die Richtung bestimmen 
  if (calcY > 0){
    calc = atan2(calcX,calcY)*180.0 / M_PI;
  }
  else if (calcY < 0){
    calc = atan2(calcX,calcY)*180.0 / M_PI;
  }
  else if (calcY == 0 && calcX < 0){
    calc = 180.0;
  }
  else if (calcY == 0 && calcX > 0){
    calc = 0.0;
  }
  else{
    calc = 1000.0;
  }
  //Werte von -180 bis 180, wir brauchen sie von 0-360 daher der Offset
  return calc + 180.0;
}


/*Kalibrierung direkt in den Registern des Kompasses
 dabei wird der roboter im 5 Sekunden Takt in jede Himmelsrichtung
 gestellt, nur bei sehr schlechten Werten verwenden
 */
void HARDc(){
  //cmps10 calib
  clearLCD();
  lcdPosition(0,0);
  LCD.print("5");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("4");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("3");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("2");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("1");
  delay(1000);

  Wire.beginTransmission(0x60);//Verbindung herstellen
  Wire.write(22);//register 22 schreiben
  Wire.write(0xF0);//befehl zum neukalibrieren
  Wire.endTransmission();//verbindung beenden

    //Nord
  clearLCD();
  lcdPosition(0,0);
  LCD.print("NORD");
  Wire.beginTransmission(0x60);//Verbindung herstellen
  Wire.write(22);//register 22 schreiben
  Wire.write(0xF5);//kompasswert neustzen
  Wire.endTransmission();//verbindung beenden
  delay(1000);

  clearLCD();
  lcdPosition(0,0);
  LCD.print("5");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("4");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("3");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("2");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("1");
  delay(1000);

  //Ost
  clearLCD();
  lcdPosition(0,0);
  LCD.print("OST");
  Wire.beginTransmission(0x60);//Verbindung herstellen
  Wire.write(22);//register 22 schreiben
  Wire.write(0xF5);//kompasswert neustzen
  Wire.endTransmission();//verbindung beenden
  delay(1000);

  clearLCD();
  lcdPosition(0,0);
  LCD.print("5");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("4");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("3");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("2");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("1");
  delay(1000);

  //West
  clearLCD();
  lcdPosition(0,0);
  LCD.print("SUED");
  Wire.beginTransmission(0x60);//Verbindung herstellen
  Wire.write(22);//register 22 schreiben
  Wire.write(0xF5);//kompasswert neustzen
  Wire.endTransmission();//verbindung beenden
  delay(1000);

  clearLCD();
  lcdPosition(0,0);
  LCD.print("5");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("4");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("3");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("2");
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("1");
  delay(1000);

  //WEST
  clearLCD();
  lcdPosition(0,0);
  LCD.print("WEST");
  Wire.beginTransmission(0x60);//Verbindung herstellen
  Wire.write(22);//register 22 schreiben
  Wire.write(0xF5);//kompasswert neustzen
  Wire.endTransmission();//verbindung beenden
  delay(1000);
}

/*
Initialisierung des Kompass und Übernahme der alten Werte aus dem EEPROM.
 Beim ersten Start müssen die Werte Manuell reingeschrieben werden oder
 über Tastenknöpfe oder ähnliche Befehele gesetzt werden, da sonst nichts
 oder 0 in der Korrekturwerten steht und der Roboter verrückt spielt.
 */
void initCmps(){
  //lese aus dem EEPROM
  NullKorrektur = EEPROMReadInt(KompassNordEEPROM);
  SUD = EEPROMReadInt(KompassSudEEPROM);
  nahnahNORDlinks = EEPROMReadInt(KompassNahNahNordLinks);
  nahnahNORDrechts = EEPROMReadInt(KompassNahNahNordRechts);

  //Falls Fehler beim lesen, übernehme direkt den Wert beim anschalten
  //sollte nicht auftreten, aber falls, dann fährt der Bot noch halbwegs ordentlich
  if (NullKorrektur > 360){
    NullKorrektur = 360-readCmps10TobsC();
  }
  //überprüfung der anderen Kompassbedingungen
  if (SUD > 360 || SUD <= -1 || nahnahNORDlinks > 360 || nahnahNORDlinks <= -1 || nahnahNORDrechts > 360 || nahnahNORDrechts <= -1){
    //Error mit LEDS!
  }
}

/*
Fehlerhafte Kalibrierungswerte kann man auf dem Display nachvollziehen
 daher wird hier nicht LEDs als Fehleranzeige gearbeitet
 */
int calibCmpsNord(){
  int check = readCmps10TobsC();//orginal Wert laden
  if (check > 360 || check <= -1){//überprüfung ob realistisch
    NullKorrektur = NullKorrektur;//alten wert beibehalten
    return 0;
  }
  else{//wert übernehmen und speichern
    NullKorrektur = check;
    EEPROMWriteInt(KompassNordEEPROM, NullKorrektur);
    return 1;
  }  
}

int calibCmpsSud(){
  int check = dRob;//manipulierten Kompasswert nehmen
  if (check > 360 || check <= -1){//siehe calibCmpsNord
    SUD = SUD;
    return 0;
  }
  else{
    SUD = check;
    EEPROMWriteInt(KompassSudEEPROM, SUD);
    return 1;
  } 
}

int calibCmpsToleranz(){
  int toleranzwert = dRob;//manipulierten wert nehmen
  if (toleranzwert > 180 && toleranzwert < 360){//bereich für links zum überprüfen
    nahnahNORDlinks = toleranzwert;
    EEPROMWriteInt(KompassNahNahNordLinks, nahnahNORDlinks);
    return 1;
  }
  else if (toleranzwert < 180 && toleranzwert > 0){//bereich für rechts zum probieren
    nahnahNORDrechts = toleranzwert;
    EEPROMWriteInt(KompassNahNahNordRechts, nahnahNORDrechts);
    return 1;
  }
  else{
    //ERROR!
    return 0;
  }
}


//mehrfach manipulierte Kompassauslesung
//dRob ist global
boolean kompass(){
  int drob360 = readCmps10TobsC();
  if (drob360 == -1){
    return 0;//Error
  }

  if (drob360 > NullKorrektur){// Anpassung: Richtung gegnerisches Tor = 0 Grad
    dRob = drob360 - NullKorrektur;
  }
  else {
    dRob = drob360 + (Richtwert - NullKorrektur);
  }
  return 1;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////EEPROM//////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/*
Funktionen zum Schrieben und Speichern von Integer auf dem EEPROM
 aus dem Forum übernommen.
 http://arduino.cc/forum/index.php?topic=100231.0;wap2
 */
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////SensorStatus/////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

/*
Da es wie gewohnt bei den German Open etwas rupiger zu geht, wurden unsere
 Sensoren teilweise beschädigt und lieferten falsche Werte und fuhren damit
 auch nicht mehr ordentlich. Um dies zu verhindern kann man Sensoren deaktivieren.
 Bei uns geschieht das über das Display und die Schalter. Dadurch wird die
 Fahreigenschaft nur minimal beintächtigt und nicht so massiv wie mit dauerhaft
 falschen Werten.
 Das ganze wird auf dem EEPROM gespeichert, damit nicht vor jedem Start der Sensor
 erneut deaktiviert werden muss.
 */

//setzt einen Sensor auf kaputt und speichert ihn im EEPROM
void SetSensorBroken(byte value){
  debugSensorFail[value] = true;
  EEPROMWriteInt(debugSensorEEPROM[value], 1337);
} 

//setzt Sensor wieder in Ordnung
void SetSensorFunkt(byte value){
  debugSensorFail[value] = false;
  EEPROMWriteInt(debugSensorEEPROM[value], 200);
}

//auslesen welche Sensoren kaputt sind, welche funktionieren
void ReadSensorsBrokenEEPROM(){
  int i;
  for (i = 0; i < 15; i++){
    if (EEPROMReadInt(debugSensorEEPROM[i]) == 1337){
      debugSensorFail[i] = true;
    }
    else{
      debugSensorFail[i] = false;
    }
  }
}

//alle Sensoren funktionieren
void SetSensorsFunktEEPROM(){
  int i;
  for (i = 0; i < 15; i++){
    debugSensorFail[i] = false;
    EEPROMWriteInt(debugSensorEEPROM[i], 200);
  }
}

/*
 Am Anfang des Prorgammes wird automatisch überprüft, ob die Werte
 alles korrekt sind und werden notfalls deaktviert. Bei Lichtsensoren
 zuverlässig. Bei Ultraschallsensoren hingegen nicht, daher werden diese
 nicht überprüft.
 */
//AutoDetect
void checkSensors(){
  int i;
  //Lichtsensoren überprüfen
  for (i=0;i<14;i++){
    if (CballR[i+shifting] < TsopLow || CballR[i+shifting] > TsopHigh){
      SetSensorBroken(i);
    }
    else{
      SetSensorFunkt(i);
    }
  }
  //Lichtschranke überprüfen
  if (analogRead(LichtschrankeRead) < 100 || analogRead(LichtschrankeRead) > 300){
    SetSensorBroken(14);
  }
  else{
    SetSensorFunkt(14);
  }
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////Display/////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/*
Grundfunktionen des Displays von Arduino übernommen.
 http://playground.arduino.cc/Learning/SparkFunSerLCD
 
 Dient zum Debugging und einstellen des Roboters.
 */
void initLCD(){
  //LCD
  pinMode(txPinDisplay, OUTPUT);

  LCD.begin(9600);
  clearLCD();
  backlightOn();
  lcdPosition(0,0);
}

void lcdPosition(int row, int col) {
  LCD.write(0xFE);   //command flag
  LCD.write((col + row*64 + 128));    //position 
}
void clearLCD(){
  LCD.write(0xFE);   //command flag
  LCD.write(0x01);   //clear command.
  delay(LCDdelay);
}
void backlightOn() {  //turns on the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(157);    //light level.
}
void backlightOff(){  //turns off the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(128);     //light level for off.
}
void serCommand(){   //a general function to call the command flag for issuing all other commands   
  LCD.write(0xFE);
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
//////////////////////////Timer//////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/*
kleiner selbstgebauter Software Timer
 */
void TimerInit(int Reset){
  switch (Reset) {
  case 0://alle Timer resetten 
    for (int k=0; k <= 10; k++){
      Timer[k] = millis();
    }
    break;
  default://Vorsichtig mit umgehen, da auch in falsche Arraypos geschrieben werden kann
    //einen Timer zurücksetzen
    Timer[Reset] = millis();
    break;
  }
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////Motoren-LOW/////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

/*
 Initialisierung der Motoren.
 http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/CD00004482.pdf
 
 Wir haben Veränderungen an der Schaltung getätigt, unsere entspricht nicht mehr
 dem Orginal in der pdf. Wir haben die Kondensatoren sowie Widerstände
 am PWM Port weggelassen, da die se nur Zeitverzögernd gewirkt haben.
 Wir wissen nicht warum es so ist, aber die Motorenansteuerung funktionierte
 so deutlich besser.
 */
void initMotor(){
  //Motoren
  //PWM Ports setzen
  pinMode(SpeedPort1,OUTPUT);
  pinMode(SpeedPort2,OUTPUT);
  pinMode(SpeedPort3,OUTPUT);

  //Richtungsports für den Motortreiber setzen
  pinMode(DirectionPort11,OUTPUT);
  pinMode(DirectionPort12,OUTPUT); 
  pinMode(DirectionPort21,OUTPUT);
  pinMode(DirectionPort22,OUTPUT);        
  pinMode(DirectionPort31,OUTPUT);
  pinMode(DirectionPort32,OUTPUT);
}


void drehM1(int velo){
  if(velo == 0){//Motor auslaufen lassen
    analogWrite(SpeedPort1, 0); 
    digitalWrite(DirectionPort11, LOW);
    digitalWrite(DirectionPort12, LOW);
  }
  else if(velo > 0){//vorwärts drehen
    velo = velo * 2.55;
    analogWrite(SpeedPort1, velo); 
    digitalWrite(DirectionPort11, LOW);
    digitalWrite(DirectionPort12, HIGH);         
  }
  else{//rückwärts drehen
    velo = abs(velo) * 2.55;
    analogWrite(SpeedPort1, velo); 
    digitalWrite(DirectionPort11, HIGH);
    digitalWrite(DirectionPort12, LOW);
  }
}

void drehM2(int velo){//siehe drehM1
  if(velo == 0){
    analogWrite(SpeedPort2, 0); 
    digitalWrite(DirectionPort21, LOW);
    digitalWrite(DirectionPort22, LOW);
  }
  else if(velo > 0){
    velo = velo * 2.55;
    analogWrite(SpeedPort2, velo); 
    digitalWrite(DirectionPort21, LOW);
    digitalWrite(DirectionPort22, HIGH);         
  }
  else{
    velo = abs(velo) * 2.55;
    analogWrite(SpeedPort2, velo); 
    digitalWrite(DirectionPort21, HIGH);
    digitalWrite(DirectionPort22, LOW);
  }
}

void drehM3(int velo){//siehe drehM1
  if(velo == 0){
    analogWrite(SpeedPort3, 0); 
    digitalWrite(DirectionPort31, LOW);
    digitalWrite(DirectionPort32, LOW);
  }
  else if(velo > 0){
    velo = velo * 2.55;
    analogWrite(SpeedPort3, velo); 
    digitalWrite(DirectionPort31, LOW);
    digitalWrite(DirectionPort32, HIGH);         
  }
  else{
    velo = abs(velo) * 2.55;
    analogWrite(SpeedPort3, velo); 
    digitalWrite(DirectionPort31, HIGH);
    digitalWrite(DirectionPort32, LOW);
  }
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////Motoren-HIGH/////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/*
 verschieden Funktionen für die Fahrtrichtung des Roboters
 
 alle durch testen entstanden
 */
void motorenleer(){//leerlauf für alle Motoren
  drehM1(0);
  drehM2(0);
  drehM3(0);
}

void motorenaus(){//Motoren bremsen
  digitalWrite(DirectionPort11, HIGH);
  digitalWrite(DirectionPort12, HIGH);
  analogWrite(SpeedPort1, 255);
  digitalWrite(DirectionPort21, HIGH);
  digitalWrite(DirectionPort22, HIGH);
  analogWrite(SpeedPort2, 255);
  digitalWrite(DirectionPort31, HIGH);
  digitalWrite(DirectionPort32, HIGH);
  analogWrite(SpeedPort3, 255);
}

void geradeaus(){
  drehM1(100);
  drehM2(100);
  digitalWrite(DirectionPort31, HIGH);   
  digitalWrite(DirectionPort32, HIGH);   
  analogWrite(SpeedPort3, 255);
}


void hinten(){
  drehM1(-100);
  drehM2(-100);
  digitalWrite(DirectionPort31, HIGH);   
  digitalWrite(DirectionPort32, HIGH);   
  analogWrite(SpeedPort3, 255);
}  


void rechts(){
  drehM1(30);
  drehM2(-30);
  drehM3(100);
}  

void links(){
  drehM1(-30);
  drehM2(30);
  drehM3(-100);
}  

void drehenrechtstor(){
  drehM1(32);
  drehM2(-32);
  drehM3(100);
  delay(100);
}

void drehenlinkstor (){
  drehM1(-32);
  drehM2(32);
  drehM3(-100);
  delay(100);
}

void startl(){
  drehM1(100);
  drehM2(-100);
  drehM3(-100);
  delay(250);
  drill = 0;
}

void startr(){
  drehM1(-100);
  drehM2(100);
  drehM3(100);
  delay(250);
  drill = 0;
}

void startvorne(){
  drehM1(100);
  drehM2(100);
  delay(300);
}

void startrechts(){
  drehM1(100);
  drehM3(60);
  delay(300);  
  drill = 2;
}

void startlinks(){
  drehM2(100);
  drehM3(-60);
  delay(300);
  drill = 1;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////HAL-Schicht//////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

/*
 Die Anweisungen wurden hier vereinfacht, da keine 100% Kopie von
 unseren Robotern enstehen sollte.
 
 Die Hardwareabstrcationlayer dient zur Abstrahierung von der Hardware.
 Der geschriebene Code ist in dieser Form auch auf den Robotern von
 2010 und 2011 vorhanden. Über die Jahre hinweg wurde hier doch immer wieder
 die Genauigkeit gesteigert, sodass eine Menge von Code in diesem Bereich
 entstand.
 
 Ziel war es die Wiederverwendbarkeit unserer Taktik zu erhöhen. Dabei
 sollte jedoch nicht immer die ganze Software umgeschrieben werden, sondern
 immer nur ein Teil, damit Zeit gesparrt wird. So war es auch möglich im
 vorhinein eines Turniers mehrere Testroboter zu bauen und die unterschiedliche
 Hardware mit der selben Taktik gegeneinander anterten zu lassen um zu schauen
 was sich durchsetzt.
 
 http://de.wikipedia.org/wiki/Hardwareabstraktionsschicht
 */
void positionierung (){

  if (x1 <= anderwand){                                                     // Feststecken verhindern
    rechts();
  }
  else if (x2 <= anderwand){
    links();
  }
  else if (y2 <= anderwand){
    hinten();
  }
  else if (x2 < XMnahlinks){                                                  // Weit rechts
    links();
  }
  else if (x2 > XMnahrechts){                                                   // Weit links
    rechts();
  }
  else if (y1 > YMnahvorn){                                                 // Weit vorne
    hinten();
  }
  else if (x2 < XMnahnahlinks){                                               // Nah rechts
    links();
  }
  else if (x2 > XMnahnahrechts){                                                // Nah links
    rechts();
  }
  else if (y1 > YMnahnahvorn){                                              // Nah vorne
    hinten();
  }
  else if (y1 < YMittehinten){                                              // Nah hinten
    geradeaus();
  }
  else {                                                                      // Torwart steht genau auf der Torwartposition, stehenbleiben
    motorenaus();
  }

}

void ballverfolgung(){  

  if (ballcap == 1){
    geradeaus();
  }
  else if (max_detector == 5 && CballR[5 + shifting] <= max_background){
    geradeaus();
  }
  else if (max_detector == 6 && CballR[6 + shifting] <= max_background){
    geradeaus();
  }
  else if (max_detector == 7 && CballR[7 + shifting] <= max_background){
    geradeaus();
  }
  else if (max_detector == 8 && CballR[8 + shifting] <= max_background){
    geradeaus();
  }
  else if (max_detector == 4){
    links();
  }
  else if (max_detector == 3){
    links();
  }
  else if (max_detector == 2){
    links();
  }
  else if (max_detector == 1){
    hinten();
  }
  else if (max_detector == 9){
    rechts();
  }
  else if (max_detector == 10){
    rechts();
  }
  else if (max_detector == 11){
    rechts();
  }   
  else if (max_detector == 12){
    hinten();
  }
  else if (max_detector == 13){
    hinten();
  }
  else if (max_detector == 0){
    hinten();
  }
}

void ballecke (){                                                               // Ball in der gegnerischen Ecke                                      
  // Ballbesitz

  if (x1 > x2){
    r = true;
    l = false;
  }
  else if (x2 >= x1){
    l = true;
    r = false;
  }

  if (ballcap == 1 && y2 <= 20 && drill == 1){
    startl();
  }
  else if (ballcap == 1 && y2 <= 20 && drill == 2){
    startr();
  }
  else if (l == true && ballcap == 1 && y2 <= 20){
    drehenrechtstor();
  }  
  else if (r == true && ballcap == 1 && y2 <= 20){
    drehenlinkstor();
  }  
}

void SensorUpdate(){
  if(readUS() == -1){//ErrorAbfangen, da I2C manchmal rumnörgelt 
    motorenaus();  
    fuaRESET();
    delay(500);
  }

  if(kompass() == 0){//ErrorAbfangen, da I2C manchmal rumnörgelt
    motorenaus();
    fuaRESET();
    delay(500);
  }

  ReadSchalter();
  readCball();

}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
//////////////////////////Bluetooth//////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void comunication(){

  //Energiesparmodus, nur alle 200ms Nachrichten senden und empfangen
  if (Timer[1] + 200 < millis()){//StatusBericht absetzen
    if (ballcap == true){
      Serial3.write('b');//habe den Ball
      myStat = 'b';
    }
    else{
      Serial3.write('d');//habe keinen Ball
      myStat = 'd';
    }
    //hierdrüber schreiben
    TimerInit(1);
  }

  if (Timer[2] + 200 < millis()){//StatusBericht auslesen

    if (Serial3.read() == 'b'){//anderer hat den Ball
      getStat = 'b';
    }
    else if (Serial3.read() == 'h'){//anderer Homepos
      getStat = 'h';
    }
    else if (Serial3.read() == 'd'){//anderer irgwo auf dem Feld
      getStat = 'd';
    }
    else{//keine Nachricht oder falsche Nachricht
      getStat = 'E';//ERROR
    }
    //nur über diesen kommi schreiben
    TimerInit(2);
  }
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
//////////////////////////LCD-MenÃ¼///////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

/*
Debugging/Anzeige von Werten
 */
void screen(){
  //Menüschalter
  if (s[5] == true){
    ebene++;
    delay(100);
  }
  if (s[1] == true){
    ebene--;
    delay(100);
  }

  //Anzeigemenüauswahl
  if (ebene == 0){//Noch kein UtermenÃ¼ gewÃ¤hlt
    led(0);

    if (s[2] == true){
      checkSensors();
    }

    if (s[4] == true){
      SetSensorsFunktEEPROM();
    }

    clearLCD();
    lcdPosition(0,0);
    LCD.print("____Main:FRT____");
    lcdPosition(1,0);
    LCD.print("Sensors::");
    if (Defekt == true){
      LCD.print("FAIL");
    }
    else{
      LCD.print("OK");
    }
    delay(20);
  }
  else if (ebene == 1){//Kompass ausgewÃ¤hlt  
    led(1);//falls Display kaputt Led zeigt Kompassauswahl

    //Taster
    if (s[2] == true){
      calibCmpsNord();
    }
    if (s[3]  == true){
      calibCmpsSud();
    }
    if (s[4]  == true){
      calibCmpsToleranz();
    }

    //Anzeige
    clearLCD();
    lcdPosition(0,0);
    LCD.print("Cmps:");
    LCD.print(dRob);
    LCD.print(":");
    LCD.print(readCmps10TobsC()); 
    lcdPosition(1,0);
    LCD.print(NullKorrektur);
    LCD.print(":");
    LCD.print(SUD);
    LCD.print(":");
    LCD.print(nahnahNORDlinks);
    LCD.print(":");
    LCD.print(nahnahNORDrechts);
    delay(20);
  }
  else if (ebene == 2){//Lichtsensoren
    led(0);

    if (s[4] == true){
      LSensor++;
      delay(100);
    }
    if (s[2]  == true){
      LSensor--;
      delay(100);
    }

    //Sensoranzeigewechsel
    if(LSensor <= -1 || LSensor > 13){
      if (LSensor >= 14){
        LSensor = 0;
      }
      else{
        LSensor = 13;      
      }
    }

    if(s[3] == true){
      SetSensorBroken(LSensor);
    }

    clearLCD();
    lcdPosition(0,0);
    LCD.print("TSOP:::");
    LCD.print(max_detector);
    lcdPosition(1,0);
    LCD.print("Sensor");
    LCD.print(LSensor);
    LCD.print("::");
    LCD.print(CballR[LSensor+shifting]);
    delay(20);
  }
  else if (ebene == 3){//Ultraschall  
    led(0);

    clearLCD();
    lcdPosition(0,0);
    LCD.print("US:x1:");
    LCD.print(x1);
    LCD.print(":x2:");
    LCD.print(x2); 
    lcdPosition(1,0);
    LCD.print("US:y1:");
    LCD.print(y1);
    LCD.print(":y2:");
    LCD.print(y2);
    delay(20);
  }
  else if (ebene == 4){//Lichtschranke
    led(0);

    clearLCD();
    lcdPosition(0,0);
    LCD.print("cap::");
    if (ballcap == true){
      LCD.print("Gotcha");
    }
    else{
      LCD.print("neien");
    }
    lcdPosition(1,0);
    LCD.print("Licht::");
    LCD.print(analogRead(LichtschrankeRead));

  }
  else if (ebene == 5){//hardcalib
    led(0);

    //Hardcalib
    clearLCD();
    lcdPosition(0,0);
    LCD.print("ATTENTION!");
    lcdPosition(1,0);
    LCD.print("HARDC!");
    delay(20);
    //beiden äußeren Tasten müssen lange gedrückt werden
    if (s[3]  == true && s[4]  == true){
      delay(500);
      if (s[3]  == true && s[4]  == true){
        //HardC ausführen
        clearLCD();
        lcdPosition(0,0);
        LCD.print("NOW CALIB!");
        delay(1000);
        HARDc();
      }
    }
  }
  else if (ebene <= -1){//weit zurückgeschaltet gehe zur höchsten ebene
    led(0);
    ebene = 5;
  }
  else{//zu hoch geschaltet gehe zur ersten ebene zurück
    led(0);
    ebene = 0;
  }

}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////Setup-Init///////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void setup(){
  Wire.begin(); 
  Serial.begin(9600);
  TimerInit(0);

  //Init aller Komponenten
  initSchalter(); 
  initUS();
  initCball();
  initCmps();
  initMotor();
  initLed();
  initLCD();

  //zeige das gestartet/resetet wurde
  led(1);
  delay(10);
  led(0);

  SensorUpdate();
  ReadSensorsBrokenEEPROM();
  if(s[6] == true){//"Schalter"/Jumper am Bluetoothmodul
    initMaster();
  }
  else{
    initSlave();
  }
}


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////Spiellogik//////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void loop(){

  SensorUpdate();
  //comunication(); wird hier nicht verwendet nur zum Testen und Experimentieren


  if(s[7] == true){//Display angesteckt... calibmode
    motorenleer();//Motoren im Leerlauf damit man den roboter rumschieben kann
    started = false;
    screen();//MenÃ¼bersicht	
  }
  else{

    if (s[3] == true){//taster zum stoppen des programms... aufstellen der bots
      started = false;
    }    

    if (started == false){//Roboter fÃ¤hrt nicht
      if (s[0] == true){//startknopf
        started = true;//roboter startet und fährt nach sensorwerten
      }
      else if (s[1] == true){//Roboter startet nach vorne
        started = true;
        startvorne();
      }  
      else if (s[2] == true){//roboter startet nach rechts
        started = true;
        startrechts();
      }
      else if (s[4] == true){//roboter startet nach links
        started = true;  
        startlinks();  
      }   
    }
    else{//Roboter fährt

      //allgemeine Logik
      if (habeball){//sieht den ball  
        ballverfolgung();
        ballecke();
      }
      else{//sonst auf position zurückfahren
        positionierung();     
      }
    }
  }
}
