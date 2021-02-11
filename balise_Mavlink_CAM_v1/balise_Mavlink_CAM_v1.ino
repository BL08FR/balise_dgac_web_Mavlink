/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Used with ESP32 + BN220, modified by Julien Launay 14/09/2020
    Using https://github.com/khancyr/TTGO_T_BEAM/tree/master/src. Thanks to Pierre Khancyr the original author of this project.
    With https://github.com/f5soh/balise_esp32/blob/droneID_FR_testing/droneID_FR.h
    Connect ESP32 Pin16 (Rx)-> BN220 white wire (Tx)
    Connect ESP32 Pin17 (Tx)-> BN220 green wire (Rx)
	
	modif de xav:
	deux balises prises en compte: esp01 (version normale et s) et esp32
	bibliothèque droneID_FR.h commune aux deux aussi

  Mavlink modification by Blaise Lapierre, to take information via Mavlink and don't use BN220.
  Xiaomi Yi cam modification by Blaise Lapierre, dirty code but it work
  
 */

//choix de la carte entre esp32 ou esp01
#define CARTE 'esp01'
//#define CARTE 'esp32'

//##################### MODIFICATION MAVLINK (global) ######################
#include "mavlink2.h"
#include "TimeLib.h"
#include <SoftwareSerial.h>
SoftwareSerial _MavLinkSerial(5, 4); // PIN TX & RX /!\What's written on the board don't necessary match GPIO pins! Using pin 0/2 don't allow to upload a sketch, you must disconnect before.

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

byte has_home_set = 0;
//unsigned long lasthomeREQ; // to delete later, unused
unsigned int lastHB;
unsigned long timetab[] = {0, 0, 0, 0, 0, 0};// 0 is first Unix time sync memory, 1 is millis() when first UNIX time was set, 2 is actual Unix time, 3 mavlink restart time, 4 last home request sent, 5 servo pwm time count
byte nbsat;
float hdopdata;
int homealt = 0;
float gpsRawAlt = 0;
//##################### CAMERA MODIFICATION (global) ########################

#include "ESP8266WiFi.h"
#include "Bounce2.h"

byte PWM_PIN = 4;
int pwm_value;
int photoPWM = 1250;
int videoPWM = 1700;
int maxPWM = 2300;
long timeinmem = 0;
int stabePWMdelay = 80;
int photorequestnumber = 0;
boolean ServoTriggerbool;
byte PWMrange;
byte prevPWMrange;
long lastphoto;
long delayBTWphotos = 5000;
boolean armed = 0;
 
WiFiClient client;
String YI_SSID;
//const int buttonPin1 = 13;          // input pin for pushbutton
//const int buttonPin2 = 12;          // input pin for pushbutton
Bounce debouncer1 = Bounce();
Bounce debouncer2 = Bounce();
boolean RecON = false;
boolean forcephotorequest = false;
//int pwm_value;
//##########################################################################

#if CARTE == 'esp01'
  #include <ESP8266WebServer.h>
  //#include <ESP8266WiFi.h>// included in camera mod
  #include <SoftwareSerial.h>  
  extern "C" 
  {
    #include "user_interface.h"
    int wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
  }
  #define LED 2 // led bleue (pour le esp-01s c'est la pin 2 (si esp-01 c'est pin 1, conflit avec le serial, donc mettre ici genre 4 pour désactiver cette fonction)
  #define ON LOW
  #define OFF HIGH
  //#define GPS_RX_PIN 0       // Brancher le fil Tx du GPS ========A EFFACER========
  //#define GPS_TX_PIN 2       // Rx gps, pas utilisé  ========A EFFACER========
  //SoftwareSerial Serial2(GPS_RX_PIN,GPS_TX_PIN);// ========A EFFACER========
#elif CARTE == 'esp32'
  #include <WiFi.h>
  extern "C"
  {
    #include "esp_wifi.h"
    esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
  }
  #include <WebServer.h>
  #define LED 2 
  #define ON HIGH
  #define OFF LOW
  //#define GPS_RX_PIN 16 // ========A EFFACER========
//  #define GPS_TX_PIN 17 // ========A EFFACER========
#else 
  #error "définition de carte invalide"
#endif

//#define GPS_BAUD_RATE 9600// ========A EFFACER========

#include "droneID_FR.h"

droneIDFR drone_idfr;

//déclaration du serveur web sur la balise
#if CARTE == 'esp01'
  ESP8266WebServer MonServeurWeb(80);
#elif CARTE == 'esp32'
  WebServer MonServeurWeb(80);
#else
  #error "définition de carte invalide"
#endif

/********************************************************************************************************************
 * MODIFIEZ LES VALEURS ICI
 ********************************************************************************************************************/
// Set these to your desired credentials.
/**
  * Le nom du point d'acces wifi CHANGEZ LE par ce que vous voulez !!!
  */
  /*construire le nom du reseau wifi avec l'adresse mac recuperee */
  /* on définie d'abord le préfixe du nom: */
const char prefixe_ssid[] = "BALISE_DGAC-";
// déclaration de la variable qui va contenir le ssid complet
char ssid[32];

// Mot de pass du wifi
const char *password = "1234567890";
/**
  * CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
  */
const char drone_id[] = "DIYEXPTHEFLYINGMADPSYCHOPATH08";

/********************************************************************************************************************
 * XAV
 ********************************************************************************************************************/
// activer ou non le DEBUG
boolean DEBUG = true;       
const int ClignoteErreur = 25;
boolean IsErreur = false;
// si erreur sur gps, on pause un certain temps et on retente, pas la peine d'interroger toutes les millisecondes...
// mis à 0 car sinon problème de lecture du gps avec les délais...
const int delayErreur = 0;

//gestion du serveur web
String messageinit = "initialisation en cours...";
String messagetrame = "initialisation en cours...";

String messagetempsUTC = "initialisation en cours...";
void handleRoot()
{  
    // Page d'accueil
    String page = "<!DOCTYPE html>"; 
    page += "<head>";
    page += "    <title>Serveur " + String(ssid) + " </title>";
    page += "    <meta http-equiv='refresh' content='3' name='viewport' content='width=device-width, initial-scale=1' charset='UTF-8'/>";
    page += "</head>";
    page += "<body lang='fr'>";
    page += "    <h1 style='color:red;font-size:5vw;'>Point d'accès: " + String(ssid) + " </h1>";
    page += "    <p style='color:blue;font-size:1vw;'>ID de la balise: " + String(drone_id) + " </p>";
    page += "    <hr>";
    page += "    <p style='color:blue;font-size:1vw;'> <form action='reset' method='POST'><input style='color:red;font-size:3vw;' type='submit' value='Reboot de la balise'></form> </p>";
    //page += "    <p style='color:blue;font-size:1vw;'> <form action=\"/reset\" method=\"POST\"><input type=\"submit\" value=\"Reboot de la balise\"></form> </p>";
    page += "    <hr>";
    page += "    <p style='color:blue;font-size:1vw;'> " + messageinit + " </p>";
    page += "    <hr>";
    page += "    <p style='color:green;font-size:1vw;'> " + messagetempsUTC + " </p>";
    page += "    <hr>";
    page += "    <p style='color:black;font-size:1vw;'> " + messagetrame + " </p>";
    page += "</body>";
    page += "</html>";  // Fin page HTML

    MonServeurWeb.send(200, "text/html", page);  // Envoie de la page HTML
}

void handleNotFound()
{
  MonServeurWeb.send(404, "text/plain","404: Not found");
}

void handleReset() 
{                          // If a POST request is made to URI /reset
  String page = "<!DOCTYPE html>"; 
    page += "<head>";
    page += "    <title>Serveur " + String(ssid) + " reset </title>";
    page += "    <meta http-equiv='refresh' content='2,url=/' name='viewport' content='width=device-width, initial-scale=1' charset='UTF-8'/>";
    page += "</head>";
    page += "<body lang='fr'>";
    page += "    <h1 style='color:red;font-size:5vw;'>Point d'accès: " + String(ssid) + " </h1>";
    page += "    <p style='color:blue;font-size:2vw;'>ID de la balise: " + String(drone_id) + " </p>";
    page += "    <hr>";
    page += "    <p style='color:green;font-size:2vw;'> Reboot de la balise en cours, page rafraichie dans quelques secondes... </p>";
    page += "    <hr>";
    page += "</body>";
    page += "</html>";  // Fin page HTML

  MonServeurWeb.send(200, "text/html", page);  // Envoie de la page HTML
  delay(5000);
  ESP.restart();              // reset de la carte
}

boolean InitWeb()
{
  MonServeurWeb.on("/", HTTP_GET, handleRoot);  // Chargement de la page d'accueil
  MonServeurWeb.on("/reset", HTTP_POST, handleReset);
  MonServeurWeb.onNotFound(handleNotFound);  // Chargement de la page "Not found"
  MonServeurWeb.begin();  // Initialisation du serveur web
  return true;
}
/********************************************************************************************************************/
// NE PAS TOUCHEZ A PARTIR D'ICI !!!
// Le wifi est sur le channel 6 conformement à la spécification
static constexpr uint8_t wifi_channel = 6;

// Ensure the AP SSID is max 31 letters
// 31 lettres maxi selon l'api, 17 caractères de l'adresse mac, reste 15 pour ceux de la chaine du début moins le caractère de fin de chaine ça fait 14, 14+17=31
static_assert((sizeof(prefixe_ssid)/sizeof(*prefixe_ssid))<=(14+1), "Prefix of AP SSID should be less than 14 letters");


// Ensure the drone_id is max 30 letters
static_assert((sizeof(drone_id)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination

// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;  // default beaconPacket size + max ssid size + max drone id frame size
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
        0x80, 0x00,							            // 0-1: Frame Control
        0x00, 0x00,							            // 2-3: Duration
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,				// 4-9: Destination address (broadcast)
        0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 10-15: Source address FAKE  // TODO should bet set manually
        0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 16-21: Source address FAKE
        0x00, 0x00,							            // 22-23: Sequence / fragment number (done by the SDK)
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,	// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
        0xB8, 0x0B,							            // 32-33: Beacon interval: set to 3s == 3000TU== BB8, bytes in reverse order  // TODO: manually set it
        0x21, 0x04,							            // 34-35: Capability info
        0x03, 0x01, 0x06,						        // 36-38: DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
        0x00, 0x20,                     				// 39-40: SSID parameter set, 0x20:maxlength:content
                                                        // 41-XX: SSID (max 32)
};

// dès que la position home a été mise, on passe à true et on y reste
byte has_set_home = 0; // ====variable homeset dans mav=== à effacer?
double home_alt = 0.0;

int8_t P1;
uint64_t beaconSec = 0;

/**
 * Phase de configuration.
 */
void setup()
{
    //définition de la led bleue
  pinMode(LED, OUTPUT);

  #if CARTE == 'esp01'
    digitalWrite(LED, ON);      
  #elif CARTE == 'esp32'
    digitalWrite(LED, OFF);
  #else 
    #error "définition de carte invalide"
  #endif

  
  //###################### MODIFICATION MAVLINK (setup) ######################
  // Serial USB connexion start
  Serial.begin(115200);
  Serial.println("Serial Begin");
  delay(5000);
  
  // MAVLink interface start
  _MavLinkSerial.begin(57600);
  Serial.println("MAVLink starting.");  
  //##################### CAMERA MODIFICATION (setup) #######################
  Serial.println("Search & connect to Camera!");
  WiFi.mode(WIFI_AP_STA); //Permets de mettre l'ESP en mode Acces Point & Station
  searchCamera();
  connectToCamera();
  //pinMode(buttonPin1, INPUT_PULLUP);
  //pinMode(buttonPin2, INPUT_PULLUP);
  //debouncer1.attach(buttonPin1); debouncer1.interval(50);
  //debouncer2.attach(buttonPin2); debouncer2.interval(50);
  //##########################################################################
    
    #if CARTE == 'esp01'
        Serial.println("Carte ESP01 !");
    #elif CARTE == 'esp32'
      Serial.println("Carte ESP32 !");
    #else 
      #error "définition de carte invalide"
    #endif
    
  //init wifi
    Serial.println("--------------------------");
    Serial.println("Mise en place du wifi:");
    messageinit = ("Mise en place du wifi OK:");messageinit+="<br>";
    //démarrage

    //WiFi.mode(WIFI_AP_STA); //Permets de mettre l'ESP en mode Acces Point & Station
    //conversion de l'adresse mac:
    String NomAPTempo = WiFi.macAddress();
    NomAPTempo.replace(":","_");
    //concat du prefixe et de l'adresse mac
    NomAPTempo = String(prefixe_ssid) + NomAPTempo;
    //transfert dans la variable globale ssid (pour pas tout changer)
    NomAPTempo.toCharArray(ssid, 32);
    // ouf! ya surement plus simple...

    //infos sur le wifi:
    if (WiFi.softAP(ssid, password, wifi_channel)==false)  // ssid, pwd, channel, hidden (facultatif), max_cnx (facultatif, de 0 à 8, 4 par défaut)
    {
      Serial.print("ECHEC de mise en place du wifi, dont le nom est: ");Serial.println(ssid);
    }

    //infos sur le wifi:
    Serial.print("Nom du point d'accès: ");Serial.println(ssid);
    //attente init correcte du wifi:
    delay(500);
    
    #if CARTE == 'esp01'
      WiFi.setOutputPower(10.5); // max 20.5dBm
      softap_config current_config;
      wifi_softap_get_config(&current_config);
      current_config.beacon_interval = 1000;
      wifi_softap_set_config(&current_config);
    #elif CARTE == 'esp32'
      wifi_config_t conf_current;
      esp_wifi_get_config(WIFI_IF_AP, &conf_current);
      // Change WIFI AP default beacon interval sending to 1s.
      conf_current.ap.beacon_interval = 1000;
      ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &conf_current));
      // paramétrage de la puissance d'émission
      esp_err_t r = esp_wifi_get_max_tx_power(&P1);
      Serial.print("Tx power Value (dBm) (doit être inférieur à 20dBm): ");
      Serial.println(P1*0.25);
      messageinit += ("Tx power Value (dBm) (doit être inférieur à 20dBm): ");messageinit+=(String(P1*0.25));messageinit+="<br>";
      Serial.println("--------------------------");
    #else
      #error "définition de carte invalide"
    #endif

    // autres paramétrages, adresse ip fixée pour avoir toujours la même:
    IPAddress Ip(192, 168, 1, 1);
    IPAddress NMask(255, 255, 255, 0);
    WiFi.softAPConfig(Ip, Ip, NMask);
  
    //affichage des infos
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("Adresse IP du point d'accès: ");
    Serial.println(myIP.toString());
    messageinit += ("Adresse IP du point d'accès: ");messageinit += myIP.toString();messageinit+="<br>";
    Serial.print("Adresse MAC du point d'accès: ");
    Serial.println(WiFi.macAddress());
    messageinit += ("Adresse MAC du point d'accès: ");messageinit += WiFi.macAddress();messageinit+="<br>";
    Serial.print("Nom de la balise: ");Serial.println(drone_id);

  // définition de l'id de la balise:
    drone_idfr.set_drone_id(drone_id);
    delay(2000);
    
  // démarrage du serveur web
    if (!InitWeb()) return;
    Serial.print("Serveur web actif sur http://");Serial.print(myIP);Serial.println("/");
    messageinit += ("Serveur web actif sur http://");messageinit += myIP.toString();messageinit += ("/");messageinit+="<br>";
    Serial.println("--------------------------");
  
} // fin setup

/**
 * Début du code principal. C'est une boucle infinie.
 */
void loop()
{
// on attend un peu que tout s'init bien (5s à partir du démarrage de la balise, en général quand on est ici c'est déjà passé les 10s)
// ne s'exec qu'une seule fois au démarrage de la balise ce délai:
	while (millis() < 10000)
	{
		digitalWrite(LED, ON);
		Serial.print(".");
		delay(100);
		digitalWrite(LED, ON);// original OFF
		delay(100);
	}

//####################### MODIFICATION MAVLINK (loop) ######################
  // RESTART MAVLINK if no heartbeat since X time
  if(millis() - lastHB > 10000 && timetab[3] == 0 && millis() > 10000){timetab[3] = millis();}// set actual time in memory if mavlink out of sync since X time and board started since X time
  if(millis() - lastHB > 10000 && millis() - timetab[3] > 1000){//if mavlink out of sync since X time, try to restart
    timetab[3] = millis(); // reset time memory of mavlink sync   
    _MavLinkSerial.begin(57600); // try restart mavlink
    Serial.println("Trying to restart MAVLINK SERIAL");// write it on serial
  }

  // TIME syncronising (syncrnize time with mavlink, then update it with millis of the board)
  unsigned long S_added_since_UNIX_was_set = ((millis() - timetab[1])/1000);// second passed since first millis/unix time sync
  if(timetab[0] != 0 && timetab[0] + S_added_since_UNIX_was_set > timetab[2]){// if one seconds more passed than time in memory
    timetab[2] = timetab[0] + S_added_since_UNIX_was_set;// set new actual time
    //Serial.println("TIME SYNC!");
    //Serial.print("S_added_since_UNIX_was_set = "); Serial.println(S_added_since_UNIX_was_set);
    //Serial.print("timetab[0]= "); Serial.println(timetab[0]);
    //Serial.print("timetab[1]= "); Serial.println(timetab[1]);
    //Serial.print("timetab[2]= "); Serial.println(timetab[2]);
    time_function();// call time function for human time conversion
    }
  
  
  // MAVLink
  /* The default UART header for your MCU */
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 195;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink)
  {
    // Record last HB update
    previousMillisMAVLink = currentMillisMAVLink;


    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      // Request streams from Pixhawk
      Serial.println("Mavlink Stream(s) requested!");
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }

  }
  // Request Home till it's set every second
  if(has_set_home < 2 && currentMillisMAVLink - timetab[4] >= 1000 && millis() - lastHB < 10000){//request home if it's not set yet, not asked since 1s and last heartbeat<10s
    timetab[4] = millis();
    request_home();
  }
  // Check reception buffer
  comm_receive();
/*
  //Request altitute message
  if(currentMillisMAVLink - timetab[5] >= 1000){
    timetab[5] = millis();
    request_altitude();
  }*/
//#########################################################################

//Serial.println("loop après comm_receive");
 //gestion serveur web
 MonServeurWeb.handleClient();  // Attente de demande du client
    
    /**
     * On regarde s'il est temps d'envoyer la trame d'identification drone: soit toutes les 3s soit si le drones s'est déplacé de 30m en moins de 3s.
     *  - soit toutes les 3s,
     *  - soit si le drone s'est déplacé de 30m en moins de 30s soit 10m/s ou 36km/h,
     *  - uniquement si la position Home est déjà définie,
     *  - et dans le cas où les données GPS sont nouvelles.
     
    static int dispfreq = millis();
    if(millis() - dispfreq > 1000){
    Serial.print("has_home_set idfr = ");Serial.print(drone_idfr.has_home_set());Serial.print("  Time to send = ");Serial.println(drone_idfr.time_to_send());
    dispfreq = millis();
    }*/
    static long tramesend = millis(); // relatif à la ligne ci-dessous
    if(millis() - tramesend >= 700)digitalWrite(LED, ON);
    if (millis() - tramesend >= 3000 || drone_idfr.has_pass_distance())// origine : if (drone_idfr.has_home_set() && drone_idfr.time_to_send()), pour test : if(millis() - tramesend >= 3000)
    { Serial.println("--------ENVOI TRAME-------");
        //boolean i = digitalRead(LED); 
        //if(i = 1)digitalWrite(LED, OFF);
        digitalWrite(LED, OFF);
      // LED allumée en fonctionnement normal,
      // deux brefs clignotements pendant l'envoi de la trame
      tramesend = millis();/*
        digitalWrite(LED, ON);
        delay(25);
        digitalWrite(LED, OFF);
        delay(25);
        digitalWrite(LED, ON);
        delay(25);
        digitalWrite(LED, OFF);
        delay(25);
        digitalWrite(LED, ON);*/

      // Ici on ecrit sur le port USB les données GPS pour visualisation seulement.
//          Serial.println("Temps UTC: " + String(gps.date.day()) + "/" + String(gps.date.month()) + "/"  + String(gps.date.year()) + " " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":"  + String(gps.time.second()));// =====A EFFACER=====
          Serial.println("Position de départ: " + drone_idfr.get_home_position());
          Serial.println("Position actuelle : " + drone_idfr.get_position_actuelle());
          messagetrame="Position actuelle : " + drone_idfr.get_position_actuelle();messagetrame+="<br>";
          Serial.println("Nb de satellites: " + String(nbsat));
          messagetrame+="Nb de satellites: " + String(nbsat);messagetrame+="<br>";
          Serial.println("hdop: " + String(hdopdata));
          messagetrame+="hdop: " + String(hdopdata);messagetrame+="<br>";
          Serial.println("--------------------------");
          messagetrame += "<hr>";

        // puis on écrit la trame qui est envoyée
        Serial.println("Send beacon");
        messagetrame += "<p style='color:black;font-size:1vw;'>";
        messagetrame+="Send beacon";messagetrame+="<br>";
        
        float time_elapsed = (float(millis() - beaconSec) / 1000);
        beaconSec = millis();

        Serial.print(time_elapsed,1); Serial.print("s Send beacon: "); Serial.print(drone_idfr.has_pass_distance() ? "Distance" : "Time");
        messagetrame+=String(time_elapsed,1);messagetrame+="s Send beacon: ";messagetrame+=String(drone_idfr.has_pass_distance() ? "Distance" : "Time");messagetrame+="<br>";
        Serial.print(" with ");  Serial.print(drone_idfr.get_distance_from_last_position_sent()); Serial.print("m Speed="); Serial.println(drone_idfr.get_ground_speed_kmh());
        messagetrame+=" with ";messagetrame+=String(drone_idfr.get_distance_from_last_position_sent());messagetrame+="m Speed=";messagetrame+=String(drone_idfr.get_ground_speed_kmh());
        messagetrame += "</p>";
        messagetrame += "<hr>";
        /**
         * On commence par renseigner le ssid du wifi dans la trame
         */
        // write new SSID into beacon frame
        const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
        beaconPacket[40] = ssid_size;  // set size
        memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
        const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker
        /**
         * On génère la trame wifi avec l'identfication
         */
        const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination
        /* Décommenter ce block pour voir la trame entière sur le port usb */
        Serial.println("beaconPacket : ");
        messagetrame += "<p style='color:black;font-size:1vw;'>";
        messagetrame+="beaconPacket : ";messagetrame+="<br>";
        for (auto i=0; i<sizeof(beaconPacket);i++)
        {
            Serial.print(beaconPacket[i], HEX);
            messagetrame+=String(beaconPacket[i]);
            // en inversant le commentaire ci-dessous et ci-dessus, on peut avoir un aperçu de la trame en "lettres"...
            //Serial.print(char(beaconPacket[i]));
            Serial.print(" ");
            messagetrame+=" ";
        }
        Serial.println(" ");
        messagetrame+=" ";
        messagetrame += "</p>";
        Serial.println("--------------------------");

        /**
         * On envoie la trame
         */
        #if CARTE == 'esp01'
          wifi_send_pkt_freedom(beaconPacket, to_send, 0);
        #elif CARTE == 'esp32'
          ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, to_send, true));
        #else 
          #error "définition de carte invalide"
        #endif
        
        /**
         * On reset la condition d'envoi
         */
        drone_idfr.set_last_send();
        
        // LED allumée en fonctionnement normal,
        // deux brefs clignotements pendant l'envoi de la trame
        /*digitalWrite(LED, OFF);
        delay(25);
        digitalWrite(LED, ON);
        delay(25);
        digitalWrite(LED, OFF);
        delay(25);
        digitalWrite(LED, ON);
        delay(25);
        digitalWrite(LED, OFF);*/
    }
}



//####################### MODIFICATION MAVLINK (post loop functions) ######################
void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 5;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RC_CHANNELS, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_RAW_CONTROLLER};
  const uint16_t MAVRates[maxStreams] = {0x20, 0x01, 0x01, 0x01};

  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(6, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    _MavLinkSerial.write(buf, len);
  }
}



void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (_MavLinkSerial.available() > 0) {
    uint8_t c = _MavLinkSerial.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      //Serial.print("DEBUG msgid:"); Serial.println(msg.msgid); // Uncomment to see all messages received (not only those used)
      // Handle message
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            lastHB = millis();
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid);Serial.println(" HEARTBEAT Received");
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
          }
          break;

        case MAVLINK_MSG_ID_SYSTEM_TIME:  // #2: System Time
          {
            if (timetab[0] == 0){
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid); Serial.print(" Mavlink Unix time message| ");
            mavlink_system_time_t unixtime;
            mavlink_msg_system_time_decode(&msg, &unixtime);
            unsigned long epoch = ((unixtime.time_unix_usec * 0.001f)/1000);
            //Serial.print(" Epoch read : "); Serial.print(epoch); Serial.print(" Timetab0 : "); Serial.println(timetab[0]);
            timetab[0] = epoch;// write first unix time got from mavlink in memory
            timetab[1] = millis();// write board time when first mavlink unix time was set
            time_function();// call function for human time conversion
            }
          }
          break;


        case MAVLINK_MSG_ID_GPS_RAW_INT: // #24: Gps raw data
          {
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid); Serial.print(" GPS_RAW_INT message | ");
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(&msg, &packet);

            nbsat = packet.satellites_visible;
            hdopdata = packet.eph * 0.01f;
            
            if(packet.fix_type < 3){
              Serial.println("Pas de fix GPS (ou perdu).");
              messagetrame = "Fix du GPS perdu...";messagetrame+="<br>";
            }
            if (packet.satellites_visible < 5 || packet.eph * 0.01f > 5.0){
              Serial.println("La précision du GPS n'est pas bonne");//if < 5 satellites & HDOP > 5
              messagetrame = "La précision du GPS n'est pas bonne";messagetrame+="<br>";
            }
            if (packet.satellites_visible < 5){
              Serial.println(" Nb de satellites (doit être supérieur à 4): " + String(packet.satellites_visible));
              messagetrame += "Nb de satellites (doit être supérieur à 4): ";messagetrame+=(String(packet.satellites_visible));messagetrame+="<br>";
            }
            if (packet.eph * 0.01f > 5.0){
              Serial.println(" Précision (doit être inférieure à 5.0): " + String(packet.eph * 0.01f));
              messagetrame += "Précision (doit être inférieure à 5.0): ";messagetrame+=(String(packet.eph * 0.01f));messagetrame+="<br>";
            }
       
            //drone_idfr.set_altitude(packet.alt * 0.001f);
            Serial.print("Type de fix GPS: ");Serial.print(packet.fix_type);
            Serial.print(" Satellites visibles: ");Serial.print(packet.satellites_visible);
            Serial.print(" HDOP: ");Serial.print(packet.eph * 0.01f);      
            Serial.print(" VDOP: ");Serial.print(packet.epv * 0.01f);  
            Serial.print(" GPS RAW Altitude MSL: ");Serial.println(packet.alt * 0.001f); 
            gpsRawAlt = packet.alt * 0.001f;
          }
          break;


        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // #33
          {
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid); Serial.print(" Global_position_int message| ");
            mavlink_global_position_int_t global_position_int;
            mavlink_msg_global_position_int_decode(&msg, &global_position_int);
            //drone_idfr.set_altitude(global_position_int.alt);
            drone_idfr.set_heading(static_cast<uint16_t>(global_position_int.hdg * 0.01f));
            
            if(homealt !=0)
            {
              drone_idfr.set_heigth(gpsRawAlt - homealt/*(global_position_int.alt - homealt)* 0.001f*/);
              Serial.print("Variable homealt set : ");Serial.println(homealt);
              Serial.print("Variable _height set : ");Serial.println(gpsRawAlt - homealt/*(global_position_int.alt - homealt)* 0.001f*/);
            }
            
            drone_idfr.set_ground_speed(sqrt(pow(global_position_int.vx, 2)+ pow(global_position_int.vy, 2)) * 0.01f);
            drone_idfr.set_current_position(global_position_int.lat, global_position_int.lon, gpsRawAlt/*(global_position_int.alt * 0.001f)*/);// =====A EFFACER===== étrange pas d'équivalent dans le prog mav
       
            Serial.print("Mavlink GPS position updated...") && Serial.print("  Lat : ") && Serial.print((global_position_int.lat * 0.001f)/10000) && Serial.print("  Long : ") && Serial.print((global_position_int.lon * 0.001f)/10000) && Serial.print("  MSL Alt : ") && Serial.print(gpsRawAlt/*global_position_int.alt * 0.001f*/)&& Serial.print("  Relative Alt : ");
            if(homealt > 0){Serial.println(gpsRawAlt-homealt);}
            else{Serial.println("no data (home altitude not set)");}

            //Serial.print("GPS alt for test == ");Serial.println(gpsAlt);
            
            if(armed == 1 && has_home_set == 0 && nbsat >= 5 && hdopdata <5){
              drone_idfr.set_home_lat_lon(global_position_int.lat, global_position_int.lon);
              drone_idfr.set_home_position(global_position_int.lat, global_position_int.lon, (gpsRawAlt/*global_position_int.alt * 0.001f*/));
              has_home_set = 1;
              homealt = gpsRawAlt/*global_position_int.alt * 0.001f*/;
              
              // on lit la position home pour controle
              Serial.println("Position de départ: " + drone_idfr.get_home_position());
              Serial.println("Stockage de la position de départ OK");
              messageinit += "<hr><p style='color:green;font-size:1vw;'>Position de départ:";messageinit+="<br>";
              messageinit+=drone_idfr.get_home_position();messageinit+="</p>";
              Serial.println("--------------------------");
            }
          }
          break;


        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: // #36: Servo output raw
          {
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid); Serial.print(" SERVO_OUTPUT_RAW message | ");
            mavlink_servo_output_raw_t packet;
            mavlink_msg_servo_output_raw_decode(&msg, &packet);
            Serial.print(" #1_");Serial.print(packet.servo1_raw);
            Serial.print(" #2_");Serial.print(packet.servo2_raw);
            Serial.print(" #3_");Serial.print(packet.servo3_raw);
            Serial.print(" #4_");Serial.print(packet.servo4_raw);
            Serial.print(" #5_");Serial.print(packet.servo5_raw);
            Serial.print(" #6_");Serial.print(packet.servo6_raw);
            Serial.print(" #7_");Serial.print(packet.servo7_raw);
            Serial.print(" #8_");Serial.print(packet.servo8_raw);
            Serial.print(" #9_");Serial.print(packet.servo9_raw);
            Serial.print(" #10_");Serial.print(packet.servo10_raw);
            Serial.print(" #11_");Serial.print(packet.servo11_raw);
            Serial.print(" #12_");Serial.print(packet.servo12_raw);
            Serial.print(" #13_");Serial.print(packet.servo13_raw);
            Serial.print(" #14_");Serial.print(packet.servo14_raw);
            Serial.print(" #15_");Serial.print(packet.servo15_raw);
            Serial.print(" #16_");Serial.println(packet.servo16_raw);
            
            if( packet.servo7_raw > photoPWM && packet.servo7_raw < videoPWM){
              Serial.println("Servo rail photo trigger PWM detected !");
              ServoTriggerbool = 1;
              PWMselector();
              ServoTriggerbool = 0;
            }

            if(packet.servo1_raw > 1200) armed = 1;
          }
          break;


        case MAVLINK_MSG_ID_RC_CHANNELS: // #65: RC channels
          {
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid); Serial.print(" RC_CHANNELS_RAW message | ");
            mavlink_rc_channels_t packet;
            mavlink_msg_rc_channels_decode(&msg, &packet);
            byte chancount = packet.chancount;
            Serial.print(" #1_");Serial.print(packet.chan1_raw);
            Serial.print(" #2_");Serial.print(packet.chan2_raw);
            Serial.print(" #3_");Serial.print(packet.chan3_raw);
            Serial.print(" #4_");Serial.print(packet.chan4_raw);
            Serial.print(" #5_");Serial.print(packet.chan5_raw);
            Serial.print(" #6_");Serial.print(packet.chan6_raw);
            Serial.print(" #7_");Serial.print(packet.chan7_raw);
            Serial.print(" #8_");Serial.print(packet.chan8_raw);
            Serial.print(" #9_");Serial.print(packet.chan9_raw);
            Serial.print(" #10_");Serial.print(packet.chan10_raw);
            Serial.print(" #11_");Serial.print(packet.chan11_raw);
            Serial.print(" #12_");Serial.print(packet.chan12_raw);
            Serial.print(" #13_");Serial.print(packet.chan13_raw);
            Serial.print(" #14_");Serial.print(packet.chan14_raw);
            Serial.print(" #15_");Serial.print(packet.chan15_raw);
            Serial.print(" #16_");Serial.print(packet.chan16_raw);
            Serial.print(" #17_");Serial.print(packet.chan17_raw);
            Serial.print(" #18_");Serial.println(packet.chan18_raw);

            if(packet.chan9_raw > photoPWM && packet.chan9_raw < videoPWM){Serial.println("RC Photo PWM detected !");}
            if(packet.chan9_raw >= videoPWM && packet.chan9_raw < maxPWM){Serial.println("RC Video PWM detected !");}            
            pwm_value = packet.chan9_raw;
            PWMselector();
          }
          break;

         
        case MAVLINK_MSG_ID_VFR_HUD:  // #74
          {
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid); Serial.print(" Vfr_hud message| "); 
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
            //drone_idfr.set_altitude(vfr_hud.alt);
            drone_idfr.set_heading(vfr_hud.heading);
            drone_idfr.set_ground_speed(vfr_hud.groundspeed * 3.6);
            Serial.print("Air speed : ") && Serial.print(vfr_hud.airspeed * 3.6) && Serial.print("  Ground speed : ") && Serial.print(vfr_hud.groundspeed * 3.6) && Serial.print(" Heading : ") && Serial.println(vfr_hud.heading);       
          }
          break;

        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:  // #49
          {
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid); Serial.print(" Global_origin message| "); 
            mavlink_gps_global_origin_t globalori;
            mavlink_msg_gps_global_origin_decode(&msg, &globalori);
            drone_idfr.set_home_lat_lon(globalori.latitude, globalori.longitude);
            drone_idfr.set_home_position(globalori.latitude, globalori.longitude, globalori.altitude);
            Serial.print("Lat : ") && Serial.print((globalori.latitude * 0.001f)/10000) && Serial.print("  Lon : ") && Serial.println((globalori.longitude * 0.001f)/10000);       
          }
          break;

        case MAVLINK_MSG_ID_HOME_POSITION:  // #242
          {
            Serial.print("DEBUG msgid:"); Serial.print(msg.msgid); Serial.print(" Home position message| "); 
            if(has_set_home < 2){
            mavlink_home_position_t packet;
            mavlink_msg_home_position_decode(&msg, &packet);
            drone_idfr.set_home_lat_lon(packet.latitude, packet.longitude);
            drone_idfr.set_home_position(packet.latitude, packet.longitude, (packet.altitude * 0.001f));
            has_set_home = 2;
            homealt = packet.altitude * 0.001f;
            Serial.print("Mavlink home position set");Serial.print("  Lat : "); Serial.print((packet.latitude * 0.001f)/10000);Serial.print("  Lon : ");Serial.print((packet.longitude * 0.001f)/10000);Serial.print(" Alt : ");Serial.println(packet.altitude * 0.001f);      
            }        
          }
          break;
          
      }
    }
  }
}


void request_home(){// request home position on mavlink
  int drone_sysid = 1;
  int drone_compid = 0;
  int my_sysid = 1;
  int my_compid = 195;
 // int compid = MAV_COMP_ID_MISSIONPLANNER; 
  int chan = 0;
  //uint8_t chan = MAVLINK_COMM_0;
  mavlink_message_t msg_send;
  mavlink_command_long_t msg_long;
  msg_long.command = MAV_CMD_GET_HOME_POSITION;
  msg_long.confirmation = 0;
  msg_long.target_system = drone_sysid;
  msg_long.target_component = drone_compid;
  msg_long.param1 = 0;
  msg_long.param2 = 0;
  msg_long.param3 = 0;
  msg_long.param4 = 0;
  msg_long.param5 = 0;
  msg_long.param6 = 0;
  msg_long.param7 = 0;

  mavlink_msg_command_long_encode_chan(my_sysid, my_compid, chan, &msg_send, &msg_long);
  Serial.println("Mavlink home position request sent !");
}


void time_function(){// convert UNIX time in human time format
  if(timetab[0] != 0){
    byte x = 2;
    if(timetab[2] == 0){x = 0;}
    Serial.print("timetab to translate in human time "); Serial.print(timetab[x]); Serial.print(" | Human time : ");
    unsigned long offset_days = offset_days * 86400;    // convert number of days to seconds
    printf("%4d-%02d-%02d %02d:%02d:%02d\n", year(timetab[x]), month(timetab[x]), day(timetab[x]), hour(timetab[x]), minute(timetab[x]), second(timetab[x])); Serial.println();
    messagetempsUTC = "Temps UTC: " + String(day(timetab[x])) + "/" + String(month(timetab[x])) + "/"  + String(year(timetab[x])) + " " + String(hour(timetab[x])) + ":" + String(minute(timetab[x])) + ":"  + String(second(timetab[x]));messagetrame+="<br>";           
  }
  else{Serial.println(" Pas de données temps disponibles");}
}

/*
void send_command_interval(){ //message interval, seem to override datastream so I don't use now

  uint8_t target_system = 1;
  uint8_t target_component = 0;
  uint16_t CMD_LONG_command = 0;
  uint8_t CMD_LONG_confirmation = 0;
  float CMD_LONG_param1 = 0;
  float CMD_LONG_param2 = 0;
  float CMD_LONG_param3 = 0;
  float CMD_LONG_param4 = 0;
  float CMD_LONG_param5 = 0;
  float CMD_LONG_param6 = 0;
  float CMD_LONG_param7 = 0;
  uint8_t system_id = 1;
  uint8_t component_id = 195;
  
  mavlink_message_t message;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Servo
  CMD_LONG_param1 = MAVLINK_MSG_ID_ALTITUDE;//MAVLINK_MSG_ID_SERVO_OUTPUT_RAW; // #36
  CMD_LONG_param2 = 100; //10000 = 100Hz 4000 = 40hz
  CMD_LONG_command = MAV_CMD_SET_MESSAGE_INTERVAL;

  mavlink_msg_command_long_pack(system_id, component_id, &message, target_system, target_component, CMD_LONG_command, CMD_LONG_confirmation, CMD_LONG_param1, CMD_LONG_param2, CMD_LONG_param3, CMD_LONG_param4, CMD_LONG_param5, CMD_LONG_param6, CMD_LONG_param7);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &message);
  _MavLinkSerial.write(buf, len);

  //Heartbeat #0
  CMD_LONG_param1 = MAVLINK_MSG_ID_HEARTBEAT;
  CMD_LONG_param2 = 1; //10000 = 100Hz 4000 = 40hz
  CMD_LONG_command = MAV_CMD_SET_MESSAGE_INTERVAL;

  mavlink_msg_command_long_pack(system_id, component_id, &message, target_system, target_component, CMD_LONG_command, CMD_LONG_confirmation, CMD_LONG_param1, CMD_LONG_param2, CMD_LONG_param3, CMD_LONG_param4, CMD_LONG_param5, CMD_LONG_param6, CMD_LONG_param7);
  len = mavlink_msg_to_send_buffer(buf, &message);
  _MavLinkSerial.write(buf, len);
}*/

/*
void request_altitude(){// request home position on mavlink
  int drone_sysid = 1;
  int drone_compid = 0;
  int my_sysid = 1;
  int my_compid = 195;
 // int compid = MAV_COMP_ID_MISSIONPLANNER; 
  int chan = 0;
  //uint8_t chan = MAVLINK_COMM_0;
  mavlink_message_t msg_send;
  mavlink_command_long_t msg_long;
  msg_long.command = 512;//MAV_CMD_REQUEST_MESSAGE;
  msg_long.confirmation = 0;
  msg_long.target_system = drone_sysid;
  msg_long.target_component = drone_compid;
  msg_long.param1 = 141;
  msg_long.param2 = 0;
  msg_long.param3 = 0;
  msg_long.param4 = 0;
  msg_long.param5 = 0;
  msg_long.param6 = 0;
  msg_long.param7 = 0;

  mavlink_msg_command_long_encode_chan(my_sysid, my_compid, chan, &msg_send, &msg_long);
  Serial.println("Mavlink Altitude request sent !");
}*/
//##################### CAMERA MODIFICATION (functions) #######################

void searchCamera() {
  //WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(10);
  int cnt = WiFi.scanNetworks();
  Serial.print("Networks: ");
  if (cnt > 0) {
    for (int i = 0; i < cnt; ++i) {
      Serial.print(WiFi.SSID(i) + ",");
      if (WiFi.SSID(i).startsWith("YDXJ_")) {
        YI_SSID = WiFi.SSID(i);
        break;
      }
    }
  }
  Serial.println();
}

void connectToCamera() {
  bool result = true;
  short retry = 30;
  const int jsonPort = 7878;
  char password[11] = "1234567890";
  char ssid[30];
  Serial.print("Con: ");
  YI_SSID.toCharArray(ssid, YI_SSID.length() + 1);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    if (retry == 0) {
      result = false;
      break;
    }
    delay(500);
    retry--;
  }
  Serial.print(" -> wifi con:");
  if (result == true) Serial.print("OK "); else Serial.print("XX ");

  if (!client.connect("192.168.42.1", jsonPort)) result = false;
  Serial.print(" IP con:");
  if (result == true) Serial.print("OK."); else Serial.print("XX.");
  Serial.println();
}

String requestToken() {
  String token;
  // This will send the request token msg to the server
  client.print("{\"msg_id\":257,\"token\":0}\n\r");
  //delay(1000);
  yield(); delay(250); yield(); delay(250); yield(); delay(250); yield(); delay(250);
  // Read all the lines of the reply from server and print them to Serial
  String response;
  while (client.available()) {
    char character = client.read();
    response.concat(character);
  }
  // Search token in to the stream
  int offset = response.lastIndexOf(':');
  if (offset != -1) {
    for (int i = offset + 1; i < response.length(); ++i) {
      if ((response.charAt(i) != ' ') && (response.charAt(i) != '}')) {
        token.concat(response.charAt(i));
      }
    }
  }

  return token;
}
 
void TakePhoto(String token) {
  photorequestnumber ++;
  if(RecON){
    RecordOFF(token);
    RecON = false;
    String token = requestToken();
    if (token.length() == 0) exit;
  }
  client.print("{\"msg_id\":769,\"token\":");
  client.print(token);
  client.print("}\n\r");
  Serial.print("Photo - Response: ");
  yield(); delay(25); yield(); delay(25); yield(); delay(25); yield(); delay(25);
  String response;
  while (client.available()) {
    char character = client.read();
    response.concat(character);
  }
  //Serial.println(response);
  if (response.startsWith("0", 8)){//if photo is done dont force photo
    forcephotorequest = false;
    photorequestnumber = 0;
    lastphoto = millis();
    Serial.print("Photo request accepted. Time = ") && Serial.println(millis());
    }
  else{
    forcephotorequest = true;}
    //Serial.print("Photo request rejected, force a new one! Time= ")&& Serial.println(millis());}//if photo isn't done, force it
  if (RecON) {
    String token = requestToken();
    if (token.length() == 0) exit;
    RecordON(token);
  }
}

void RecordON(String token) {
  client.print("{\"msg_id\":513,\"token\":");
  client.print(token);
  client.print("}\n\r");
  Serial.println("RecON - Response: ");
  yield(); delay(250); yield(); delay(250); yield(); delay(250); yield(); delay(250);
  String response;
  while (client.available()) {
    char character = client.read();
    response.concat(character);
  }
  //Serial.println(response);
  if (response.startsWith("0", 8))// if camera return video record is accepted recon= true, if not recon= false (next loop will ask again to record)
    RecON = true;
  else{RecON = false;}
  //Serial.print("RecOn in recordon function = ") && Serial.println(RecON);
  
}

void RecordOFF(String token) {
  client.print("{\"msg_id\":514,\"token\":");
  client.print(token);
  client.print("}\n\r");
  //Serial.print("RecOFF - Response: ");
  yield(); delay(250); yield(); delay(250); yield(); delay(250); yield(); delay(250);
  String response;
  while (client.available()) {
    char character = client.read();
    response.concat(character);
  }
  //Serial.println(response);
}



void PWMselector(){

  //Print pwm_value
  Serial.print("PWM read =") && Serial.println(pwm_value);
  
  //Detect if pwm is from rc or servo pin and set delay
  //if(ServoTriggerbool == 1)stabePWMdelay = 0;
  //else{stabePWMdelay = 80;}
  //stabePWMdelay = 80;

  //Detect pwm variation and reset time counter if range change (range 0=do nothing 1=photo 2=video)
  if(pwm_value < photoPWM || pwm_value > maxPWM)PWMrange = 0;
  if(pwm_value >= photoPWM && pwm_value < videoPWM)PWMrange = 1;
  if(pwm_value >= videoPWM)PWMrange = 2;
  if(PWMrange != prevPWMrange)timeinmem = millis();
  prevPWMrange = PWMrange;
  
  //If photorequest number is high, camera problem so abandon the attempt to take picture
  if(photorequestnumber >= 10){
    photorequestnumber = 0;
    forcephotorequest = false;
  }
  
  //1)to stop record if low pwm
  if(PWMrange < 2 && RecON == true && millis() - timeinmem >= stabePWMdelay) {//do it if pwm last stablePWMdelay, same logic for photo(2) and video(3)
    //if(timeinmem == 0){timeinmem = millis();}
    //if(millis() - timeinmem >= stabePWMdelay){
    String token = requestToken();
    if (token.length() != 0) {
      RecordOFF(token);
      //timeinmem = 0;
      RecON = false;//tell video record stopped
    }
   }


  //2)to take photo if pwm in photo range after small delay in range
  if(PWMrange == 1 && millis() - timeinmem >= stabePWMdelay /*&& millis() - lastphoto > delayBTWphotos*/ || forcephotorequest == true || ServoTriggerbool == 1 /*&& millis() - lastphoto > delayBTWphotos*/){
    String token = requestToken();
    if (token.length() != 0){
      Serial.print("Photo requested Time= ") && Serial.println(millis());
      //TakePhoto(token);
      shoot(token);
    }
  }


  //3)to take video if within pwm range (stop with one or two
  if(PWMrange == 2 && RecON == false && millis() - timeinmem >= stabePWMdelay ) {
      String token = requestToken();
      if (token.length() != 0) {
        RecordON(token);
      }
  }
  yield();
}
//#########################################################################################

void shoot(String token) {

  String INcamera = "";
  int conn = 0, state = 0;
  /*String*/ token = requestTokenTWO();//bl
  client.flush();
  client.print("{\"msg_id\":769,\"token\":");
  client.print(token);
  client.print("}\n\r");
  unsigned long whiletime = millis();//bl
  while (millis()-whiletime <= 3000) {
    if (INcamera.substring(24, 32) == "vf_start") {// original : vf_start
      //delay(800); // show "CAPTURED" string on OSD screen for 0.8s, if camera confirmed capturing
      break;
    }
    INcamera = "";
    while (!client.available() && millis()-whiletime <= 3000) {/*Serial.println("client not available while");*/}//bl (seulement serialprint)
    while (client.available() && millis()-whiletime <= 3000) {

      char k = client.read();
      INcamera += k;
      if (k == '}') {
        Serial.println(INcamera);// bl
#ifdef DEBUG_CAMERA_OUTPUT
        Serial.println(INcamera);
#endif
      }
      //Serial.println("client available while");//bl
    }
    //Serial.println("shoot while");//bl
  }

#ifdef DEBUG
  Serial.println("Photo shot finished : "); Serial.println(millis());
  forcephotorequest = false;
  photorequestnumber = 0;
  lastphoto = millis();
#endif

  //delay(1000);
}


String requestTokenTWO() {
  String token = "" ;
  String INcamera = "";
  //Serial.println("stark token");
  // This will send the request token msg to the server
//  digitalWrite(BUILTIN_LED1, !digitalRead(BUILTIN_LED1)); // blink
  //digitalWrite(BUILTIN_LED1, LOW);
  while (1) {

    if (INcamera.substring(29, 34) == "param") {
      //param finden und string davor abschneiden
      int startid = INcamera.indexOf("param");
      String reststring = INcamera.substring(startid);
      // 7 weil param": wegschneiden und dann starten, solange kein leer, Klammer oder , gefunden wird
      for (int i = 8; i < reststring.length(); ++i) {
        //wenn leer, klammer Ende, oder , dann ist der param bereich zuende
        if ((reststring.charAt(i) == ' ') || (reststring.charAt(i) == '}') || (reststring.charAt(i) == ',')) {
          //Serial.println("Ende gefunden");
          break;
        }
        token.concat(reststring.charAt(i));
      }
      break;
    } else {
      //Serial.println("token request done: ");
      client.print("{\"msg_id\":257,\"token\":0}\n\r");
    }

    INcamera = "";

    while (!client.available()) {}
    //counter_while_client++;
    //yield();

    while (client.available()) {
      char k = client.read();
      INcamera += k;
      if (k == '}') {
        //Serial.print(INcamera);
      }
    }
    //Serial.println(INcamera);
  }
  //digitalWrite(BUILTIN_LED1, HIGH);
  //Serial.println("While ende");
  //Serial.println(token);
  return token;
}
