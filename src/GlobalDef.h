#ifndef _MYDEF_H
#define _MYDEF_H
#include <Arduino.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <Adafruit_Sensor.h>
#include "DHT.h"         // Para el sensor de temperatura y humedad DHT

#include "ChannelCtrl.h"
#include "ShedulesManager.h"

#include <BH1750.h>
#include <Wire.h>

#define APP_CPU 1
#define PRO_CPU 0

#define ACT 2             // Pin led
#define BTN_BOOT 0        // Boton Boot

// Definiciones de pines
#define DHT_PIN 32        // Pin del sensor DHT
#define LM35_PIN 35        // Pin Analogico 0 "A0" para sensor LM35
//#define BUTTON_PIN 0     // Pin del pulsador

#define IN1 33
#define IN2 32
#define IN3 35
#define IN4 34
#define IN5 18
#define IN6 19

//PINES PARA CONEXION LORA
#define RXD2 26 
#define TXD2 27 

//Pines I2C
#define SCL 14
#define SDA 13


#define CH1 4
#define CH2 16
#define CH3 17  
#define CH4 5

// #define CH1 13
// #define CH2 14
// #define CH3 27  
// #define CH4 26


#define BTN1 18
#define BTN2 19
#define BTN3 21
#define BTN4 22



// String SSID = "-__GOINN_S";    
// String PASS = "90079192";

String SSID = "Moto_AH";    
String PASS = "12345678";

// String SSID = "MILO";    
// String PASS = "MILOVALERO17";   

// String SSID = "ESP32_Hotspot";                       
// String PASS = "12345678";                  

 /*const char* mqtt_server = "192.168.20.105";  //Mosquitto  
 const char* id = "Esp";
 const char* user = "prueba";
 const char* codepass = "123456";*/

 const char* mqtt_server = "eiaiotdario.cloud.shiftr.io";  //node02.myqtthub.com  192.168.4.1
 const char* id = "Esp";
 const char* user = "eiaiotdario";   
 const char* codepass = "SKWpvAWoAsagTuzp"; 

// const char* mqtt_server = "mycelium-1.cloud.shiftr.io";  //node02.myqtthub.com  192.168.4.1
//  const char* id = "Esp";
//  const char* user = "mycelium-1";   
//  const char* codepass = "pkaYpnJZZz92twZb"; 

// const char* mqtt_server = "node02.myqtthub.com";  
// const char* id = "EspPrueba";
// const char* user = "prueba";   
// const char* codepass = "123456"; 

TaskHandle_t xReadSensorsTaskHandle, xSchedulesTaskHandle, xMqttTaskHandle, xReceiveTaskHandle;
//SemaphoreHandle_t xSemaphore;

WiFiClient espClient;
PubSubClient client(espClient);
String _topic, _payload;

ShedulesManager myRTC;
ESP32Time rtc(0);
Event eventsChannel1[6];

bool enableChFlag[8] = {false,false,false,false,false,false,false,false};
ChannelCtrl channels;
ChannelServer chServer;

unsigned long lastMsg = 0;
const long interval = 5000; // 5 segundos

uint8_t chCont = 0;

// Configuración del sensor DHT
DHT dht(DHT_PIN, DHT11);

uint16_t globalAccount = 1;  //SE INICIALIZA EN EL PRIMER ESTADO QUE VA A ENVIAR, SE ASUME QUE TODOS EMPIEZAN EN CERO

float tempLm35 = 0;

BH1750 lightMeter;

volatile unsigned long lastDebounceTime1 = 0;
volatile unsigned long lastDebounceTime2 = 0;
volatile unsigned long lastDebounceTime3 = 0;
volatile unsigned long lastDebounceTime4 = 0;
volatile unsigned long lastInterruptTime1 = 0;


const unsigned long debounceDelay = 200; 


uint8_t sensors[6];  //Se almacena el estado de cada pin
uint8_t sensorsLora[8];  //Se almacena el estado de cada pin

const char topicChannels   [] =  "userID/getway/channels";
const char topicSensores   [] =  "userID/getway/sensores";
const char topicEvents   [] =  "userID/getway/events";
const char topicLed        [] =  "userID/getway/led";
const char topicLora        [] =  "userID/getway/lora";


void ReadSensorsTask(void *pvParameters);
void ReceiveTask(void *pvParameters);
void ButtonPressTask(void *pvParameters);
void SchedulesTask(void *pvParameters);
void MqttTask(void *pvParameters);
void I2cTask(void *pvParameters);
void testHwm(char * taskName); //Me permite conocer cuanta RAM NO se esta Usando en una tarea "taskName"

bool initWiFi(String SSID, String PASS);
float readLm35(uint8_t analogPin);
template <typename T>
void printMsg(String label, T val);

void callback(char* topic, byte* payload, unsigned int length);
void reconnect(void);
void onReceiveFunction(void);
void receiveLoraData(void);
String extractFragment(String input);

void IRAM_ATTR handleButtonPress1(void) {
    unsigned long currentTime = millis();
    if (currentTime - lastInterruptTime1 > debounceDelay) {
        Serial.println("Botón 1 presionado");

        channels.chState[0] = !channels.chState[0];
        client.publish("userid/getway/ch1", String(channels.chState[0]).c_str());

        digitalWrite(channels.chPinHW[0],channels.chState[0]);
        
    }
}


void testHwm(char * taskName){
  static int stack_hwm, stack_hwm_temp;

  stack_hwm_temp = uxTaskGetStackHighWaterMark(nullptr);
  if( !stack_hwm || (stack_hwm_temp < stack_hwm)){
      stack_hwm = stack_hwm_temp;
      Serial.printf("%s Memoria RAM libre: %u\n", taskName, stack_hwm);
  }
}

void ReadSensorsTask(void *pvParameters) {

    while(1) {

         JsonDocument doc;
         char jsonBuffer[512];
         JsonArray sensorsJson = doc["sensors"].to<JsonArray>();
        //tempLm35 = readLm35(LM35_PIN);
        //Serial.print("TemperaturaLM35: ");Serial.print(int(tempLm35));Serial.println(" °C");

        sensorsJson.add(int(random(15,28)));
        sensorsJson.add(readLm35(LM35_PIN));
        sensorsJson.add(int(random(15,28)));
        sensorsJson.add(int(random(15,28)));

        serializeJson(doc, jsonBuffer);
        //client.publish(topicSensores, payload.c_str());

        client.publish(topicSensores, jsonBuffer);

        vTaskDelay(30000 / portTICK_PERIOD_MS);

        // Reading temperature or humidity takes about 250 milliseconds!
        // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    }
}

void ButtonPressTask(void *pvParameters) {

    while(1) {

        JsonDocument doc;
        char jsonBuffer[512];
        JsonArray channelsJson = doc["channels"].to<JsonArray>();

        // Lee el estado de los botones
        int reading1 = digitalRead(BTN1);
        int reading2 = digitalRead(BTN2);
        int reading3 = digitalRead(BTN3);
        int reading4 = digitalRead(BTN4);

        // ----- Debounce para el botón 1 -----
        if (reading1 == LOW && (millis() - lastDebounceTime1) > debounceDelay) {
            lastDebounceTime1 = millis(); // Actualiza el tiempo del debounce
            Serial.println("Botón 1 presionado");

            channels.chState[0] = !channels.chState[0];
            //client.publish("userid/getway/ch1", String(channels.chState[0]).c_str());

            digitalWrite(channels.chPinHW[0],channels.chState[0]);
            channelsJson.add(channels.chState[0]);
            channelsJson.add(channels.chState[1]);
            channelsJson.add(channels.chState[2]);
            channelsJson.add(channels.chState[3]);
            serializeJson(doc, jsonBuffer);
            client.publish(topicChannels, jsonBuffer);
            // Realiza la acción para el botón 1
        }

        if (reading2 == LOW && (millis() - lastDebounceTime2) > debounceDelay) {
            lastDebounceTime2 = millis(); // Actualiza el tiempo del debounce
            Serial.println("Botón 2 presionado");

            channels.chState[1] = !channels.chState[1];
            
            //client.publish("userid/getway/ch2", String(channels.chState[1]).c_str());

            digitalWrite(channels.chPinHW[1],channels.chState[1]);
            channelsJson.add(channels.chState[0]);
            channelsJson.add(channels.chState[1]);
            channelsJson.add(channels.chState[2]);
            channelsJson.add(channels.chState[3]);
            serializeJson(doc, jsonBuffer);
            client.publish(topicChannels, jsonBuffer);
            // Realiza la acción para el botón 1
        }

        // ----- Debounce para el botón 2 -----
        if (reading3 == LOW && (millis() - lastDebounceTime3) > debounceDelay) {
            lastDebounceTime3 = millis(); // Actualiza el tiempo del debounce
            Serial.println("Botón 3 presionado");
            channels.chState[2] = !channels.chState[2];
            //client.publish("userid/getway/ch3", String(channels.chState[2]).c_str());
            digitalWrite(channels.chPinHW[2],channels.chState[2]);
            channelsJson.add(channels.chState[0]);
            channelsJson.add(channels.chState[1]);
            channelsJson.add(channels.chState[2]);
            channelsJson.add(channels.chState[3]);
            serializeJson(doc, jsonBuffer);
            client.publish(topicChannels, jsonBuffer);

            
            // Realiza la acción para el botón 2
        }

        if (reading4 == LOW && (millis() - lastDebounceTime4) > debounceDelay) {
            lastDebounceTime4 = millis(); // Actualiza el tiempo del debounce
            Serial.println("Botón 4 presionado");

            channels.chState[3] = !channels.chState[3];
            //client.publish("userid/getway/ch4", String(channels.chState[3]).c_str());
            digitalWrite(channels.chPinHW[3],channels.chState[3]);
            channelsJson.add(channels.chState[0]);
            channelsJson.add(channels.chState[1]);
            channelsJson.add(channels.chState[2]);
            channelsJson.add(channels.chState[3]);
            serializeJson(doc, jsonBuffer);
            client.publish(topicChannels, jsonBuffer);
            // Realiza la acción para el botón 1
        }

    }
}

void SchedulesTask(void *pvParameters) {
    
    printMsg("Schedules Activoo!!!","");
    Serial.println("Hora actualizada: "); Serial.print(rtc.getTime()); Serial.print(", Dia: "); Serial.println(rtc.getDay());

    //channels.chInit(1,CH1);
    //channels.chInit(2,CH2);
    //channels.chInit(3,CH3);
    //channels.chInit(4,CH4);

    eventsChannel1[0].state  = false;
    eventsChannel1[0].action = true;
    eventsChannel1[0].hour = 21;
    eventsChannel1[0].min = 48;
    eventsChannel1[0].days[0] = true;
    eventsChannel1[0].days[1] = true;
    eventsChannel1[0].days[2] = true;
    eventsChannel1[0].days[3] = true;
    eventsChannel1[0].days[4] = true;
    eventsChannel1[0].days[5] = true;
    eventsChannel1[0].days[6] = true;

    eventsChannel1[1].state  = false;
    eventsChannel1[1].action = false;
    eventsChannel1[1].hour = 21;
    eventsChannel1[1].min = 49;
    eventsChannel1[1].days[0] = true;
    eventsChannel1[1].days[1] = true;
    eventsChannel1[1].days[2] = true;
    eventsChannel1[1].days[3] = true;
    eventsChannel1[1].days[4] = true;
    eventsChannel1[1].days[5] = true;
    eventsChannel1[1].days[6] = true;

    //yRTC.printEventTimes(eventsChannel1,6);

    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    while(1) {
        /*chServer.num = 1;
        chServer.state = false;
        channels.chEnableHw(chServer);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        chServer.num = 2;
        chServer.state = false;
        channels.chEnableHw(chServer);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        chServer.num = 3;
        chServer.state = false;
        channels.chEnableHw(chServer);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        chServer.num = 4;
        chServer.state = false;
        channels.chEnableHw(chServer);
        vTaskDelay(100 / portTICK_PERIOD_MS);*/

        chServer.num = 1;
        
        uint8_t numEvents = sizeof(eventsChannel1) / sizeof(eventsChannel1[0]);
        chServer.state= myRTC.stateDefine(eventsChannel1,numEvents,channels.chState[chServer.num-1]);
        channels.chEnableHw(chServer);

        //enableChFlag[ch-1] = myRTC.stateDefine(eventsChannel1,numEvents,enableChFlag[ch-1]);  // la funcion stateDefine solo modifica al canal 1, se debe cambia REBISAR!!
        //digitalWrite(CH1,enableChFlag[ch-1]);
        //digitalWrite(ACT,enableChFlag[ch-1]);
        testHwm("SchedulesTask"); // consume 512 bite     
                   
        
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void I2cTask(void *pvParameters){

  Wire.begin(SDA,SCL);
  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));

  while (1){
        float lux = lightMeter.readLightLevel();
        Serial.print("Light: ");
        Serial.print(lux);
        Serial.println(" lx");

        client.publish("userID/getway/i2c/lux", String(lux).c_str());
        vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
  

}

void MqttTask(void *pvParameters){
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    while (1){
        if (!client.connected()) {
            reconnect();
        }
        client.loop();
        digitalWrite(ACT,HIGH);
        //testHwm("MqttTask"); // consume 512 bites
    }
}

void ReceiveTask(void *pvParameters){
    
    while (1){
        
            Serial.println("RECEIVE ACTIVOO...............");
            // if(_topic.equals(topicLed)){
            //     if(_payload.equals("on")){
            //         digitalWrite(ACT,HIGH);
            //     }else if(_payload.equals("off")){
            //         digitalWrite(ACT,LOW);
            //     }else{

            //     }   
            //}
            if(_topic.equals(topicChannels)){
                JsonDocument doc;
                deserializeJson(doc, _payload);

                channels.chState[0] = doc["channels"][0];
                channels.chState[1] = doc["channels"][1];
                channels.chState[2] = doc["channels"][2];
                channels.chState[3] = doc["channels"][3];

                printMsg(String(channels.chPinHW[0]),channels.chState[0]);
                printMsg(String(channels.chPinHW[1]),channels.chState[1]);
                printMsg(String(channels.chPinHW[2]),channels.chState[2]);
                printMsg(String(channels.chPinHW[3]),channels.chState[3]);

                digitalWrite(channels.chPinHW[0],channels.chState[0]);
                digitalWrite(channels.chPinHW[1],channels.chState[1]);
                digitalWrite(channels.chPinHW[2],channels.chState[2]);
                digitalWrite(channels.chPinHW[3],channels.chState[3]);

            }else if(_topic.equals(topicSensores)){
                JsonDocument doc;
                deserializeJson(doc, _payload);

                sensors[0] = doc["s1"];
                sensors[1] = doc["s2"];
                sensors[2] = doc["s3"];
                sensors[3] = doc["s4"];
                sensors[4] = doc["s5"];
                sensors[5] = doc["s6"];
            }else if(_topic.equals(topicEvents)){
                JsonDocument doc;
                deserializeJson(doc, _payload);

                uint8_t event = doc["channel"];
                eventsChannel1[event-1].state  = doc["state"];
                eventsChannel1[event-1].action = doc["s1"];
                eventsChannel1[event-1].hour   = doc["s1"];
                eventsChannel1[event-1].min    = doc["s1"];

                JsonArray daysArray = doc["days"];
                for (size_t i = 0; i < 7; i++) eventsChannel1[event-1].days[i] = daysArray[i];
                
            }
            
         
            vTaskSuspend(xReceiveTaskHandle);

            //testHwm("ReceiveTask"); // consume 512 bite 
    }
}

float readLm35(uint8_t analogPin){

  int valorSensor = 0;
  float temp=0;
  float tempAux=0;
  
  for (size_t i = 0; i < 3; i++){
    valorSensor =  analogRead(analogPin);       // Lee el valor analógico del sensor LM35
    float tempAux = float(valorSensor) * (4095.0 / 3330.0);  // Convierte el valor analógico a temperatura en grados Celsius
    temp = temp + ((tempAux/10)+3.3);
  }

  temp = temp/3;

  //valorSensor = valorSensor / 5;
  //valorSensor = analogRead(LM35_PIN);       // Lee el valor analógico del sensor LM35

  // En el caso del LM35, cada 10 mV se corresponde con 1°C
  return temp;
}

template <typename T>
void printMsg(String label, T val) {
  Serial.print(label); 
  Serial.println(val);
}

bool initWiFi(String SSID, String PASS){  

    bool state=false;    
    boolean bandStateWifi = false;
    unsigned char tamano1= SSID.length();
    char nombreRed[tamano1];

    SSID.toCharArray(nombreRed,tamano1+1);

    unsigned char tamano2= PASS.length();
    char passRed[tamano2];

    PASS.toCharArray(passRed,tamano2+1);
    
    WiFi.mode(WIFI_STA);                      // Estacion 
    //WiFi.mode(WIFI_AP);                     // Punto de Acceso 
    //WiFi.mode(WIFI_MODE_APSTA);             // Ambos 

    WiFi.begin(nombreRed,passRed);                   // Inicializamos el WiFi con nuestras credenciales.
    Serial.print("***** INCIANDO PROCESO DE CONEXIÓN A RED: ");Serial.print(nombreRed);Serial.print(" *****");Serial.println(" ");
 

    while(WiFi.status() != WL_CONNECTED){     // Se quedata en este bucle hasta que el estado del WiFi sea diferente a desconectado.  
        Serial.print(".");
        delay(100);
        //ledBlinkMillis(200);

    }

    if(WiFi.status() == WL_CONNECTED){        // Si el estado del WiFi es conectado entra al If
        
        Serial.println();
        Serial.println();
        Serial.println("Conexion exitosa!!!");
        Serial.println("");
        Serial.print("Tu IP es: ");
        Serial.println(WiFi.localIP());
        //digitalWrite(ACT,LOW);

        myRTC.updateDatetime();
        Serial.println("**************************************************");Serial.println();

        state = true;
    }else{
        state = false;
        Serial.println("error WIFI 1");
        //fuente.encenderTodoLuces(255,0,0);
        //fuente.ledBlinkMillis(50);
        //contIntentos = 0;
        //initWiFi(ssidWifi,passWifi);
    }

    return state;
}

void callback(char* topic, byte* payload, unsigned int length) {
    String conc_payload_;

    for (int i = 0; i < length; i++) {
      conc_payload_ += (char)payload[i];
    }
    _topic = topic;
    _payload = conc_payload_;

    Serial.println(_topic);
    Serial.println(_payload);

    vTaskResume(xReceiveTaskHandle);
}

void reconnect(void) {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Intentando conectar MQTT...");


    if (client.connect(id, user, codepass)) {
      Serial.println("connected");
      client.subscribe(topicChannels);
    } else {
      Serial.print("falla, estado=");
      Serial.print(client.state());
      Serial.println(" Intentando en 5 seg");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void onReceiveFunction(void) {
    receiveLoraData();
}

/*
void receiveLoraData(void){

    digitalWrite(ACT, LOW); //led OFF: Empieza la transmisión
    //timerAlarmDisable(timer1); //Desabilito el timer de cuenta autonoma porque ya se recibio mensaje y deja de necesitarse
    //showTime=0; // Se garantiza que la proxima vez que se activa el conteo autonomo empieza en cero.

    // This is a callback function that will be activated on UART RX events
    String textoEntrada=Serial2.readString(); //Almacena los datos recibidos del emisor 
    //Serial.print("MensajeFull: "); Serial.println(textoEntrada);
    
    // Se LLamada a la función para extraer el fragmento que se encuentra en medio de dos asteriscos 
    String extractedFragment = extractFragment(textoEntrada);
    //Serial.print("Fragmento: "); Serial.println(extractedFragment);

    //Como se sabe que el fragmento entre los dos astericos es un numero, se convierte a entero
    globalAccount = atoi(extractedFragment.c_str());


    //Logica de avance de estado segun la cuenta glogal
    //globalAccountLogic();

    digitalWrite(ACT, HIGH); //led OFF: Empieza la transmisión
    //startShowTime = millis();
    
    Serial.print("Estado: "); Serial.println(globalAccount);
    client.publish("Esp1/cuenta", String(globalAccount).c_str());
}
*/
void receiveLoraData(void){

    digitalWrite(ACT, LOW); //led OFF: Empieza la transmisión
 
    String _payloadLora=Serial2.readString(); //Almacena los datos recibidos del emisor 
    //Serial.print("MensajeFull: "); Serial.println(textoEntrada);
    String extractedFragment = extractFragment(_payloadLora);

    JsonDocument doc;
    deserializeJson(doc,extractedFragment);

    sensorsLora[0] = doc["hum"];
    sensorsLora[1] = doc["temp"];

    printMsg("Hum: ", sensorsLora[0]);
    printMsg("Temp: ", sensorsLora[1]);
    
    // Se LLamada a la función para extraer el fragmento que se encuentra en medio de dos asteriscos 
    //String extractedFragment = extractFragment(textoEntrada);
    //Serial.print("Fragmento: "); Serial.println(extractedFragment);

    //Como se sabe que el fragmento entre los dos astericos es un numero, se convierte a entero
    //globalAccount = atoi(extractedFragment.c_str());


    //Logica de avance de estado segun la cuenta glogal
    //globalAccountLogic();

    client.publish("userID/getway/lora/cuenta", extractedFragment.c_str());

    digitalWrite(ACT, HIGH); //led OFF: Empieza la transmisión
    //startShowTime = millis();
    
    //Serial.print("Estado: "); Serial.println(globalState);
}

// Función para extraer el fragmento de texto entre asteriscos
String extractFragment(String input) {
    int startIndex = input.indexOf('*');
    int endIndex = input.indexOf('*', startIndex + 1);

    // Verificar que ambos asteriscos fueron encontrados
    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
        return input.substring(startIndex + 1, endIndex);
    } else {
        return "No se encontró el fragmento";
    }
}






#endif //_MYDEF_H
