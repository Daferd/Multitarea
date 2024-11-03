#include <Arduino.h>
#include "GlobalDef.h"



/*void IRAM_ATTR handleButtonPress(void) {
    // Verificar cuál botón activó la interrupción
    if (digitalRead(channels.btnPinHW[0]) == LOW) {
        Serial.println("Botón 1 presionado");
        channels.chState[0] = !channels.chState[0];
        digitalWrite(channels.chPinHW[0],channels.chState[0]);
        // Realiza acción para el botón 1
    } else if (digitalRead(channels.btnPinHW[1]) == LOW) {
        Serial.println("Botón 2 presionado");
        // Realiza acción para el botón 2
    }
}*/

void setup() {
    Serial.begin(9600);
    //attachInterrupt(BTN1,handleButtonPress1,FALLING);
    //attachInterrupt(BTN2,handleButtonPress2,FALLING);
    //attachInterrupt(BTN3,handleButtonPress3,FALLING);
    //attachInterrupt(BTN4,handleButtonPress4,FALLING);

    

    initWiFi(SSID, PASS);

    pinMode(ACT,OUTPUT);
    digitalWrite(ACT,LOW);
    //pinMode(BTN_BOOT,INPUT_PULLUP);
    //attachInterrupt(BTN_BOOT,handleButtonPress() ,FALLING);

     channels.chInit(1,CH1);

     //pinMode(CH1,INPUT_PULLDOWN);
     channels.chInit(2,CH2);
     channels.chInit(3,CH3);
     channels.chInit(4,CH4);

    //channels.btnInit(1,BTN1);
    //channels.btnInit(3,BTN3);
    pinMode(BTN1,INPUT_PULLUP);
    pinMode(BTN2,INPUT_PULLUP);
    pinMode(BTN3,INPUT_PULLUP);
    pinMode(BTN4,INPUT_PULLUP);

    //dht.begin();

    
    xTaskCreatePinnedToCore(MqttTask       , "MqttTask",        4000, NULL, 1, &xMqttTaskHandle,         APP_CPU);
    delay(100);
    xTaskCreatePinnedToCore(ReceiveTask       , "ReceiveTask",        6000, NULL, 1, &xReceiveTaskHandle,         APP_CPU);
    vTaskSuspend(xReceiveTaskHandle);
    delay(100);
    xTaskCreatePinnedToCore(ReadSensorsTask, "ReadSensorsTask", 4000, NULL, 1, &xReadSensorsTaskHandle,  APP_CPU);
    delay(100);
    xTaskCreatePinnedToCore(I2cTask       , "I2cTask",        4000, NULL, 1, NULL,         APP_CPU);
    delay(100);
    xTaskCreatePinnedToCore(ButtonPressTask, "ButtonPressTask", 4000, NULL, 1, NULL,  APP_CPU);
    delay(100);
    //xTaskCreatePinnedToCore(SchedulesTask  , "SchedulesTask",   4000, NULL, 1, &xSchedulesTaskHandle,    APP_CPU);

    Serial2.begin(115200, SERIAL_8N1, RXD2,TXD2);  // Rx = 4, Tx = 5 will work for ESP32, S2, S3 and C3
    Serial2.onReceive(onReceiveFunction, true); //Se habilita la interrupción de la UART 2
    delay(100);

    //xSemaphore = xSemaphoreCreateMutex();
}

void loop() {
    //Serial.println("Loop");

    /*vTaskDelay(2000 / portTICK_PERIOD_MS);

    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }


    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.println(F("°C "));   */ 
}
