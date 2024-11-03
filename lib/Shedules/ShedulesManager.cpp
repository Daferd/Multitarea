#include "ShedulesManager.h"

ShedulesManager::ShedulesManager(/* args */){
}

ShedulesManager::~ShedulesManager(){
}

//C O N T R O L  D E  H O R A R I O S (Timer)
uint8_t ShedulesManager::updateDatetime(void){
    uint8_t seconds = 0;
    WiFiUDP ntpUDP;
    NTPClient timeClient(ntpUDP, "pool.ntp.org");

    timeClient.begin();
    timeClient.setTimeOffset(-18000);
    timeClient.update();
    delay(200);
    rtc.setTime(timeClient.getEpochTime());
    Serial.print("Hora actualizada: "); Serial.println(rtc.getTime());

    //Se sincronización el timer con el reloj
    seconds = 60 - rtc.getSecond();
    //delay(seconds * 1000);
    

    return seconds;
}

bool ShedulesManager::timerAction(Event eventAux, bool stateCurrent){
  bool action = stateCurrent;
  uint8_t currentDay = rtc.getDayofWeek();
  uint8_t currentHour = rtc.getHour(true);
  uint8_t currentMinute = rtc.getMinute();

  if (eventAux.days[currentDay]) {
    if (currentHour > eventAux.hour || (currentHour == eventAux.hour && currentMinute >= eventAux.min)) {
      action = eventAux.action;
    }
  }
  return action;
}

bool ShedulesManager::stateDefine(Event events[], uint8_t numEvents, bool stateCurrent){

    bool stateDef = stateCurrent;
    uint8_t sortedIndices[numEvents];
    getSortedIndices(events, numEvents, sortedIndices);

    // Imprimir los índices ordenados
    /*Serial.println("Índices ordenados:");
    for (uint8_t i = 0; i < numEvents; ++i) {
        Serial.print(sortedIndices[i]);
        Serial.print(" ");
    }
    Serial.println();*/

    for (uint8_t i = 0; i < numEvents; ++i) {
        uint8_t index = sortedIndices[i];
        if (events[index].state) {
            stateDef = timerAction(events[index], stateDef);
        }
    }

  return stateDef;
}

void ShedulesManager::swapEvents(Event &event1, Event &event2) {
    Event temp = event1;
    event1 = event2;
    event2 = temp;
}

void ShedulesManager::sortEventsByTime(Event events[], int size) {
    for (int i = 0; i < size - 1; i++) {
        int minIndex = i;
        for (int j = i + 1; j < size; j++) {
            if (events[j].hour < events[minIndex].hour ||
                (events[j].hour == events[minIndex].hour && events[j].min < events[minIndex].min)) {
                minIndex = j;
            }
        }
        if (minIndex != i) {
            swapEvents(events[i], events[minIndex]);
        }
    }
}

void ShedulesManager::printEventTimes(Event events[], int size) {
    Serial.println("Event Times:");
    for (int i = 0; i < size; i++) {
        Serial.print("Event ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(events[i].hour);
        Serial.print(":");
        if (events[i].min < 10) {
            Serial.print("0");
        }
        Serial.print(events[i].min);
        Serial.print(", State: "); 
        Serial.print(events[i].state); 
        Serial.print(", Action: "); 
        Serial.print(events[i].action); 
        Serial.print(", D: "); 
        Serial.print(events[i].days[0]);
        Serial.print(", L: "); 
        Serial.print(events[i].days[1]);
        Serial.print(", M: "); 
        Serial.print(events[i].days[2]);
        Serial.print(", W: "); 
        Serial.print(events[i].days[3]);
        Serial.print(", J: "); 
        Serial.print(events[i].days[4]);
        Serial.print(", V: "); 
        Serial.print(events[i].days[5]);
        Serial.print(", S: "); 
        Serial.println(events[i].days[6]);
       

    }
}

void ShedulesManager::getSortedIndices(const Event events[], uint8_t numEvents, uint8_t indices[]) {
  for (uint8_t i = 0; i < numEvents; ++i) {
    indices[i] = i;
  }

  for (uint8_t i = 0; i < numEvents - 1; ++i) {
    for (uint8_t j = 0; j < numEvents - i - 1; ++j) {
      if ((events[indices[j]].hour > events[indices[j + 1]].hour) || 
          (events[indices[j]].hour == events[indices[j + 1]].hour && events[indices[j]].min > events[indices[j + 1]].min)) {
        uint8_t temp = indices[j];
        indices[j] = indices[j + 1];
        indices[j + 1] = temp;
      }
    }
  }
}
