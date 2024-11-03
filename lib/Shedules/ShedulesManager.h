#ifndef SHEDULESMANAGER_H
#define SHEDULESMANAGER_H

#include <ESP32Time.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define CHANNELS_TOTAL 2
#define TIMERS_TOTAL 6


//Variable y Estructuras para el control del Timer
struct Event {
  uint8_t min;
  uint8_t hour;
  bool days[7];
  bool state;
  bool action;
};

class ShedulesManager{
    private:
        

    public:
        ShedulesManager(/* args */);
        ~ShedulesManager();

        //Variables para configurar la hora mediante Wifi
        ESP32Time rtc;
        uint8_t updateDatetime(void);
        bool timerAction(Event eventAux, bool stateCurrent);
        bool stateDefine(Event events[], uint8_t numEvents ,bool stateCurrent);
        void swapEvents(Event &event1, Event &event2);
        void sortEventsByTime(Event events[], int size);
        void printEventTimes(Event events[], int size);
        void getSortedIndices(const Event events[], uint8_t numEvents, uint8_t indices[]);
};




#endif //SHEDULESMANAGER