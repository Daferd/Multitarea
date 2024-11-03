#ifndef _CHANNELCTRL_H
#define _CHANNELCTRL_H

#include <Arduino.h>

struct ChannelServer {
    uint8_t num;
    bool state;
    bool action;
};

class ChannelCtrl{
private:
    
        
public:

    //En el indice de cada arreglo esta intrinseco el numero del canal
    uint8_t chPinHW[4];  //Se debe definir el pin de hardware especifico para la tarjeta esp32 que se este utilizando 
    bool chState[4];  //Se almacena el estado de cada pin
    uint8_t btnPinHW[4];

    ChannelCtrl();
    ~ChannelCtrl();

    void chInit(uint8_t chDef, uint8_t pinDef);
    void chEnableHw(ChannelServer ch);
    void chEnableInvHw(ChannelServer ch);

    //static void IRAM_ATTR handleButtonPress(void);  // Función estática para manejar la interrupción
    void btnInit(uint8_t btnDef, uint8_t pinDef);
    void btnEnableHw(uint8_t btnSel);
};




#endif //_CHANNELCTRL_H