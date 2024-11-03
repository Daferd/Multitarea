#include <ChannelCtrl.h> 

ChannelCtrl::ChannelCtrl(){}

ChannelCtrl::~ChannelCtrl(){}

void ChannelCtrl::chInit(uint8_t chDef, uint8_t pinDef){
    chPinHW[chDef-1] = pinDef;
    chState[chDef-1] = false;

    pinMode(chPinHW[chDef-1],OUTPUT);
    digitalWrite(chPinHW[chDef-1], chState[chDef-1]);
}

void ChannelCtrl::chEnableHw(ChannelServer ch){
    chState[ch.num-1] = ch.state;
    digitalWrite(chPinHW[ch.num-1], chState[ch.num-1]);
}

void ChannelCtrl::chEnableInvHw(ChannelServer ch){
    chState[ch.num-1] = ch.state;
    digitalWrite(chPinHW[ch.num-1], !chState[ch.num-1]);
}

void ChannelCtrl::btnInit(uint8_t btnDef, uint8_t pinDef){
    btnPinHW[btnDef-1] = pinDef;
    pinMode(btnPinHW[btnDef-1],INPUT_PULLDOWN);
    //attachInterrupt(btnPinHW[btnDef-1],handleButtonPress,FALLING);
}

void ChannelCtrl::btnEnableHw(uint8_t btnSel){
    
    chState[btnSel-1] = !chState[btnSel-1];
    digitalWrite(chPinHW[btnSel-1], chState[btnSel-1]);
    
}
