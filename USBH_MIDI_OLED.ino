//#include <MIDI.h>
//#include <Usb.h>
//#include <usbhub.h>
#include <usbh_midi.h>
#include <MicroLCD.h>
#include <Arduino.h>
#include <Wire.h>

#define cursor_status 64
#define cursor_number 60
#define init_line_number 5
#define end_line_number 9
#define debounce 150
#define numButtons 5
#define ZOOM_PID 0x15F
#define ID_ZOOM_G5 0x5B
#define ZG3_CURRENT_PATCH_DATA_SIZE 120
#define maxBank 2

void intro();
void writeBank(byte bank);
void presets();
void banks();
void writeNumber(int num);


uint8_t _readBuffer[MIDI_MAX_SYSEX_SIZE] = { 0 };
const byte zoom_request_id[] = { 0xF0, 0x7E, 0x00, 0x06, 0x01, 0xF7 };
uint8_t _deviceID = 0x64;  //0x64 zoom g1xon
//MIDI_CREATE_DEFAULT_INSTANCE();
USB Usb;
USBH_MIDI Midi(&Usb);

LCD_SSD1306 lcd;

const byte pedalIdx[] = { 36, 50, 64, 78, 92 };
//const byte pushButtons[] = { 5, 17, 16, 15, 14 };  //invertido
const byte pushButtons[] = { 3, 17, 16, 15, 14 };
const int pinBank = 2;
//const int pinTuner = 11;
//const byte outPin5 = 13;
byte buttonPushCounter[] = { 0, 0, 0, 0, 0 };               // contador para o número de vezes que o botão foi pressionado
static bool prevButtonState[numButtons] = {};               // estado anterior do botao
static bool currentButtonState[numButtons] = {};            // estado atual do botao
static unsigned long prevTimeButtonState[numButtons] = {};  // tempo do estado anterior do botao

bool buttonBankState; //estado atual do botão que muda de banco
bool prevBankState; //estado anterior do botão que muda de banco
unsigned long prevTimeBankState;

int bankNumber = 0;

const byte fav = 10;  // valor do preset favorito, no caso da g1On, seria o B0

byte status_usb;
void setup()

{

  Serial.begin(115200);

  status_usb = USB_STATE_DETACHED;
  pinMode(pinBank, INPUT_PULLUP);

  for (int d = 0; d < numButtons; d++) {
    pinMode(pushButtons[d], INPUT_PULLUP);
    delay(10);
  }

  lcd.begin();
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.print("Status:");

  //Usb.Init();
  //pinMode(pinTuner, INPUT_PULLUP);
  byte retry = 100;
  while (Usb.Init(2000) == -1) {

    writeNumber(retry);
    retry++;
  };

  Midi.attachOnInit(intro);

  bankNumber = 0;
  writeBank(bankNumber);
}

/**********************************************************************/
void loop()  //ok
{
  Usb.Task();
  if ((Usb.getUsbTaskState() != status_usb)) {
    status_usb = Usb.getUsbTaskState();
    writeStatus(status_usb);
  }
  presets();
  banks();
}

void banks() {
  buttonBankState = digitalRead(pinBank);
  if ((millis() - prevTimeBankState) > debounce) {
    if (buttonBankState != prevBankState) {
      prevTimeBankState = millis();
      if (buttonBankState == LOW) {
        bankNumber++;
        resetCounter();
        if (bankNumber > maxBank) {
          bankNumber = 0;
        }
        writeBank(bankNumber);
      }
      prevBankState = buttonBankState;
    }
  }
}

void presets() {

  for (byte i = 0; i < numButtons; i++) {

    currentButtonState[i] = digitalRead(pushButtons[i]);
    if ((millis() - prevTimeButtonState[i]) > debounce) {
      if (currentButtonState[i] != prevButtonState[i]) {
        prevTimeButtonState[i] = millis();
        if (currentButtonState[i] == LOW) {


          if (buttonPushCounter[i] == 0) {

            //SendMIDI(midiAdress[i]);
            SendMIDI(i + (numButtons * bankNumber));
            resetCounter();
            buttonPushCounter[i]++;
            writeNumber(i + (numButtons * bankNumber));
          } else {

            SendMIDI(fav);
            resetCounter();
            writeNumber(fav);
          }
          writeBank(bankNumber);
        }

        prevButtonState[i] = currentButtonState[i];
      }  // delay(debounce);  // anti rebond
    }
  }
}

void sendBytes(uint8_t *aBytes, const __FlashStringHelper *aMessage = NULL) {
  Usb.Task();
  Midi.SendData(aBytes);
}

void readResponse() {
  uint16_t recv_read = 0;
  uint16_t recv_count = 0;
  uint8_t rcode = 0;
  delay(400);
  Usb.Task();
  do {
    rcode = Midi.RecvData(&recv_read, (uint8_t *)(_readBuffer + recv_count));
    if (rcode == 0) {
      recv_count += recv_read;
    }
  } while (/*recv_count < MIDI_MAX_SYSEX_SIZE &&*/ rcode == 0);

  Serial.print("\nRecv count:");
  Serial.println(recv_count);
  Serial.println("Data Byte (antes tratamento):");
  for (int i = 0; i < sizeof(_readBuffer); i++) {
    Serial.print(_readBuffer[i], HEX);
    Serial.print(" ");
  }

  for (int i = 0, j = 0; i < MIDI_MAX_SYSEX_SIZE; i++) {
    _readBuffer[j++] = _readBuffer[++i];
    _readBuffer[j++] = _readBuffer[++i];
    _readBuffer[j++] = _readBuffer[++i];
  }
  Serial.println("\nData Byte:");
  for (int i = 0; i < sizeof(_readBuffer); i++) {
    Serial.print(_readBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println("\nEnd");
}

//Forçar a comunicão via usb
//Aguardar pedal inicializar
// Tempo de iniciar baseado na Zoom G1_On
void intro() {

  Serial.println("\n id do produto");
  Serial.println(Midi.idProduct(), HEX);

  //if the device is a zoom multieffect, catch the ID
  if (Midi.idProduct() == 0x15F) {
    //Product ID valeton GP200LT
    sendBytes(zoom_request_id, F("REQ PATCH DATA"));
    readResponse();
    Serial.print("\nDevice ID: ");
    Serial.println(_readBuffer[6], HEX);
    _deviceID = _readBuffer[6];

    delay(500);
    SendMIDIintro(fav);
    writeNumber(fav);
    writeBank(bankNumber);
  }
}


/**********************************************************************/
void resetCounter() {
  for (byte j = 0; j < 5; j++) {
    buttonPushCounter[j] = 0;
  }
}
/**********************************************************************/
void SendMIDI(byte number) {


  //Se o controlador reconhece o dispositivo perfeitamente, então o comando midi é enviado
  if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {
    if (status_usb != USB_STATE_RUNNING) {
      status_usb = USB_STATE_RUNNING;
      writeStatus(status_usb);
    }

    SendMIDIintro(number);  //
  }

  //este estado do host acontece quando o arduino/host não reconheceu a pedaleira, serve como indicador
  //this host state happans when the arduino/host doesn't reconized the zoom g1xon, is an indicator
  else {  // if( Usb.getUsbTaskState() == USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE ){
    resetCounter();
    if (status_usb == USB_STATE_RUNNING) {
      status_usb = Usb.getUsbTaskState();
      writeStatus(status_usb);
    }
  }
}


void SendMIDIintro(byte number) {
  byte Message[2];
  byte message1[3];


  if (_deviceID == ID_ZOOM_G5) {
    //byte message1[3];
    message1[0] = 0xB0;
    message1[1] = 0x00;
    message1[2] = 0x00;
    Midi.SendData(message1);
    delay(10);
    message1[1] = 0x20;
    message1[2] = number / 3;  // LSB
    Midi.SendData(message1);
    delay(10);
    //byte Message[2];         // Construct the midi message (2 bytes)
    Message[0] = 0xC0;        // 0xC0 is for Program Change
    Message[1] = number % 3;  // Number is the program/patch
    Midi.SendData(Message);   // Send the message

  } else {
    //byte Message[2];         // Construct the midi message (2 bytes)
    Message[0] = 0xC0;       // 0xC0 is for Program Change
    Message[1] = number;     // Number is the program/patch
    Midi.SendData(Message);  // Send the message
    //MIDI.sendControlChange(12,number, 0);
  }
}



void writeNumber(int num) {
  limparLinhaNumero();
  lcd.setFontSize(FONT_SIZE_XLARGE);
  lcd.setCursor(0, init_line_number);
  lcd.setCursor(cursor_number, init_line_number);
  lcd.printLong(num);
  //delay(50);
}

void limparLinhaNumero() {
  for (int i = init_line_number; i < end_line_number; i++) {
    lcd.clearLine(i);
  }
}

void writeBank(byte bank) {
  lcd.setFontSize(FONT_SIZE_LARGE);
  lcd.setCursor(0, init_line_number);

  lcd.printInt(bank);
}

void writePedal(int numberPedal, bool status) {
  //limparLinhaNumero();
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(pedalIdx[numberPedal - 1], init_line_number);
  lcd.printLong(numberPedal);
}

void writeStatus(byte status) {
  for (int i = 0; i < 3; i++) {
    lcd.clearLine(i);
  }

  lcd.setCursor(0, 1);
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.print("Status:");

  lcd.setFontSize(FONT_SIZE_SMALL);
  if (status == USB_STATE_RUNNING) {
    lcd.setCursor(cursor_status, 2);
    lcd.print("running");
  } else {
    lcd.setCursor(cursor_status, 1);
    switch (status) {
      case USB_STATE_DETACHED:
        lcd.print("desligado");
        break;
      case USB_DETACHED_SUBSTATE_ILLEGAL:
        lcd.print("ILEGAL");
        break;
      case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE:
        lcd.print("Wait USB");
        Usb.busprobe();
        delay(10);
        break;
      case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
        lcd.print("reset");
        break;
      case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE:
        lcd.print("reset OK");
        break;
      case USB_ATTACHED_SUBSTATE_WAIT_RESET:
        lcd.print("wait rst");
        break;
      case USB_STATE_CONFIGURING:
        lcd.print("CONFIG..");
        break;
      case USB_DETACHED_SUBSTATE_INITIALIZE:
        lcd.print("Initialing..");
        break;
      case USB_STATE_ERROR:
        lcd.print("ERRO");
        Usb.Init();
        break;
      default:
        lcd.print("unknow");
        break;
    }
  }
}
