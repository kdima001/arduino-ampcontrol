#include <Streaming.h>
#include <avr/wdt.h>
#include "ACS712.h"
#include <IRremote.hpp>

// Контроллер включения отключения аудиосборки по наличию/отсутсвия аудиосигнала
// слушает аналог на A1 и A2
// при обнаружении сигнала
// 	1. запускается цикл включения
//		включает канал 1 (фильтры/преды/ЦАП)
//		взводит таймер1 до включения канала 2 (усилители)
//		после обнуления таймера1 если сигнал есть переходит к включает канал2
//		взводит таймер2 (защита от выключения сразу)
//	при пропадании звука
//	запускаем цикл выключения (если таймер2 истёк)
//		выключаем канал 2
//		взводим таймер1
//		после обнуления таймера1 выключаем канал2
//		взводим таймер2 (защитный)
//
//Состояния:
//				нет звука								(светодио не светится)
//				есть звук, цикл включения				(светодиод медленно мигает)
//				есть звук, включено						(светится непрерывно)
//				нет звука, цикл выключения				(светодиод быстро мигает)


//индикаторы (исп. одноцветный диод)
// LED_BUILTIN

#define PRE 9  // нагрузка 1 - преды
#define AMP 8  // нагрузка 2 - усилители

//константы интервалов в мсек
#define POOL_INT 100

// порог тишины
#define MIN_LEVEL 175
#define HISTERESIL_LEVEL 10
#define ZERO_LEVEL 174
// порог тока
#define MIN_CURRENT 500
#define HISTERESIL_CURRENT 100
#define ZERO_CURRENT_VALUE 300

// Переменные пуллинга
unsigned long
  curMillis,
  prevMillisLED = 0,
  prevMillis1 = 0,
  startLearnMillis = 0,
  waitLearnMillis;

//---------------------------------------------------------------------------
uint8_t SerialDBG, TimeON_RO;
uint16_t level, currentMillis;

ACS712 acSensor(A1, 5.0, 1023, 185);

//---------------------------------------------------------------------------
enum mSTATE_t {  //режим работы
  WAIT,          //слушаем и ждем (выключены все каналы)
  ON_01,         //включен 1 выход
  ON_11,         //включен 1+2 выход
  OFF_10,        //цикл выключения 2 выход отключен
  OFF_00         //цикл выключения 2+1 выходы отключены
};
mSTATE_t mSTATE;

void PrintStatus(void) {
  Serial << F(" AMP_STATUS = ");
  switch (mSTATE) {
    case WAIT: Serial << F("WAIT") << endl; break;
    case ON_01: Serial << F("ON_01") << endl; break;
    case ON_11: Serial << F("ON_11") << endl; break;
    case OFF_10: Serial << F("OFF_10") << endl; break;
    case OFF_00: Serial << F("OFF_00") << endl; break;
  };
}

//***************************************************************************

#define IR_PIN 2
#define IR_LEARN 3
#define T_WAIT_LEARN 60000
#define T_WAIT_LEARN_BLINK 1000

//---------------------------------------------------------------------------
enum LEARN_IR_t {      //режим запоминания кнопок
  DISABLED,            //не включен
  LEARN_VOL_UP,        //запоминаем +
  LEARN_VOL_UP_END,    //переходим к -
  LEARN_VOL_DOWN,      //запоминаем -
  LEARN_VOL_DOWN_END,  //запоминаем + и -
};
LEARN_IR_t LEARN_IR;

void PrintStatusLearn(void) {
  Serial << F(" LEARN = ");
  switch (LEARN_IR) {
    case DISABLED: Serial << F("DISABLED") << endl; break;
    case LEARN_VOL_UP: Serial << F("LEARN_VOL_UP") << endl; break;
    case LEARN_VOL_DOWN: Serial << F("LEARN_VOL_DOWN") << endl; break;
  };
}

uint16_t learnVol[20];  // +addr1, +command1, +addr2, +command2, +addr3, +command3, +addr4, +command4, +addr5, +command5,
                        // -addr1, -command1, -addr2, -command2, -addr3, -command3, -addr4, -command4, -addr5, -command5,

uint8_t currentLearnItem;
IRData learnIrData;

void ProcessStateLearnIr(void) {
  switch (LEARN_IR) {
    //********************
    case DISABLED:
      if (digitalRead(IR_LEARN) == HIGH) {
        LEARN_IR = LEARN_VOL_UP;
        startLearnMillis = curMillis;
      }
      break;

    //********************
    case LEARN_VOL_UP:
      // Если прошло отведённое время
      if (curMillis - startLearnMillis > T_WAIT_LEARN) {
        LEARN_IR = DISABLED;
      }

      if (learnIrData.protocol != UNKNOWN) {
        // есть распознанная кнопка
        learnVol[currentLearnItem++] = learnIrData.address;
        learnVol[currentLearnItem++] = learnIrData.command;
        learnIrData.protocol = UNKNOWN;
      }

      if (currentLearnItem >= 10) {
        // насобирали достаточно для обучения (5 команд * 2)
        LEARN_IR = LEARN_VOL_UP_END;
        waitLearnMillis = curMillis;
      }
      break;

    //********************
    case LEARN_VOL_UP_END:
      if (curMillis - waitLearnMillis > T_WAIT_LEARN_BLINK) {
        LEARN_IR = LEARN_VOL_DOWN;
      }
      break;

    //********************
    case LEARN_VOL_DOWN:
      // Если прошло отведённое время
      if (curMillis - startLearnMillis > T_WAIT_LEARN) {
        LEARN_IR = DISABLED;
      }

      if (learnIrData.protocol != UNKNOWN) {
        // есть распознанная кнопка
        learnVol[currentLearnItem++] = learnIrData.address;
        learnVol[currentLearnItem++] = learnIrData.command;
        learnIrData.protocol = UNKNOWN;
      }

      if (currentLearnItem >= 20) {
        // насобирали достаточно для обучения (5 команд * 2 * 2)
        LEARN_IR = LEARN_VOL_DOWN_END;
        // проверить совпадает ли 4 из 5 команд на + и на -
        bool match = true;
        for (uint8_t i = 1; i < 5; i += 2) {
          if (learnVol[0] != learnVol[i]
              || learnVol[1] != learnVol[i + 1]
              || learnVol[10] != learnVol[i + 10]
              || learnVol[11] != learnVol[i + 11]) {
            match = false;
            break;
          }
        }
        if (match) {
          // коды совпали - радость то какая !!!
          Serial << F("IR learning successful") << endl;
          waitLearnMillis = curMillis;
          LEARN_IR = LEARN_VOL_DOWN_END;
        } else {
          // не совпали коды - пичаль
          Serial << F("IR learning error") << endl;
          LEARN_IR = DISABLED;
        }
      }
      break;

    //********************
    case LEARN_VOL_DOWN_END:
      if (curMillis - waitLearnMillis > T_WAIT_LEARN_BLINK) {
        // EEPROM.WRITE
        LEARN_IR = DISABLED;
      }
      break;      
  }
}

//---------------------------------------------------------------------------
void (*Reset)(void) = 0;  // Reset CON function

//---------------------------------------------------------------------------
// длинная буфера усреднения
#define BUF_LENGTH 256
// буфера усреднения
int valuesAudioBuffer[BUF_LENGTH];
int valuesCurrentBuffer[BUF_LENGTH];
int bufferPosition = 0;
void ProcessSensors(void) {
  if (bufferPosition >= BUF_LENGTH) {
    bufferPosition = 0;
  }
  analogRead(A0);
  analogRead(A0);
  analogRead(A0);
  valuesAudioBuffer[bufferPosition] = analogRead(A0);
  valuesAudioBuffer[bufferPosition] = valuesAudioBuffer[bufferPosition] < ZERO_LEVEL ? 0 : valuesAudioBuffer[bufferPosition] - ZERO_LEVEL;

  analogRead(A1);
  analogRead(A1);
  analogRead(A1);
  valuesCurrentBuffer[bufferPosition] = acSensor.mA_peak2peak(50, 1);
  valuesCurrentBuffer[bufferPosition] = valuesCurrentBuffer[bufferPosition] < ZERO_CURRENT_VALUE ? 0 : valuesCurrentBuffer[bufferPosition] - ZERO_CURRENT_VALUE;
  bufferPosition++;

  uint32_t valAudio = 0;
  uint32_t valCurrent = 0;
  for (int i = 0; i < BUF_LENGTH; i++) {
    valAudio += valuesAudioBuffer[i];
    valCurrent += valuesCurrentBuffer[i];
  }
  level = valAudio / BUF_LENGTH;
  currentMillis = valCurrent / BUF_LENGTH;
}

//---------------------------------------------------------------------------
struct LED_MODE_t {
  bool blink;
  uint16_t t1, t2;  // свечения, пауза
  uint8_t r, g, b;
};

enum LM_t {
  LM_WAIT,
  LM_ON_01,
  LM_ON_11,
  LM_OFF_10,
  LM_OFF_00
};
LM_t LED_MODE;

LED_MODE_t lm[5] = {
  { 0, 0, 0, 0, 0, 0 },       //LM_WAIT,    не горит
  { 1, 50, 500, 255, 0, 0 },  //LM_ON_01, 	редко мигает 0,5сек
  { 0, 0, 0, 255, 0, 0 },     //LM_ON_11, 	горит
  { 1, 50, 250, 255, 0, 0 },  //LM_OFF_10,  часто мигает
  { 1, 50, 50, 255, 0, 0 }    //LM_OFF_11,  очень часто мигает
};

//---------------------------------------------------------------------------
void setRGB(uint8_t p_Y, uint8_t p_G, uint8_t p_B) {
  //инвертнем ибо общий анод и 0 зажигает, а 255 гасит диод
  digitalWrite(LED_BUILTIN, p_Y);
}

//---------------------------------------------------------------------------
void ProcessLED() {
  if (lm[LED_MODE].blink) {
    //Если цикл режима светодиода кончился - ставим цикл в начало
    if (curMillis - prevMillisLED > lm[LED_MODE].t1 + lm[LED_MODE].t2)
      prevMillisLED = curMillis;

    if ((prevMillisLED < curMillis) && (curMillis <= (prevMillisLED + lm[LED_MODE].t1))) {  //фаза удержания яркости
      setRGB(lm[LED_MODE].r, lm[LED_MODE].g, lm[LED_MODE].b);
    } else {
      if (prevMillisLED + lm[LED_MODE].t1 < curMillis && curMillis <= prevMillisLED + lm[LED_MODE].t1 + lm[LED_MODE].t2) {  //фаза выключения
        setRGB(0, 0, 0);
      }
    }
  } else
    setRGB(lm[LED_MODE].r, lm[LED_MODE].g, lm[LED_MODE].b);
}

//---------------------------------------------------------------------------
void PrintSensors() {
  Serial << F("state:") << mSTATE << F(", ");
  Serial << F("led:") << LED_MODE << F(", ");
  Serial << F("current:") << currentMillis << F(", ");
  Serial << F("level:") << level;
  Serial << endl;
}
//---------------------------------------------------------------------------
bool isSensorLevelUp(void) {
  bool sound = level > MIN_LEVEL + HISTERESIL_LEVEL;
  bool current = currentMillis > MIN_CURRENT + HISTERESIL_CURRENT;
  return sound || current;
}

bool isSensorLevelDown(void) {
  bool sound = level < MIN_LEVEL - HISTERESIL_LEVEL;
  bool current = currentMillis < MIN_CURRENT - HISTERESIL_CURRENT;
  return sound && current;
}

uint8_t checkCount = 0;
#define WAIT_CHECK_COUNT 50
#define ON_11_CHECK_COUNT 20
#define OFF_CHECK_COUNT 200

void ProcessStateAmp(void) {
  switch (mSTATE) {
    //********************
    case WAIT:
      if (isSensorLevelUp()) {
        checkCount++;
      } else {
        checkCount = 0;
      }
      if (checkCount >= WAIT_CHECK_COUNT) {
        checkCount = 0;
        digitalWrite(PRE, HIGH);
        mSTATE = ON_01;
      }
      break;
    //********************
    case ON_01:
      checkCount++;
      if (checkCount >= ON_11_CHECK_COUNT) {
        checkCount = 0;
        digitalWrite(AMP, HIGH);
        mSTATE = ON_11;
      }
      break;
    //********************
    case ON_11:
      if (isSensorLevelDown()) {
        checkCount++;
      } else {
        checkCount = 0;
      }
      if (checkCount >= WAIT_CHECK_COUNT) {
        digitalWrite(AMP, LOW);
        mSTATE = OFF_10;
        checkCount = 0;
      }
      break;
    //********************
    case OFF_10:
      checkCount++;
      if (checkCount >= OFF_CHECK_COUNT) {
        checkCount = 0;
        digitalWrite(PRE, LOW);
        mSTATE = OFF_00;
      }
      break;
    //********************
    case OFF_00:
      checkCount++;
      if (checkCount >= WAIT_CHECK_COUNT) {
        checkCount = 0;
        mSTATE = WAIT;
      }
      break;
  }
}

//---------------------------------------------------------------------------
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    // Пришлось придумать символ-заменитель переводу строки - в кач-ве параметра в cmd \n передать не удалось
    if (inChar == '\n' || inChar == '@' || inChar == ';') {
      inputString.replace('@', '\n');
      inputString.replace(';', '\n');
      stringComplete = true;
    }
  }
}

//--------------------------------------------------------------------------
void SelfTest(void) {
  Serial << endl;
  ProcessSensors();

  PrintSensors();
  Serial << F("Test outputs:") << endl;

  wdt_reset();  // не забываем сбросить смотрящую собаку

  Serial << F("PRE channel ON...");
  digitalWrite(PRE, HIGH);
  delay(1000);
  Serial << F("OFF") << endl;
  digitalWrite(PRE, LOW);
  Serial << F("AMP channel ON...");
  digitalWrite(AMP, HIGH);
  delay(1000);
  Serial << F("OFF") << endl;
  digitalWrite(AMP, LOW);

  wdt_reset();  // не забываем сбросить смотрящую собаку

  Serial << F("Self test complete") << endl;
}

//---------------------------------------------------------------------------
void setup() {
  uint8_t tmpi;

  wdt_enable(WDTO_8S);

  pinMode(PRE, OUTPUT);
  digitalWrite(PRE, LOW);
  pinMode(AMP, OUTPUT);
  digitalWrite(AMP, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IR_LEARN, INPUT_PULLUP);

  analogReference(DEFAULT);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);

  SerialDBG = 1;
  Serial.begin(115200);
  inputString.reserve(200);

  mSTATE = WAIT;

  LED_MODE = LM_WAIT;

  for (int i = 0; i < BUF_LENGTH; i++) {
    valuesAudioBuffer[i] = MIN_LEVEL;
    valuesCurrentBuffer[i] = ZERO_CURRENT_VALUE;
  }

  PrintStatus();
}

//--------------------------------------------------------------------------
void loop() {

  wdt_reset();  // не забываем сбросить смотрящую собаку

  ProcessLED();

  curMillis = millis();

  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial << F("IR decoding error") << endl;
      // Если идёт обучение запихать команду в буфер обучения
      if (LEARN_IR == LEARN_VOL_UP || LEARN_IR == LEARN_VOL_DOWN) {
        memcpy(&IrReceiver.decodedIRData, &learnIrData, sizeof(IRData));
      }
    } else {

      IrReceiver.printIRResultShort(&Serial);
    }
    IrReceiver.resume();
  }

  //проверим на переполнение
  if (prevMillis1 > curMillis || prevMillisLED > curMillis) {
    prevMillis1 = 0;
    prevMillisLED = 0;
  }

  //Еже-действия
  if (curMillis - prevMillis1 > POOL_INT) {
    prevMillis1 = curMillis;

    ProcessSensors();

    ProcessStateAmp();

    ProcessStateLearnIr();

    // print the string when a newline arrives:
    if (stringComplete) {
      inputString.toLowerCase();
      inputString.trim();
      Serial << inputString << endl;
      if (inputString.equals(F("help"))) {
        Serial << F("comands:") << endl;
        Serial << F("diag") << endl;
        Serial << F("debug=on") << endl;
        Serial << F("debug=off") << endl;
      }
      if (inputString.equals(F("status")) || inputString.equals(F("diag"))) {
        PrintSensors();
      }
      if (inputString.equals(F("debug=on")) || inputString.equals(F("debug on"))) SerialDBG = 1;
      if (inputString.equals(F("debug=off")) || inputString.equals(F("debug off"))) SerialDBG = 0;
      // clear the string:
      inputString = "";
      stringComplete = false;
    }

    if (SerialDBG) PrintSensors();
  }

  switch (mSTATE) {
    case WAIT: LED_MODE = LM_WAIT; break;
    case ON_01: LED_MODE = LM_ON_01; break;
    case ON_11: LED_MODE = LM_ON_11; break;
    case OFF_10: LED_MODE = LM_OFF_10; break;
    case OFF_00: LED_MODE = LM_OFF_00; break;
  }
}
