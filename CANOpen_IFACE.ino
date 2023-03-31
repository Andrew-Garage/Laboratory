extern "C" {
#include "MicroMenu.h"
}
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C main_lcd(0x38, 16, 2);
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Bris.h>

#include "esp_err.h"
#include "esp_log.h"

#include "ModbusMessage.h"
#include "ModbusServerRTU.h"
#include "ModbusTypeDefs.h"
#include "CoilData.h"
#include <Logging.h>

#include "driver/gpio.h"
#include "driver/twai.h"

#include <SPI.h>
#define BME_MISO 100  // Убираем это из SPI
#define BME_CS 101    // и это тоже

/* ------------------------------------------------------------------------------------------ Useful DEFinitions ---------------------------------*/

#define SPI_CURRENT_PIN 32
#define SPI_VOLTAGE_PIN 33
#define CURRENT H_Regs[1]
#define VOLTAGE H_Regs[2]
#define ENCODER H_Regs[5]

#define B 125
#define R 115
#define I 28
#define S 57

/* ------------------------------------------------------------------------------------------ CAN-bus DEFinitions ---------------------------------*/

#define IFACE_NodeID 0x01
#define PDA_NodeID 0x02
#define HVT_NodeID 0x03

#define COB_ID_MASK 0x780 // Оставляет биты 0111 1000 0000 
#define NODE_ID_MASK 0x7F // Оставляет биты 0000 0111 1111 
#define NMT_MONITORING 0x700

/* ------------------------------------------------------------------------------------------ RUSSIAN letters ---------------------------------*/

#define Burn "\xA8\x70\x6F\xB6\xB8\xB4"     // Прожиг
#define Afterburn "\xE0\x6F\xB6\xB8\xB4"      // Дожиг
#define Acoustic "\x41\xBA\x79\x63\xBF\xB8\xBA\x61"     // Акустика
#define HV_Test "\x42\x42\xA5"      // ВВИ
#define Phase "\xAA\x61\xB7\x61"      // Фаза

#define Stage "\x43\xBF\x79\xBE\x65\xBD\xC4"     // Ступень
#define Timer "\x54\x61\xB9\xBC\x65\x70"     // Таймер
#define Burn_countdown "\x4F\xB2\x70. \x6F\xBF\x63\xC0\xB5\xBF"  // Обр. отсчёт
#define Mode "\x50\x65\xB6\xB8\xBC"     // Режим
#define Method "\x4D\x65\xBF\x6F\xE3"   // Метод
#define Current_Type "\x54\xB8\xBE \xBF\x6F\xBA\x61"  // Тип тока

#define Start "\xA4\x61\xBE\x79\x63\xBA"     // Запуск

#define Set_Phase "\x42\xC3\xB2\x6F\x70 \xE4\x61\xB7\xC3:"  // Выбор фазы
#define Set_Stage "\x42\xC3\xB2\x6F\x70 \x63\xBF\x79\xBE\x65\xBD\xB8:"  // Выбор ступени
#define Set_Timer "\xA9\x63\xBF. \xBF\x61\xB9\xBC\x65\x70\x61"  // Уст. таймера
#define Set_Timer_Mode "\x50\x65\xB6\xB8\xBC \x6F\xBF\x63\xC0\xB5\xBF\x61" // Режим отсчёта
#define Set_Mode "\x42\xC3\xB2\x6F\x70 \x70\x65\xB6\xB8\xBC\x61" // Выбор режима
#define Set_Current_Type "\x42\xC3\xB2\x6F\x70 \xBF\x6F\xBA\x61" // Выбор тока

/* ------------------------------------------------------------------------------------------ NOTIFICATIONS ---------------------------------*/

#define IN_BIT 0x01
#define OUT_BIT 0x02
#define UP_BIT 0x04
#define DOWN_BIT 0x08
#define ENTER_BIT 0x10

/* ------------------------------------------------------------------------------------------ GLOBAL variables ---------------------------------*/

typedef enum {
  INITIALIZING = 0x00,  // После первоначальной загрузки передается boot_up сообщение, что все хорошо.
  STOPPED = 0x04,  // Прием/передача не происходит. За исключением node_quarding(устар.) и heartbeat.
  OPERATIONAL = 0x05,  // Переходит в это состояние при приеме start_remote_node.
  PRE_OPERATIONAL = 0x7F,  // Обмен PDO невозможен. Можно изменять словарь через SDO, принимать/отправлять emeregency, SYNC, time-stamp и NMT.
} module_status_t; 

byte led_display[3] = { 0, 0, 0 };

ModbusServerRTU MBserver(Serial2, 2000);

TaskHandle_t menu, button_Handler, encoder_Handler, seven_seg_display, CAN_tx, CAN_rx, heartbeat_task;
static QueueHandle_t CAN_rx_queue;

uint32_t ulNotifiedValue;
bool changing_Value = 0;

/* ------------------------------------------------------------------------------------------ CAN MESSAGES ---------------------------------*/

twai_message_t NMT_Message;

/* ------------------------------------------------------------------------------------------ DICTIONARY ---------------------------------*/

//Адрес в Tesla |       0|       1|       2|       3|       4|       5|       6|       7|       8|       9|

CoilData H_Coils("      1        0        0        0        0        1        1        0        0        0 \
/* 0_Прожиг  0* | Br_auto|      --|      --|      --|      --|      --|      --|      --|      --|      --|*/ \
                        0        0        0        0        0        0        0        0        0        0 \
/* 1_Дожиг   1* |      --|      --|      --|      --|      --|      --|      --|      --|      --|      --|*/ \
                        0        0        0        0        0        0        0        0        0        0 \
/* 2_Ак-ка   2* | Ac_auto| Ac_Mode|      --|      --|      --|      --|      --|      --|      --|      --|*/ \
                        0        0        0        0        0        0        0        0        0        0 \
/* 3_ВВИ     3* |      AC|HVT_auto|      --|      --|      --|      --|      --|      --|      --|   START|*/ \
                        0        0        0        0        0        0        0        0        0        0 \
/* 4_Сервис  4* |      --|      --|      --|      --|      --|      --|      --|      --|      --|      --|*/ \
                        0        0        0        0        0        0        0        0        0        0 \
/* 5_Конфиг  5* |      --|      --|      --|      --|      --|      --|      --|      --|      --|      --|*/ ");

/**/

//Адрес в Tesla  |       0|       2|       4|       6|       8|      10|      12|      14|      16|      18|    СМЕЩЕНИЕ +2 байта для FLOAT!

float H_Regs[] = {       2,       5,       0,       0,       0,       0,       0,       0,       0,       0, \
/* 0_Прожиг      |Br_Stage| minutes|      --|      --|      --|      --|      --|      --|      --|      --|*/
                         5,       0,       0,       0,       0,       0,       0,       0,       0,       0, \
/* 1_Дожиг    2* | minutes| seconds|      --|      --|      --|      --|      --|      --|      --|      --|*/
                         1,       5,       0,       0,       0,       0,       0,       0,       0,       0, \
/* 2_Ак-ка    4* |Ac_Stage| minutes|      --|      --|      --|      --|      --|      --|      --|      --|*/
                         5,       0,       0,       0,       0,       0,       0,       0,       0,       0, \
/* 3_ВВИ      6* | minutes|      --|      --|      --|      --|      --|      --|      --|      --|      --|*/
                      0x04,    0x04,    0x04,    0x04,    0x04,    0x04,    0x04,       0,       0,       0, \
/* 4_Сервис   8* | PDA_mdl| Burn_st|  ABr_st|   Ac_st| HVT_mdl|  HVT_st|IFACEmdl|      --|      --|      --|*/};


/* ------------------------------------------------------------------------------------------ OD DEFinitions ---------------------------------*/

#define PDA_module        H_Regs[40]
#define Burn_state        H_Regs[41]
#define Afterburn_state   H_Regs[42]
#define Acoustic_state    H_Regs[43]
#define HVT_module        H_Regs[44]
#define HVT_state         H_Regs[45]
#define IFACE_module      H_Regs[46]

typedef struct {
  CoilData *ACDC;
  CoilData *HVT_auto;
  float *minutes;
} Start_parameters_t;

Start_parameters_t HVT_Start_parameters = {.ACDC = &H_Coils + 30, .HVT_auto = &H_Coils + 31, .minutes = &H_Regs[30]}; // Работает

typedef struct {
  uint8_t Node_address;  
  float *Node_state;
  String Node_name;
} Node_item;

Node_item PDA = {.Node_address = PDA_NodeID, .Node_state = &PDA_module, .Node_name = "PDA_module"};
Node_item HVT = {.Node_address = HVT_NodeID, .Node_state = &HVT_module, .Node_name = "HVT_module"};
Node_item IFACE = {.Node_address = IFACE_NodeID, .Node_state = &IFACE_module, .Node_name = "IFACE_module"};

Node_item basic_config[] = {PDA, HVT,}; // Основные модули, должны инициализироваться при включении лаборатории
Node_item full_config[] = {PDA, HVT,};  // Полная конфигурация, должны опрашиваться все время после успешной инициализации

/* ------------------------------------------------------------------------------------------ MODBUS functions ---------------------------------*/

ModbusMessage FC01_ReadCoil(ModbusMessage request) {
  //Serial.println("Read coil!");     // Считывает все койлы за раз (даже NULL) если выставить "максимум регистров"
  ModbusMessage response;  // Считывает 3 и 2 тэга, если "последовательные регистры"
  uint16_t start = 0;      // Считывает по одному, если "регистры 1 тэга"
  uint16_t numCoils = 0;
  request.get(2, start, numCoils);

  if (start + numCoils <= H_Coils.coils()) {
    vector<uint8_t> coilset = H_Coils.slice(start, numCoils);
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)coilset.size(), coilset);
  } else {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}

ModbusMessage FC05_WriteCoil(ModbusMessage request) {
  //Serial.println("Write coil!");
  ModbusMessage response;
  uint16_t start = 0;
  uint16_t state = 0;
  request.get(2, start, state);

  if (start <= H_Coils.coils()) {
    if (state == 0x0000 || state == 0xFF00) {  // FF00 = 65280 это "1"
      if (H_Coils.set(start, state)) {
        response = ECHO_RESPONSE;
        if(start == 39 && state == 0xFF00) HVT_Start();
      } else {
        response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);
      }
    } else {
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    }
  } else {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }

  return response;
}

ModbusMessage FC03_ReadReg(ModbusMessage request) {
  //Serial.println("Read reg!");
  ModbusMessage response;
  uint16_t address;  // requested register address
  uint16_t words;    // requested number of registers

  request.get(2, address);
  request.get(4, words);
  address = address / 2;  // приводим к адресации H_Reg как массива c++
  words = words / 2;      // приводим к количеству ячеек массива H_Reg (будет считываться на 1 меньше из-за условия в цикле for)

  if (address + words <= sizeof(H_Regs) / sizeof(H_Regs[0])) {
    // Looks okay. Set up message with serverID, FC and length of data
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(4 * words));  // добавлять можно только побайтово, поэтому добавляем сколько uint8_t
    // Fill response with requested data
    for (uint16_t i = address; i < address + words; i++) {
      response.add(H_Regs[i]);
    }
  } else {
    //No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}

ModbusMessage FC16_WriteReg(ModbusMessage request) {
  //Serial.println("Write reg!");
  ModbusMessage response;
  float received_num = 0;
  uint16_t data[2];
  uint16_t address;

  request.get(2, address);

  if (address / 2 <= sizeof(H_Regs) / sizeof(H_Regs[0])) {
    request.get(7, data[1]);  // Старшие байты
    request.get(9, data[0]);  // Младшие байты
    memcpy(&received_num, data, 4);
    H_Regs[address / 2] = received_num;
    response = ECHO_RESPONSE;
  } else {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}

/* ------------------------------------------------------------------------------------------ LCD DISPLAY functions ---------------------------------*/

void next_Item(){
  if(*(&Menu_GetCurrentMenu()->Next->Text)){
    main_lcd.setCursor(0, 1);
    main_lcd.print(*(&Menu_GetCurrentMenu()->Next->Text));
  }
}

void Br_Params() {
  main_lcd.setCursor(0, 1);
  if (H_Regs[0] == 1) main_lcd.print("2\xBA\x42");   // 2kB
  if (H_Regs[0] == 2) main_lcd.print("7\xBA\x42");   // 7kB
  if (H_Regs[0] == 3) main_lcd.print("25\xBA\x42");  // 20kB

  main_lcd.setCursor(5, 1);
  if (H_Regs[1] < 10) {
    main_lcd.print("0");
    main_lcd.print((uint16_t)H_Regs[1]);
  } else main_lcd.print((uint16_t)H_Regs[1]);
  main_lcd.print(":00");

  main_lcd.setCursor(11, 1);
  main_lcd.print("\x54:");
  if (H_Coils[0]) main_lcd.print("ON");
  else main_lcd.print("OFF");
}

void Br_stageChg() {
  changing_Value = 1;

  if ((ulNotifiedValue & UP_BIT) != 0) {
    if (H_Regs[0] + 1 <= 3) H_Regs[0]++;
  }

  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    if (H_Regs[0] - 1 >= 1) H_Regs[0]--;
  }

  main_lcd.clear();

  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Stage);

  main_lcd.setCursor(0, 1);
  if (H_Regs[0] == 1) main_lcd.print("[2] 7  25  (\xBA\x42)");
  if (H_Regs[0] == 2) main_lcd.print(" 2 [7] 25  (\xBA\x42)");
  if (H_Regs[0] == 3) main_lcd.print(" 2  7 [25] (\xBA\x42)");
}

void Br_timerChg() {
  changing_Value = 1;
  if ((ulNotifiedValue & UP_BIT) != 0) {
    if (H_Regs[1] + 1 <= 60) H_Regs[1]++;
  }
  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    if (H_Regs[1] - 1 >= 1) H_Regs[1]--;
  }
  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Timer);
  main_lcd.setCursor(5, 1);
  main_lcd.print((uint16_t)H_Regs[1]);
  main_lcd.print(" \xBC\xB8\xBD."); // мин.
}

void Br_modeChg() {
  changing_Value = 1;

  if ((ulNotifiedValue & UP_BIT) != 0) {
    H_Coils.set(0, !H_Coils[0]);
  }

  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    H_Coils.set(0, !H_Coils[0]);
  }

  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Timer_Mode);
  
  main_lcd.setCursor(5, 1);
  if(H_Coils[0]) main_lcd.print("ON");
  else main_lcd.print("OFF");
}

void Abr_Params() {
  main_lcd.setCursor(0, 1);
  main_lcd.print("\x50\x61\xB2\x6F\xBF\x61 "); // "Работа"

  if (H_Regs[10] < 10) {
    main_lcd.print("0");
    main_lcd.print((uint16_t)H_Regs[10]);
  } else main_lcd.print((uint16_t)H_Regs[10]);

  main_lcd.print(":");

  if (H_Regs[11] < 10) {
    main_lcd.print("0");
    main_lcd.print((uint16_t)H_Regs[11]);
  } else main_lcd.print((uint16_t)H_Regs[11]);
}

void Ac_Params() {
  main_lcd.setCursor(0, 1);
  if(H_Coils[21]) main_lcd.print("\x41\xBA\x79"); // Аку
  else main_lcd.print("\xA5\xE0\x4D");  // ИДМ
  
  main_lcd.setCursor(4, 1);
  if(H_Regs[20] == 1) main_lcd.print(" 5\xBA"); //5кВ
  if(H_Regs[20] == 2) main_lcd.print("10\xBA"); //10кВ
  if(H_Regs[20] == 3) main_lcd.print("20\xBA"); //20кВ

  main_lcd.setCursor(8, 1);
  if(H_Coils[20]) main_lcd.print("\x41\xB3\xBF"); // Авто
  else main_lcd.print("\x50\x79\xC0");  // Руч

  main_lcd.setCursor(12, 1);
  main_lcd.print((uint16_t)H_Regs[21]);
  main_lcd.print("c");
}

void Ac_stageChg() {
  changing_Value = 1;
  if ((ulNotifiedValue & UP_BIT) != 0) {
    if (H_Regs[20] + 1 <= 3) H_Regs[20]++;
  }
  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    if (H_Regs[20] - 1 >= 1) H_Regs[20]--;
  }
  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Stage);
  main_lcd.setCursor(0, 1);
  if (H_Regs[20] == 1) main_lcd.print("[5] 10  20  (\xBA\x42)"); // "[5] 10  20  кВ"
  if (H_Regs[20] == 2) main_lcd.print(" 5 [10] 20  (\xBA\x42)");
  if (H_Regs[20] == 3) main_lcd.print(" 5  10 [20] (\xBA\x42)");
}

void Ac_modeChg() {
  changing_Value = 1;
  if ((ulNotifiedValue & UP_BIT) != 0) {
    H_Coils.set(20, !H_Coils[20]);
  }
  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    H_Coils.set(20, !H_Coils[20]);
  }
  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Mode); // Выбор режима (Авто/Ручной)
  main_lcd.setCursor(0, 1);
  if (H_Coils[20]) main_lcd.print("[\x41\xB3\xBF\x6F] \x50\x79\xC0\xBD\x6F\xB9");
  else main_lcd.print(" \x41\xB3\xBF\x6F [\x50\x79\xC0\xBD\x6F\xB9]");
}

void Ac_timerChg() {
  changing_Value = 1;
  if ((ulNotifiedValue & UP_BIT) != 0) {
    if (H_Regs[21] + 1 <= 999) H_Regs[21]++;
  }
  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    if (H_Regs[21] - 1 >= 1) H_Regs[21]--;
  }
  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Timer);
  main_lcd.setCursor(5, 1);
  main_lcd.print((uint16_t)H_Regs[21]);
  main_lcd.print(" \x63\x65\xBA."); // сек.
}

void Ac_mtdChg() {
  changing_Value = 1;
  if ((ulNotifiedValue & UP_BIT) != 0) {
    H_Coils.set(21, !H_Coils[21]);
  }
  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    H_Coils.set(21, !H_Coils[21]);
  }
  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Mode); // Выбор режима (Акустика/ИДМ)
  main_lcd.setCursor(0, 1);
  if (H_Coils[21]) main_lcd.print("[\x41\xBA\x79\x63\xBF\xB8\xBA\x61] \xA5\xE0\x4D");
  else main_lcd.print(" \x41\xBA\x79\x63\xBF\xB8\xBA\x61 [\xA5\xE0\x4D]");
}

void HVT_Params() {
  main_lcd.setCursor(0, 1);
  if (H_Coils[30]) main_lcd.print("AC");
  else main_lcd.print("DC");

  main_lcd.setCursor(4, 1);
  if (H_Coils[31]) main_lcd.print("\x41\xB3\xBF\x6F");
  else main_lcd.print("\x50\x79\xC0");

  main_lcd.setCursor(9, 1);
  main_lcd.print((uint16_t)H_Regs[30]);
  main_lcd.print("\xBC");
}

void HVT_currChg() {
  changing_Value = 1;
  if ((ulNotifiedValue & UP_BIT) != 0) {
    H_Coils.set(30, !H_Coils[30]);
  }
  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    H_Coils.set(30, !H_Coils[30]);
  }
  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Current_Type);
  main_lcd.setCursor(3, 1);
  if (H_Coils[30]) main_lcd.print("[AC]  DC");
  else main_lcd.print(" AC  [DC]");
}

void HVT_modeChg() {
  changing_Value = 1;
  if ((ulNotifiedValue & UP_BIT) != 0) {
    H_Coils.set(31, !H_Coils[31]);
  }
  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    H_Coils.set(31, !H_Coils[31]);
  }
  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Mode);
  main_lcd.setCursor(0, 1);
  if (H_Coils[31]) main_lcd.print("[\x41\xB3\xBF\x6F] \x50\x79\xC0\xBD\x6F\xB9");
  else main_lcd.print(" \x41\xB3\xBF\x6F [\x50\x79\xC0\xBD\x6F\xB9]");
}

void HVT_timerChg() {
  changing_Value = 1;
  if ((ulNotifiedValue & UP_BIT) != 0) {
    if (H_Regs[30] + 1 <= 999) H_Regs[30]++;
  }
  if ((ulNotifiedValue & DOWN_BIT) != 0) {
    if (H_Regs[30] - 1 >= 1) H_Regs[30]--;
  }
  main_lcd.clear();
  main_lcd.setCursor(0, 0);
  main_lcd.print(Set_Timer); // Уст. таймера
  main_lcd.setCursor(5, 1);
  main_lcd.print((uint16_t)H_Regs[30]);
  main_lcd.print(" \xBC\xB8\xBD."); // мин.
}

void HVT_Start() {
  Serial.println(H_Coils[39]);
  Serial.println("HVT started!");

  Serial.print("FROM HVT_Start_parameters! ACDC = ");
  Serial.print((bool)&HVT_Start_parameters.ACDC);
  Serial.print(" HVT_auto = ");
  Serial.print((bool)&HVT_Start_parameters.HVT_auto);
  Serial.print(" minutes = ");
  Serial.println(*HVT_Start_parameters.minutes);

  xQueueSend(CAN_rx_queue, &HVT_Start_parameters, 1000);
  H_Coils.set(39, (bool)0);
  Serial.println(H_Coils[39]);
}

/* ------------------------------------------------------------------------------------------ MENU configuration ---------------------------------*/


//       (           Name,        Next,    Previous,      Parent,       Child,     SelFunc,     EntFunc, Text)
MENU_ITEM(         Menu_1,      Menu_2,   NULL_MENU,   NULL_MENU,     Menu_11,   next_Item,        NULL, Burn);
   MENU_ITEM(     Menu_11,     Menu_12,   NULL_MENU,      Menu_1,   NULL_MENU,   Br_Params, Br_stageChg, Stage);
   MENU_ITEM(     Menu_12,     Menu_13,     Menu_11,      Menu_1,   NULL_MENU,   Br_Params, Br_timerChg, Timer);
   MENU_ITEM(     Menu_13,     Menu_14,     Menu_12,      Menu_1,   NULL_MENU,   Br_Params,  Br_modeChg, Burn_countdown);
   MENU_ITEM(     Menu_14,   NULL_MENU,     Menu_13,      Menu_1,   NULL_MENU,   Br_Params,        NULL, Start); 

MENU_ITEM(         Menu_2,      Menu_3,      Menu_1,   NULL_MENU,     Menu_21,   next_Item,        NULL, Afterburn);
    MENU_ITEM(    Menu_21,   NULL_MENU,   NULL_MENU,      Menu_2,   NULL_MENU,  Abr_Params,        NULL, Start);

MENU_ITEM(         Menu_3,      Menu_4,      Menu_2,   NULL_MENU,     Menu_31,   next_Item,        NULL, Acoustic);
   MENU_ITEM(     Menu_31,     Menu_32,   NULL_MENU,      Menu_3,   NULL_MENU,   Ac_Params, Ac_stageChg, Stage);
   MENU_ITEM(     Menu_32,     Menu_33,     Menu_31,      Menu_3,   NULL_MENU,   Ac_Params,  Ac_modeChg, Mode);
   MENU_ITEM(     Menu_33,     Menu_34,     Menu_32,      Menu_3,   NULL_MENU,   Ac_Params, Ac_timerChg, Timer);
   MENU_ITEM(     Menu_34,     Menu_35,     Menu_33,      Menu_3,   NULL_MENU,   Ac_Params,   Ac_mtdChg, Method);
   MENU_ITEM(     Menu_35,   NULL_MENU,     Menu_34,      Menu_3,   NULL_MENU,   Ac_Params,        NULL, Start);

MENU_ITEM(         Menu_4,   NULL_MENU,      Menu_3,   NULL_MENU,     Menu_41,   next_Item,         NULL, HV_Test);
   MENU_ITEM(     Menu_41,     Menu_42,   NULL_MENU,      Menu_4,   NULL_MENU,  HVT_Params,  HVT_currChg, Current_Type);
   MENU_ITEM(     Menu_42,     Menu_43,     Menu_41,      Menu_4,   NULL_MENU,  HVT_Params, HVT_timerChg, Timer);
   MENU_ITEM(     Menu_43,     Menu_44,     Menu_42,      Menu_4,   NULL_MENU,  HVT_Params,  HVT_modeChg, Mode);
   MENU_ITEM(     Menu_44,   NULL_MENU,     Menu_43,      Menu_4,   NULL_MENU,  HVT_Params,    HVT_Start, Start);


/* ------------------------------------------------------------------------------------------ USEFUL functions ---------------------------------*/


static void Generic_Write(const char *Text) {
  if (Text) {
    main_lcd.clear();
    main_lcd.setCursor(0, 0);
    main_lcd.print("> ");
    main_lcd.print(Text);
  }
}

bool _isTimer(unsigned long startPush, unsigned long period){
    unsigned long currentTime;
    currentTime = millis();
    if (currentTime>= startPush) 
    {
        return (currentTime>=(startPush + period));
    }
     else 
    {
        return (currentTime >=(4294967295-startPush+period));
    }
}

void setup() {
  Serial.begin(115200);  // For debug
  while (!Serial) {}
  Serial.println("__ Serial OK __");

  IFACE_module = INITIALIZING;

  //Initialize configuration structures using macro initializers
  //twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_2, GPIO_NUM_15, TWAI_MODE_NORMAL);
  twai_general_config_t g_config = { .mode = TWAI_MODE_NORMAL, .tx_io = GPIO_NUM_2, .rx_io = GPIO_NUM_15, \
                                     .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 5, \
                                     .rx_queue_len = 5, .alerts_enabled = TWAI_ALERT_ALL, .clkout_divider = 0, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  //Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    printf("Driver installed\n");
  } else {
    printf("Failed to install driver\n");
    return;
  }

  //Start TWAI driver
  if (twai_start() == ESP_OK) {
    printf("Driver started\n");
  } else {
    printf("Failed to start driver\n");
    return;
  }

  CAN_rx_queue = xQueueCreate(1, sizeof(Start_parameters_t));

  Serial2.begin(115200, SERIAL_8N1, GPIO_NUM_16, GPIO_NUM_17);  // поменял местами рх тх!

  MBserver.registerWorker(0x01, READ_COIL, &FC01_ReadCoil);             // Читает по нескольку
  MBserver.registerWorker(0x01, WRITE_COIL, &FC05_WriteCoil);           // Пишет по одному
  MBserver.registerWorker(0x01, READ_HOLD_REGISTER, &FC03_ReadReg);     // Читает по нескольку
  MBserver.registerWorker(0x01, WRITE_MULT_REGISTERS, &FC16_WriteReg);  // Пишет по одному

  MBserver.start(0);  // core 1 is default
  MBUlogLvl = 1;
  MBserver.skipLeading0x00();

  xTaskCreatePinnedToCore(menu_code, "menu", 10000, NULL, 1, &menu, 1);
  vTaskDelay(pdMS_TO_TICKS(30));
  xTaskCreatePinnedToCore(button_Handler_code, "button_Handler", 10000, NULL, 1, &button_Handler, 1);
  vTaskDelay(pdMS_TO_TICKS(30));
  xTaskCreatePinnedToCore(encoder_Handler_code, "encoder_Handler", 10000, NULL, 1, &encoder_Handler, 1);
  vTaskDelay(pdMS_TO_TICKS(30));
  xTaskCreatePinnedToCore(seven_seg_display_code, "seven_seg_display", 10000, NULL, 1, &seven_seg_display, 1);
  vTaskDelay(pdMS_TO_TICKS(30));
  xTaskCreatePinnedToCore(CAN_rx_code, "CAN_rx", 10000, NULL, 6, &CAN_rx, 0);
  vTaskDelay(pdMS_TO_TICKS(30));
  xTaskCreatePinnedToCore(CAN_tx_code, "CAN_tx", 10000, NULL, 7, &CAN_tx, 0);
  vTaskDelay(pdMS_TO_TICKS(30));
}

void loop() {}



void button_Handler_code(void *pvParameters) {
  int pressed_button = 0;
  pinMode(36, INPUT);
  unsigned long startPush = 0UL;
  bool send_once = 0;
  int period = 300;
  bool in = 0;
  for (;;) {
    pressed_button = analogRead(36);

    if (pressed_button < 200 && send_once == 0) {
      send_once = 1;
      Serial.println("Out");
      xTaskNotify(menu, OUT_BIT, eSetBits);
    } else if ((385 < pressed_button && pressed_button < 785) && send_once == 0 || in == 1) {
      if (in == 0) {
        startPush = millis();
        in = 1;
      }
      if ((385 < pressed_button && pressed_button < 785) && in == 1 && _isTimer(startPush, period)) {
        send_once = 1;
        in = 0;
        Serial.println("Enter button");
        xTaskNotify(menu, ENTER_BIT, eSetBits);
      }
      //if ((pressed_button > 4000) && in == 1 && _isTimer(startPush, 50)) {
      if ((pressed_button > 4000) && in == 1 && _isTimer(startPush, 60)) {
        send_once = 1;
        in = 0;
        Serial.println("In");
        xTaskNotify(menu, IN_BIT, eSetBits);
      }
    } else if ((1110 < pressed_button && pressed_button < 1510) && send_once == 0) {
      send_once = 1;
      Serial.println("Down");
      xTaskNotify(menu, DOWN_BIT, eSetBits);
    } else if ((1845 < pressed_button && pressed_button < 2245) && send_once == 0) {
      send_once = 1;
      Serial.println("Up");
      xTaskNotify(menu, UP_BIT, eSetBits);
    } else if ((2740 < pressed_button && pressed_button < 3140) && send_once == 0) {
      send_once = 1;
      Serial.println("Encoder button");
      xTaskNotify(menu, ENTER_BIT, eSetBits);
    }
    if (pressed_button > 4000 && send_once == 1) {
      send_once = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

void menu_code(void *pvParameters) {
  main_lcd.init();
  main_lcd.backlight();
  BaseType_t xResult;

  bool qwe = 0;

  Menu_SetGenericWriteCallback(Generic_Write);
  Menu_Navigate(&Menu_1);

  for (;;) {
    xResult = xTaskNotifyWait(pdFALSE,          /* Не очищать биты на входе. */
                              ULONG_MAX,        /* Очистка всех бит на выходе. */
                              &ulNotifiedValue, /* Сохраняет значение оповещения. */
                              pdMS_TO_TICKS(500));
    
    if (xResult == pdPASS && changing_Value == 0) {
      if ((ulNotifiedValue & UP_BIT) != 0) {
        //Serial.println("UP_BIT");
        Menu_Navigate(MENU_PREVIOUS);
      }
      if ((ulNotifiedValue & DOWN_BIT) != 0) {
        //Serial.println("DOWN_BIT");
        Menu_Navigate(MENU_NEXT);
      }
      if ((ulNotifiedValue & IN_BIT) != 0) {
        //Serial.println("IN_BIT");
        Menu_Navigate(MENU_CHILD);
      }
      if ((ulNotifiedValue & OUT_BIT) != 0) {
        //Serial.println("OUT_BIT");
        Menu_Navigate(MENU_PARENT);
      }
      if ((ulNotifiedValue & ENTER_BIT) != 0) {
        //Serial.println("ENTER_BIT");
        Menu_EnterCurrentItem();
        //changing_Value = 1;
      }
    }
    if (xResult == pdPASS && changing_Value == 1) {
      if ((ulNotifiedValue & UP_BIT) != 0) {
        Menu_EnterCurrentItem();
      }
      if ((ulNotifiedValue & DOWN_BIT) != 0) {
        Menu_EnterCurrentItem();
      }
      if ((ulNotifiedValue & OUT_BIT) != 0) {
        changing_Value = 0;
        Menu_Navigate(Menu_GetCurrentMenu());
      }
    }
    //Serial.println("tick");
  }
}

void encoder_Handler_code(void *pvParameters) {
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  bool flag34 = 0;
  bool flag35 = 0;
  for (;;) {
    if (!digitalRead(34)) {              // Когда прошло "прерывание" по FALLING, есть 2 варианта:
      if (digitalRead(35) && !flag34) {  // 1. Вторая линия в HIGH && это "вход" в пару сигналов, т.е. до этого прерывания не было
        flag34 = 1;                      // тогда помечаем, что прерывание было
      }

      if (!digitalRead(35) && flag35) {
        //ENCODER++;  // Значит плюсуем/минусуем
        xTaskNotify(menu, UP_BIT, eSetBits); // По часовой (плюсуем)
      }
      flag35 = 0;
    }

    if (!digitalRead(35)) {  // Все. Дальше ждем прерывание по второй линии.
      if (digitalRead(34) && !flag35) {
        flag35 = 1;
      }

      if (!digitalRead(34) && flag34) {
        //ENCODER--;
        xTaskNotify(menu, DOWN_BIT, eSetBits); // Против часовой (минусуем)
      }
      flag34 = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void seven_seg_display_code(void *pvParameters) {
  SPI.begin();
  pinMode(SPI_CURRENT_PIN, OUTPUT);
  pinMode(SPI_VOLTAGE_PIN, OUTPUT);
  int speed = 200;
  for (;;) {
    send_to_display(~B, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~R, ~B, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~I, ~R, ~B, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~S, ~I, ~R, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~0, ~S, ~I, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~0, ~0, ~S, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~0, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));
    
    send_to_display(~0, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~B, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));
    
    send_to_display(~0, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~R, ~B, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~0, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~I, ~R, ~B, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~0, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~S, ~I, ~R, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~0, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~S, ~I, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~0, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~S, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed));

    send_to_display(~0, ~0, ~0, SPI_VOLTAGE_PIN);
    send_to_display(~0, ~0, ~0, SPI_CURRENT_PIN);
    vTaskDelay(pdMS_TO_TICKS(speed * 10));
  }
}

void CAN_tx_code(void *pvParameters) {
  init_CAN();
  Start_parameters_t HVT_Start_parameters2;
  for (;;) {
    xQueueReceive(CAN_rx_queue, &HVT_Start_parameters2, portMAX_DELAY);
    Serial.print("OD!  ACDC = ");
    Serial.print(H_Coils[30]);
    Serial.print("  ");
    Serial.print("HVT_auto = ");
    Serial.print(H_Coils[31]);
    Serial.print("  ");
    Serial.print("minutes = ");
    Serial.println(H_Regs[30]);

    Serial.print("HVT_Start_parameters2! ACDC = ");
    Serial.print((bool)&HVT_Start_parameters2.ACDC);
    Serial.print(" HVT_auto = ");
    Serial.print((bool)&HVT_Start_parameters2.HVT_auto);
    Serial.print(" minutes = ");
    Serial.println(*HVT_Start_parameters2.minutes);
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void CAN_rx_code(void *pvParameters) {
  twai_message_t rx_msg;
  for (;;) {
    if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
      Serial.println("Received message");
      if ((rx_msg.identifier & COB_ID_MASK) == NMT_MONITORING) {
        if ((rx_msg.identifier & NODE_ID_MASK) == PDA_NodeID) {
          PDA_module = rx_msg.data[0];
          Burn_state = rx_msg.data[1];
          Afterburn_state = rx_msg.data[2];
          Acoustic_state = rx_msg.data[3];
          Serial.print("PDA register: ");
          Serial.print((uint8_t)PDA_module, HEX);
          Serial.print("  ");
          Serial.print((uint8_t)Burn_state, HEX);
          Serial.print("  ");
          Serial.print((uint8_t)Afterburn_state, HEX);
          Serial.print("  ");
          Serial.println((uint8_t)Acoustic_state, HEX);
        }
        if ((rx_msg.identifier & NODE_ID_MASK) == HVT_NodeID) {
          HVT_module = rx_msg.data[0];
          HVT_state = rx_msg.data[1];
          Serial.print("HVT register: ");
          Serial.print((uint8_t)HVT_module, HEX);
          Serial.print("  ");
          Serial.println((uint8_t)HVT_state, HEX);
        }
      }
    }
    //Serial.println("waiting for CAN message");
  }
}

void heartbeat_task_code(void *pvParameters) {
  Serial.println("heartbeat_task started");
  int full_number_of_modules = sizeof(full_config) / sizeof(full_config[0]);
  for (;;) {
    for (int i = 0; i < full_number_of_modules; i++) {
        NMT_Message.identifier = NMT_MONITORING + full_config[i].Node_address;
        NMT_Message.data_length_code = 0;
        if(twai_transmit(&NMT_Message, 1000) == ESP_OK) Serial.print("Send heartbeat to ");
        else Serial.print("Not send heartbeat to ");
        Serial.println(full_config[i].Node_name);

        vTaskDelay(pdMS_TO_TICKS(10)); // Задержка чтобы IFACE мог принять ответ. Но вооще по Serial ответ мгновенный
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void init_CAN() {
  Serial.println("init started");
  vTaskDelay(pdMS_TO_TICKS(100));

  int basic_number_of_modules = sizeof(basic_config) / sizeof(basic_config[0]);

  NMT_Message.data_length_code = 1;
  NMT_Message.data[0] = INITIALIZING;  // State. (0-й байт придет первым т.к. little-endian)

  unsigned long init_start = millis();

  while (!_isTimer(init_start, pdMS_TO_TICKS(3000))) {
    for (int i = 0; i < basic_number_of_modules; i++) {
      if (*basic_config[i].Node_state == PRE_OPERATIONAL) continue;

      NMT_Message.identifier = basic_config[i].Node_address;

      if (twai_transmit(&NMT_Message, 100) == ESP_OK) Serial.println("init request sent");
      else Serial.println("init request NOT sent");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    if ((uint8_t)PDA_module == PRE_OPERATIONAL && (uint8_t)HVT_module == PRE_OPERATIONAL) {
      xTaskCreatePinnedToCore(heartbeat_task_code, "heartbeat_task", 10000, NULL, 7, &heartbeat_task, 0);
      vTaskDelay(pdMS_TO_TICKS(30));
      Serial.println("init successful");
      break;
    }

    // xTaskCreatePinnedToCore(heartbeat_task_code, "heartbeat_task", 10000, NULL, 7, &heartbeat_task, 0);
    // vTaskDelay(pdMS_TO_TICKS(30));

  }

  if ((uint8_t)PDA_module != PRE_OPERATIONAL && (uint8_t)HVT_module != PRE_OPERATIONAL) {
    Serial.println("init failed");
  }
}













