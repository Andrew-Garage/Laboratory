#include <Arduino.h>
#include "ModbusMessage.h"
#include "HardwareSerial.h"
#include "ModbusServerRTU.h"
#include "ModbusTypeDefs.h"
#include "CoilData.h"

#define LOCAL_LOG_LEVEL LOG_LEVEL_VERBOSE
#define LOG_LEVEL LOG_LEVEL_VERBOSE
#include "Logging.h"

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C main_lcd(0x38, 16, 2);

#include <SPI.h>
#define BME_MISO           100    // Убираем это из SPI
#define BME_CS             101    // и это тоже

#define SPI_CURRENT_PIN    32
#define SPI_VOLTAGE_PIN    33
#define CURRENT            H_Reg[1]
#define VOLTAGE            H_Reg[2]

#define PC_CONNECTED       8
#define EXIT_button        1
#define ENTER_button       2
#define ON_button          7
#define OFF_button         6
#define ENCODER_button     5
#define PUSH_TIME          100



bool btn1, btn2, btn3, btn4, btn5;
unsigned long start_push = 0UL;

TaskHandle_t builtin_Led, button_Handler, seven_seg_Display, lcd_Display, check_PC_connection;

CoilData myCoils("0         0         0         0         0         1         1         0         0");  // Порядок тэгов 0 -> 8
//             (Led,    Ext_b,    Ent_b,     NULL,     NULL,  Encoder,     On_b,    Off_b,  PC_conn, )
float H_Reg[] = {98.7, 12.3, 34.5, 56.7, 78.9,}; 
//              (   0,    2,    4,    6,    8, )    СМЕЩЕНИЕ +2 байта для FLOAT!
       
byte led_display[3] = {0, 0, 0};

// Create a ModbusRTU server instance listening on Serial2 with 20000ms timeout
ModbusServerRTU MBserver(Serial2, 2000);

ModbusMessage FC01_ReadCoil(ModbusMessage request){
  //Serial.println("Read coil!");     // Считывает все койлы за раз (даже NULL) если выставить "максимум регистров"
  ModbusMessage response;           // Считывает 3 и 2 тэга, если "последовательные регистры"
  uint16_t start = 0;               // Считывает по одному, если "регистры 1 тэга"
  uint16_t numCoils = 0;
  request.get(2, start, numCoils);    
  
  if(start + numCoils <= myCoils.coils()){
    vector<uint8_t> coilset = myCoils.slice(start, numCoils);
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)coilset.size(), coilset);
  }
  else{
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);        
  }
  return response;
}

ModbusMessage FC05_WriteCoil(ModbusMessage request){
  //Serial.println("Write coil!");
  ModbusMessage response;
  uint16_t start = 0;
  uint16_t state = 0;
  request.get(2, start, state);  

  if(start <= myCoils.coils()){
      if(state == 0x0000 || state == 0xFF00){   // FF00 = 65280 это "1"
         if(myCoils.set(start, state)){
           response = ECHO_RESPONSE;
         }
         else{
           response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);             
         }         
      }
      else{      
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);   
      }
  }
  else{
  response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);  
  }

  return response;  
}

ModbusMessage FC03_ReadReg(ModbusMessage request){
  //Serial.println("Read reg!");
  ModbusMessage response;
  uint16_t address;           // requested register address
  uint16_t words;             // requested number of registers

  request.get(2, address);  
  request.get(4, words);
  address = address / 2; // приводим к адресации H_Reg как массива c++
  words = words / 2;     // приводим к количеству ячеек массива H_Reg (будет считываться на 1 меньше из-за условия в цикле for)

  if (address + words <= sizeof(H_Reg)/sizeof(H_Reg[0])) {
    // Looks okay. Set up message with serverID, FC and length of data
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(4 * words)); // добавлять можно только побайтово, поэтому добавляем сколько uint8_t
    // Fill response with requested data
    for (uint16_t i = address; i < address + words; i++) {
      response.add(H_Reg[i]);
    }
  } else {
    //No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}

ModbusMessage FC16_WriteReg(ModbusMessage request){
  //Serial.println("Write reg!");
  ModbusMessage response;  
  float received_num = 0;
  uint16_t data[2];
  uint16_t address;
  
  request.get(2, address);

  if (address <= sizeof(H_Reg)/sizeof(H_Reg[0])) {
    request.get(7, data[1]);    // Старшие байты
    request.get(9, data[0]);    // Младшие байты
    memcpy(&received_num, data, 4);
    //Serial.println(received_num);

    H_Reg[address / 2] = received_num;
    response = ECHO_RESPONSE;
  } else {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}

ModbusMessage FC_ANY(ModbusMessage request){
  printing(request);
  return ECHO_RESPONSE;
}

void setup() {
  Serial.begin(115200); // For debug
  while (!Serial) {}
  Serial.println("__ Serial OK __");

  xTaskCreatePinnedToCore(builtin_Led_code,"Task0",10000,NULL,1,&builtin_Led, 0);
  vTaskDelay(pdMS_TO_TICKS(100));

  xTaskCreatePinnedToCore(button_Handler_code,"Task1",10000,NULL,1,&button_Handler, 0);
  vTaskDelay(pdMS_TO_TICKS(100));

  xTaskCreatePinnedToCore(seven_seg_Display_code,"Task2",10000,NULL,1,&seven_seg_Display, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  xTaskCreatePinnedToCore(lcd_Display_code,"Task3",10000,NULL,1,&lcd_Display, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  xTaskCreatePinnedToCore(check_PC_connection_code,"Task4",10000,NULL,1,&check_PC_connection, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  Serial2.begin(115200, SERIAL_8N1, GPIO_NUM_16, GPIO_NUM_17);      // поменял местами рх тх!

  MBserver.registerWorker(0x01, READ_COIL, &FC01_ReadCoil);               // Читает по нескольку
  MBserver.registerWorker(0x01, WRITE_COIL, &FC05_WriteCoil);             // Пишет по одному
  MBserver.registerWorker(0x01, READ_HOLD_REGISTER, &FC03_ReadReg);       // Читает по нескольку
  MBserver.registerWorker(0x01, WRITE_MULT_REGISTERS, &FC16_WriteReg);    // Пишет по одному
  MBserver.registerWorker(0x01, 0x00, &FC_ANY);
  
  // Start ModbusRTU background task
  delay(500);
  MBserver.start();       
  MBserver.skipLeading0x00();
}

void loop() {
}

void builtin_Led_code(void * pvParameters){                  //CORE 0
pinMode(2, OUTPUT);
  for(;;){
    digitalWrite(2, myCoils[0]);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void button_Handler_code(void * pvParameters){               //CORE 0
btn1, btn2, btn3, btn4, btn5 = 0;
int pressed_button = 0;
pinMode(36, INPUT);
  for(;;){
    //check_button(analogRead(36));
    pressed_button = analogRead(36);

    if(pressed_button < 200){
      myCoils.set(EXIT_button, (bool) 1);
      myCoils.set(ENTER_button, (bool) 0);
      myCoils.set(OFF_button, (bool) 0);
      myCoils.set(ON_button, (bool) 0);
      myCoils.set(ENCODER_button, (bool) 0);
      }
    else if(385 < pressed_button && pressed_button < 785){
      myCoils.set(EXIT_button, (bool) 0);
      myCoils.set(ENTER_button, (bool) 1);
      myCoils.set(OFF_button, (bool) 0);
      myCoils.set(ON_button, (bool) 0);
      myCoils.set(ENCODER_button, (bool) 0);
      }
    else if(1110 < pressed_button && pressed_button < 1510){
      myCoils.set(EXIT_button, (bool) 0);
      myCoils.set(ENTER_button, (bool) 0);
      myCoils.set(OFF_button, (bool) 1);
      myCoils.set(ON_button, (bool) 0);
      myCoils.set(ENCODER_button, (bool) 0);
      }
    else if(1845 < pressed_button && pressed_button < 2245){
      myCoils.set(EXIT_button, (bool) 0);
      myCoils.set(ENTER_button, (bool) 0);
      myCoils.set(OFF_button, (bool) 0);
      myCoils.set(ON_button, (bool) 1);
      myCoils.set(ENCODER_button, (bool) 0);
      }
    else if(2740 < pressed_button && pressed_button < 3140){
      myCoils.set(EXIT_button, (bool) 0);
      myCoils.set(ENTER_button, (bool) 0);
      myCoils.set(OFF_button, (bool) 0);
      myCoils.set(ON_button, (bool) 0);
      myCoils.set(ENCODER_button, (bool) 1);
    }
    else{
      myCoils.set(EXIT_button, (bool) 0);
      myCoils.set(ENTER_button, (bool) 0);
      myCoils.set(OFF_button, (bool) 0);
      myCoils.set(ON_button, (bool) 0);
      myCoils.set(ENCODER_button, (bool) 0);
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void seven_seg_Display_code(void * pvParameters){            //CORE 1
SPI.begin();
pinMode(32, OUTPUT);
pinMode(33, OUTPUT);
  for(;;){
    // ТОК
    change_order_of_digits(CURRENT, led_display);  // Изменяем порядок цифр и добавляем точку (float значение, byte arr куда пишем результат)
    send_to_display(~led_display[0], ~led_display[1], ~led_display[2], SPI_CURRENT_PIN); // 
    vTaskDelay(pdMS_TO_TICKS(200));

    // НАПРЯЖЕНИЕ
    change_order_of_digits(VOLTAGE, led_display);
    send_to_display(~led_display[0], ~led_display[1], ~led_display[2], SPI_VOLTAGE_PIN);
    vTaskDelay(pdMS_TO_TICKS(200));         
  }
}

void lcd_Display_code(void * pvParameters){                  //CORE 1
main_lcd.init();
main_lcd.backlight();
int word_len;
  for(;;){
    if(myCoils[PC_CONNECTED]){
      if(String("PC connected").length() != word_len) main_lcd.clear();
      word_len = String("PC connected").length();
      main_lcd.setCursor(int((16 - word_len)/2), 0);
      main_lcd.print(String("PC connected"));
    } else{
      if(String("Menu").length() != word_len) main_lcd.clear();
      word_len = String("Menu").length();
      main_lcd.setCursor(int((16 - word_len)/2), 0);
      main_lcd.print(String("Menu"));
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void check_PC_connection_code(void * pvParameters){          //CORE 1
int old_messages = 0;
int counter = 0;
  for(;;){
    if(old_messages != MBserver.getMessageCount()){
      old_messages = MBserver.getMessageCount();
      counter = 0;
      myCoils.set(PC_CONNECTED, (bool) 1);
    } else{
      counter++;
      if(counter > 1){
        myCoils.set(PC_CONNECTED, (bool) 0);
      }
    }
    //Serial.println(uxTaskGetNumberOfTasks());
    //TaskHandle_t builtin_Led, button_Handler, seven_seg_Display, lcd_Display, check_PC_connection;
    /*Serial.print("LED ");
    Serial.println(uxTaskGetStackHighWaterMark(builtin_Led));
    Serial.print("Button ");
    Serial.println(uxTaskGetStackHighWaterMark(button_Handler));
    Serial.print("7seg display ");
    Serial.println(uxTaskGetStackHighWaterMark(seven_seg_Display));
    Serial.print("LCD display ");
    Serial.println(uxTaskGetStackHighWaterMark(lcd_Display));
    Serial.print("PC ");
    Serial.println(uxTaskGetStackHighWaterMark(check_PC_connection));
    Serial.println();*/
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}












byte convert_to_bitSequence (int num){
  switch (num) {
   case 0:  return 63;
   case 1:  return 6;
   case 2:  return 91;
   case 3:  return 79;
   case 4:  return 102;
   case 5:  return 109;
   case 6:  return 125;
   case 7:  return 7;
   case 8:  return 127;
   case 9:  return 111;
   default: return 64;   // Dash
  }
}

void send_to_display(byte n0, byte n1, byte n2, int pin){
  digitalWrite(pin, 0);
  SPI.transfer(n2);
  SPI.transfer(n1);
  SPI.transfer(n0);
  digitalWrite(pin, 1);
}

int change_order_of_digits(float num, byte *arr){
  if(num >= 100){
    int int_num = (int)num;
    arr[0] = convert_to_bitSequence(int_num % 10) + 128;
    arr[1] = convert_to_bitSequence((int_num % 100) / 10);
    arr[2] = convert_to_bitSequence((int_num % 1000) / 100);
  }
  else if(100 > num && num >= 10){
    num = num * 10;
    int int_num = (int)num;
    arr[0] = convert_to_bitSequence(int_num % 10);
    arr[1] = convert_to_bitSequence((int_num % 100) / 10) + 128;
    arr[2] = convert_to_bitSequence((int_num % 1000) / 100);
  }
  else if(10 > num && num >= 0){
    num = num * 100;
    int int_num = (int)num;
    arr[0] = convert_to_bitSequence(int_num % 10);
    arr[1] = convert_to_bitSequence((int_num % 100) / 10);
    arr[2] = convert_to_bitSequence((int_num % 1000) / 100) + 128;
  }
  else{
    return 0;
  }
  return 1;
}

void check_button(int ButtonIN){
  if(ButtonIN < 200){                                     // (1) Если нажата кнопка EXIT, заходим сюда.
      if (btn1){                                          // (5) Если уже заходили по нажатию EXIT, то попадем сюда.
            if(_isTimer(start_push, PUSH_TIME)){          // (6) Пока держим кнопку, будем биться в этот if, пока _isTimer() не вернет 1.
			        myCoils.set(EXIT_button, (bool) 1);         // (7) Считаем, что кнопка EXIT нажата.
			      }
      } else{                                             // (2) Переменная бтн1 изначально равна 0, значит зайдем сюда.
          btn1 = 1;                                       // (3) Взводим ее. Типо зашли.
          start_push = millis();                          // (4) И запоминаем во сколько зашли.
      }
  } else{                                                 // Пока EXIT не нажата...
		  btn1 = 0;                                           // ... флаг бтн1 и значение "нажатости" кнопки EXIT постоянно зануляются.
		  myCoils.set(EXIT_button, (bool) 0);                 // Нужно приведение типов (bool), иначе ругается.
		}
    
  if(385 < ButtonIN && ButtonIN < 785){
      if (btn2){
            if(_isTimer(start_push, PUSH_TIME)){
			        myCoils.set(ENTER_button, (bool) 1);
			      }
      } else{
          btn2 = 1;
          start_push = millis();
      }
  } else{
		  btn2 = 0;
		  myCoils.set(ENTER_button, (bool) 0);
		}

  if(1110 < ButtonIN && ButtonIN < 1510){
      if (btn3){
            if(_isTimer(start_push, PUSH_TIME)){
			        myCoils.set(OFF_button, (bool) 1);
			}
      } else{
          btn3 = 1;
          start_push = millis();
        }
  } else{
		  btn3 = 0;
		  myCoils.set(OFF_button, (bool) 0);
		}

  if(1845 < ButtonIN && ButtonIN < 2245){
      if (btn4){
            if(_isTimer(start_push, PUSH_TIME)){
			        myCoils.set(ON_button, (bool) 1);
     			 }
	} else{
      btn4 = 1;
      start_push = millis();
    }
  } else{
		  btn4 = 0;
		  myCoils.set(ON_button, (bool) 0);
		}

  if(2740 < ButtonIN && ButtonIN < 3140){
      if (btn5){
            if(_isTimer(start_push, PUSH_TIME)){
			        myCoils.set(ENCODER_button, (bool) 1);
     			 }
	} else{
      btn5 = 1;
      start_push = millis();
    }
  } else{
		  btn5 = 0;
		  myCoils.set(ENCODER_button, (bool) 0);
		}
}

bool _isTimer(unsigned long startTime, unsigned long period){
    unsigned long currentTime;
    currentTime = millis();
    if (currentTime>= startTime){
        return (currentTime>=(startTime + period));
    } else{
        return (currentTime >=(4294967295-startTime+period));
      }
}


void printing (ModbusMessage request){
  uint16_t a1;
  uint16_t a2;
  uint16_t a3;
  uint16_t a4;
  uint16_t a5;
  uint16_t a6;
  uint16_t a7;
  uint16_t a8;
  uint16_t a9;
  uint16_t a10;
  uint16_t a11;
  uint16_t a12;
  uint16_t a13;
  uint16_t a14;
  uint16_t a15;
  uint16_t a16;

  request.get(1, a1);
  request.get(2, a2);
  request.get(3, a3);
  request.get(4, a4);
  request.get(5, a5);
  request.get(6, a6);
  request.get(7, a7);
  request.get(8, a8);
  request.get(9, a9);
  request.get(10, a10);
  request.get(11, a11);
  request.get(12, a12);
  request.get(13, a13);
  request.get(14, a14);
  request.get(15, a15);
  request.get(16, a16);

  Serial.print("SERVER ID: ");
  Serial.println(request.getServerID());
  Serial.print("FUNC CODE: ");
  Serial.println(request.getFunctionCode());

  Serial.print("1 - ");
  Serial.println(a1);
  Serial.print("2 - ");
  Serial.println(a2);
  Serial.print("3 - ");
  Serial.println(a3);
  Serial.print("4 - ");
  Serial.println(a4);
  Serial.print("5 - ");
  Serial.println(a5);
  Serial.print("6 - ");
  Serial.println(a6);
  Serial.print("7 - ");
  Serial.println(a7, HEX);
  Serial.print("8 - ");
  Serial.println(a8);
  Serial.print("9 - ");
  Serial.println(a9, HEX);
  Serial.print("10 - ");
  Serial.println(a10);
  Serial.print("11 - ");
  Serial.println(a11);
  Serial.print("12 - ");
  Serial.println(a12);
  Serial.print("13 - ");
  Serial.println(a13);
  Serial.print("14 - ");
  Serial.println(a14);
  Serial.print("15 - ");
  Serial.println(a15);
  Serial.print("16 - ");
  Serial.println(a16);
}

ModbusMessage FC06(ModbusMessage request){
  Serial.println("Write Reg!");

  ModbusMessage response;
  uint16_t address;
  uint16_t value;
  request.get(2, address);        // requested register address
  request.get(4, value);          // requested number of registers
  
  if(H_Reg [address] = value){
      response = ECHO_RESPONSE;
      //response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);
    }
    else{
    Serial.println("Write ERROR!");
    response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);             
    } 
  return response;
}

ModbusMessage FC04(ModbusMessage request){
  Serial.println("Read float!");
  ModbusMessage response;
  uint16_t address;
  uint16_t value;
  request.get(2, address);        // requested register address
  request.get(4, value);          // requested number of registers

    // Looks okay. Set up message with serverID, FC and length of data
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(value * 2));
    // Fill response with requested data
    response.add(H_Reg[address]);

  return response;
}
