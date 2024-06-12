#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#ifdef ESP32_C3_SUPERMINI
  #define SCK_PIN      1
  #define MOSI_PIN     4
  #define MISO_PIN     2
  #define CAN0_CS_PIN  3
#elif ESP32
  #define SCK_PIN      16
  #define MOSI_PIN     4
  #define MISO_PIN     2
  #define CAN0_CS_PIN  15
#elif NRF52840_PROMICRO
  #define SCK_PIN      PIN_P0_17
  #define MOSI_PIN     PIN_P0_20
  #define MISO_PIN     PIN_P0_22
  #define CAN0_CS_PIN  PIN_P1_04
#endif

struct can_frame canMsg;
SPIClass *spi;
MCP2515 *CAN0;
int cntr = 0;
unsigned long oldTime = 0;

uint8_t recv_id[2047];

void testPin(uint8_t pin) {
  Serial.print("Pin test: ");
  Serial.println(pin);
  pinMode(pin, OUTPUT);
  while (true) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(100);
    digitalWrite(pin, LOW);
    delayMicroseconds(200);
  }
}

void zero_recv_id() {
  for(int i=0; i<sizeof(recv_id); i++) {
    recv_id[i] = 0;
  }
}

void setup()
{
  Serial.begin(115200);
  // while (!Serial1)
  // {
  //   /* code */
  // }
  

  delay(1000);
  Serial.println("CAN monitor Start");
  // PrintFileNameDateTime();
  
    
  Serial.print("MOSI: ");
  Serial.println(MOSI_PIN);
  Serial.print("MISO: ");
  Serial.println(MISO_PIN);
  Serial.print("SCK: ");
  Serial.println(SCK_PIN);
  Serial.print("SS: ");
  Serial.println(CAN0_CS_PIN);  

  // testPin(CAN0_CS_PIN);

  #ifdef ESP32_C3_SUPERMINI
    spi = new SPIClass(FSPI);
    // spi = SPI(FSPI);
    spi->begin(SCK_PIN, MISO_PIN, MOSI_PIN, CAN0_CS_PIN);
    spi->setHwCs(false);
  #elif ESP32
    spi = new SPIClass();
    spi->begin(SCK_PIN, MISO_PIN, MOSI_PIN, CAN0_CS_PIN);
    spi->setHwCs(false);
  #elif NRF52840_PROMICRO
    spi = new SPIClass(NRF_SPIM3, MISO_PIN, SCK_PIN, MOSI_PIN);
    spi->begin();
  #endif

  CAN0 = new MCP2515(CAN0_CS_PIN, 100000000U, spi);
 
  int error = 1;

  for(;error!=0;) {
    delay(500);
    // Serial.print(" CAN status: ");
    // Serial.println(CAN0->getStatus());
    error = 0;
    if (CAN0->reset()==MCP2515::ERROR_OK) {
      Serial.println("CAN0: MCP2515 reset OK");
    } else {
      Serial.println("CAN0: MCP2515 reset ERROR!");
      error++;
      continue;
    }
    if (CAN0->setBitrate(CAN_500KBPS, MCP_8MHZ)==MCP2515::ERROR_OK) {
      Serial.println("CAN0: MCP2515 speed set: 500 KBPS");
    } else {
      Serial.println("CAN0: MCP2515 speed set ERROR!");
      error++;
      continue;      
    }
      
    // if (CAN0->setNormalMode()==MCP2515::ERROR_OK) {
    //   Serial.println("MCP2515 normal mode set");
    // } else {
    //   Serial.println("CAN0: MCP2515 normal mode set ERROR!");
    //   error++;
    //   continue;      
    // }

    if (CAN0->setListenOnlyMode()==MCP2515::ERROR_OK) {
    // if (CAN0->setLoopbackMode()==MCP2515::ERROR_OK) {
      Serial.println("CAN0: MCP2515 monitor mode set");
    } else {
      Serial.println("CAN0: MCP2515 monitor mode set ERROR!");
      error++;
      continue;      
    }
    

  }

  delay(1000);

  // CAN0->checkReceive()
 
  Serial.print(" CAN status: ");
  Serial.println(CAN0->getStatus());
  Serial.print(" CAN error flags: ");
  Serial.println(CAN0->getErrorFlags());
  // CAN0->modifyRegister(MCP2515::MCP_RXB0CTRL);
}
 
u_int64_t next_frame_ts  = 0;
bool new_id_only = false;

void print_frame() {
  u_int64_t ts = millis();
  Serial.print(ts);
  Serial.print("\tCAN0:");
  Serial.printf(" 0x%03x", canMsg.can_id & 0x7ff);
  if (canMsg.can_id & CAN_EFF_FLAG) {
    Serial.printf(" E");      
  } else if (canMsg.can_id & CAN_RTR_FLAG) {
    Serial.printf(" R");
  } else {
    Serial.printf("  ");
  }

  Serial.print(" [");
  Serial.print(canMsg.can_dlc);  
  Serial.print("] ");
  for (int i=0; i<canMsg.can_dlc; i++) {
    Serial.printf("%02x ", canMsg.data[i]);
  }
  for (int i=canMsg.can_dlc; i<8; i++) {
    Serial.print("-- ");
  }
  Serial.print("| ");
  for (int i=0; i<canMsg.can_dlc; i++) {
    int c = canMsg.data[i];
    if (isascii(c)) {
      Serial.print((char)c);
    } else {
      Serial.print(".");
    }
  }
  for (int i=canMsg.can_dlc; i<8; i++) {
    Serial.print(" ");
  }
  Serial.println();
}

void loop() {  
  if (Serial.available()) {
    int c = Serial.read();
    switch(c) {
      case 'n':
      Serial.println("\rOK");
      new_id_only = true;
      zero_recv_id();
      break;
      default:
      Serial.println("?");
    }
  }
  // if (next_frame_ts<ts) {
  //   next_frame_ts = ts+500;
  //   canMsg.can_id = 1;
  //   canMsg.can_dlc = 1;
  //   canMsg.data[0] = next_frame_ts & 0xff;
  //   MCP2515::ERROR e = CAN0->sendMessage(&canMsg);
  //   if (e!=MCP2515::ERROR_OK) {
  //     Serial.print(" CAN TX error: ");
  //     Serial.println(e);
  //   }
  // }

  // if (CAN0->checkError()) {
  //   Serial.print("CAN error TX: ");    
  //   Serial.print(CAN0->errorCountTX());
  //   Serial.print(" RX: ");    
  //   Serial.print(CAN0->errorCountRX());
  //   Serial.println();
  // }
  if (CAN0->readMessage(&canMsg) == MCP2515::ERROR_OK) {
    cntr++;
    int id = canMsg.can_id & 0x7ff;
    if (new_id_only) {
      if (!recv_id[id]) {
        recv_id[id]++;
        print_frame();
      }
    } else {
      print_frame();
    }
  }
 
  // if ((millis()-oldTime)>1000) {
  //   oldTime = millis();
  //   Serial.print(cntr);
  //   Serial.println(" msg/sec");
  //   cntr = 0;      
  // }
}