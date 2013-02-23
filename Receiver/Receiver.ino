#include <cc2500_REG.h>
#include <cc2500_VAL.h>
#include <SPI.h>
// Definition of interrupt names
#include < avr/io.h >
// ISR interrupt service routine
#include < avr/interrupt.h >

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0xBF
#define SRES           0x30




const int GDO0_PIN = 9;     // the number of the GDO0_PIN pin
int GDO0_State = 0;         
int led = 5;
int led_gnd = 4;

int led_ext = 6;

int ledValue = 20;

volatile int looponce=LOW;





void setup() {
 

 
  
  pinMode(SS,OUTPUT);
  pinMode(led_gnd, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(led_ext, OUTPUT);
  analogWrite(led,ledValue);
  analogWrite(led_ext,ledValue);
  digitalWrite(led_gnd, LOW);
  
  Serial.begin(9600);

  SPI.begin();
  digitalWrite(SS,HIGH);
  // initialize the pushbutton pin as an input:
 // pinMode(buttonPin, INPUT);     
 // pinMode(GDO0_PIN, INPUT);     
  
  attachInterrupt(0, loopnow, RISING);

  Serial.println("Starting..");
  init_CC2500();
  //Read_Config_Regs();
}

void loop()
{
  
  //change this back to calling RxData_RF(); and no interrupt calls.
  RxData_RF();

  if(looponce==HIGH)
  {
  RxData_RF();
  looponce=LOW;  
  }
   

}

void loopnow()
{
 looponce=HIGH; 
}


void RxData_RF(void) 
{

  
  int PacketLength=0;
  // RX: enable RX
  SendStrobe(CC2500_RX);
  byte state = SendStrobe(00);
  //Serial.println(state,HEX);
   
   if(state>>0x4==1) //mode is RX mode
   {
     int count=0;
     // Wait for GDO0 to be set -> sync transmitted
      while (!GDO0_State)
      {
          // read the state of the GDO0_PIN value:
          GDO0_State = digitalRead(GDO0_PIN);
          delay(1);
          count++;
          
          if(count>1000) {
            flushRX();
           //  Serial.println("ERR NO DATA"); 
            return;
          }
       }
       
       // Wait for GDO0 to be cleared -> end of packet
       while (GDO0_State)
       {
           // read the state of the GDO0_PIN value:
           GDO0_State = digitalRead(GDO0_PIN);
       }
       
      // state = SendStrobe(00);
       
    // Serial.println("Reading Bytes"); 
     int data;
     int i=0;
    // Read data from RX FIFO and store in rxBuffer
    while((state & 0xF) > 0)
    {    
    
      digitalWrite(SS,LOW);
      while (digitalRead(MISO) == HIGH) {
      };
      state = SPI.transfer(CC2500_RXFIFO);
      delay(1);
      unsigned char data = SPI.transfer(0);
      digitalWrite(SS,HIGH);
      
      Serial.println(data, DEC);
      if(i==0)
      {
        if(data<10) data=0;
        ledValue = data;
        analogWrite(led,ledValue);
        analogWrite(led_ext,ledValue);
      }
      
     if((state >> 4) > 1)
     {
      flushRX();
      Serial.print("ERR STATE = "); 
     Serial.println(state >> 4);  //prints state
     return; 
     }
    i++;
    
    }
    Serial.println();
   // Serial.println(state,HEX);
   //  delay(1000);
    
   // Serial.println();
 
   // flushRX();
  
   }else
   {
    Serial.println("ERR RX MODE NOT ENABLED");  
   }

}// Rf RxPacket

void flushRX()
{
    // Make sure that the radio is in IDLE state before flushing the FIFO
  // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
   delay(10); 
   SendStrobe(CC2500_IDLE);
   delay(10);
   // Flush RX FIFO
   SendStrobe(CC2500_FRX); 
   delay(10); 
}

void WriteReg(char addr, char value)
{
  digitalWrite(SS,LOW);

  while (digitalRead(MISO) == HIGH) {
  };

  SPI.transfer(addr);
  delay(1);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
}

char ReadReg(char addr)
{
  byte stat=0xFF;
  return ReadReg(addr, stat);
}

char ReadReg(char addr, byte &stat)
{
  
  addr = addr | 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  byte tmp = SPI.transfer(addr);
  if(stat!=0xFF) stat = tmp;
  delay(1);
  char data = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return data;  
}

char SendStrobe(char strobe)
{
  digitalWrite(SS,LOW);

  while (digitalRead(MISO) == HIGH) {
  };

  char result =  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(1);
  return result;
}


void init_CC2500()
{
  SendStrobe(SRES);
  WriteReg(REG_IOCFG2,VAL_IOCFG2);
  WriteReg(REG_IOCFG1,VAL_IOCFG1);
  WriteReg(REG_IOCFG0,VAL_IOCFG0);

  WriteReg(REG_FIFOTHR,VAL_FIFOTHR);
  WriteReg(REG_SYNC1,VAL_SYNC1);
  WriteReg(REG_SYNC0,VAL_SYNC0);
  WriteReg(REG_PKTLEN,VAL_PKTLEN);
  WriteReg(REG_PKTCTRL1,VAL_PKTCTRL1);
  WriteReg(REG_PKTCTRL0,VAL_PKTCTRL0);
  WriteReg(REG_ADDR,VAL_ADDR);
  WriteReg(REG_CHANNR,VAL_CHANNR);
  WriteReg(REG_FSCTRL1,VAL_FSCTRL1);
  WriteReg(REG_FSCTRL0,VAL_FSCTRL0);
  WriteReg(REG_FREQ2,VAL_FREQ2);
  WriteReg(REG_FREQ1,VAL_FREQ1);
  WriteReg(REG_FREQ0,VAL_FREQ0);
  WriteReg(REG_MDMCFG4,VAL_MDMCFG4);
  WriteReg(REG_MDMCFG3,VAL_MDMCFG3);
  WriteReg(REG_MDMCFG2,VAL_MDMCFG2);
  WriteReg(REG_MDMCFG1,VAL_MDMCFG1);
  WriteReg(REG_MDMCFG0,VAL_MDMCFG0);
  WriteReg(REG_DEVIATN,VAL_DEVIATN);
  WriteReg(REG_MCSM2,VAL_MCSM2);
  WriteReg(REG_MCSM1,VAL_MCSM1);
  WriteReg(REG_MCSM0,VAL_MCSM0);
  WriteReg(REG_FOCCFG,VAL_FOCCFG);

  WriteReg(REG_BSCFG,VAL_BSCFG);
  WriteReg(REG_AGCCTRL2,VAL_AGCCTRL2);
  WriteReg(REG_AGCCTRL1,VAL_AGCCTRL1);
  WriteReg(REG_AGCCTRL0,VAL_AGCCTRL0);
  WriteReg(REG_WOREVT1,VAL_WOREVT1);
  WriteReg(REG_WOREVT0,VAL_WOREVT0);
  WriteReg(REG_WORCTRL,VAL_WORCTRL);
  WriteReg(REG_FREND1,VAL_FREND1);
  WriteReg(REG_FREND0,VAL_FREND0);
  WriteReg(REG_FSCAL3,VAL_FSCAL3);
  WriteReg(REG_FSCAL2,VAL_FSCAL2);
  WriteReg(REG_FSCAL1,VAL_FSCAL1);
  WriteReg(REG_FSCAL0,VAL_FSCAL0);
  WriteReg(REG_RCCTRL1,VAL_RCCTRL1);
  WriteReg(REG_RCCTRL0,VAL_RCCTRL0);
  WriteReg(REG_FSTEST,VAL_FSTEST);
  WriteReg(REG_PTEST,VAL_PTEST);
  WriteReg(REG_AGCTEST,VAL_AGCTEST);
  WriteReg(REG_TEST2,VAL_TEST2);
  WriteReg(REG_TEST1,VAL_TEST1);
  WriteReg(REG_TEST0,VAL_TEST0);
  /*  
   WriteReg(REG_PARTNUM,VAL_PARTNUM);
   WriteReg(REG_VERSION,VAL_VERSION);
   WriteReg(REG_FREQEST,VAL_FREQEST);
   WriteReg(REG_LQI,VAL_LQI);
   WriteReg(REG_RSSI,VAL_RSSI);
   WriteReg(REG_MARCSTATE,VAL_MARCSTATE);
   WriteReg(REG_WORTIME1,VAL_WORTIME1);
   WriteReg(REG_WORTIME0,VAL_WORTIME0);
   WriteReg(REG_PKTSTATUS,VAL_PKTSTATUS);
   WriteReg(REG_VCO_VC_DAC,VAL_VCO_VC_DAC);
   WriteReg(REG_TXBYTES,VAL_TXBYTES);
   WriteReg(REG_RXBYTES,VAL_RXBYTES);
   WriteReg(REG_RCCTRL1_STATUS,VAL_RCCTRL1_STATUS);
   WriteReg(REG_RCCTRL0_STATUS,VAL_RCCTRL0_STATUS);
   */
}

void Read_Config_Regs(void)
{ 
  Serial.println("Configuration registers");
  Serial.println(ReadReg(REG_IOCFG2),HEX);
  delay(1000);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
  delay(1000);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
  delay(1000);
  /* Serial.println(ReadReg(REG_FIFOTHR),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_SYNC1),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_SYNC0),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_PKTLEN),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_PKTCTRL1),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_PKTCTRL0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_ADDR),HEX);
   delay(10);
   Serial.println(ReadReg(REG_CHANNR),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FSCTRL1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FSCTRL0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FREQ2),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FREQ1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FREQ0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_MDMCFG4),HEX);
   delay(10);
   Serial.println(ReadReg(REG_MDMCFG3),HEX);
   delay(10);
   Serial.println(ReadReg(REG_MDMCFG2),HEX);
   delay(10);
   Serial.println(ReadReg(REG_MDMCFG1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_MDMCFG0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_DEVIATN),HEX);
   delay(10);
   Serial.println(ReadReg(REG_MCSM2),HEX);
   delay(10);
   Serial.println(ReadReg(REG_MCSM1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_MCSM0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FOCCFG),HEX);
   delay(10);
   
   Serial.println(ReadReg(REG_BSCFG),HEX);
   delay(10);
   Serial.println(ReadReg(REG_AGCCTRL2),HEX);
   delay(10);
   Serial.println(ReadReg(REG_AGCCTRL1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_AGCCTRL0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_WOREVT1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_WOREVT0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_WORCTRL),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FREND1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FREND0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FSCAL3),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FSCAL2),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FSCAL1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FSCAL0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_RCCTRL1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_RCCTRL0),HEX);
   delay(10);
   Serial.println(ReadReg(REG_FSTEST),HEX);
   delay(10);
   Serial.println(ReadReg(REG_PTEST),HEX);
   delay(10);
   Serial.println(ReadReg(REG_AGCTEST),HEX);
   delay(10);
   Serial.println(ReadReg(REG_TEST2),HEX);
   delay(10);
   Serial.println(ReadReg(REG_TEST1),HEX);
   delay(10);
   Serial.println(ReadReg(REG_TEST0),HEX);
   delay(10);
  /*
   Serial.println(ReadReg(REG_PARTNUM),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_VERSION),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_FREQEST),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_LQI),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_RSSI),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_MARCSTATE),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_WORTIME1),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_WORTIME0),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_PKTSTATUS),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_VCO_VC_DAC),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_TXBYTES),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_RXBYTES),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_RCCTRL1_STATUS),HEX);
   delay(1000);
   Serial.println(ReadReg(REG_RCCTRL0_STATUS),HEX);
   delay(1000);
   */
}


