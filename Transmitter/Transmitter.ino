#include <cc2500_REG.h>
#include <cc2500_VAL.h>
#include <stdlib.h>	// for itoa() call

#include <SPI.h>
#include <string>

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0xBF


const int buttonHighPin = 3;
const int buttonLowPin = 5;
const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status

const int GDO0_PIN = 9;     // the number of the GDO0_PIN pin
int GDO0_State = 0;         // variable for reading the pushbutton status

int incomingByte = 0;   // for incoming serial data
unsigned char txData[50];

int sensorPin = A0; 
int sensorValue = 0; 

int led = 6;
int led_gnd = 7;
int ledValue = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(SS,OUTPUT);
  SPI.begin();
  digitalWrite(SS,HIGH);
  
    pinMode(led_gnd, OUTPUT);
  pinMode(led, OUTPUT);
  analogWrite(led,ledValue);
  digitalWrite(led_gnd, LOW);
  
  pinMode(buttonLowPin, OUTPUT);
  digitalWrite(buttonLowPin, LOW);
  
  pinMode(buttonHighPin, OUTPUT);
  digitalWrite(buttonHighPin, HIGH);
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);     
  pinMode(GDO0_PIN, INPUT);     

  Serial.println("Starting..");
  init_CC2500();
 // Read_Config_Regs();
  
  // TxData_RF(No_of_Bytes);    //  Transmit No_of_Bytes-1
}

void loop()
{
    
    
    buttonState = digitalRead(buttonPin);
     sensorValue = analogRead(sensorPin) >> 2;  
     analogWrite(led,sensorValue);
     
    
//  To start transmission
   // while (buttonState)
   //   {
        // read the state of the pushbutton value:
        buttonState = digitalRead(buttonPin);
         Serial.println();
        delay(10);
        char buf[5];
        char data[50] = "Hello User. Sensor value is -";
         strcat(data, itoa(sensorValue, buf, 10));
         strcat(data, "-");
        TxData_RF((byte)sensorValue);    //  Transmit data value
         Serial.println((byte)sensorValue);
        delay(100);
   //   }
          
}

void TxData_RF(String data)
{
   unsigned char string_data[data.length()];
   for(int i=0;i<data.length();i++)
   {
     string_data[i]=data[i];
   }
  TxData_RF(string_data, data.length());
}

void TxData_RF(byte data)
{
  byte newdata[1];
  newdata[0]=data;
  TxData_RF(newdata, 1);
}


//  Send slide strobe
void TxData_RF( unsigned char data[], int length)
{
      // Make sure that the radio is in IDLE state before flushing the FIFO
      SendStrobe(CC2500_IDLE);
      // Flush TX FIFO
      SendStrobe(CC2500_FTX);

  
      // SIDLE: exit RX/TX
      SendStrobe(CC2500_IDLE);
       Serial.print("Transmitting: ");
        Serial.write(data, length);
        Serial.println();
      
      //first byte is data length for variable packet length mode
      WriteReg(CC2500_TXFIFO, length);
      
      //load up the rest of the bytes
      for(int i = 0; i < length; i++)
      {	  
          WriteReg(CC2500_TXFIFO, data[i]);
      }
      // STX: enable TX
      SendStrobe(CC2500_TX);
      
      // Wait for GDO0 to be set -> sync transmitted
      while (!GDO0_State)
      {
          // read the state of the GDO0_PIN value:
          GDO0_State = digitalRead(GDO0_PIN);
          //Serial.println("GDO0 = 0");
       }
       
       // Wait for GDO0 to be cleared -> end of packet
       while (GDO0_State)
       {
           // read the state of the GDO0_PIN value:
           GDO0_State = digitalRead(GDO0_PIN);
           //Serial.println("GDO0 = 1");
       }
}// Rf TX Packet 
	

void WriteReg(char addr, char value)
{
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
    };
    
  SPI.transfer(addr);
  delay(10);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
}

char ReadReg(char addr)
{
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);
  delay(10);
  char y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

char SendStrobe(char strobe)
{
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
    };
    
  char result =  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(10);
  return result;
}


void init_CC2500()
{
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
  Serial.println(ReadReg(REG_IOCFG2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
   delay(10);
/* Serial.println(ReadReg(REG_FIFOTHR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_SYNC1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_SYNC0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTLEN),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTCTRL1),HEX);
   delay(10);
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

