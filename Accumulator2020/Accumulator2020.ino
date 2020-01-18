  /*!

Accumulator code for 2020 as developed by Chad McColm. Source originally from demo files provided by LTC.

*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include <mcp_can.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <OneWire.h> 
#include <DallasTemperature.h>

// Define enabled and disabled to use rather than 1 and 0 to make things more easy to read
#define ENABLED 1
#define DISABLED 0

// Function List
void readVoltages();
void setStatuses();
void balanceCells();
void readCurrent();
void sendCAN();
void readTemperatures();
void printCells(uint8_t datalog_en);

// Configuration parameters for ICs and ADCs
const uint8_t TOTAL_IC = 3;// Number of ICs in the daisy chain
const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;//MD_7KHZ_3KHZ; //MD_26HZ_2KHZ;//MD_7KHZ_3KHZ; // See LTC6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See LTC6811_daisy.h for Options

// Define CAN message IDs
#define AMS_CAN_ID_data 105
#define AMS_CAN_ID_cell 112

// Define Hybrid master GPIO pins
#define AMS_STATUS_PIN 7        // AMS Status (ENABLED is good)
#define precharge_pin 6         // Starts the precharge of the AIRs
#define air_toggle_pin 2        // Turns on AIRs if ENABLED
#define midpack_input_pin 3     // Input for if TS is switched on
#define midpack_output_pin 4    // When ENABLED, pack may be charged
#define temp_output_pin 5       //output of temp sensor reading
#define pack_current_in_pin A1  // Current sensor input
#define SPI_CS_PIN 9            // Chip select pin of CAN shield

// Define cell limits
#define hardUpperLimit 2.7
#define softUpperLimit 2.6

// Configuration of the temperature sensors
OneWire oneWire(temp_output_pin);
DallasTemperature sensors(&oneWire);

// For CAN initialization
MCP_CAN CAN(SPI_CS_PIN);

// Timers for the events
unsigned long lastReadVoltages = 0;
unsigned long lastReadTemperatures = 0;
unsigned long lastSetStatus = 0;
unsigned long lastReadCurrent = 0;
unsigned long lastBalanceCells = 0;
unsigned long lastSendCAN = 0;
unsigned long startPrecharge = 0;
int prechargeDuration = 4000;

// Period at which to run the main loop events in milliseconds
int readVoltagesPeriod = 100;
int readTemperaturesPeriod = 1000;
int setStatusPeriod = 1000;
int readCurrentPeriod = 50;
int balanceCellsPeriod = 15;
int sendCANPeriod = 150;

// Global battery variables received from stack
cell_asic bms_ic[TOTAL_IC];

//Different states of accumulator isolation relays
typedef enum
{
  OffState,
  PrechargeState,
  OnState
  
}AIRState;

// Global variables for tracking accumulator condition
AIRState nextAIRState = OffState; // Set the default state of AIRs to Off
int AMSStatus = DISABLED;
int regenStatus = DISABLED;
int minCellNumber;
float minCellVoltage;
int maxCellNumber;
float maxCellVoltage;
float packVoltage;
float packCurrent;
float packTemperature1;
float packTemperature2;

void setup(){
  
  // Begin serial communications
  Serial.begin(115200);

  // Start the Dallas Temperature sensors library
  sensors.begin();
  
  // Enable the watchdog with a 4 second overflow
  wdt_enable(WDTO_4S);

  // Set up the CAN
  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
      Serial.println("CAN BUS Shield init fail");
      Serial.println(" Init CAN BUS Shield again");
      delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");

  // Make place for QuickEval software to connect
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock

  // Generate and write a base configuration to chips
  LTC681x_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC,bms_ic);
  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC,bms_ic);

  // Set pinmodes for Hybrid controlled things
  pinMode(AMS_STATUS_PIN, OUTPUT);
  pinMode(precharge_pin, OUTPUT);
  pinMode(air_toggle_pin, OUTPUT);
  pinMode(midpack_input_pin, INPUT);
  pinMode(midpack_output_pin, OUTPUT);
  pinMode(pack_current_in_pin, INPUT);
  
}

void loop(){

  unsigned long currentLoopTime = millis();

  // Read the voltages
  if(currentLoopTime > lastReadVoltages + readVoltagesPeriod){
    readVoltages();
//    Serial.print("Max Cell Voltage: \t");
//    Serial.println(maxCellVoltage, 4);
//    Serial.print("Min Cell Voltage: \t");
//    Serial.println(minCellVoltage, 4);
//    Serial.print("Max Cell Number: \t");
//    Serial.println(maxCellNumber);
//    Serial.print("Min Cell Number: \t");
//    Serial.println(minCellNumber);
//    Serial.print("Pack Voltage: \t");
//    Serial.println(packVoltage, 4);
//    printCells(DISABLED);
    lastReadVoltages = currentLoopTime;
  }
  
  // Read the temperatures
  if(currentLoopTime > lastReadTemperatures + readTemperaturesPeriod){
    readTemperatures();
    lastReadTemperatures = currentLoopTime;
  }

  // Set the pack statuses
  if(currentLoopTime > lastSetStatus + setStatusPeriod){
    setStatuses();
    lastSetStatus = currentLoopTime;
  }

  // Read the current
  if(currentLoopTime > lastReadCurrent + readCurrentPeriod){
    readCurrent();
    lastReadCurrent = currentLoopTime;
  }

  // Balance the cells
  if(currentLoopTime > lastBalanceCells + balanceCellsPeriod){
    //balanceCells();
    lastBalanceCells = currentLoopTime;
  }

  // Send CAN
  if(currentLoopTime > lastSendCAN + sendCANPeriod){
    sendCAN();
    lastSendCAN = currentLoopTime;
  }

  // Run the finite state machine for the Precharge/AIR states
  switch (nextAIRState){
    case OffState:
      
      // Write the pins
      digitalWrite(precharge_pin, DISABLED);
      digitalWrite(air_toggle_pin, DISABLED);
  
      // Transition to precharge state if AMS good
      if(AMSStatus == ENABLED && digitalRead(midpack_input_pin)){
        nextAIRState = PrechargeState;
        startPrecharge = millis();
      }

      break;
      
    case PrechargeState:
    
      // Write the pins
      digitalWrite(precharge_pin, ENABLED);
      digitalWrite(air_toggle_pin, DISABLED);

      // Transition to on state if precharge timer complete
      if(millis() - startPrecharge > prechargeDuration && digitalRead(midpack_input_pin)) nextAIRState = OnState;

      // Transition to off state if AMS bad
      if(AMSStatus == DISABLED || !digitalRead(midpack_input_pin)) nextAIRState = OffState;
      
      break;
      
    case OnState:
      
      // Write the pins
      digitalWrite(precharge_pin, DISABLED);
      digitalWrite(air_toggle_pin, ENABLED);

      // Transition to off state if AMS bad
      if(AMSStatus == DISABLED || !digitalRead(midpack_input_pin)) nextAIRState = OffState;
      
      break; 
      
  }

  // Manage the midpack relay based on TSV switch
  if(digitalRead(midpack_input_pin)) digitalWrite(midpack_output_pin, ENABLED);
  else digitalWrite(midpack_output_pin, DISABLED);

  // Reset the watchdog timer
  wdt_reset();
  
}

void readVoltages(){
  
  // Wakup the boards and read the cells to slaves
  wakeup_idle(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
  LTC6811_pollAdc();
  
  // Make sure boards still awake and read the slaves to master
  wakeup_idle(TOTAL_IC);
  int8_t error = 0;
  error = LTC6811_rdcv(0, TOTAL_IC,bms_ic);

  // Loop through the cells to determine the pack voltage, min cell voltage, and max cell voltage
  packVoltage = 0;
  minCellVoltage = 100;
  minCellNumber = 0;
  maxCellVoltage = 0;
  maxCellNumber = 0;
  int currentCellNumber = 1;
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){
    for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++){
        
        float currentCellVoltage = bms_ic[current_ic].cells.c_codes[i]*0.0001;
        if(currentCellVoltage < minCellVoltage){
          minCellVoltage = currentCellVoltage;
          minCellNumber = currentCellNumber;
        }
        if(currentCellVoltage > maxCellVoltage){
          maxCellVoltage = currentCellVoltage;
          maxCellNumber = currentCellNumber;
        }
        packVoltage += currentCellVoltage;
        currentCellNumber++;
        
      }
  }
  
}

void setStatuses(){

  // if highest cell has exceeded hard upper limit, shut down the pack
  if(maxCellVoltage > hardUpperLimit) AMSStatus = DISABLED;
  else AMSStatus = ENABLED;
  digitalWrite(AMS_STATUS_PIN, AMSStatus);
  
  // if highest cell has exceeded soft upper limit, stop regen
  if(maxCellVoltage > softUpperLimit) regenStatus = DISABLED;
  else regenStatus = ENABLED;
  
}

void balanceCells() {
  
  //clear the last set discharge bits
  clear_discharge(TOTAL_IC,bms_ic);
  
  // Go through every IC
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){

    // Create a place for the discharge flags and go through each cell on that IC
    bool dischargeMe[12];
    for (int i=0; i<12; i++){

      // Get the cell voltage and check against the upper limit and the distance from miminum to see if discharging necessary
      float currentCellVoltage = bms_ic[current_ic].cells.c_codes[i]*0.0001;
      if(currentCellVoltage > (softUpperLimit + hardUpperLimit)/2 || currentCellVoltage > minCellVoltage + 0.1) dischargeMe[i] = 1;
      else dischargeMe[i] = 0;
      
    }

    // Sets discharge registers
    LTC6811_set_cfgr_dis(current_ic, bms_ic, dischargeMe);
    
  }

  //Write the config to the chip
  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC,bms_ic);
  
}

// Based on the datasheet for the current sensor in the accumulator
void readCurrent() {
  
  int adc_in = analogRead(pack_current_in_pin);
  float adc_voltage = adc_in/1024.0*5.0;
  packCurrent = (adc_voltage-2.5)*(1.0/0.004) + 1.22;  //This formula is from the datasheet for current sensor i=(v-(Vsupply/2))*(1/g)*(5/Vsupply), the 1.22 is just an offset value
  
}

// Send out pack data over CAN bus
void sendCAN() {

  // Output values to command line
  Serial.print("Voltage: ");
  Serial.print(packVoltage);
  Serial.print("\t Temperature 1: ");
  Serial.print(packTemperature1);
  Serial.print("\t Temperature 2: ");
  Serial.print(packTemperature2);
  Serial.print("\t Pack Current: ");
  Serial.println(packCurrent);
  
  // Structure the data message:
  uint8_t data[8];
  // Send 100*packVoltage in two bytes as an integer
  data[0] = (int)(packVoltage*100) >> 8;
  data[1] = (int)(packVoltage*100);
  // Send 100*packTemperature in two bytes as an integer
  data[2] = (int)(packTemperature1);
  data[3] = (int)(packTemperature2);
  // AMS status   0b01 bit
  // REGEN status 0b10 bit
  data[4] = AMSStatus | regenStatus << 1;
  //TS current    2bytes 100*amps
  data[5] = (int)(packCurrent*100) >> 8;
  data[6] = (int)(packCurrent*100);

  // Structure the data message:
  uint8_t data2[8];
  // Max Cell Number
  data2[0] = maxCellNumber;
  // Max Cell Voltage * 1000 in 2 bytes
  data2[1] = (int)(maxCellVoltage*1000) >> 8;
  data2[2] = (int)(maxCellVoltage*1000);
  // Min Cell Number
  data2[3] = minCellNumber;
  // Min Cell Voltage * 1000 in 2 bytes
  data2[4] = (int)(minCellVoltage*1000) >> 8;
  data2[5] = (int)(minCellVoltage*1000);

  
  CAN.sendMsgBuf(AMS_CAN_ID_data, 0, 8, data);
  CAN.sendMsgBuf(AMS_CAN_ID_cell, 0, 8, data2);

  spi_enable(SPI_CLOCK_DIV16); // Restores the required SPI clock speed
  
}

// Read temperatures from Dallas one wire temperature sensors
void readTemperatures(){
  
  packTemperature1 = sensors.getTempCByIndex(0);
  packTemperature2 = sensors.getTempCByIndex(1);
  
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
 *************************************************************/
void printCells(uint8_t datalog_en)
{

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      Serial.print(", ");
      for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++)
      {

        Serial.print(" C");
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].cells.c_codes[i]*0.0001,4);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {
      Serial.print("Cells, ");
      for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++)
      {
        Serial.print(bms_ic[current_ic].cells.c_codes[i]*0.0001,4);
        Serial.print(",");
      }

    }
  }
  Serial.println();
}
