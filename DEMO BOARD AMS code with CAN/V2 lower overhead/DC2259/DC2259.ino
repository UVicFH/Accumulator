/*
 * LTC demo board 6811 cell montioring
 * 
 * AMS code
 *  Original demo code supplied by LTC
 *  Highly modified, stripped and custom functinos added
 * 
 * BY: Akshdeep Maan
 */

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include <SPI.h>
#include <avr/wdt.h>
#include "AMS.h"
#include <OneWire.h> 
#include <DallasTemperature.h>

void print_cells();
void check_error(int error);
/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/

//ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //MD_27KHZ_14KHZ;//MD_7KHZ_3KHZ; //MD_26HZ_2KHZ; // See LTC6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; // See LTC6811_daisy.h for Options

/**********************************************************
  Setup Temp Sensors:
  1) Setup a oneWire instance to communicate with any OneWire devices  
     (not just Maxim/Dallas temperature ICs)
  2) Pass our oneWire reference to Dallas Temperature
  3) Addresses of the sensor is set when code finishes setup
***********************************************************/
OneWire oneWire(temp_output_pin);
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1;
DeviceAddress sensor2;

/************************************
  END SETUP
*************************************/

/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the LTC6811
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/

cell_asic bms_ic[TOTAL_IC];

int loopcnt = 0;

/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
  int8_t error = 0;
  delay(1000);
  Serial.begin(115200);
  CANSetup();
  quikeval_SPI_connect();
  //SPI.setDataMode(3);
  spi_enable(SPI_CLOCK_DIV128); // This will set the Linduino to have a 1MHz Clock
  LTC6811_init_cfg(TOTAL_IC, bms_ic);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6811_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,dccBits_a);
  }
  LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
  LTC681x_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC,bms_ic);
  wakeup_sleep(TOTAL_IC);
  //write config
  LTC6811_wrcfg(TOTAL_IC,bms_ic);
  wakeup_idle(TOTAL_IC);
  error = LTC6811_rdcfg(TOTAL_IC,bms_ic);
  check_error(error);
  wakeup_sleep(TOTAL_IC);
  error = LTC6811_rdcfg(TOTAL_IC,bms_ic);
  check_error(error);
  ams_status_setup();
  watchdogSetup();
  sensors.begin(); 
  
  //finds the addresses of the temp sensors sets them in the code
  byte addr[8];
  byte i;
  int sensor_num = 1;
  
  while(oneWire.search(addr)){
    for (i = 0; i < 8; i++) {  
      if(sensor_num == 1){
        sensor1[i] = (uint8_t)addr[i];
      }else if(sensor_num == 2){
        sensor2[i] = (uint8_t)addr[i];
        Serial.println("found them");
      }else{
        if(debugging) {
          Serial.println();
          Serial.println("sum ting wong...shouldnt get here unless there are more sensors hooked up");
        }
      }
    }
    sensor_num++;
  }
}

/*!**********************************************************************
 \brief  Inititializes hardware and variables for watchdog
 ***********************************************************************/
void watchdogSetup(void)
{
  //disables all interrupts on the microcontroller so that configuration is never disrupted and left unfinished.
  cli();
  //reset watchdog timer
  wdt_reset();

/*
 WDTCSR configuration:
 WDIE = 1: Interrupt Enable
 WDE = 1 :Reset Enable
 See table for time-out variations:
 WDP3 = 0 :For 1000ms Time-out
 WDP2 = 1 :For 1000ms Time-out
 WDP1 = 1 :For 1000ms Time-out
 WDP0 = 0 :For 1000ms Time-out
*/
// Enter Watchdog Configuration mode:
WDTCSR |= B00011000;
// Set Watchdog settings:
WDTCSR = B01001110;
//re-enable interrupts
sei();
}

/*!*********************************************************************
  \brief main loop
***********************************************************************/
void loop()
{
  chargebrd = digitalRead(midpack_input_pin);
  //These functions are only valid if charge control is on and fucntioning
  if(chargebrd){
    //If no error and the pack currently being precharged, this will turn on the AIRS.
    if ((millis() > (current_precharge_millis+precharge_time)) && precharged){
      precharged = 0;
      digitalWrite(air_toggle_pin, 1);
      digitalWrite(precharged_pin, 0);
    }

    if( millis() > 3500) {
      //Turn on the mid pack relay
      digitalWrite(midpack_output_pin, 1);
      midpack_status = 1;
    }
  } else {
    //Turn on the mid pack relay
    digitalWrite(midpack_output_pin, 0);
    midpack_status = 0;
  }

//  if( millis() > 2000) {
//      while(true){
//        Serial.println(loopcnt++);
//      }
//  }

  //To account for errors in reading a error counter is updated where ever a error occurs. if the error is deadly, the function would stop ams right there and then
  //Otherwise this counter is used. if it counts too high, we trip the ams otherwise assume a glitch and carry on with our lives.
  if (   abs(millis()-current_error_millis) > error_time_allowed    ){
    if ((error_cnt >= max_error_cnt) && (error_cnt != max_record)) { 
      set_ams_status(false);
      max_record = error_cnt;
      current_error_millis = millis();
    }
    else{
      set_ams_status(true);
      error_cnt = 0;
      max_record = error_cnt;
      current_error_millis = millis();
    }
    Serial.println("er ch");
  }
  
  //start monitoring all of the cells and and sensors, and outputing them over can and signal wires.
  if( abs(millis()-current_millis) > meas_time ){

    //read cell temp
    read_temperature();
    
    //read new cell voltages and actual BMS tasks.
    read_voltages();
    
    //read current sensor/
    read_current();  
    
    //format and send over can
    CAN_send();

    //Serial debugging print statements, can be removed once code is finalized and finished, hahahahahahahahahahahahahahahaha, never gonna happen.
    if(1) {
      Serial.println();
      Serial.print("AMS STAUS: ");
      Serial.print(ams_status);
      Serial.println();
      Serial.print("REG STAUS: ");
      Serial.print(regen_status);
      Serial.println();
      Serial.print("Error count: ");
      Serial.print(error_cnt);
      Serial.println();
      
//      Serial.println();
//      Serial.print("midpack status: ");
//      Serial.print(midpack_status);
//      Serial.println();
//      
      Serial.println();
      Serial.print("pack voltage: ");
      Serial.print(pack_voltage);
      Serial.println();

      Serial.println();
      Serial.print("pack current: ");
      Serial.print(pack_current_draw);
      Serial.println();   

//      Serial.println();
//      Serial.print("charge ctrl: ");
//      Serial.print(digitalRead(midpack_input_pin));
//      Serial.println(); 

      Serial.println();
      Serial.print("Temperature Sensor #1: ");
      Serial.print(temp_sen1);
      Serial.print("Temperature Sensor #2: ");
      Serial.print(temp_sen2);
      Serial.println();

//      Serial.println("Discharging or not:");
//      for(int x=0; x<TOTAL_IC; x++) {
//        for(int i=0; i<12; i++){
//          Serial.print("C");
//          Serial.print(i);
//          Serial.print(": ");
//          Serial.print(cell_discharging[x][i]);
//          Serial.print("  ");
//        }
//      Serial.println();
//      }

    }
    
    //the order of the data being read back seemed to be wrong so this function is used to correct that. it works but not 100% sure exacttly why. just leave it in untill you know for sure.
    for(int x=0; x<TOTAL_IC; x++){
      bms_ic[x].isospi_reverse = 0;
    }

    //Check max cell voltage to determine if regen is allowed or not
    set_regen_status();

    //Update the time
    current_millis = millis();
  }

  //resetting watchdog timer
  wdt_reset();
  
  //balancing the cells is as often as possible since the discharge timmers are not used. this keeps the ltc chips discharging as required.
  balance_cells();
  
  //Processor dead time
  delay(100);

}

/*!*****************************************
  \brief watchdog interrupt can exicute code before resetting
*******************************************/
ISR(WDT_vect)
{
//shouldnt use println because you dont want to any code that could have an error since it could stop the watchdog from resetting
//recommended to set a flag on the arduino but im not sure which one 
//Serial.println("watchdog has reset arduino*****************************************************");
}


/*!************************************************************
  \brief Prints cell voltage codes to the serial port
 *************************************************************/
void print_cells()
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
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
}

//Function to check error flag and print PEC error message
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
    error_cnt++;
  }
}

//*****************************************************************************************************************
//                                     CODE FOR UVIC HYBRID AMS                                                    
//*****************************************************************************************************************
void ams_status_setup() {
  pinMode(AMS_STATUS_PIN, OUTPUT);
  pinMode(precharged_pin, OUTPUT);
  pinMode(air_toggle_pin, OUTPUT);
  pinMode(midpack_input_pin, INPUT);
  pinMode(midpack_output_pin, OUTPUT);
  pinMode(pack_current_in_pin, INPUT);
}

void set_ams_status(bool val) {
  if(DEAD_charge){
    ams_status = 1;
    digitalWrite(AMS_STATUS_PIN, 1);
    if(!AIRS_state && chargebrd ) {
      precharged = 1;
      digitalWrite(precharged_pin, precharged);
      current_precharge_millis = millis();
    }
  }else{
    //Start precharging to turn pack on if no error
    if( val && !AIRS_state && chargebrd ) {
      precharged = 1;
      digitalWrite(precharged_pin, precharged);
      current_precharge_millis = millis();
      AIRS_state = 1; 
    }
    //toggle air status with current status
    ams_status = val; // AMS Fault for Can
    digitalWrite(AMS_STATUS_PIN, val); // AMS Fault for RPDU
    //turn off pack if error
    if(!ams_status) {
      digitalWrite(air_toggle_pin, val);
      AIRS_state = val;
    }
  }
};

void set_regen_status(){
  // Set regen status based on whether we exceed soft upper limit  -This is only for the mabx and motor controller since it just needs to not trip ams fault.
  if(max_cell_voltage < cell_soft_upper_limit){
     regen_status = true;
  } else {
     regen_status = false;
  } 
}

// CAN Setup
void CANSetup() {
  for(;;)
  {
    if(CAN_OK == CAN.begin(CAN_500KBPS))
    {
      //Serial.println("CAN BUS INIT GOOD");
      break;
    }
    else
    {
      //Serial.println("CAN BUS INIT FAIL, RETRY");
      delay(100);
    }
  }
  
};

void CAN_send() {
  //packet 0x69
  uint8_t data_msg[8];
  // pack voltage 2bytes 100*voltage
  data_msg[0] = (int)(pack_voltage*100)>>8;
  data_msg[1] = (int)(pack_voltage*100);
  // pack temp    2bytes 100*temp
  data_msg[2] = (int)(((temp_sen1+temp_sen2)/2)*100) >>8;
  data_msg[3] = (int)(((temp_sen1+temp_sen2)/2)*100);
  // AMS status   0x01 bit
  // REGEN status 0x02 bit
  data_msg[4] = ams_status | regen_status<<1;
  //TS current    2bytes 100*amps
  data_msg[5] = (int)(pack_current_draw*100)>>8;
  data_msg[6] = (int)(pack_current_draw*100);

  //packet 0x70
  uint8_t cell_msg[8];
  //cell max num   1bytes cell#
  cell_msg[0] = max_cell_num;
  //cell volts max 2bytes 1000*votls
  cell_msg[1] = (int)(max_cell_voltage*1000) >>8;
  cell_msg[2] = (int)(max_cell_voltage*1000);
  //cell min num   1bytes cell#
  cell_msg[3] = min_cell_num;
  //cell volts min 2bytes 1000*votls
  cell_msg[4] = (int)(min_cell_voltage*1000)>>8;
  cell_msg[5] = (int)(min_cell_voltage*1000);

  
  CAN.sendMsgBuf(AMS_CAN_ID_data, 0, 8, data_msg);
  delay(50);
  CAN.sendMsgBuf(AMS_CAN_ID_cell, 0, 8, cell_msg);
  delay(50);
};

void balance_cells() {
  //clear the last set discharge bits
  clear_discharge(TOTAL_IC,bms_ic);
  bool cell[12];
  
  //This sorts the bytes into which cell is over voltage, and if so sets the discharge bit on, in the config register
  for(int each_ic=0; each_ic<TOTAL_IC; each_ic++){
    for(int i=0; i<12; i++){
      //This checks if any cell above upper limit and discharges it
      if (cell_data[(each_ic*12)+i+1] >  cell_soft_upper_limit) {
        cell[i] = 1;
        cell_discharging[each_ic][i] = 1;
      }
      else {
        cell[i] = 0;
        cell_discharging[each_ic][i] = 0;
      }

    //This checks if any cell to too far above the others and discharges it
    if (    (cell_data[(each_ic*12)+i+1] >  (min_cell_voltage+cell_balance_window)) && (cell_data[(each_ic*12)+i+1] > low_safety_cell_v)    ) {
        cell[i] = 1;
        cell_discharging[each_ic][i] = 1;
      }
      
    }
    LTC681x_set_cfgr_dis(each_ic, bms_ic, cell);
  } 

  //Write the config to the chip
  //wake up ltc chip
  wakeup_sleep(TOTAL_IC);
  //write config
  LTC6811_wrcfg(TOTAL_IC,bms_ic);
  
};

//This is based on the current sesnor in the AMS box. it outputs a voltage level 0-5V based on the current. there are two chanels, we use the bigger one, duh.
//The formula is very simple, just take the read value on adc, convert to voltage, and feed to formula. look at datasheet for more details.
void read_current() {
  int adc_in = analogRead(pack_current_in_pin);
  float adc_voltage = adc_in/1024.0*5.0;
  pack_current_draw = (adc_voltage-2.5)*(1.0/0.004) + 1.22;  //This formula is from the datasheet for current sensor i=(v-(Vsupply/2))*(1/g)*(5/Vsupply), the 1.22 is just an offset value
};

void read_temperature(){
  sensors.begin();
  sensors.requestTemperatures(); // Send the command to get temperature readings
  temp_sen1 = sensors.getTempC(sensor1);
  temp_sen2 = sensors.getTempC(sensor2);
  if(( temp_sen1 || temp_sen2) > max_cell_temp) error_cnt++;
  if(( temp_sen1 || temp_sen2) < min_cell_temp) error_cnt++;
  
};

void read_voltages() {
  int8_t error = 0;
  uint32_t conv_time = 0;
  spi_enable(SPI_CLOCK_DIV128);

  //wakeup_sleep(TOTAL_IC);
  //LTC6811_wrcfg(TOTAL_IC,bms_ic);

  // ADC Cell Measurement and conversion
  wakeup_sleep(TOTAL_IC);
  wakeup_idle(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
  wakeup_sleep(TOTAL_IC);
  wakeup_idle(TOTAL_IC);
  conv_time = LTC6811_pollAdc();
  
  if(debugging){
    Serial.print(F("cell conversion completed in:"));
    Serial.print(((float)conv_time/1000), 1);
    Serial.println(F("mS"));
    Serial.println();
  }
 
  // Store cell voltages in cell_data array
  wakeup_sleep(TOTAL_IC);
  wakeup_idle(TOTAL_IC);
  error = LTC6811_rdcv(0, TOTAL_IC,bms_ic); // Set to read back all cell voltage registers
  check_error(error);
  print_cells();

  //re order the cell voltages into another array to ease of use.
  for(int each_ic=0; each_ic<TOTAL_IC; each_ic++){
    for(int i=0; i<12; i++){
      cell_data[(each_ic*12)+i+1] =  bms_ic[each_ic].cells.c_codes[i]*0.0001,4;
    }
  }

  // Calulate Pack Voltage, minimum voltage and max cell voltage
  pack_voltage = 0;
  min_cell_voltage = 10;
  max_cell_voltage = 0;
  
  for(int i=1; i<=total_cells; i++){
    pack_voltage += cell_data[i];
    if(cell_data[i] > max_cell_voltage) {
      // Record highest cell voltage
      max_cell_voltage = cell_data[i];
      max_cell_num = i;
    }
    if(cell_data[i] < min_cell_voltage) {
      // Record lowest cell voltage
      min_cell_voltage = cell_data[i];
      min_cell_num = i;
    }
    if((cell_data[i] >= cell_hard_upper_limit) && (cell_data[i] <= 6.0)) {
      //add an error count  
      error_cnt++;      
    }
    if((cell_data[i] <= cell_under_limit) && (i == 24)    ) {
      //add an error count
      error_cnt++;    
    } 
//    if ((cell_data[i] <= cell_under_limit) && (i == 12)    ){
//      cell_data[i] = cell_data[i]+1.2; 
//    } 
//    if (cell_data[i] <= cell_under_limit) {
//      error_cnt++; 
//    }
  }

};
