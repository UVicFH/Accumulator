/*!
Linear Technology DC2259 Demonstration Board
LTC6811-1: Battery stack monitor


@verbatim

NOTES
 Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.
   Ensure all jumpers on the demo board are installed in their default positions from the factory.
   Refer to Demo Manual DC2259.

USER INPUT DATA FORMAT:
 decimal : 1024
 hex     : 0x400
 octal   : 02000  (leading 0)
 binary  : B10000000000
 float   : 1024.0
@endverbatim

http://www.linear.com/product/LTC6811-1

http://www.linear.com/product/LTC6811-1#demoboards

Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright 2017 Linear Technology Corp. (LTC)
 */


/*! @file
    @ingroup LTC6811-1
*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include <SPI.h>

#include "AMS.h"

#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

char get_char();
void print_menu();
void read_config_data(uint8_t cfg_data[][6], uint8_t nIC);
void print_cells(uint8_t datalog_en);
void print_open();
void print_config();
void print_rxconfig();
void print_aux(uint8_t datalog_en);
void print_stat();
void check_error(int error);
/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/

//ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;//MD_7KHZ_3KHZ; //MD_26HZ_2KHZ;//MD_7KHZ_3KHZ; // See LTC6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; // See LTC6811_daisy.h for Options

const uint16_t MEASUREMENT_LOOP_TIME = 500;//milliseconds(mS)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; // Over voltage threshold ADC Code. LSB = 0.0001
const uint16_t UV_THRESHOLD = 30000; // Under voltage threshold ADC Code. LSB = 0.0001

//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_STAT = DISABLED; //This is ENABLED or DISABLED
const uint8_t PRINT_PEC = DISABLED; //This is ENABLED or DISABLED
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


//change to 1 to export values to excel 
bool export_excel = 1;


/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
  Serial.begin(115200);
  //CANSetup();
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV128); // This will set the Linduino to have a 1MHz Clock
  LTC681x_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC,bms_ic);
  ams_status_setup();
  //print_menu();
}

/*!*********************************************************************
  \brief main loop
***********************************************************************/
void loop()
{
  
  
  //on startup, enable the ams box and output voltage since the only reason to start tsms is to use the ams.
  if (millis() > 5000 && !precharged){
    precharged = 1;
    digitalWrite(precharged_pin, precharged);
    digitalWrite(air_toggle_pin, 1);
  }
 
  //To account for errors in reading a error counter is updated where ever a error occurs. if the error is deadly, the function would stop ams right there and then
    //Otherwise this counter is used. if it counts too high, we trip the ams otherwise assume a glitch and carry on with our lives.
    if (   abs(millis()-current_error_millis) > error_time_allowed    ){
      if (error_cnt >= max_error_cnt) { 
        set_ams_status(false);
      }
      else{
        error_cnt = 0;
        current_error_millis = millis();
      }
      
    }

    
  //start monitoring all of the cells and and sensors, and outputing them over can and signal wires.
  if( abs(millis()-current_millis) > meas_time ){
    
    //read new cell voltages and actual BMS tasks.
    read_voltages(); 
      
    //read current sensor
    read_current();  
    
    //format and send over can
    CAN_send();
    
    //exporting to excel 
    if(export_excel){
      
      //printing values as stated in the print statement
      Serial.print((String) millis() +  ams_status+" , "+ regen_status+" , "+ error_cnt+" , "+ midpack_status+" , "+pack_current_draw +" , ");
    
      //printing discharge values
      for(int x=0; x<TOTAL_IC; x++) {
        for(int i=0; i<12; i++){
          Serial.print(cell_discharging[x][i]);
          Serial.print(" , ");
        }
      }
    
      //printing cell voltages
      for(int each_ic=0; each_ic<TOTAL_IC; each_ic++){
        for(int i=0; i<12; i++){
          cell_data[(each_ic*12)+i+1] =  bms_ic[each_ic].cells.c_codes[i]*0.0001,4;
          Serial.print(bms_ic[each_ic].cells.c_codes[i]*0.0001,4);
          Serial.print(" , ");
        }
      }
      Serial.println("");    
    }else{
      //Serial debugging print statements, can be removed once code is finalized and finished, hahahahahahahahahahahahahahahaha, never gonna happen.
      Serial.println();
      Serial.print("AMS STATUS: ");
      Serial.print(ams_status);
      Serial.println();
      Serial.print("REG STATUS: ");
      Serial.print(regen_status);
      Serial.println();
      Serial.print("Error count: ");
      Serial.print(error_cnt);
      Serial.println();
    
      Serial.println();
      Serial.print("midpack status: ");
      Serial.print(midpack_status);
      Serial.println();
    
      Serial.println();
      Serial.print("pack voltage: ");
      Serial.print(pack_voltage);
      Serial.println();

      Serial.println();
      Serial.print("pack current: ");
      Serial.print(pack_current_draw);
      Serial.println();
    
      print_discharge_status();
    }

      
    
    
    //the order of the data being read back seemed to be wrong so this function is used to correct that. it works but not 100% sure exacttly why. just leave it in untill you know for sure.
    for(int x=0; x<TOTAL_IC; x++){
      bms_ic[x].isospi_reverse = 0;
    }
   
    current_millis = millis();
  }

  //balancing the cells is as often as possible since the discharge timmers are not used. this keeps the ltc chips discharging as required.
  balance_cells();
  delay(100);

}


/*!*****************************************
  \brief executes the user command
*******************************************/

void run_command(uint32_t cmd)
{
  int8_t error = 0;
  uint32_t conv_time = 0;
  uint32_t user_command;
  int8_t readIC=0;
  char input = 0;
  switch (cmd)
  {

    case 1: // Write Configuration Register
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC,bms_ic);
      print_config();
      break;

    case 2: // Read Configuration Register
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC,bms_ic);
      check_error(error);
      print_rxconfig();
      break;

    case 3: // Start Cell ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      if(!export_excel){
        Serial.print(F("cell conversion completed in:"));
        Serial.print(((float)conv_time/1000), 1);
        Serial.println(F("mS"));
        Serial.println();
      }
      break;

    case 4: // Read Cell Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdcv(0, TOTAL_IC,bms_ic); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      break;

    case 5: // Start GPIO ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_TO_CONVERT);
      LTC6811_pollAdc();
      Serial.println(F("aux conversion completed"));
      Serial.println();
      break;

    case 6: // Read AUX Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdaux(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
      check_error(error);
      print_aux(DATALOG_DISABLED);
      break;

    case 7: // Start Status ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
      LTC6811_pollAdc();
      Serial.println(F("stat conversion completed"));
      Serial.println();
      break;

    case 8: // Read Status registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdstat(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
      check_error(error);
      print_stat();
      break;

    case 9: // Loop Measurements
      Serial.println(F("transmit 'm' to quit"));
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC,bms_ic);
      while (input != 'm')
      {
        if (Serial.available() > 0)
        {
          input = read_char();
        }

        measurement_loop(DATALOG_DISABLED);

        delay(MEASUREMENT_LOOP_TIME);
      }
      //print_menu();
      break;

    case 10: // Run open wire self test
      print_pec();

      break;

    case 11: // Read in raw configuration data
      LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
      break;

    case 12:  // Run the ADC/Memory Self Test
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(CELL,ADC_CONVERSION_MODE,bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and CELL Memory \n"));

      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(AUX,ADC_CONVERSION_MODE, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and AUX Memory \n"));

      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(STAT,ADC_CONVERSION_MODE, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and STAT Memory \n"));
      print_menu();
      break;

    case 13: // Enable a discharge transistor
      Serial.println(F("Please enter the Spin number"));
      readIC = (int8_t)read_int();
      LTC6811_set_discharge(readIC,TOTAL_IC,bms_ic);
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC,bms_ic);
      print_config();
      break;

    case 14: // Clear all discharge transistors
      clear_discharge(TOTAL_IC,bms_ic);
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC,bms_ic);
      print_config();
      break;

    case 15: // Clear all ADC measurement registers
      wakeup_sleep(TOTAL_IC);
      LTC6811_clrcell();
      LTC6811_clraux();
      LTC6811_clrstat();
      Serial.println(F("All Registers Cleared"));
      break;

    case 16: // Run the Mux Decoder Self Test
      wakeup_sleep(TOTAL_IC);
      LTC6811_diagn();
      delay(5);
      error = LTC6811_rdstat(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
      check_error(error);
      error = 0;
      for (int ic = 0; ic<TOTAL_IC; ic++)
      {
        if (bms_ic[ic].stat.mux_fail[0] != 0) error++;
      }
      if (error==0) Serial.println(F("Mux Test: PASS "));
      else Serial.println(F("Mux Test: FAIL "));

      break;

    case 17: // Run ADC Overlap self test
      wakeup_sleep(TOTAL_IC);
      error = (int8_t)LTC6811_run_adc_overlap(TOTAL_IC,bms_ic);
      if (error==0) Serial.println(F("Overlap Test: PASS "));
      else Serial.println(F("Overlap Test: FAIL"));
      break;

    case 18: // Run ADC Redundancy self test
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE,AUX,TOTAL_IC, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in AUX Measurement \n"));

      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE,STAT,TOTAL_IC, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in STAT Measurement \n"));
      break;

    case 19:
      LTC6811_run_openwire(TOTAL_IC, bms_ic);
      print_open();
      break;

    case 20: //Datalog print option Loop Measurements
      Serial.println(F("transmit 'm' to quit"));
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC,bms_ic);
      while (input != 'm')
      {
        if (Serial.available() > 0)
        {
          input = read_char();
        }

        measurement_loop(DATALOG_ENABLED);

        delay(MEASUREMENT_LOOP_TIME);
      }
      print_menu();
      break;

    case 'm': //prints menu
      print_menu();
      break;

    default:
      Serial.println(F("Incorrect Option"));
      break;
  }
}

void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  if (WRITE_CONFIG == ENABLED)
  {
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcfg(TOTAL_IC,bms_ic);
    print_config();
  }

  if (READ_CONFIG == ENABLED)
  {
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_rdcfg(TOTAL_IC,bms_ic);
    check_error(error);
    print_rxconfig();
  }

  if (MEASURE_CELL == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcv(0, TOTAL_IC,bms_ic);
    check_error(error);
    print_cells(datalog_en);

  }

  if (MEASURE_AUX == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdaux(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
    check_error(error);
    print_aux(datalog_en);
  }

  if (MEASURE_STAT == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdstat(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
    check_error(error);
    print_stat();
  }

  if (PRINT_PEC == ENABLED)
  {
    print_pec();
  }

}


/*!*********************************
  \brief Prints the main menu
***********************************/
void print_menu()
{
  Serial.println(F("Please enter LTC6811 Command"));
  Serial.println(F("Write Configuration: 1            | Reset PEC Counter: 11 "));
  Serial.println(F("Read Configuration: 2             | Run ADC Self Test: 12"));
  Serial.println(F("Start Cell Voltage Conversion: 3  | Set Discharge: 13"));
  Serial.println(F("Read Cell Voltages: 4             | Clear Discharge: 14"));
  Serial.println(F("Start Aux Voltage Conversion: 5   | Clear Registers: 15"));
  Serial.println(F("Read Aux Voltages: 6              | Run Mux Self Test: 16"));
  Serial.println(F("Start Stat Voltage Conversion: 7  | Run ADC overlap Test: 17"));
  Serial.println(F("Read Stat Voltages: 8             | Run Digital Redundancy Test: 18"));
  Serial.println(F("loop Measurements: 9              | Run Open Wire Test: 19"));
  Serial.println(F("Read PEC Errors: 10               |  Loop measurements with datalog output: 20"));
  Serial.println();
  Serial.println(F("Please enter command: "));
  Serial.println();
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
if(!export_excel){

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
}

/*!****************************************************************************
  \brief Prints Open wire test results to the serial port
 *****************************************************************************/
void print_open()
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (bms_ic[current_ic].system_open_wire == 0)
    {
      Serial.print("No Opens Detected on IC: ");
      Serial.print(current_ic+1, DEC);
      Serial.println();
    }
    else
    {
      for (int cell=0; cell<bms_ic[0].ic_reg.cell_channels+1; cell++)
      {
        if ((bms_ic[current_ic].system_open_wire &(1<<cell))>0)
        {
          Serial.print(F("There is an open wire on IC: "));
          Serial.print(current_ic + 1,DEC);
          Serial.print(F(" Channel: "));
          Serial.println(cell,DEC);
        }
      }
    }
  }
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      for (int i=0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(bms_ic[current_ic].aux.a_codes[5]*0.0001,4);
      Serial.println();
    }
    else
    {
      Serial.print("AUX, ");

      for (int i=0; i < 6; i++)
      {
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_stat()
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(" SOC:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[0]*0.0001*20,4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[1]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[2]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[3]*0.0001,4);
    Serial.println();
  }

  Serial.println();
}

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6811
 to the serial port.
 ********************************************************************************/
void print_config()
{
  int cfg_pec;

  Serial.println(F("Written Configuration: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6,&bms_ic[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
  }
  Serial.println();
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6811 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
  Serial.println(F("Received Configuration "));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}

void print_pec()
{
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.println("");
    Serial.print(bms_ic[current_ic].crc_count.pec_count,DEC);
    Serial.print(F(" : PEC Errors Detected on IC"));
    Serial.println(current_ic+1,DEC);
  }
}


void serial_print_hex(uint8_t data)
{
  if (data< 16)
  {
    Serial.print("0");
    Serial.print((byte)data,HEX);
  }
  else
    Serial.print((byte)data,HEX);
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


// hex conversion constants
char hex_digits[16]=
{
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

// global variables

char hex_to_byte_buffer[5]=
{
  '0', 'x', '0', '0', '\0'
};               // buffer for ASCII hex to byte conversion
char byte_to_hex_buffer[3]=
{
  '\0','\0','\0'
};

char read_hex()
// read 2 hex characters from the serial buffer and convert
// them to a byte
{
  byte data;
  hex_to_byte_buffer[2]=get_char();
  hex_to_byte_buffer[3]=get_char();
  get_char();
  get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return(data);
}

char get_char()
{
  // read a command from the serial port
  while (Serial.available() <= 0);
  return(Serial.read());
}

//*****************************************************************************************************************
//                                     CODE FOR UVIC HYBRID AMS                                                    
//*****************************************************************************************************************
void ams_status_setup() {
  pinMode(AMS_STATUS_PIN, OUTPUT);
  pinMode(precharged_pin, INPUT);
  pinMode(air_toggle_pin, OUTPUT);
  pinMode(midpack_input_pin, INPUT);
  pinMode(midpack_output_pin, OUTPUT);
  pinMode(pack_current_in_pin, INPUT);
  digitalWrite(AMS_STATUS_PIN, HIGH);
  
}

void set_ams_status(bool val) {
  ams_status = val; // AMS Fault for Can
  digitalWrite(AMS_STATUS_PIN, val); // AMS Fault for RPDU
  digitalWrite(air_toggle_pin, val);
};



// CAN Setup
void CANSetup() {
  for(;;)
  {
    if(CAN_OK == CAN.begin(CAN_500KBPS))
    {
      Serial.println("CAN BUS INIT GOOD");
      break;
    }
    else
    {
      Serial.println("CAN BUS INIT FAIL, RETRY");
      delay(100);
    }
  }
  
};

void CAN_send() {
  
  //packet 0x51
  uint8_t data_msg[8];
  // pack voltage 2bytes 100*voltage
  data_msg[0] = (int)(pack_voltage*100)>>8;
  data_msg[1] = (int)(pack_voltage*100);
  // pack temp    2bytes 100*temp
  data_msg[2] = (int)((temp_sen1+temp_sen2)*100) >>8;
  data_msg[3] = (int)((temp_sen1+temp_sen2)*100);
  // AMS status   0x01 bit
  // REGEN status 0x02 bit
  data_msg[4] = ams_status | regen_status<<1;
  //TS current    2bytes 100*amps
  data_msg[5] = (int)(pack_current_draw*100)>>8;
  data_msg[6] = (int)(pack_current_draw*100);

  //packet 0x52
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
  delay(10);
  CAN.sendMsgBuf(AMS_CAN_ID_cell, 0, 8, cell_msg);
  spi_enable(SPI_CLOCK_DIV128);
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
    if (    (cell_data[(each_ic*12)+i+1] >  (min_cell_voltage+cell_balance_window)) && (min_cell_voltage > low_safety_cell_v)    ) {
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

//This is a debugging functions, it tells us which cell if any is currenttly being discharged. !!! WARNING, discharge led may not be lit on ltc board due to low voltages, KEEP IN MIND WHEN DEBUGGING !!!
void print_discharge_status() {
  Serial.println("Discharging or not:");
    for(int x=0; x<TOTAL_IC; x++) {
      for(int i=0; i<12; i++){
        Serial.print("C");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(cell_discharging[x][i]);
        Serial.print("  ");
      }
      Serial.println();
    }

};

//This is based on the current sesnor in the AMS box. it outputs a voltage level 0-5V based on the current. there are two chanels, we use the bigger one, duh.
//The formula is very simple, just take the read value on adc, convert to voltage, and feed to formula. look at datasheet for more details.
void read_current() {
  int adc_in = analogRead(pack_current_in_pin);
  float adc_voltage = adc_in/1024.0*5.0;
  pack_current_draw = (adc_voltage-2.5)*(1.0/0.004) + 1.22;  //This formula is from the datasheet for current sensor i=(v-(Vsupply/2))*(1/g)*(5/Vsupply), the 1.22 is just an offset value
};

void read_voltages() {
  
  // ADC Cell Measurement and conversion
  run_command(3);
 
  // Store cell voltages in cell_data array
  run_command(4);

  //re order the cell voltages into another array to ease of use.
  if(!export_excel){
    for(int each_ic=0; each_ic<TOTAL_IC; each_ic++){
      for(int i=0; i<12; i++){
        cell_data[(each_ic*12)+i+1] =  bms_ic[each_ic].cells.c_codes[i]*0.0001,4;
          Serial.print("Cell #");
          Serial.print(i+1);
          Serial.print("=");
          Serial.print(bms_ic[each_ic].cells.c_codes[i]*0.0001,4);
      }
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
    if(cell_data[i] >= cell_hard_upper_limit) {
      //add an error count
      error_cnt++;      
    }
  }
  
  // Set regen status based on whether we exceed soft upper limit  -This is only for the mabx and motor controller since it just needs to not trip ams fault.
  if(max_cell_voltage < cell_soft_upper_limit){
     regen_status = true;
  } else {
     regen_status = false;
  } 

  // Allow pack to be charged externally. should be always on, but this functions stops charging up to the point of damage.
  if(  (max_cell_voltage < (cell_hard_upper_limit - cell_hyst)) || (pack_voltage < (cell_soft_upper_limit*total_cells))  ) {
    digitalWrite(midpack_output_pin, true);
    midpack_status = true;
    if ( ams_status == false) {
      set_ams_status(true);
    }
  }
  else {
    digitalWrite(midpack_output_pin, false);
    midpack_status = false;
  }
};
