#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC6811_daisy.h"

#define ENABLED 1
#define DISABLED 0

// ATMEGA pins
const uint8_t AMS_STATUS_PIN = 5;
const uint8_t PRECHARGED_PIN = 4;
const uint8_t MIDPACK_OUT_PIN = 6;
const uint8_t MIDPACK_ALLOWED_PIN = 7;
const uint8_t SPI_MOSI_PIN = 11;
const uint8_t SPI_MISO_PIN = 12;
const uint8_t SPI_SCK_PIN = 13;
const uint8_t SPI_CAN_CS_PIN = 9;
const uint8_t SPI_LTC_CS_PIN = 3;
const uint8_t CT_SENSE_1_PIN = 0;
const uint8_t CT_SENSE_2_PIN = 1;

// Number of LTC6811 chips in the daisy chain
const uint8_t NUM_CHIPS = 1;

// ADC command configurations
// See ltc6811_daisy.h for options
const uint8_t ADC_OPT = ADC_OPT_DISABLED;
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;
const uint8_t ADC_DCP = DCP_DISABLED;
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;

const uint16_t MEASUREMENT_LOOP_TIME = 500; // ms

// Under-voltage and over-voltage thresholds
const uint16_t OV_THRESHOLD = 27000; // 2.7 V
const uint16_t UV_THRESHOLD = 10000; // 1.0 V

// Loop measurement setup
const uint8_t WRITE_CONFIG = DISABLED;
const uint8_t READ_CONFIG = DISABLED;
const uint8_t MEASURE_CELL = ENABLED;
const uint8_t MEASURE_AUX = DISABLED;
const uint8_t MEASURE_STAT = DISABLED;

/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the ltc6811
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
uint16_t cell_codes[NUM_CHIPS][CELL_CHANNELS];
/*!<
  The cell codes will be stored in the cell_codes[][12] array in the following format:
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

uint16_t aux_codes[NUM_CHIPS][AUX_CHANNELS];
/*!<
 The GPIO codes will be stored in the aux_codes[][6] array in the following format:
 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/

uint16_t stat_codes[NUM_CHIPS][4];
/*!<
 The GPIO codes will be stored in the aux_codes[][6] array in the following format:
 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/

uint8_t tx_cfg[NUM_CHIPS][6];
/*!<
  The tx_cfg[][6] stores the ltc6811 configuration data that is going to be written
  to the ltc6811 ICs on the daisy chain. The ltc6811 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:
 |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |
*/

uint8_t rx_cfg[NUM_CHIPS][8];
/*!<
  the rx_cfg[][8] array stores the data that is read back from a ltc6811-1 daisy chain.
  The configuration data for each IC  is stored in blocks of 8 bytes. Below is an table illustrating the array organization:
|rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
|---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
|IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/

void setup()
{
  delay(5000);
  Serial.begin(115200);
  spi_enable(SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
  init_cfg();  //initialize the 681x configuration array to be written

  pinMode(AMS_STATUS_PIN, OUTPUT);
  digitalWrite(AMS_STATUS_PIN, HIGH);

  // Slave-select pin for the LTC6811. Active low to enable SPI communication
  pinMode(SPI_LTC_CS_PIN, OUTPUT);
  digitalWrite(SPI_LTC_CS_PIN, LOW);
}

void loop()
{
  // Send a wakeup command to the LTC6811
  wakeup_sleep();

  // Print the configuration for all LTC6811 chips in the chain (for debugging)
  // The tx_cfg array contains the chip configuration set in init_cfg()
  Serial.print("Configuration being written:\t");
  for (int i = 0; i < 6; i++) {
    Serial.print(tx_cfg[0][i]);
    Serial.print("\t");
  }
  Serial.println();
  ltc6811_wrcfg(NUM_CHIPS, tx_cfg); // Write the desired config to the LTC6811s

  delay(500);

  // Send another wakeup command
  wakeup_sleep();

  // Read back the chip config (should be the same as what was written)
  int8_t pec_code = ltc6811_rdcfg(NUM_CHIPS, rx_cfg);

  // Print the chip config read back for troubleshooting
  if (pec_code == -1) {
    Serial.print("Configuration being read:\t");
    for (int i = 0; i < 6; i++) {
      Serial.print(rx_cfg[0][i]);
      Serial.print("\t");
    }
    Serial.println();
  } else {
    Serial.println("PEC error when reading config");
  }

  delay(MEASUREMENT_LOOP_TIME);
}

void init_cfg()
{
  uint16_t uv_val = (UV_THRESHOLD/16)-1;
  uint16_t ov_val = (OV_THRESHOLD/16);
  for (int i = 0; i < NUM_CHIPS; i++) {
    tx_cfg[i][0] = 0xFC | ADC_OPT;
    tx_cfg[i][1] = (uint8_t)(uv_val&0xFF);
    tx_cfg[i][2] = (uint8_t)((ov_val&0x00F)|((uv_val&0xF00)>>8));
    tx_cfg[i][3] = (uint8_t)((ov_val&0xFF0)>>4);
    tx_cfg[i][4] = 0x00;
    tx_cfg[i][5] = 0x00;

  }

}

/*
 * TODO: Call this once reading/writing chip config is working
 */
void read_and_print_voltages()
{
  // Start cell voltage conversion
  ltc6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);

  // Wait until ADC is complete and then wake up chips from their idle state
  ltc6811_pollAdc();
  wakeup_idle();

  // Store the cell voltages in the cell_codes array and check for errors
  int8_t error = ltc6811_rdcv(0, NUM_CHIPS, cell_codes);
  if (error == -1) {
    Serial.println(F("A PEC error was detected in the received data"));
  }

  // Print the cell voltages
  for (int chip = 0; chip < NUM_CHIPS; chip++) {
    Serial.print("Chip ");
    Serial.print(chip);
    Serial.print(": ");

    for (int cell = 0; cell < CELL_CHANNELS; cell++) {
      Serial.print(cell_codes[chip][cell]);
      Serial.print(" ");
    }

    Serial.println();
  }
}

