// Include for CAN
#include <mcp_can.h>
#include <mcp_can_dfs.h>


// Define pins for AMS functions
#define AMS_STATUS_PIN 7
#define AMS_CAN_ID_data 0x69
#define AMS_CAN_ID_cell 0x70

// Define pins for air and control board
#define precharged_pin 6        //output for if TS is pre charged
#define air_toggle_pin 2        //output to toggle the AIR contactors
#define midpack_input_pin 3     //input says if TS is on or not
#define midpack_output_pin 4    //output to allow charging the pack externally
#define temp_output_pin 5       //output of temp sensor reading
#define pack_current_in_pin A1  //input for the current sensor on pack 
#define SPI_CS_PIN 9            //depends on CAN shield
MCP_CAN CAN(SPI_CS_PIN);

//#define CCP_A2D_BATCH_READ1 0x1b

const uint8_t TOTAL_IC = 3;//!<number of ICs in the daisy chain
const int total_cells = TOTAL_IC*12;

bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool gpioBits_a[5] = {false,false,false,false,false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
uint16_t UV=0; //!< Under-voltage Comparison Voltage
uint16_t OV=0; //!< Over-voltage Comparison Voltage
bool dccBits_a[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool dctoBits[4] = {true, false, true, false}; //!< Discharge time value // Dcto 0,1,2,3 // Programed for 4 min 
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */


/*!*********************************************************************
  \AMS variables
***********************************************************************/
int debugging = 0;      //FOR serial debugging print statements
bool DEAD_charge = 1;   //FOR manual relay turn on for charging dead pack

int precharged = 0;
int precharge_time = 1000;
unsigned long current_precharge_millis = 0;
int midpack_status = 0;
bool chargebrd = 0;

float cell_data[total_cells+1];

float cell_hard_upper_limit = 2.70;
float cell_soft_upper_limit = 2.60;
float cell_under_limit = 1.05;
float cell_hyst = 0.1;
float cell_balance_window = 0.05;
float low_safety_cell_v = 1.2;

bool cell_discharging[TOTAL_IC][12];

bool ams_status = false;
bool regen_status = false;
bool AIRS_state = false;

int error_cnt = 0;
int max_error_cnt = 5;
int max_record = 0;
unsigned long current_error_millis = 0;
int error_time_allowed = 4000;  //This means if you get 5 errors in 2 sec, something is wrong

int max_cell_temp = 65;
int min_cell_temp = 0;
float temp_sen1;
float temp_sen2;

int max_cell_num;
int min_cell_num;
float max_cell_voltage;
float min_cell_voltage;
float pack_voltage;

float pack_current_draw;

unsigned long current_millis;
unsigned long current_can_millis;
int can_time = 10;
int meas_time = 50; //momintor pack every 2sec

