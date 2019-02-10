// Include for CAN
#include <mcp_can.h>
#include <mcp_can_dfs.h>


// Define pins for AMS functions
#define AMS_STATUS_PIN 7
#define AMS_CAN_ID_data 0x51
#define AMS_CAN_ID_cell 0x52

// Define pins for air and control board

#define precharged_pin 1        //input for if TS is pre charged
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

/*!*********************************************************************
  \AMS variables
***********************************************************************/

int precharged = 0;
int midpack_status = 0;

float cell_data[total_cells+1];

float cell_hard_upper_limit = 2.70;
float cell_soft_upper_limit = 2.60;
float cell_soft_under_limit = 1.0;
float cell_hyst = 0.1;
float cell_balance_window = 0.05;
float low_safety_cell_v = 0.5;

bool cell_discharging[TOTAL_IC][12];

bool ams_status = true;
bool regen_status = false;
int error_cnt = 0;
int max_error_cnt = 5;
int current_error_millis;
int error_time_allowed = 2000;  //This means if you get 5 errors in 2 sec, something is wrong

int max_cell_num;
int min_cell_num;
float max_cell_voltage;
float min_cell_voltage;

float pack_voltage;

float temp_sen1;
float temp_sen2;

float pack_current_draw;

unsigned long current_millis;
int meas_time = 100; //momintor pack every 2sec

/*!*********************************************************************
  \AMS functions
***********************************************************************/
void read_voltages();
