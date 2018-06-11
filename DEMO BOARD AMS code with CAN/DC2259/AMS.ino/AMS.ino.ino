// Define pins for AMS functions
#define MOSFET_STATUS_PIN 7
#define KELLY_KDHE_SEND_CAN_ID 0x6B
#define KELLY_KDHE_RESPONSE_CAN_ID 0x73
#define AMS_CAN_ID 0x69
#define CCP_A2D_BATCH_READ1 0x1b


pinMode(MOSFET_STATUS_PIN, OUTPUT);
  digitalWrite(MOSFET_STATUS_PIN, HIGH);
//  pinMode(9,OUTPUT);
//  digitalWrite(9,HIGH);


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
}

void CAN_send() {}

void cell_discharge() {
  // Cell Balancing
  if (Serial.available())           // Check for user input
  {
    uint32_t user_command;
    user_command = read_int();      // Read the user command
    Serial.println(user_command);
    run_command(user_command);
  }
  bms_ic[0].config.tx_data[4] = 0xFF;
  bms_ic[0].config.tx_data[5] = 0x0F;
  bms_ic[1].config.tx_data[4] = 0xFF;
  bms_ic[1].config.tx_data[5] = 0x0F;
  bms_ic[2].config.tx_data[4] = 0xFF;
  bms_ic[2].config.tx_data[5] = 0x0F;
  
 // bms_ic[0].config.tx_data[4] = 0;
  //bms_ic[0].config.tx_data[5] = 0;
  //bms_ic[1].config.tx_data[4] = 0;
  //bms_ic[1].config.tx_data[5] = 0;
  //bms_ic[2].config.tx_data[4] = 0;
  //bms_ic[2].config.tx_data[5] = 0;
  
  run_command(1);
  delay(10);
  run_command(2);
  Serial.print(bms_ic->isospi_reverse);
  delay(10);
  run_command(3);
  delay(10);
  run_command(4);
  delay(500);
}

void read_voltages() {
  // ADC Cell Measurement and conversion
  run_command(3);
  // Store cell voltages in cell_data array (1 indexed)
  run_command(4);
}

void 

  Serial.println("Starting the precharge!");
  // CCP_A2D_BATCH_READ1 is what we are checking
  
  bool preCharging = true;
  unsigned char preChargeSendMsg[1] = { CCP_A2D_BATCH_READ1 };
  unsigned char preChargeRcvMsg[5];
  unsigned char len = 0;
  int8_t error = 0;
  
  while (preCharging)
  {

    CAN.sendMsgBuf(KELLY_KDHE_SEND_CAN_ID, 0, 1, preChargeSendMsg);
    
    // We may need a delay here
    
    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
      CAN.readMsgBuf(&len, preChargeRcvMsg);
      unsigned long canID = CAN.getCanId();  
      if(canID == KELLY_KDHE_RESPONSE_CAN_ID)
      {
        float kellyVoltage = (float) preChargeRcvMsg[4]/1.39;
        Serial.print("Float voltage is: ");
        Serial.print(preChargeRcvMsg[4]);
        Serial.print(" | ");
        Serial.println(kellyVoltage);


