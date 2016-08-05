/* BME680 Basic Example Code  
 by: Kris Winer
 date: June 20, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 This sketch uses SDA/SCL on pins 16/17, respectively, and it uses the ESP8285.
 The BME680 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have 4K7 pull-up resistors (to 3.3V).
 
 Hardware setup:
 SDA ----------------------- 17
 SCL ----------------------- 16
 
  */
#include "i2c_t3.h"   

// BME680 registers
#define BME680_CALIB_DATA00             0x00   // eight bytes of device-specific calibration data 0x00 - 0x07
#define BME680_FIELD_0_MEAS_STATUS_0    0x1D
#define BME680_FIELD_0_MEAS_STATUS_1    0x1E
#define BME680_FIELD_0_PRESS_MSB        0x1F
#define BME680_FIELD_0_PRESS_LSB        0x20
#define BME680_FIELD_0_PRESS_XLSB       0x21
#define BME680_FIELD_0_TEMP_MSB         0x22
#define BME680_FIELD_0_TEMP_LSB         0x23
#define BME680_FIELD_0_TEMP_XLSB        0x24
#define BME680_FIELD_0_HUM_MSB          0x25
#define BME680_FIELD_0_HUM_LSB          0x26

#define BME680_FIELD_0_GAS_RL_MSB       0x2A
#define BME680_FIELD_0_GAS_RL_LSB       0x2B

#define BME680_FIELD_1_MEAS_STATUS_0    0x2E
#define BME680_FIELD_1_MEAS_STATUS_1    0x2F
#define BME680_FIELD_1_PRESS_MSB        0x30
#define BME680_FIELD_1_PRESS_LSB        0x31
#define BME680_FIELD_1_PRESS_XLSB       0x32
#define BME680_FIELD_1_TEMP_MSB         0x33
#define BME680_FIELD_1_TEMP_LSB         0x34
#define BME680_FIELD_1_TEMP_XLSB        0x35
#define BME680_FIELD_1_HUM_MSB          0x36
#define BME680_FIELD_1_HUM_LSB          0x37

#define BME680_FIELD_2_GAS_RL_MSB       0x3B
#define BME680_FIELD_2_GAS_RL_LSB       0x3C

#define BME680_FIELD_2_MEAS_STATUS_0    0x3F
#define BME680_FIELD_2_MEAS_STATUS_1    0x40
#define BME680_FIELD_2_PRESS_MSB        0x41
#define BME680_FIELD_2_PRESS_LSB        0x42
#define BME680_FIELD_2_PRESS_XLSB       0x43
#define BME680_FIELD_2_TEMP_MSB         0x44
#define BME680_FIELD_2_TEMP_LSB         0x45
#define BME680_FIELD_2_TEMP_XLSB        0x46
#define BME680_FIELD_2_HUM_MSB          0x47
#define BME680_FIELD_2_HUM_LSB          0x48

#define BME680_FIELD_2_GAS_RL_MSB       0x4C
#define BME680_FIELD_2_GAS_RL_LSB       0x4D

#define BME680_IDAC_HEAT_X              0x50 // 10 IDAC byte values 0x50 - 0x59
#define BME680_RES_HEAT_X               0x5A // 10 RES byte values  0x5A - 0x63
#define BME680_GAS_WAIT_X               0x64 // 10 WAIT byte values 0x64 - 0x6D
#define BME680_GAS_WAIT_SHARED          0x6E  
#define BME680_RES_HEAT_CTRL            0x6F  
#define BME680_CTRL_GAS_0               0x70  
#define BME680_CTRL_GAS_1               0x71  
#define BME680_CTRL_HUM                 0x72  
#define BME680_STATUS                   0x73  
#define BME680_CTRL_MEAS                0x74  
#define BME680_CONFIG                   0x75  
#define BME680_CTRL_PROG                0x76  

#define BME680_CALIB_DATA_80            0x80  // more calibration data 0x80 - 0xA1

#define BME680_ID                       0xD0  //should return 0x61
#define BME680_RESET                    0xE0   

#define BME680_CALIB_DATA_E1            0xE1  // more calibration data 0xE1 - 0xF0

#define BME680_CTRL_HUM                 0xF2   
#define BME680_SPI_MEM_PAGE             0xF3   

#define BME680_CALIB_ADDR_1             0x89  // 25 bytes of calibration data for I2C
#define BME680_CALIB_ADDR_2             0xE1  // 16 bytes of calibration data for I2C

#define BME680_ADDRESS                  0x76   // Address of BME680 altimeter when ADO = 0 (default)


#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

enum Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum Hosr {
  H_OSR_00 = 0,  // no op
  H_OSR_01,
  H_OSR_02,
  H_OSR_04,
  H_OSR_08,
  H_OSR_16
};

enum Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode {
  BME680Sleep = 0,
  Forced,
  Parallel,
  Sequential
};

enum SBy {
  t_00_6ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_10ms,
  t_20ms,
};

enum GWaitMult {
  gw_1xmult = 0,
  gw_4xmult,
  gw_16xmult,
  gw_64xmult
};

// Specify BME680 configuration
uint8_t Posr = P_OSR_16, Hosr = H_OSR_01, Tosr = T_OSR_02, Mode = Forced, IIRFilter = BW0_042ODR, SBy = t_10ms;     // set pressure amd temperature output data rate
// Gas sensor configuration
uint8_t GWaitMult = gw_1xmult; // choose gas sensor wait time multiplier
uint8_t numHeatPts = 0x01; // one heat set point
// choose gas wait time in milliseconds x gas wait multiplier 0x00 | 0x59 == 100 ms gas wait time
uint8_t gasWait[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // must choose at least one non-zero wait time  
uint8_t resHeat[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // must choose at least one non-zero wait time  

// Data arrays for conversion of raw gas measurements into resistance
float const_array1[16] = {1, 1, 1, 1, 1, 0.99, 1, 0.992, 1, 1, 0.998, 0.995, 1, 0.99, 1, 1};
double const_array2[16] = {8000000.0, 4000000.0, 2000000.0, 1000000.0, 499500.4995, 248262.1648, 125000.0, 
63004.03226, 31281.28128, 15625.0, 7812.5, 3906.25, 1953.125, 976.5625, 488.28125, 244.140625};

// t_fine carries fine temperature as global value for BME680
int32_t t_fine;

float Temperature, Pressure, Humidity; // stores BME680 pressures sensor pressure and temperature
uint32_t rawPress, rawTemp;   // pressure and temperature raw count output for BME680
uint16_t rawHumidity, rawGasResistance;  // variables to hold raw BME680 humidity and gas resistance values

// BME680 compensation parameters
uint8_t  dig_P10, dig_H6;
uint16_t dig_T1, dig_P1, dig_H1, dig_H2;
int16_t  dig_T2, dig_P2, dig_P4, dig_P5, dig_P8, dig_P9, dig_GH2;
int8_t   dig_T3, dig_P3, dig_P6, dig_P7, dig_H3, dig_H4, dig_H5, dig_H7, dig_GH1, dig_GH3;
float   temperature_C, temperature_F, pressure, humidity, altitude, resistance; // Scaled output of the BME680

uint32_t delt_t = 0, count = 0, sumCount = 0, slpcnt = 0;  // used to control display output rate

uint8_t status0, status1, status2;

void setup()
{

  Serial.begin(115200);  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);

  pinMode(myLed, OUTPUT);

  I2Cscan(); // should detect BME680 at 0x76
  
  // Read the WHO_AM_I register of the BME680 this is a good test of communication
  byte f = readByte(BME680_ADDRESS, BME680_ID);  // Read WHO_AM_I register for BME680
  Serial.print("BME680 "); 
  Serial.print("I AM "); 
  Serial.print(f, HEX); 
  Serial.print(" I should be "); 
  Serial.println(0x61, HEX);
  Serial.println(" ");
  
  delay(1000); 

  if(f == 0x61) {
   
  writeByte(BME680_ADDRESS, BME680_RESET, 0xB6); // reset BME680 before initialization
  delay(100);

  BME680TPHInit(); // Initialize BME680 Temperature, Pressure, Humidity sensors
  Serial.println("Calibration coeficients:");
  Serial.print("dig_T1 ="); 
  Serial.println(dig_T1);
  Serial.print("dig_T2 ="); 
  Serial.println(dig_T2);
  Serial.print("dig_T3 ="); 
  Serial.println(dig_T3);
  Serial.print("dig_P1 ="); 
  Serial.println(dig_P1);
  Serial.print("dig_P2 ="); 
  Serial.println(dig_P2);
  Serial.print("dig_P3 ="); 
  Serial.println(dig_P3);
  Serial.print("dig_P4 ="); 
  Serial.println(dig_P4);
  Serial.print("dig_P5 ="); 
  Serial.println(dig_P5);
  Serial.print("dig_P6 ="); 
  Serial.println(dig_P6);
  Serial.print("dig_P7 ="); 
  Serial.println(dig_P7);
  Serial.print("dig_P8 ="); 
  Serial.println(dig_P8);
  Serial.print("dig_P9 ="); 
  Serial.println(dig_P9);
  Serial.print("dig_P10 ="); 
  Serial.println(dig_P10);
  Serial.print("dig_H1 ="); 
  Serial.println(dig_H1);
  Serial.print("dig_H2 ="); 
  Serial.println(dig_H2);
  Serial.print("dig_H3 ="); 
  Serial.println(dig_H3);
  Serial.print("dig_H4 ="); 
  Serial.println(dig_H4);
  Serial.print("dig_H5 ="); 
  Serial.println(dig_H5);
  Serial.print("dig_H6 ="); 
  Serial.println(dig_H6);
  Serial.print("dig_H7 ="); 
  Serial.println(dig_H7);
  Serial.print("dig_GH1 ="); 
  Serial.println(dig_GH1);
  Serial.print("dig_GH2 ="); 
  Serial.println(dig_GH2);
  Serial.print("dig_GH3 ="); 
  Serial.println(dig_GH3);

  // Configure the gas sensor
  gasWait[0] = GWaitMult | 0x59;  // define gas wait time for heat set point 0x59 == 100 ms
  Serial.print("gas wait time = 0x"); Serial.println(gasWait[0], HEX);
  resHeat[0] = BME680_TT(200);    // define temperature set point in degrees Celsius for resistance set point 0
  Serial.print("resistance Heat = "); Serial.println(resHeat[0]);
  BME680GasInit();

 }
  else Serial.println(" BME680 not functioning!");
  
  delay(1000);  
}
 

void loop()
{  
   // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 1000) { // update LCD once per second independent of read rate

    // Check status registers
     status0 = readByte(BME680_ADDRESS, BME680_FIELD_0_MEAS_STATUS_0);
      if(status0 & 0x80) Serial.println("New data in field 0!");
      if(status0 & 0x40) Serial.println("Measuring field 0 gas data!");
      if(status0 & 0x20) Serial.println("Conversion in progress!");
      Serial.print("Gas measurement Index = "); Serial.println(status0 & 0x0F);
      
    writeByte(BME680_ADDRESS, BME680_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
    rawTemp =   readBME680Temperature();
    temperature_C = (float) BME680_compensate_T(rawTemp)/100.;
    rawPress =  readBME680Pressure();
    pressure = (float) BME680_compensate_P(rawPress)/100.; // Pressure in mbar
    rawHumidity =   readBME680Humidity();
    humidity = (float) BME680_compensate_H(rawHumidity)/1024.;
    rawGasResistance = readBME680GasResistance();
    resistance = (float) BME680_compensate_Gas(rawGasResistance);
 
      Serial.println("BME680:");
      Serial.print("Altimeter temperature = "); 
      Serial.print( temperature_C, 2); 
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = "); 
      Serial.print(9.*temperature_C/5. + 32., 2); 
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); 
      Serial.print(pressure, 2);  
      Serial.println(" mbar");// pressure in millibar
      altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));
      Serial.print("Altitude = "); 
      Serial.print(altitude, 2); 
      Serial.println(" feet");
      Serial.print("Altimeter humidity = "); 
      Serial.print(humidity, 1);  
      Serial.println(" %rH");// relative humidity
      Serial.print("Gas Sensor raw resistance = "); 
      Serial.print(rawGasResistance);  
      Serial.println(" ");
      Serial.print("Gas Sensor resistance = "); 
      Serial.print(resistance, 1);  
      Serial.println(" Ohm");// gas sensor resistance in Ohm
      Serial.println(" ");

      digitalWrite(myLed, !digitalRead(myLed));
      count = millis(); 
    }
    
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

 uint32_t readBME680Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME680_ADDRESS, BME680_FIELD_0_TEMP_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

uint32_t readBME680Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME680_ADDRESS, BME680_FIELD_0_PRESS_MSB, 3, &rawData[0]);  
  return (uint32_t) (((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

uint16_t readBME680Humidity()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BME680_ADDRESS, BME680_FIELD_0_HUM_MSB, 2, &rawData[0]);  
  return (uint16_t) (((uint16_t) rawData[0] << 8 | rawData[1]) );
}

uint16_t readBME680GasResistance()
{
  uint8_t rawData[2];  // 10-bit gas resistance register data stored here
  readBytes(BME680_ADDRESS, BME680_FIELD_0_GAS_RL_MSB, 2, &rawData[0]);  
  if(rawData[1] & 0x20) Serial.println("Field 0 gas data valid"); 
  return (uint16_t) (((uint16_t) rawData[0] << 2 | (0xC0 & rawData[1]) >> 6) );

}


void BME680TPHInit()
{
  // Configure the BME680 Temperature, Pressure, Humidity sensors
  // Set H oversampling rate
  writeByte(BME680_ADDRESS, BME680_CTRL_HUM, 0x07 & Hosr);
  // Set T and P oversampling rates and sensor mode
  writeByte(BME680_ADDRESS, BME680_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  writeByte(BME680_ADDRESS, BME680_CONFIG, SBy << 5 | IIRFilter << 2);
 
  // Read and store calibration data
  uint8_t calib[41] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  readBytes(BME680_ADDRESS, BME680_CALIB_ADDR_1, 25, &calib[0]);
  readBytes(BME680_ADDRESS, BME680_CALIB_ADDR_2, 16, &calib[25]);
 // temperature compensation parameters
  dig_T1 = (uint16_t)(((uint16_t) calib[34] << 8) | calib[33]);
  dig_T2 = ( int16_t)((( int16_t) calib[2] << 8) | calib[1]);
  dig_T3 = (  int8_t)             (calib[3]);
 // pressure compensation parameters
  dig_P1 = (uint16_t)(((uint16_t) calib[6] << 8) | calib[5]);
  dig_P2 = ( int16_t)((( int16_t) calib[8] << 8) | calib[7]);
  dig_P3 =  ( int8_t)             (calib[9]);
  dig_P4 = ( int16_t)((( int16_t) calib[12] << 8) | calib[11]);
  dig_P5 = ( int16_t)((( int16_t) calib[14] << 8) | calib[13]);
  dig_P6 =  ( int8_t)             (calib[16]);
  dig_P7 =  ( int8_t)             (calib[15]);  
  dig_P8 = ( int16_t)((( int16_t) calib[20] << 8) | calib[19]);
  dig_P9 = ( int16_t)((( int16_t) calib[22] << 8) | calib[21]);
  dig_P10 = (uint8_t)             (calib[23]);
// humidity compensation parameters
  dig_H1 =  (uint16_t)(((uint16_t) calib[27] << 4) | (calib[26] & 0x0F));
  dig_H2 =  (uint16_t)(((uint16_t) calib[25] << 4) | (calib[26] >> 4));
  dig_H3 =  (int8_t) calib[28];
  dig_H4 =  (int8_t) calib[29];
  dig_H5 =  (int8_t) calib[30];
  dig_H6 = (uint8_t) calib[31];
  dig_H7 =  (int8_t) calib[32];
// gas sensor compensation parameters
  dig_GH1 =  (int8_t) calib[37];
  dig_GH2 = ( int16_t)((( int16_t) calib[36] << 8) | calib[35]);
  dig_GH3 =  (int8_t) calib[38];
}

void BME680GasInit()  // Initialize BME680 gas sensor
{
  // Configure the BME680 Gas Sensor
  writeByte(BME680_ADDRESS, BME680_CTRL_GAS_1, 0x10 | numHeatPts - 1); // write number of heater set points
  // Set gas sampling wait time and target heater resistance
  for(uint8_t ii = 0; ii < numHeatPts; ii++) 
  {
    writeByte(BME680_ADDRESS, (BME680_GAS_WAIT_X + ii), gasWait[ii]);
    writeByte(BME680_ADDRESS, (BME680_RES_HEAT_X + ii), resHeat[ii]);
  }
  Serial.print("CTRL_GAS_1 = 0x"); Serial.println(readByte(BME680_ADDRESS, BME680_CTRL_GAS_1), HEX);
  Serial.print("gas wait = 0x"); Serial.println(readByte(BME680_ADDRESS, BME680_GAS_WAIT_X), HEX);
  Serial.print("res heat = 0x"); Serial.println(readByte(BME680_ADDRESS, BME680_RES_HEAT_X));
}

// Returns register code to be written to register BME680_RES_HEAT_CTRL for a user specified target temperature TT
// where TT is the target temperature in degrees Celsius
uint8_t BME680_TT(uint16_t TT) // TT is between 200 and 400  
{
  uint8_t res_heat_x = 0;
  double var1 = 0.0, var2 = 0.0, var3 = 0.0, var4 = 0.0, var5 = 0.0;
  uint16_t par_g1 = ((uint16_t) readByte(BME680_ADDRESS, 0xEC) << 8) | readByte(BME680_ADDRESS, 0xEB);
  uint8_t  par_g2 = readByte(BME680_ADDRESS, 0xED);
  uint8_t  par_g3 = readByte(BME680_ADDRESS, 0xEE);
  uint8_t  res_heat_range = (readByte(BME680_ADDRESS, 0x02) & 0x30) >> 4;
  uint8_t res_heat_val = readByte(BME680_ADDRESS, 0x00);
  var1 = ((double) par_g1/ 16.0) + 49.0;
  var2 = (((double)par_g2 / 32768.0) * 0.0005) + 0.00235;
  var3 = (double)par_g3 / 1024.0;
  var4 = var1 * (1.0 + (var2 * (double)TT));
  var5 = var4 + (var3 * 25.0); // use 25 C as ambient temperature
  res_heat_x = (uint8_t)(((var5 * (4.0/(4.0 * (double)res_heat_range))) - 25.0) * 3.4 / ((res_heat_val * 0.002) + 1));
  return res_heat_x;
  }

  
  // Compensate Raw Gas ADC values to obtain resistance
float BME680_compensate_Gas(uint16_t gas_adc)
{
  uint8_t gasRange = readByte(BME680_ADDRESS, BME680_FIELD_0_GAS_RL_LSB) & 0x0F;
  Serial.print("gas range = "); Serial.println(gasRange);
  double var1 = 0, gas_switch_error = 1.0;
  var1 =  (1340.0 + 5.0 * gas_switch_error) * const_array1[gasRange];
  float gas_res = var1 * const_array2[gasRange] / (gas_adc - 512.0 + var1);
  return gas_res;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t BME680_compensate_T(uint32_t adc_T)
{
  int32_t var1 = 0, var2 = 0, var3 = 0, T = 0;
  var1 = ((int32_t) adc_T >> 3) - ((int32_t)dig_T1 << 1); 
  var2 = (var1 * (int32_t)dig_T2) >> 11;
  var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t) dig_T3 << 4)) >> 14;
  t_fine = var2 + var3;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns the value in Pascal(Pa)
// Output value of "96386" equals 96386 Pa =
//  963.86 hPa = 963.86 millibar
int32_t BME680_compensate_P(uint32_t adc_P)
{
  int32_t var1 = 0, var2 = 0, var3 = 0, var4 = 0, P = 0;
  var1 = (((int32_t) t_fine) >> 1) - 64000;
  var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t) dig_P6) >> 2;
  var2 = var2 + ((var1 * (int32_t)dig_P5) << 1);
  var2 = (var2 >> 2) + ((int32_t) dig_P4 << 16);
  var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t) dig_P3 << 5)) >> 3) + (((int32_t) dig_P2 * var1) >> 1);
  var1 = var1 >> 18;
  var1 = ((32768 + var1) * (int32_t) dig_P1) >> 15;
  P = 1048576 - adc_P;
  P = (int32_t)((P - (var2 >> 12)) * ((uint32_t)3125));
  var4 = (1 << 31);
  
  if(P >= var4)
    P = (( P / (uint32_t) var1) << 1);
  else
    P = ((P << 1) / (uint32_t) var1);
    
  var1 = ((int32_t) dig_P9 * (int32_t) (((P >> 3) * (P >> 3)) >> 13)) >> 12;
  var2 = ((int32_t)(P >> 2) * (int32_t) dig_P8) >> 13;
  var3 = ((int32_t)(P >> 8) * (int32_t)(P >> 8) * (int32_t)(P >> 8) * (int32_t)dig_P10) >> 17;
  P = (int32_t)(P) + ((var1 + var2 + var3 + ((int32_t)dig_P7 << 7)) >> 4);
  
  return P;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22integer and 10fractional bits).
// Output value of “47445”represents 47445/1024 = 46.333%RH
int32_t BME680_compensate_H(uint32_t adc_H)
{
  int32_t var1 = 0, var2 = 0, var3 = 0, var4 = 0, var5 = 0, var6 = 0, H = 0, T = 0;

  T = (((int32_t) t_fine * 5) + 128) >> 8;
  var1 = (int32_t) adc_H  - ((int32_t) ((int32_t)dig_H1 << 4)) - (((T * (int32_t) dig_H3) / ((int32_t)100)) >> 1);
  var2 = ((int32_t)dig_H2 * (((T * (int32_t)dig_H4) / 
         ((int32_t)100)) + (((T * ((T * (int32_t)dig_H5) / 
         ((int32_t)100))) >> 6) / ((int32_t)100)) + (int32_t)(1 << 14))) >> 10;
  var3 = var1 * var2;
  var4 = ((((int32_t)dig_H6) << 7) + ((T * (int32_t) dig_H7) / ((int32_t)100))) >> 4;
  var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
  var6 = (var4 * var5) >> 1;

  H = (var3 + var6) >> 12;

  if (H > 102400) H = 102400; // check for over- and under-flow
  else if(H < 0) H = 0;

  return H;
}

// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// I2C read/write functions for the BMP280 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
//	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
