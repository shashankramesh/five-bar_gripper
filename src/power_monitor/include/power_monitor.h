#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>	
#include <time.h>
#include "smbus.h"
#include <iostream>

namespace rpi_power_monitor
{
  // Config Register (R/W)
  __u8 _REG_CONFIG = 0x00;

  // Shunt Voltage Register (R)
  __u8 _REG_SHUNTVOLTAGE = 0x01;

  // Bus Voltage Register (R)
  __u8 _REG_BUSVOLTAGE = 0x02;

  // POWER REGISTER (R)
  __u8 _REG_POWER = 0x03;

  // CURRENT REGISTER (R)
  __u8 _REG_CURRENT = 0x04;

  // CALIBRATION REGISTER (R/W)
  __u8 _REG_CALIBRATION = 0x05;

  struct
  {
    __u8 RANGE_16V = 0X00; // set bus voltage range to 16V
    __u8 RANGE_32V = 0x01; // set bus voltage range to 32V (default)
  } BusVoltageRange;

  struct
  {
    __u8 DIV_1_40MV = 0x00; // shunt prog. gain set to  1, 40 mV range
    __u8 DIV_2_80MV = 0x01; // shunt prog. gain set to /2, 80 mV range
    __u8 DIV_4_160MV = 0x02; // shunt prog. gain set to /4, 160 mV range
    __u8 DIV_8_320MV = 0x03; // shunt prog. gain set to /8, 320 mV range
  } Gain;

  struct
  {
    __u8 ADCRES_9BIT_1S = 0x00; //  9bit,   1 sample,     84us
    __u8 ADCRES_10BIT_1S = 0x01; // 10bit,   1 sample,    148us
    __u8 ADCRES_11BIT_1S = 0x02; // 11 bit,  1 sample,    276us
    __u8 ADCRES_12BIT_1S = 0x03; // 12 bit,  1 sample,    532us
    __u8 ADCRES_12BIT_2S = 0x09; // 12 bit,  2 samples,  1.06ms
    __u8 ADCRES_12BIT_4S = 0x0A; // 12 bit,  4 samples,  2.13ms
    __u8 ADCRES_12BIT_8S = 0x0B; // 12bit,   8 samples,  4.26ms
    __u8 ADCRES_12BIT_16S = 0x0C; // 12bit,  16 samples,  8.51ms
    __u8 ADCRES_12BIT_32S = 0x0D; // 12bit,  32 samples, 17.02ms
    __u8 ADCRES_12BIT_64S = 0x0E; // 12bit,  64 samples, 34.05ms
    __u8 ADCRES_12BIT_128S = 0x0F; // 12bit, 128 samples, 68.10ms
  } ADCResolution;

  struct
  {
    __u8 POWERDOW                = 0x00; // power down
    __u8 SVOLT_TRIGGERED         = 0x01; // shunt voltage triggered
    __u8 BVOLT_TRIGGERED         = 0x02; // bus voltage triggered
    __u8 SANDBVOLT_TRIGGERED     = 0x03; // shunt and bus voltage triggered
    __u8 ADCOFF                  = 0x04; // ADC off
    __u8 SVOLT_CONTINUOUS        = 0x05; // shunt voltage continuous
    __u8 BVOLT_CONTINUOUS        = 0x06; // bus voltage continuous
    __u8 SANDBVOLT_CONTINUOUS    = 0x07; // shunt and bus voltage continuous
  } Mode;

  class RpiPowerMonitor
  {

    public: 
      RpiPowerMonitor(int i2c_bus = 1, __u8 addr = 0x40)
      {
        address = addr;

        // Opening I2C Bus
        char *filename = (char*)"/dev/i2c-1";
        if ((file_i2c = open(filename, O_RDWR)) < 0)
        {
          //ERROR HANDLING: you can check errno to see what went wrong
          std::cout << "Failed to open the i2c bus" << std::endl;
          return;
        }

        if (ioctl(file_i2c, I2C_SLAVE, address) < 0)
        {
          std::cout << "Failed to acquire bus access and/or talk to slave.\n" << std::endl;
          //ERROR HANDLING; you can check errno to see what went wrong
          return;
        }

        _cal_value = 0;
        _current_lsb = 0;
        _power_lsb = 0;
        set_calibration_32V_2A();
      }

      void getShuntVoltage(float &voltage)
      {

        write(_REG_CALIBRATION, _cal_value);
        read(_REG_SHUNTVOLTAGE, value);

        if(value > 32767)
          value -= 65535;

        voltage = value * 0.01;

      }

      void getBusVoltage_V(float &voltage)
      { 

        write(_REG_CALIBRATION, _cal_value);
        read(_REG_BUSVOLTAGE, value);
        value = 0;

        read(_REG_BUSVOLTAGE, value);

        voltage = (value >> 3) * 0.004;

      }


      void getCurrent_mA(float &current)
      {

        read(_REG_CURRENT, value);
        if(value > 32767)
            value -= 65535;

        current = value * _current_lsb;
      
      }

      void getPower_W(float &power)
      {

        read(_REG_POWER, value);
        if(value > 32767)
            value -= 65535;

        power = value * _power_lsb;

      }
      
      void getVIP(float &voltage, float &current, float &power)
      {
        write(_REG_CALIBRATION, _cal_value);

        read(_REG_BUSVOLTAGE, value);
        value = 0;

        read(_REG_BUSVOLTAGE, value);

        voltage = (value >> 3) * 0.004;

        read(_REG_CURRENT, value);
        if(value > 32767)
            value -= 65535;

        current = value * _current_lsb;

        read(_REG_POWER, value);
        if(value > 32767)
            value -= 65535;

        power = value * _power_lsb;
      }

      void getIP(float &current, float &power)
      {
        //write(_REG_CALIBRATION, _cal_value);

        read(_REG_CURRENT, value);
        if(value > 32767)
            value -= 65535;

        current = value * _current_lsb;

        read(_REG_POWER, value);
        if(value > 32767)
            value -= 65535;

        power = value * _power_lsb;
      }

    private:

      void read(__u8 address, unsigned int& data)
      {
        union i2c_smbus_data reddata;

        reddata.block[0] = 2;
        
        __s32 err = i2c_smbus_access(file_i2c, I2C_SMBUS_READ, address, I2C_SMBUS_I2C_BLOCK_BROKEN, &reddata);

        if (err < 0)
          std::cout << "Error reading from power monitor" << std::endl;
        
        data = (reddata.block[1]*256) + reddata.block[2];
      }

      void write(__u8 address, unsigned int data)
      {
        union i2c_smbus_data writedata;
        
        writedata.block[0] = 2;
        writedata.block[2] = data & 0xFF;
        writedata.block[1] = (data & 0xFF00) >> 8;
        
        __s32 err = i2c_smbus_access(file_i2c, I2C_SMBUS_WRITE, address, I2C_SMBUS_I2C_BLOCK_BROKEN, &writedata);
        
        if (err < 0)
          std::cout << "Error writing to power monitor" << std::endl;
      }

      void set_calibration_32V_2A()
      {
        _current_lsb = 0.1;
        _cal_value = 4096;
        _power_lsb = 0.002;

        write(_REG_CALIBRATION, _cal_value);

        bus_voltage_range = BusVoltageRange.RANGE_32V;
        gain = Gain.DIV_8_320MV;
        bus_adc_resolution = ADCResolution.ADCRES_12BIT_2S;
        shunt_adc_resolution = ADCResolution.ADCRES_12BIT_2S;
        mode = Mode.SANDBVOLT_CONTINUOUS;
        auto config = bus_voltage_range << 13 | \
                gain << 11 | \
                bus_adc_resolution << 7 | \
                shunt_adc_resolution << 3 | \
                mode;
                
        write(_REG_CONFIG, config);
      }

      __u8 address;
      unsigned int _cal_value;
      double _current_lsb = 0;
      double _power_lsb = 0;
      int file_i2c;
      int length;

      __u8 gain;
      __u8 bus_voltage_range;
      __u8 bus_adc_resolution;
      __u8 shunt_adc_resolution;
      __u8 mode;

      unsigned int value;

  };
}


