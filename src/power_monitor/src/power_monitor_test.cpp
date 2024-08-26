#include "power_monitor.h"
#include <stdio.h>
#include <iostream>
#include <chrono>

using namespace std::chrono;

int main(void)
{

  rpi_power_monitor::RpiPowerMonitor pm1(1, 0x40);
  
  float bus_voltage1 = 0;
  float shunt_voltage1 = 0;
  float current1 = 0;
  float power1 = 0;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
  double time = 0;

  while(true)
  {
    ti = high_resolution_clock::now();

/*    pm1.getBusVoltage_V(bus_voltage1);
    pm1.getShuntVoltage(shunt_voltage1);
    pm1.getCurrent_mA(current1);
    pm1.getPower_W(power1);*/

    pm1.getIP(current1, power1);

    std::cout << "Bus voltage: " << bus_voltage1 << std::endl;
    std::cout << "Shunt voltage: " << shunt_voltage1 << std::endl;
    std::cout << "Current: " << current1 << std::endl;
    std::cout << "Power: " << power1 << std::endl;
    std::cout << std::endl;
    
    sleep(0.001);
    
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    std::cout << time << std::endl;
    
  }

  return 0;
}
