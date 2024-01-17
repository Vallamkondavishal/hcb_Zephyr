
#include "hcb_protocol.h"
#include "hcbapi.h"
#include "math.h"
#include "unistd.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <string>
#include <fstream>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef MILLISLEEP
#define MILLISLEEP(X)                                                          \
  do                                                                           \
  {                                                                            \
    std::chrono::milliseconds tim2((X));                                       \
    std::this_thread::sleep_for(tim2);                                         \
  } while (0)
#endif

class hcb_driver
{

public:
  // Constructor: Initializes the handle member variable.
  hcb_driver() : handle() { }

  // Destructor: Stops the thread and closes the HID handle.
  ~hcb_driver()

  {
    stopThread();
    hcb_close(handle);
  }

  // Connects to the HID device, opens the handle, and spawns a communication thread.
  void connect()
  {
    handle = hcb_open(0x2fe3, 0x100);
    spawn_thread();
  }
  
  // Sends a "hello" command to the HID device.
  void send_hello() { hcb_send_hello(handle); }

  // Sends a set point command to the HID device with an array of values.
  void send_set_point(uint32_t val[]) { hcb_set(handle, val); }
  
  // Spawns a thread for continuous communication with the HID device.
  void spawn_thread()
  {
    std::cout << "start driver thread" << std::endl;
    workerThread = std::thread([this]() {
      hcb_protocol_t msg;
      while (continuing)
      {
        auto res = hcb_read_packet(handle, &msg);
        if (res == 1)
        {
          switch (msg.header.message_id)
          {
            case HCB_ID_COMMAND:
              std::cout << "HCB_ID_COMMAND" << std::endl;
              break;
            case HCB_ID_HELLO_REPLY:
              std::cout << "HCB_ID_HELLO_REPLY : " << msg.hello_reply.ver
                        << std::endl;
              break;
            case HCB_ID_UNSOLICITED_TIME:
              std::cout << "HCB_ID_UNSOLICITED_TIME" << std::endl;
              break;
            case HCB_ID_ADC:
              //std::cout << "HCB_ID_ADC" << std::endl;
              memcpy(&this->currRead, &msg.adc, sizeof(hcb_msg_adc_t));
              break;
            case HCB_ID_FT:
              memcpy(&this->currftRead, &msg.ft, sizeof(hcb_msg_ft_t));
              break;
          }
        }
      }
    });
  }
  

  // Stops the communication thread.
  void stopThread()
  {
    this->continuing = false;
    if (this->workerThread.joinable())
    {
      this->workerThread.join();
    }
  }

  // Retrieves ADC data from the current read.
  hcb_msg_adc_t getAdc() { return currRead; }

  // Retrieves FT data from the current read.
  hcb_msg_ft_t getFt() {return currftRead;}
private:
  // HID device handle.
  void* handle;

  // Thread for continuous communication with the HID device.
  std::thread workerThread {};

  // Atomic flag indicating whether the communication thread should continue.
  std::atomic<bool> continuing { true };

  // Current ADC read data.
  hcb_msg_adc_t currRead;

  // Current FT read data.
  hcb_msg_ft_t currftRead;
};

// Function to convert ADC values from bananas to bars.
uint32_t fromBananasToBar(uint16_t val)
{
  return (uint32_t) ((val - 400)*350)/1600;
}

//Main Function
int main()
{
  // Create an instance of the hcb_driver class.
  hcb_driver driver;

  // Connect to the HID device and start the communication thread.
  driver.connect();

  // Open a file for writing CSV data.
  std::ofstream myfile;
  myfile.open("pid_pressione.csv");
  myfile << "Time, set_point,  pression 1, pression bar 1, pwm1, current 1, pression 2, pression bar 2, pwm2, current 2, pression 3, pression bar 3, pwm3, current 3, pression 4, pression bar 4, pwm4, current 4, pression 5, pression bar 5, pwm5, current 5 \n";

  // Sleep for 6 seconds.
  MILLISLEEP(6000);

  // Set initial values and send set point command.
  uint32_t start_vals[] = {0, 0, 0, 0, 0, 0, 5000, 0};
  driver.send_set_point(start_vals);

  // Sleep for 6 seconds.
  MILLISLEEP(6000);

  // Set set_point value and values array for continuous communication loop.
  uint32_t set_point = 70;
  uint32_t vals[] = {70, 70, 70, 70, 70, 70, 5000, 0};

  // Record start time for elapsed time calculation.
  auto start = std::chrono::system_clock::now();

  // Continuous communication loop.
  int k = 0;
  while (k < 300)
  {
    auto end= std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    // Write CSV data.
    myfile << elapsed_seconds.count() <<",";
    myfile << set_point << ","; 
    for(int i = 0; i < 6; i++){
      myfile << (float)driver.getAdc().val[i] << "," <<fromBananasToBar(driver.getAdc().val[i])<< ","<< (float)driver.getAdc().val[8 + i]<<","<<(float)driver.getAdc().val[16 + i] << ",";
      std::cout << "BANANAS:"<<(float)driver.getAdc().val[i] << "\n\n";
      std::cout << "BAR:" <<fromBananasToBar(driver.getAdc().val[i]) << "\n\n\n";
    }

    // Send set point command and sleep for 100 milliseconds.
    myfile << "\n";
    driver.send_set_point(vals);
    MILLISLEEP(100);
    k++;
    std::cout << "\n\n \n K : " << k << "\n\n";
  }  


  // Reset values and send set point command for the second part of the loop.
  uint32_t vals1[] = {0, 0, 0, 0, 0, 0,5000,0};
  set_point =0;
  k = 0;
  while(k < 150){
    auto end= std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    // Write CSV data.
    myfile << elapsed_seconds.count() <<",";
    myfile << set_point << ","; 
    for(int i = 0; i < 6; i++){
      myfile << (float)driver.getAdc().val[i] << "," <<fromBananasToBar(driver.getAdc().val[i])<< ","<< (float)driver.getAdc().val[8 + i]<<","<<(float)driver.getAdc().val[16 + i] << ",";
      std::cout << (float)driver.getAdc().val[i] << "\n\n";
      std::cout << "BAR:" <<fromBananasToBar(driver.getAdc().val[i]) << "\n\n\n";
    }

    myfile << "\n";

    // Send set point command and sleep for 100 milliseconds.
    driver.send_set_point(vals1);
    MILLISLEEP(100);
    k++;
    std::cout << "\n\n \n K : " << k << "\n\n";
  }  

  // Final part of the loop with set point set to 70.
  k = 0;
  set_point =70;
  while(k < 300){
    auto end= std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    // Write CSV data.
    myfile << elapsed_seconds.count() <<",";
    myfile << set_point << ","; 
    for(int i = 0; i <6; i++){
      myfile << (float)driver.getAdc().val[i] << "," <<fromBananasToBar(driver.getAdc().val[i])<< ","<< (float)driver.getAdc().val[8 + i]<<","<<(float)driver.getAdc().val[16 + i] << ",";
      std::cout << (float)driver.getAdc().val[i] << "\n\n";
      std::cout << "BAR:" <<fromBananasToBar(driver.getAdc().val[i]) << "\n\n\n";
    }
    myfile << "\n";

    // Send set point command and sleep for 100 milliseconds.
    driver.send_set_point(vals);
    MILLISLEEP(100);
    k++;
    std::cout << "\n\n \n K : " << k << "\n\n";
  }  

  // Close the CSV file.
  myfile.close();
  return 0;
}