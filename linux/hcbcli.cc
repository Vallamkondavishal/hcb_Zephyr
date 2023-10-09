
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
  hcb_driver() : handle() { }
  ~hcb_driver()
  {
    stopThread();
    hcb_close(handle);
  }

  void connect()
  {
    handle = hcb_open(0x2fe3, 0x100);
    spawn_thread();
  }

  void send_hello() { hcb_send_hello(handle); }

  void send_set_point(uint32_t val[]) { hcb_set(handle, val); }

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

  void stopThread()
  {
    this->continuing = false;
    if (this->workerThread.joinable())
    {
      this->workerThread.join();
    }
  }
  hcb_msg_adc_t getAdc() { return currRead; }
  hcb_msg_ft_t getFt() {return currftRead;}
private:
  void* handle;
  std::thread workerThread {};
  std::atomic<bool> continuing { true };
  hcb_msg_adc_t currRead;
  hcb_msg_ft_t currftRead;
};
uint32_t fromBananasToBar(uint16_t val){
  return (uint32_t) ((val - 400)*350)/1600;
}
int main()
{
  hcb_driver driver;
  driver.connect();
  std::ofstream myfile;
  myfile.open ("pid_pressione.csv");
  myfile << "Time, set_point,  pression 1, pression bar 1,pwm1, current 1 ,pression 2,pression bar 2,pwm2, current 2 ,pression 3,pression bar 3,pwm3, current 3 ,pression 4,pression bar 4,pwm4, current 4 ,pression 5,pression bar 5, pwm5, current 5 \n";
  MILLISLEEP(6000);
  uint32_t start_vals[] = {0,0,0,0,0,0,5000,0};
  driver.send_set_point(start_vals);
  MILLISLEEP(6000);
  uint32_t set_point =70;
  uint32_t vals[] = {70, 70, 70, 70, 70, 70,5000,0};
  int k = 0;
  auto start = std::chrono::system_clock::now();
  while(k < 300){
    auto end= std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    myfile << elapsed_seconds.count() <<",";
    myfile << set_point << ","; 
    for(int i = 0; i < 6; i++){
      myfile << (float)driver.getAdc().val[i] << "," <<fromBananasToBar(driver.getAdc().val[i])<< ","<< (float)driver.getAdc().val[8 + i]<<","<<(float)driver.getAdc().val[16 + i] << ",";
      std::cout << "BANANAS:"<<(float)driver.getAdc().val[i] << "\n\n";
      std::cout << "BAR:" <<fromBananasToBar(driver.getAdc().val[i]) << "\n\n\n";
    }
    myfile << "\n";
    driver.send_set_point(vals);
    MILLISLEEP(100);
    k++;
    std::cout << "\n\n \n K : " << k << "\n\n";
  }  

  uint32_t vals1[] = {0, 0, 0, 0, 0, 0,5000,0};
  set_point =0;
  k = 0;
  while(k < 150){
    auto end= std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    myfile << elapsed_seconds.count() <<",";
    myfile << set_point << ","; 
    for(int i = 0; i < 6; i++){
      myfile << (float)driver.getAdc().val[i] << "," <<fromBananasToBar(driver.getAdc().val[i])<< ","<< (float)driver.getAdc().val[8 + i]<<","<<(float)driver.getAdc().val[16 + i] << ",";
      std::cout << (float)driver.getAdc().val[i] << "\n\n";
      std::cout << "BAR:" <<fromBananasToBar(driver.getAdc().val[i]) << "\n\n\n";
    }

    myfile << "\n";
    driver.send_set_point(vals1);
    MILLISLEEP(100);
    k++;
    std::cout << "\n\n \n K : " << k << "\n\n";
  }  
  k = 0;
  set_point =70;
  while(k < 300){
    auto end= std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    myfile << elapsed_seconds.count() <<",";
    myfile << set_point << ","; 
    for(int i = 0; i <6; i++){
      myfile << (float)driver.getAdc().val[i] << "," <<fromBananasToBar(driver.getAdc().val[i])<< ","<< (float)driver.getAdc().val[8 + i]<<","<<(float)driver.getAdc().val[16 + i] << ",";
      std::cout << (float)driver.getAdc().val[i] << "\n\n";
      std::cout << "BAR:" <<fromBananasToBar(driver.getAdc().val[i]) << "\n\n\n";
    }
    myfile << "\n";
    driver.send_set_point(vals);
    MILLISLEEP(100);
    k++;
    std::cout << "\n\n \n K : " << k << "\n\n";
  }  
  myfile.close();
  return 0;
}