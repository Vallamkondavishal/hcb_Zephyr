#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <string>

#include "hcb_protocol.h"
#include "hcbapi.h"
#include "unistd.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <signal.h>
// It is important ncurses is defined last... does strange things with "OK"
#include <ncurses.h>

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

  void send_set_point(uint32_t val[])
  {
    std::lock_guard<std::mutex> guard(mtx);
    hcb_set(handle, val);
  }

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
              break;
            case HCB_ID_HELLO_REPLY:
              break;
            case HCB_ID_UNSOLICITED_TIME:
              break;
            case HCB_ID_ADC:{
              std::lock_guard<std::mutex> guard(mtx);
              memcpy(&this->currRead, &msg.adc, sizeof(hcb_msg_adc_t));
              break;
            }
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

  hcb_msg_adc_t getAdc()
  {
    std::lock_guard<std::mutex> guard(mtx);
    return currRead;
  }

  hcb_msg_ft_t getFt() {return currftRead;}

private:
  void* handle;
  std::thread workerThread {};
  std::atomic<bool> continuing { true };
  hcb_msg_adc_t currRead {};
  std::mutex mtx;
  hcb_msg_ft_t currftRead {};
};

hcb_driver hcb;
hcb_msg_adc_t currRead {};
hcb_msg_ft_t currftRead;
uint32_t val[8] { 0,0,0 ,0,0,0, 0,0};

/////////////////////////////////////////////////////////////
const char mainTitle[] = "HCB MONITOR";
#define TITLELEN (sizeof(mainTitle) / sizeof(char))

const char errMsg[] = "Enlarge your Console !!!";
#define ERRLEN (sizeof(errMsg) / sizeof(char))

#define CENTER_X(dimx, len)                                                    \
  ((dimx) > ((int)len) ? (dimx) / 2 - ((int)len) / 2 : 0)

/////////////////////////////////////////////////////////////
/**
 * @brief
 *
 */
static void finish(int)
{
  endwin();
  /* do your non-curses wrapup here */
  exit(0);
}

/**
 * Init the terminal screen
 */
void initScreen()
{
  initscr();
  (void)signal(SIGINT, finish); /* arrange interrupts to terminate */
  keypad(stdscr, TRUE);         /* We get F1, F2 etc..          */
  noecho();
  curs_set(FALSE);
  timeout(40);
  cbreak();
}

enum UI_Top_State
{
  MAIN_MENU,
};

/**
 * @brief stuff for the TUI
 *
 */
struct Context
{
  UI_Top_State state = MAIN_MENU;
  std::string infoRow = "";
  std::string titleRow = "";
  float lastTime { 0.f };
  float timestamp { 0.f };
  int32_t position { 0 };
  bool warning { false };
  bool error { false };
  int new_x {};              //< x dimension of the terminal
  int new_y {};              //< y dimension of the termanal
  bool logEnabled { false }; //< Am I logging
  bool prevCalibOnGoing { false };
  bool calibOnGoing { false };
};

static Context context;

//******************************************
// MAIN_MENU STATUS

int mainMenuDisplay(int firstRow)
{
  auto row = firstRow;
  return ++row;
}

#define MAX_USEC 8330u
#define MIN_USEC 0u

void incv_10(uint32_t& v)
{
  v += 10;
  v = std::clamp(v, MIN_USEC, MAX_USEC);
}

void incv_1(uint32_t& v)
{
  v += 1;
  v = std::clamp(v, MIN_USEC, MAX_USEC);
}

void decv_10(uint32_t& v)
{
  if (v >= 10)  v -= 10;
  v = std::clamp(v, MIN_USEC, MAX_USEC);
}
void decv_1(uint32_t& v)
{
  if (v >= 1)  v -= 1;
  v = std::clamp(v, MIN_USEC, MAX_USEC);
}

void mainMenuInputHandler(int ch)
{
  switch (ch)
  {
    case 'w':
      incv_10(val[0]);
      break;
    case 's':
      decv_10(val[0]);
      break;
    case 'W':
      incv_1(val[0]);
      break;
    case 'S':
      decv_1(val[0]);
      break;

    case 'e':
      incv_10(val[1]);
      break;
    case 'd':
      decv_10(val[1]);
      break;
    case 'E':
      incv_1(val[1]);
      break;
    case 'D':
      decv_1(val[1]);
      break;

    case 'r':
      incv_10(val[2]);
      break;
    case 'f':
      decv_10(val[2]);
      break;
    case 'R':
      incv_1(val[2]);
      break;
    case 'F':
      decv_1(val[2]);
      break;

    case 't':
      incv_10(val[3]);
      break;
    case 'g':
      decv_10(val[3]);
      break;
    case 'T':
      incv_1(val[3]);
      break;
    case 'G':
      decv_1(val[3]);
      break;

    case 'y':
      incv_10(val[4]);
      break;
    case 'h':
      decv_10(val[4]);
      break;
    case 'Y':
      incv_1(val[4]);
      break;
    case 'H':
      decv_1(val[4]);
      break;

    case 'u':
      incv_10(val[5]);
      break;
    case 'j':
      decv_10(val[5]);
      break;
    case 'U':
      incv_1(val[5]);
      break;
    case 'J':
      decv_1(val[5]);
      break;

    case 'i':
      incv_10(val[6]);
      break;
    case 'k':
      decv_10(val[6]);
      break;
    case 'I':
      incv_1(val[6]);
      break;
    case 'K':
      decv_1(val[6]);
      break;

    case 'o':
      incv_10(val[7]);
      break;
    case 'l':
      decv_10(val[7]);
      break;
    case 'O':
      incv_1(val[7]);
      break;
    case 'L':
      decv_1(val[7]);
      break;
    
    case 'q':
    for(int i = 0; i <= 5; i++){
      incv_10(val[i]);
    }
    break;
    case 'a':
    for(int i = 0; i <= 5; i++){
      decv_10(val[i]);
    }
    break;
    case 'Q':
    for(int i = 0; i <= 5; i++){
      incv_1(val[i]);
    }
    break;
    case 'A':
    for(int i = 0; i <= 5; i++){
      decv_1(val[i]);
    }
    break;


    case KEY_F(3):
      break;
    case KEY_F(4):
      break;
    case KEY_F(5):
      MILLISLEEP(5);
      break;
    case KEY_F(6):
      finish(0);
      MILLISLEEP(5);
      break;
    case KEY_F(7):
      break;
    default:
      break;
  }
  return;
}

//####################
//******************************************

int menuDisplay(int firstRow)
{
  switch (context.state)
  {
    case MAIN_MENU:
      return mainMenuDisplay(firstRow);
      break;
  }
  return firstRow;
}

/**/
int displayContextRows(int firstRow)
{
  if (context.lastTime <= context.timestamp)
    return firstRow;

  auto row = firstRow;
  attron(A_BOLD);
  mvprintw(row++, 2, context.titleRow.c_str());
  attroff(A_BOLD);

  mvprintw(row, 0, " %s ", context.infoRow.c_str());

  return ++row;
}

/**
 * @brief
 *
 * @param firstRow
 * @return int
 */
uint32_t fromBananasToBar(uint16_t val_p){
  int32_t valbar = ((val_p - 400)*350)/1600;
  if(valbar < 0) return 0;
  return (uint32_t) valbar;
}

uint32_t fromBananasTomillAmper(uint16_t val_c){
  return (uint32_t)((val_c * 1.25)/(0.05*20*2));
}
int periodicDisplay(int firstRow)
{
  auto row = firstRow;
  attron(A_BOLD);
  mvprintw(row++, 2, "Adc Data");
  attroff(A_BOLD);
   mvprintw(row++, 00, "            % 6d     % 6d     % 6d     % 6d     % 6d     % 6d     % 6d     % 6d",1,2,3,4,5,6,7,8);
  

  mvprintw(++row, 00, "adc [%d]   : % 6d bar % 6d bar % 6d bar % 6d bar % 6d bar % 6d bar % 6d bar % 6d bar",
  0, fromBananasToBar(currRead.val[0]), fromBananasToBar(currRead.val[1]),
  fromBananasToBar(currRead.val[2]), fromBananasToBar(currRead.val[3]),
  fromBananasToBar(currRead.val[4]), fromBananasToBar(currRead.val[5]),
  fromBananasToBar(currRead.val[6]), fromBananasToBar(currRead.val[7]));

  mvprintw(++row, 00, "adc [%d]   : % 6d mA  % 6d mA  % 6d mA  % 6d mA  % 6d mA  % 6d mA  % 6d mA  % 6d mA",
    2, fromBananasTomillAmper(currRead.val[0 + (2 * 8)]), fromBananasTomillAmper(currRead.val[1 + (2 * 8)]),
    fromBananasTomillAmper(currRead.val[2 + (2 * 8)]), fromBananasTomillAmper(currRead.val[3 + (2 * 8)]),
    fromBananasTomillAmper(currRead.val[4 + (2 * 8)]), fromBananasTomillAmper(currRead.val[5 + (2 * 8)]),
    fromBananasTomillAmper(currRead.val[6 + (2 * 8)]), fromBananasTomillAmper(currRead.val[7 + (2 * 8)]));
  attron(A_BOLD);
  row += 2;
  mvprintw(row++, 2, "Pwm");

  mvprintw(++row, 00, "adc [%d]   : % 6d     % 6d     % 6d     % 6d     % 6d    % 6d     % 6d     % 6d   ",
  1, currRead.val[0 + (1 * 8)], currRead.val[1 + (1 * 8)],
  currRead.val[2 + (1 * 8)], currRead.val[3 + (1 * 8)],
  currRead.val[4 + (1 * 8)], currRead.val[5 + (1 * 8)],
  currRead.val[6 + (1 * 8)], currRead.val[7 + (1 * 8)]);

  row += 2;
  mvprintw(row++, 2, "Setpoint Data");
  attroff(A_BOLD);
  mvprintw(++row, 00, "Setpoint [us]: % 6d  % 6d  % 6d  % 6d  % 6d  % 6d  % 6d  % 6d",
      val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
  mvprintw(++row, 00, "cmds                w/s     e/d     r/f    t/g     y/h     u/j     i/k     o/l ");
  
  row += 2;
  mvprintw(row++, 2, "Force & Torque");

  mvprintw(++row, 00, "F/T   : % 6d     % 6d     % 6d     % 6d     % 6d    % 6d ",
  1, currftRead.val[0], currftRead.val[1],
  currftRead.val[2], currftRead.val[3],
  currftRead.val[4], currftRead.val[5]);
  
  return ++row;
}
/**
 * @brief
 *
 * @return int
 */
int statusLines()
{
  auto row = context.new_y - 1;

  attron(A_BOLD | A_REVERSE);
  mvprintw(row, 0, "F1");
  attroff(A_BOLD | A_REVERSE);
  mvprintw(row, 2, context.logEnabled ? " Disable Log" : " Enable Log ");

  attron(A_BOLD | A_REVERSE);
  mvprintw(row, 25, "m");
  attroff(A_BOLD | A_REVERSE);
  mvprintw(row, 26, " Main Menu");

  attron(A_BOLD | A_REVERSE);
  mvprintw(row, 37, "q");
  attroff(A_BOLD | A_REVERSE);
  mvprintw(row, 38, " Quit");

  return 0;
}

/**
 * Draw the current screen
 **/
void drawScreen()
{
  int new_x, new_y;
  getmaxyx(stdscr, new_y, new_x);
  context.new_x = new_x;
  context.new_y = new_y;

  if (new_x < 80 || new_y < 25)
  {
    attron(A_BOLD);
    mvprintw(new_y / 2, CENTER_X(new_x, ERRLEN), errMsg);
    attroff(A_BOLD);
    return;
  }

  attron(A_BOLD);
  mvprintw(1, CENTER_X(new_x, TITLELEN), mainTitle);
  attroff(A_BOLD);

  auto nextRow = menuDisplay(3);

  nextRow = periodicDisplay(++nextRow);

  nextRow = displayContextRows(++nextRow);

  statusLines();
}

/**
 * @brief
 *
 * @param hemc
 * @param ch
 * @return true continue program
 * @return false exit from the program
 */
bool handleInput(int ch)
{
  // If no input returns
  if (ch < 1)
    return true;

  // Global Commands

  if (ch == KEY_F(1))
  {
    context.logEnabled = !context.logEnabled;
    // TODO logging stuff
    return true;

  } else if (ch == 'x')
  {
    return false;
  } else if (ch == 'm')
  {
    context.state = MAIN_MENU;
    return true;
  };
  switch (context.state)
  {
    case MAIN_MENU:
      mainMenuInputHandler(ch);
      break;
    default:
      break;
  }
  return true;
}

/**
 * @brief main loop utility
 **/
void hcb_loop()
{
  initScreen();

  int ch = 0;
  do
  {
    hcb.send_set_point(val);
    currRead = hcb.getAdc();
    currftRead = hcb.getFt();
    erase();
    drawScreen();

    refresh();
    ch = getch();
  } while (handleInput(ch));

  endwin();
}

/**
 * @brief Cli HEMC Monitor Utility
 * Interact via CAN with the HEMC board via
 * a ncurses enabled CLI
 */
int main(int, char**)
{
  
  try
  {
    hcb.connect();
    hcb_loop();
  } catch (...)
  {
    std::cerr << "ERROR!" << std::endl;
    finish(0);
  }

  return 0;
}
