#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <string>


#include "hcb_protocol.h"
#include "hcbapi.h"         // Contain definitions and declarations related to a custom communication protocol
#include "unistd.h"         // Provides access to the POSIX operating system API.

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <signal.h>
// It is important ncurses is defined last... does strange things with "OK"
#include <ncurses.h>  // This line includes the ncurses library for creating text-based user interfaces in a console window.

#ifndef MILLISLEEP
#define MILLISLEEP(X)                        //Defination of macro which represents the number of milliseconds to sleep                                   
  do                                                                           
  {                                                                            
    std::chrono::milliseconds tim2((X));     //This represents the duration of time to sleep in milliseconds.          
    std::this_thread::sleep_for(tim2);       //Pause the execution of the current thread for the specified duration         
  } while (0)
#endif

class hcb_driver
{

public:

  // Constructor: Initializes member variables
  hcb_driver() : handle() { }

  // Destructor: Stops the communication thread and closes the device handle
  ~hcb_driver()
  {
    stopThread();
    hcb_close(handle);
  }

  // Connects to the device and starts the communication thread
  void connect()
  {
    handle = hcb_open(0x2fe3, 0x100);
    spawn_thread();
  }

  // Sends a "hello" command to the device
  void send_hello() { hcb_send_hello(handle); }

  // Sends a set point to the device
  void send_set_point(uint32_t val[])
  {
    // Use a mutex to protect shared resources during multithreaded access
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
                // Handle command message
              break;
            case HCB_ID_HELLO_REPLY:
                // Handle hello reply message
              break;
            case HCB_ID_UNSOLICITED_TIME:
                // Handle unsolicited time message
              break;
            case HCB_ID_ADC:{
                 // Use a mutex to protect shared resources during multithreaded access
              std::lock_guard<std::mutex> guard(mtx);
              memcpy(&this->currRead, &msg.adc, sizeof(hcb_msg_adc_t));
              break;
            }
            case HCB_ID_FT:
                // Handle force/torque message
              memcpy(&this->currftRead, &msg.ft, sizeof(hcb_msg_ft_t));
              break;
          }
        }
      }
    });
  }
  
  // Stops the communication thread
  void stopThread()
  {
    this->continuing = false;
    if (this->workerThread.joinable())
    {
      this->workerThread.join();
    }
  }
  
  // Gets the latest ADC (Analog-to-Digital Converter) data
  hcb_msg_adc_t getAdc()
  {
     // Use a mutex to protect shared resources during multithreaded access
    std::lock_guard<std::mutex> guard(mtx);
    return currRead;
  }

  // Gets the latest force/torque data
  hcb_msg_ft_t getFt() {return currftRead;}

private:
  void* handle;                          // Device handle
  std::thread workerThread {};           // Thread for communication
  std::atomic<bool> continuing { true }; // Atomic flag for thread continuation
  hcb_msg_adc_t currRead {};             // Latest ADC data
  std::mutex mtx;                        // Mutex for protecting shared resources
  hcb_msg_ft_t currftRead {};            // Latest force/torque data
};

// Instantiate an instance of the hcb_driver class
hcb_driver hcb;

// Additional variables for storing ADC and force/torque data
hcb_msg_adc_t currRead {};
hcb_msg_ft_t currftRead;

// Array for storing set point values
uint32_t val[8] { 0,0,0 ,0,0,0, 0,0};

/////////////////////////////////////////////////////////////
// Constant string representing the main title
const char mainTitle[] = "HCB MONITOR";

// Macro to calculate the length of the main title
#define TITLELEN (sizeof(mainTitle) / sizeof(char))

// Constant string representing an error message
const char errMsg[] = "Enlarge your Console !!!";

// Macro to calculate the length of the error message
#define ERRLEN (sizeof(errMsg) / sizeof(char))

// Macro to center the title or error message horizontally on the console
#define CENTER_X(dimx, len) \
  ((dimx) > ((int)len) ? (dimx) / 2 - ((int)len) / 2 : 0)


/////////////////////////////////////////////////////////////
/**
 * @brief Signal handler for SIGINT (Ctrl+C)
 *
 * @param[in] Unused parameter (required for signal handler)
 */
static void finish(int)
{
  endwin();    // Close the curses library
  /* do your non-curses wrapup here */
  exit(0);    // Exit the program with code 0
} 

/**
 *  Initialize the terminal screen using curses library
 */
void initScreen()
{
  initscr();                    // Initialize the curses library
  (void)signal(SIGINT, finish); /* arrange interrupts to terminate */  // Set the signal handler for SIGINT to the finish function
  keypad(stdscr, TRUE);         /* We get F1, F2 etc..          */     // Enable special keys like F1, F2, etc.
  noecho();                     // Don't echo keypresses to the screen
  curs_set(FALSE);              // Hide the cursor
  timeout(40);                  // Set a timeout for getch() to 40 milliseconds
  cbreak();                     // Disable line buffering (receive keypresses immediately)
}


// Enumeration representing UI states
enum UI_Top_State
{
  MAIN_MENU,
};

/**
 * @brief Structure representing the context of the terminal user interface (TUI)
 *
 */
struct Context //manualization of the terminal context
{
  UI_Top_State state = MAIN_MENU; // Initial state is MAIN_MENU
  std::string infoRow = "";       // String for informational messages
  std::string titleRow = "";      // String for title or header
  float lastTime { 0.f };         // Last timestamp
  float timestamp { 0.f };        // Current timestamp
  int32_t position { 0 };         // Position indicator
  bool warning { false };         // Flag for warning state
  bool error { false };           // Flag for error state
  int new_x {};                   // x dimension of the terminal
  int new_y {};                   // y dimension of the terminal
  bool logEnabled { false };      // Flag indicating if logging is enabled
  bool prevCalibOnGoing { false }; // Previous calibration state
  bool calibOnGoing { false };     // Current calibration state
};

static Context context; // Static instance of the Context structure

//******************************************
// MAIN_MENU STATUS

// Function to display the main menu and return the next row
int mainMenuDisplay(int firstRow)
{
  auto row = firstRow;
  return ++row;
}

// Constants defining the maximum and minimum values for microseconds
#define MAX_USEC 8330u
#define MIN_USEC 0u

// Functions to increment and decrement values by 10 and 1, respectively
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

// Function to handle user input in the main menu
void mainMenuInputHandler(int ch)
{
    // Switch statement to handle different keypresses
  switch (ch)
  {

    // Increment and decrement values for different parameters based on keypresses
    // 'w', 's', 'W', 'S' handle val[0]
    // 'e', 'd', 'E', 'D' handle val[1]
    // ...
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
    // Increment all values by 10
    for(int i = 0; i <= 5; i++){
      incv_10(val[i]);
    }
    break;

    case 'a':
     // Decrement all values by 10
    for(int i = 0; i <= 5; i++){
      decv_10(val[i]);
    }
    break;

    case 'Q':
    // Increment all values by 1
    for(int i = 0; i <= 5; i++){
      incv_1(val[i]);
    }
    break;

    case 'A':
    // Decrement all values by 1
    for(int i = 0; i <= 5; i++){
      decv_1(val[i]);
    }
    break;


    // Additional cases for special keys (not explained in the comments)
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

// Function to select and display the appropriate menu based on the UI state
int menuDisplay(int firstRow)
{
  switch (context.state)
  {
    case MAIN_MENU:
      // Call the mainMenuDisplay function and return the updated row
      return mainMenuDisplay(firstRow);
      break;
  }
  return firstRow;
}

/**/
// Function to display context-related information in the terminal
int displayContextRows(int firstRow)
{
    // Check if an update is needed
  if (context.lastTime <= context.timestamp)
    return firstRow;

  auto row = firstRow;

  // Display the title row with bold formatting
  attron(A_BOLD);
  mvprintw(row++, 2, context.titleRow.c_str());
  attroff(A_BOLD);

  // Display the information row
  mvprintw(row, 0, " %s ", context.infoRow.c_str());

  return ++row;
}

/**
 * @brief  Convert ADC values from bananas to bars
 *
 * @param firstRow  val_p ADC value in bananas
 * @return uint32_t ADC value in bars
 */
uint32_t fromBananasToBar(uint16_t val_p){
  int32_t valbar = ((val_p - 400)*350)/1600;
  if(valbar < 0) return 0;
  return (uint32_t) valbar;
}

uint32_t fromBananasTomillAmper(uint16_t val_c){
  return (uint32_t)((val_c * 1.25)/(0.05*20*2));
}
// Function to display periodic updates in the terminal
int periodicDisplay(int firstRow)
{
  auto row = firstRow;

  // Display ADC Data
  attron(A_BOLD);
  mvprintw(row++, 2, "Adc Data");
  attroff(A_BOLD);
   mvprintw(row++, 00, "            % 6d     % 6d     % 6d     % 6d     % 6d     % 6d     % 6d     % 6d",1,2,3,4,5,6,7,8);
  
  // Display ADC values converted to bars and milliamperes
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

  // Display Pwm values  
  attron(A_BOLD);
  row += 2;
  mvprintw(row++, 2, "Pwm");

  mvprintw(++row, 00, "adc [%d]   : % 6d     % 6d     % 6d     % 6d     % 6d    % 6d     % 6d     % 6d   ",
  1, currRead.val[0 + (1 * 8)], currRead.val[1 + (1 * 8)],
  currRead.val[2 + (1 * 8)], currRead.val[3 + (1 * 8)],
  currRead.val[4 + (1 * 8)], currRead.val[5 + (1 * 8)],
  currRead.val[6 + (1 * 8)], currRead.val[7 + (1 * 8)]);

  // Display Setpoint Data
  row += 2;
  mvprintw(row++, 2, "Setpoint Data");
  attroff(A_BOLD);
  mvprintw(++row, 00, "Setpoint [us]: % 6d  % 6d  % 6d  % 6d  % 6d  % 6d  % 6d  % 6d",
      val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
  mvprintw(++row, 00, "cmds                w/s     e/d     r/f    t/g     y/h     u/j     i/k     o/l ");


  // Display Force & Torque values
  row += 2;
  mvprintw(row++, 2, "Force & Torque");

  mvprintw(++row, 00, "F/T   : % 6d     % 6d     % 6d     % 6d     % 6d    % 6d ",
  1, currftRead.val[0], currftRead.val[1],
  currftRead.val[2], currftRead.val[3],
  currftRead.val[4], currftRead.val[5]);
  
  return ++row;
}
/**
 * @brief  Displays status lines at the bottom of the screen.
 *
 * @return int
 */
int statusLines()
{
  auto row = context.new_y - 1;

  // Display "Enable/Disable Log" status
  attron(A_BOLD | A_REVERSE);
  mvprintw(row, 0, "F1");
  attroff(A_BOLD | A_REVERSE);
  mvprintw(row, 2, context.logEnabled ? " Disable Log" : " Enable Log ");
  
  // Display "Main Menu" status
  attron(A_BOLD | A_REVERSE);
  mvprintw(row, 25, "m");
  attroff(A_BOLD | A_REVERSE);
  mvprintw(row, 26, " Main Menu");
  
  // Display "Quit" status
  attron(A_BOLD | A_REVERSE);
  mvprintw(row, 37, "q");
  attroff(A_BOLD | A_REVERSE);
  mvprintw(row, 38, " Quit");

  return 0;
}

/**
 * Draw the current screen based on the context and menu displays.
 **/
void drawScreen()
{
  int new_x, new_y;
  getmaxyx(stdscr, new_y, new_x);
  context.new_x = new_x;
  context.new_y = new_y;
 
  // If the screen size is too small, display an error message
  if (new_x < 80 || new_y < 25)
  {
    attron(A_BOLD);
    mvprintw(new_y / 2, CENTER_X(new_x, ERRLEN), errMsg);
    attroff(A_BOLD);
    return;
  }
  
  // Display the main title
  attron(A_BOLD);
  mvprintw(1, CENTER_X(new_x, TITLELEN), mainTitle);
  attroff(A_BOLD);

  // Display the menu and context information
  auto nextRow = menuDisplay(3);

  nextRow = periodicDisplay(++nextRow);

  nextRow = displayContextRows(++nextRow);
  
  // Display status lines at the bottom
  statusLines();
}

/**
 * @brief  Handles user input based on the current state.
 *
 * @param hemc 
 * @param ch The input chapter.
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
    // Toggle log status on F1
    context.logEnabled = !context.logEnabled;
    // TODO: logging stuff
    return true;

  } else if (ch == 'x') // Exit program on 'x'
  {
    return false;
  } else if (ch == 'm')
  {
    context.state = MAIN_MENU; // Switch to Main Menu on 'm'
    return true;
  };

  // Handle input based on the current state
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
 * @brief main loop utility for the HCB Monitor.
 **/
void hcb_loop()
{
  initScreen();

  int ch = 0;
  do
  {
    // Send set points, get ADC and Force/Torque data
    hcb.send_set_point(val);
    currRead = hcb.getAdc();
    currftRead = hcb.getFt();

    // Refresh the screen
    erase();
    drawScreen();

    refresh();
    // Get user input
    ch = getch();
  } while (handleInput(ch)); // Continue loop based on user input

  endwin();// End curses mode
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
    // Connect to the HCB board and start the main loop
    hcb.connect();
    hcb_loop();
  } catch (...)
  {
    std::cerr << "ERROR!" << std::endl;
    finish(0);
  }

  return 0;
}
