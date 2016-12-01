/*
 * @author JACOB AYLWRD
 * @brief This program is a starting point for the briefcase assignment for EN0572.
 *        The project must be inside the EN0572 workspace in order to build correctly.
 *        A basic framework for a uCOS-II program is provided. There are some task declarations
 *        and some use of devices. These are not meant to be definitive. You should modify the
 *				program structure to suit your requirements. Some tasks may not be required. Other tasks
 *				may need to be added. You will certainly need to add some concurrency control.
 */

#include <stdbool.h>	//Boolean type and values
#include <ucos_ii.h>	//Real-time deterministic multitasking kernel for microcontroller
#include <mbed.h>			//SDK provides software platform and libraries
#include <display.h>	//Graphic Display Manager
#include "buffer.h"		
#include <MMA7455.h>
#define ACCEL_DIFFERENCE 15 //Accelerometer threshold difference
//#include "LPC407x_8x_177x_8x.h"	//Device Header
//#include "RTE_Components.h"			//Component Selection

/*
*********************************************************************************************************
*                                            APPLICATION TASK PRIORITIES
*********************************************************************************************************
*/

typedef enum {	//Enumeration APP_TASK Priority: BUTTONS, POT, LED1&2, LCD, TIME, ACC
	APP_TASK_BUTTONS_PRIO = 7,
	APP_TASK_POT_PRIO,	//Potentiometer knob provides variable resistance
  APP_TASK_LED1_PRIO,	//LED1
  APP_TASK_LED2_PRIO,	//LED2
	APP_TASK_LCD_PRIO,	//Liquid crystal Display
	APP_TASK_TIME_PRIO,	//Time
	APP_TASK_ACC_PRIO		//Accelerometer measuring the acceleration of a moving body
} taskPriorities_t; 	//Change Priority of task t
// Output Task
//#define APP_TASK_LCD_PRIO 
//#define APP_TASK_LED1_PRIO
//#define APP_TASK_LED2_PRIO

/*
*********************************************************************************************************
*                                            APPLICATION TASK STACKS
*********************************************************************************************************
*/
//Buttons event: STK size
#define  APP_TASK_BUTTONS_STK_SIZE           256 //pdata is limited to 256 bytes
//Potentiometer knob event: STK size 
#define  APP_TASK_POT_STK_SIZE               256 //pdata is limited to 256 bytes
//LED1 event: STK size
#define  APP_TASK_LED1_STK_SIZE              256 //pdata is limited to 256 bytes
//LED2 event: STK size
#define  APP_TASK_LED2_STK_SIZE  						 256 //pdata is limited to 256 bytes
//Liquid Crystal Display event: STK size
#define  APP_TASK_LCD_STK_SIZE               256 //pdata is limited to 256 bytes
//Time event: STK size
#define  APP_TASK_TIME_STK_SIZE              256 //pdata is limited to 256 bytes
//Accelerometer event: STK size
#define  APP_TASK_ACC_STK_SIZE               256 //pdata is limited to 256 bytes
//Each task has stack entries of this type. (located src/uC/os_cpu.h)
static OS_STK appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE];	//Buttons stack 				[256]
static OS_STK appTaskPotStk[APP_TASK_POT_STK_SIZE];					//Potentiometer stack [256]
static OS_STK appTaskLED1Stk[APP_TASK_LED1_STK_SIZE];				//LED1 stack					[256]
static OS_STK appTaskLED2Stk[APP_TASK_LED2_STK_SIZE];				//LED2 stack					[256]
static OS_STK appTaskLCDStk[APP_TASK_LCD_STK_SIZE];					//LCD stack						[256]
static OS_STK appTaskTimeStk[APP_TASK_TIME_STK_SIZE];				//Time stack					[256]
static OS_STK appTaskAccStk[APP_TASK_ACC_STK_SIZE];					//Accelerometer stack	[256]

/*
*********************************************************************************************************
*                                            APPLICATION FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void appTaskButtons(void *pdata);  //Memory type declare variable Buttons
static void appTaskPot(void *pdata);			//Memory type declare variable Potentiometer
static void appTaskLED1(void *pdata);			//Memory type declare variable LED1
static void appTaskLED2(void *pdata);			//Memory type declare variable Led2
static void appTaskLCD(void *pdata); 			//Memory type declare variable LCD 
static void appTaskTime(void *pdata);			//Memory type declare variable Time
static void appTaskAcc(void *pdata);			//Memory type declare variable Accelerometer

/*
*********************************************************************************************************
*                                            APPLICATION TASK FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            GLOBAL TYPES AND VARIABLES 
*********************************************************************************************************
*/

typedef enum { 	//Enumeration RINGBUFFER: LED1&2, POT, BUTTONS, ACC, TIME,
	RB_LED1 = 0,	//Ringbuffer (FIFO-queue) LED1
	RB_LED2,			//Ringbuffer (FIFO-queue) LED2
	RB_POT,				//Ringbuffer (FIFO-queue) Potentiometer
	RB_BUTTONS,		//Ringbuffer (FIFO-queue) Buttons
	RB_ACC,				//Ringbuffer (FIFO-queue) Accelerometer
	RB_TIME,			//Ringbuffer (FIFO-queue) Time
} deviceNames_t;//Device Names generical string

typedef enum {	//Enumeration JOYSTICK: JLEFT, JRIGHT, JUP, JDOWN, JCENTER
	JLEFT = 0,		//Joystick JLEFT 		= LEFT
	JRIGHT,				//Joystick JRIGHT 	= RIGHT
	JUP,					//Joystick JUP			= UP
	JDOWN,				//Joystick JDOWN		= DOWN
	JCENTER				//Joystick JCENTER	= Center
} buttonId_t;		//buttonID_t Button ID generical string

enum { //Enumeration FLASH DELAY: MIN, INITIAL, MAX, STEP
	//1 Second = 1000 Milliseconds
	FLASH_MIN_DELAY     = 1, 		//MIN_DELAY 		= 0.001 Seconds
	FLASH_INITIAL_DELAY = 500, 	//INITIAL_DELAY = 0.5 Seconds
	FLASH_MAX_DELAY     = 1000, //MAX_DELAY			=	1.0 Seconds 
	FLASH_DELAY_STEP    = 50 		//DELAY_STEP		= 0.05 Seconds
};

//static bool briefcaseLocked 			= false; //briefcaseLocked 			DEFAULT = FALSE
//static bool briefcaseMoving 			= false; //briefcaseMoving 			DEFAULT = FALSE
//static bool briefcaseMovingOnce 	= false; //briefcaseMovingOnce 	DEFAULT = FALSE
//static bool LED1Flashing 					= false; //LED1Flashing					DEFAULT = FALSE
//static bool LED2Flashing					= false; //LED2Flashing					DEFAULT = FALSE
static bool buttonPressedAndReleased(buttonId_t button); //Button Pressed detected
static void incDelay(void);															 //Increment delay .05 Seconds
static void decDelay(void);															 //Decrement delay .05 Seconds
static void barChart(float value); 											 //Float-point value for (+-) .05
//Configuring Accelerometer
MMA7455 acc(P0_27, P0_28);	//ModeMeasurement & Range
bool accInit(MMA7455& acc); //Display true/false
//Exact-width integer type: typedef signed long int int32_t
int32_t accVal1;						//accVal1 = x-coordinate 
int32_t accVal2; 						//accVal2 = y-coordinate
int32_t accVal3;						//accVal3 = z-coordinate

float potVal;								//Potentiometer value Float-point value

/*
*********************************************************************************************************
*                                       ADDITIONAL GLOBAL TYPES AND VARIABLES 
*********************************************************************************************************
*/
//'locked' character string size decleration
char locked[10]; 				//Character variable 'locked' Array size 10
//'moving' character string size decleration
char moving[10];				//Character variable 'moving' Array size 10
//'alarm' character string size decleration
char alarm[3];					//Character variable 'alarm' Array size 3
//'buttonPressed' button pressed released event 
int buttonPressed;			//Integer type variable 'buttonPressed'
//'interval' variable used for the potentiometer value formula 
float interval;					//Float variable 'interval'
//'timer' variable used for alarm formula holds float value
float timer;						//Float variable 'timer'
//'countTimer' variable decleration start value
float countTimer	= 0;	//Float cariable 'CountTimer' equal to 0
//'lock' variable decleration start value
int lock 					= 0;	//Integer type variable 'lock' equal to 0
//'secure' variable decleration start value
int secure				= 0;	//Integer type variable 'secure' equal to 0
//'bMoving' board movement detection variable 
int bMoving;						//Integer type 'bMoving'

//Joystick/Locking variables 
int count		= 1;	//Increment value decrement value by 1 
int first		= 0;	//DEFAULT first digit  	= 0
int second	= 0;	//DEFAULT second digit 	= 0
int third		= 0;	//DEFAULT third digit		= 0
int fourth	= 0;	//DEFAULT fourth digit 	= 0

//Current 4-digit pin (1234)
int digitOne		= 1; //pin digit 1 set to 1
int digitTwo		= 2; //pin digit 2 set to 2
int digitThree	= 3; //pin digit 3 set to 3
int digitFour		= 4; //pin digit 4 set to 4

static DigitalOut led1(P1_18); //Configure digital output pin LED1
static DigitalOut led2(P0_13); //Configure digital output pin LED2
static DigitalIn buttons[] = {P5_0, P5_4, P5_2, P5_1, P5_3}; // Read values of digital input pins
//P5_0,1,2,3,4 = {LEFT, RIGHT, UP, DOWN, CENTER}
static AnalogIn potentiometer(P0_23); //Read external voltage applied to the analog input pin potentiometer
static Display *d = Display::theDisplay();
//LED Alarm flashing
static bool flashing[2] = {false, false};
static int32_t flashingDelay[2] = {FLASH_INITIAL_DELAY, FLASH_INITIAL_DELAY};

/*
*********************************************************************************************************
*                                            GLOBAL FUNCTION DEFINITIONS
*********************************************************************************************************
*/

int main() {
  d->fillScreen(BLACK);						//LCD Screen color 
	d->setTextColor(GREEN, BLACK);	//LCD Text color
  /* Initialise the OS */
  OSInit();                                                   
 	
  /* Create the tasks */
  OSTaskCreate(appTaskButtons,                               
               (void *)0,
               (OS_STK *)&appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE - 1],
               APP_TASK_BUTTONS_PRIO); //Global task Buttons created 
  OSTaskCreate(appTaskPot,                               
               (void *)0,
               (OS_STK *)&appTaskPotStk[APP_TASK_POT_STK_SIZE - 1],
               APP_TASK_POT_PRIO); //Global task Potentiometer created
	OSTaskCreate(appTaskLED1,                               
               (void *)0,
               (OS_STK *)&appTaskLED1Stk[APP_TASK_LED1_STK_SIZE - 1],
               APP_TASK_LED1_PRIO); //Global task LED1 created
  OSTaskCreate(appTaskLED2,                               
               (void *)0,
               (OS_STK *)&appTaskLED2Stk[APP_TASK_LED2_STK_SIZE - 1],
               APP_TASK_LED2_PRIO); //Global task LED2 created
  OSTaskCreate(appTaskLCD,                               
               (void *)0,
               (OS_STK *)&appTaskLCDStk[APP_TASK_LED2_STK_SIZE - 1],
               APP_TASK_LCD_PRIO);	//Global task LCD created 
	OSTaskCreate(appTaskTime,                               
               (void *)0,
               (OS_STK *)&appTaskTimeStk[APP_TASK_TIME_STK_SIZE - 1],
               APP_TASK_TIME_PRIO);	//Global task Time created
	OSTaskCreate(appTaskAcc,                               
               (void *)0,
               (OS_STK *)&appTaskAccStk[APP_TASK_ACC_STK_SIZE - 1],
               APP_TASK_ACC_PRIO);	//Global task Accelerometer created							 
  safeBufferInit(); //Naive buffer function implement operations to the buffer
	//IF STATEMENT: if Accelerometer is created (above OSTaskCreate) print "Accelerometer initialised"					 
	if (accInit(acc)) {
		d->printf("MSG: Accelerometer Initialised");
	} 	
	//ELSE STATEMENT: else error message print "Could not initialise accelerometer"
	else {
    d->printf("MSG: Accelerometer Failed To Initialise");
	}
	
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive to this segment of code!!! */ 
  return 0;      
}

/*
*********************************************************************************************************
*                                            APPLICATION TASK DEFINITIONS
*********************************************************************************************************
*/

static void appTaskButtons(void *pdata) {
  /* Start the OS ticker -- must be done in the highest priority task */
  SysTick_Config(SystemCoreClock / OS_TICKS_PER_SEC); //Generates interrupt request on a regular basis
	//Context switching is required here inorder to support multiple tasking
  /* Application task main loop */ 
	//WHILE LOOP: Button pressed/released event
	while (true) {
		 message_t msg;
			if (buttonPressedAndReleased(JLEFT)) {
				buttonPressed = 1; //Left Joystick Pressed/Released
			}
			else if (buttonPressedAndReleased(JRIGHT)) { 
				buttonPressed = 2; //Right Joystick Pressed/Released
			}
			else if (buttonPressedAndReleased(JUP)) {
				buttonPressed = 3; //Up Joystick Pressed/Released
			}
			else if (buttonPressedAndReleased(JDOWN)) {
				buttonPressed = 4; //Down Joystick Pressed/Released
			}
			else if (buttonPressedAndReleased(JCENTER)) {
				buttonPressed = 5; //Center Joystick Pressed/REleased
			}	
		msg.id = RB_BUTTONS; 					//Ringbuffer BUTTONS message ID
		msg.data[0] = buttonPressed; 	//Message data Contains Array 0 
		safeBufferPut(&msg); 					//Naive buffer implements operation add/put to buffer
    OSTimeDlyHMSM(0,0,0,100); 		//Hours Minutes Seconds Milliseconds(100)
  }
}

static void appTaskAcc(void *pdata) {
	//WHILE LOOP: Accelerometer TRUE = movement detected (bMoving) 
	while(true) {
		acc.read(accVal1, accVal2, accVal3); //Accelerometer reads x,y,z values
		//IF STATEMENT:((y-coordinate > 10)  absolute 
		//							(x-coordinate > 10)  absolute
		//							(y-coordinate < -10) absolute
		//							(x-coordinate < -10) absolute
		//							(z-coordinate > 20))
    if ((accVal2 > 10) || (accVal1  > 10) || (accVal2 < -10) || (accVal1  < -10) || (accVal3  > 20)){
			message_t msg;						//txt: Moving
			msg.id = RB_ACC; 					//Ringbuffer Accelerometer message ID
			msg.fdata[0] = 1;					//(bMoving) Movement therefore text reads 'Moving'
			safeBufferPut(&msg);			//Naive buffer implements operation add/put to buffer
	    OSTimeDlyHMSM(0,0,0,100);	//Hours Minutes Seconds Milliseconds(100)
		}
		//ELSE STATEMENT:((y-coordinate < 10)  and
		//								(x-coordinate < 10)  and
		//								(y-coordinate > -10) and
		//								(x-coordinate > -10) and
		//								(z-coordinate < 20))
		else if ((accVal2 < 10) && (accVal1  < 10) && (accVal2 > -10) && (accVal1  > -10) && (accVal3  < 20)){
			message_t msg; 						//txt. Not Moving
			msg.id = RB_ACC; 					//Ringbuffer Accelerometer message ID
			msg.fdata[0] = 0;					//(bMoving) No Movement therefore text reads 'Not Moving'
			safeBufferPut(&msg);			//Naive buffer implements operation add/put to buffer
	    OSTimeDlyHMSM(0,0,0,100); //Hours Minutes Seconds Milliseconds(100)
		}
   }
}   

static void appTaskLED1(void *pdata) {
	message_t msg;
	//WHILE LOOP: Implement flashing LED1 while true
  while (true) {
		//IF STATEMENT: 
		if (flashing[0]) {
      led1 = !led1; //LED1 = ¬LED1 (loop LED on/off)
		}
		msg.id = RB_LED1;								//Ringbuffer LED1 message ID
		msg.data[0] = flashing[0];			//Temp loop rotation to implement delay
		msg.data[1] = flashingDelay[0];	//Temp loop cont.
		safeBufferPut(&msg);						//Naive buffer implements operation add/put to buffer
    OSTimeDly(flashingDelay[0]);		//.5 sec
  }
}

static void appTaskLED2(void *pdata) {
	message_t msg;
	//WHILE LOOP: Implement flashing LED2 while true
  while (true) {
		//IF STATEMENT:
		if (flashing[1]) {
      led2 = !led2; //LED2 = ¬LED2 (loop LED on/off)
		}
		msg.id = RB_LED2;								//Ringbuffer LED2 message ID
		msg.data[0] = flashing[1];			//Temp loop rotation to implement delay
		msg.data[1] = flashingDelay[1];	//Temp loop cont. 
		safeBufferPut(&msg);						//Naive buffer implements operation add/put to buffer
    OSTimeDly(flashingDelay[1]);		//.5 sec
  } 
} 
  
static void appTaskPot(void *pdata) {
	//WHILE LOOP: potentiameter movement  = TRUE
	while (true) {
		message_t msg;
		potVal = 1.0F - potentiometer.read(); //Potentiameter value = Float-point value 1.0 - potentiameter.read()
		interval = ((potVal*110)+10);					//Potentiameter value 120 
		msg.id = RB_TIME;											//Ringbuffer TIME message ID
		msg.fdata[0] = interval;							//Message fdata equal 'interval' (Float-point)
		msg.id = RB_POT; 											//Ringbuffer potentiameter message ID
		safeBufferPut(&msg);									//Naive buffer implements operation and/put to buffer
	  OSTimeDlyHMSM(0,0,0,100);							//Hour Minute Second Millisecond(100)
		}			
	}                       
//TODO: implement while loop so that interval cannot be changed while board is locked
static void appTaskTime(void *pdata) {
	//WHILE LOOP: Time (Alarm timer) 
	while(true){
		message_t msg;
		//Potentiameter: Ringbuffer 'RB_POT' and 'RB_TIME' swap values
		potVal = 1.0F - potentiometer.read(); //Potentiameter value = Float-point value 1.0 - potentiameter.read()
		interval = ((potVal*110)+10);					//Potentiameter MAX value = 120 
		msg.id = RB_TIME;											//Ringbuffer TIME message ID
		msg.fdata[0] = interval;							//Message fdata equal 'interval' (float-point)
		safeBufferPut(&msg);									//Naive buffer implements operation and/put to buffer
		OSTimeDlyHMSM(0,0,1,0);								//Hour Minute Second(1) Millisecond
	}
}

static void appTaskLCD(void *pdata) {
	uint8_t status; //Exact-width integer type: typedef signed long int int32_t 'status'
	message_t msg;
	//strcpy STRING copy 
	strcpy(alarm , "OFF"); 									//STRING alarm replaces "Alarm" to "OFF"
	strcpy(locked , "UNLOCKED");  					//STRING locked replaces "locked" to "unlocked"
	d->fillScreen(BLACK);										//LCD Screen color
	d->setTextColor(GREEN, BLACK);					//LCD Text color
	d->setCursor(160, 70);									//"Assignment EN0572" display location
	d->printf("Assignement EN0572");				//"Assignment EN0572" print statement
	d->setCursor(167,85);										//"Jacob Aylward" display location
	d->printf("Jacob Aylward");							//"Jacob Aylward" print statement
	d->drawRect(100, 100, 180, 100, GREEN); //Rectangle Color and display location/size
  while (true) {
		//WHILE LOOP:
	d->setCursor(145, 115);												//"Alarm" display location
	d->printf("Alarm: %s", alarm); 								//Print "Alarm: __"
	d->setCursor(145, 125);												//"Interval" display location
	d->printf("Interval: %3.0F\n", interval);	 		//FIX FLASHING VALUES WHEN UNLOCKED	
	d->setCursor(145, 135);												//"Time" display location 
	d->printf("Time: %3.0F / %3.0F\n", countTimer, interval);//Print "Time: _/_"
	d->setCursor(145, 145);												//"Case" display location
	d->printf("Case: %s %s",locked); 							//Print "Case: ______"
	d->setCursor(145, 155);												//"Briefcase" display location
	d->printf("Briefcase: %s",moving); 						//Print "Briefcase: __"
	d->setCursor(145, 165);												//"Code" display location
	d->printf("Code: %d %d %d %d", first, second, third, fourth);//Print "Code: 0 0 0 0"
	d->drawRect(140, 100, 180, 100, GREEN);				//Rectangle color and display location/size
	d->setCursor(100,200);												//"Acc" display location
  d->printf("Acc = (%05d, %05d, %05d)", accVal1, accVal2, accVal3);//Print "Acc = (_,_,_)"
	safeBufferGet(&msg);													//Naive buffer implements operation and/put to buffer
		switch (msg.id) { //Construnct test variable against set of constants
			case RB_BUTTONS : { //Case 1: Ringbuffer Buttons 
				//IF STATEMENT: if msg.data (buttons) equals 3, secure status = FALSE 
				if ((msg.data[0] == 3) && (secure == 0)){
					lock = 1;	//Variable 'lock' equals 1 
					strcpy (locked , "LOCKED  "); //STRING copy locked replaces "__" to "LOCKED  "
					break; 
				}
				//ELSE IF STATEMENT: if msg.data (buttons) equals 4, secure status = FALSE
				else if((msg.data[0] == 4) && (secure == 0)){
					lock = 0; //Variable 'lock' equals 0
					strcpy (locked , "UNLOCKED"); //STRING copy locked replaces "__" to "UNLOCKED"
					break;
				}
				//ELSE IF STATEMENT: if msg.data (buttons) equals 2 and variable lock equals 1 then secure equals true
				else if((msg.data[0] == 2) && (secure == 0) && (lock == 1)){				
					secure = true; //Sets variable secure (bool) = TRUE
					break;
				}
				//IF STATEMENT: if variable count greater(1) and variable secure equals 1 and msg.data equals 2 count(decrements)
				if((count > 1) && (secure == 1) && (msg.data[0] == 2)){
					count--; //Count decrements
					buttonPressed = 0;
				}
				//ELSE IF STATEMENT: if variable count less(4) and secure equals 1 and msg.data equals 1 count(increments)
				else if((count < 4) && (secure == 1) && (msg.data[0] == 1)){
					count++; //Count increments
					buttonPressed = 0;
				}
				//IF STATEMENT: if variable secure equals 1 and msg.data equals 3 (enter if statement) for incrementing passcode digits
				if ((secure == 1) && (msg.data[0] == 3)) {
					//IF STATEMENT: if variable count equals 1 and first(pin digit variable) less(9)
					if((count == 1) && (first < 9)){
					first++; //Increment first pin digit by 1, 0->9
					buttonPressed = 0;
					}
					//ELSE IF STATEMENT: if variable count equals 2 and second(pin digit variable) less(9)
					else if((count == 2) && (second < 9)){
					second++; //Increment second pin digit by 1, 0->9
					buttonPressed = 0;	
					}
					//ELSE IF STATEMENT: if variable count equals 3 and third(pin digit variable) less(9)
					else if((count == 3) && (third < 9)){
					third++; //Increment third pin digit by 1, 0->9
					buttonPressed = 0;	
					}
					//ELSE IF STATEMENT: if variable count equals 4 and fourth(pin digit variable) less(9)
					else if((count == 4) && (fourth < 9)){
					fourth++; //Increment fourth pin digit by 1, 0->9
					buttonPressed = 0;	
					}
				}
				//IF STATEMENT: if variable secure equals 1 and msg.data equals 4 (enter if statement) for decrementing passcode digits
				if((secure == 1) && (msg.data[0] == 4)) {
					//IF STATEMET: if variable count equals 1 and first(pin digit variable) greater(0)
					if((count == 1) && (first > 0)){
					first--;//Decrement first pin digit by 1, 0<-9
					buttonPressed = 0;
					}
					//ELSE IF STATEMENT: if variable count equals 2 and second(pin digit variable) greater(0)
					else if((count == 2) && (second > 0)){
					second--;//Decrement second pin digit by 1, 0<-9
					buttonPressed = 0;
					}
					//ELSE IF STATEMENT: if variable count equals 3 and second(pin digit variable) greater(0)
					else if((count == 3) && (third > 0)){
					third--;//Decrement third pin digit by 1, 0<-9
					buttonPressed = 0;
					}
					//ELSE IF STATEMENT: if variable count equals 4 and second(pin digit variable) greater(0)
					else if((count == 4) && (fourth > 0)){
					fourth--;//Decrement fourth pin digit by 1, 0<-9
					buttonPressed = 0;
					}
				}
				//IF STATEMENT: DEFAULT setup 
				if((secure == 1) && (msg.data[0] == 5)) {
					if((first == digitOne) && (second == digitTwo)&& (third == digitThree)&& (fourth == digitFour)){
						first 	= 0; //Variable 'first' 	DEFAULT = 0
						second 	= 0; //Variable 'second'	DEFAULT = 0
						third 	= 0; //Variable 'third'		DEFAULT = 0
						fourth 	= 0; //Variable 'fourth'	DEFAULT = 0
						secure 	= 0; //Variable 'secure'	DEFAULT = 0
						lock 		= 0; //Variable 'lock'		DEFAULT = 0
						countTimer = 0;								//countTimer 	DEFAULT = 0
						count = 1;										//count				DEFAULT = 1
						bMoving = 0;									//bMoving			DEFAULT = 0
						flashing[0] = flashing[0];		
						flashing[1] = flashing[1];
						strcpy (locked , "LOCKED  "); //STRING copy variable 'locked' display "LOCKED" (DEFAULT)
						strcpy (alarm , "OFF");				//STRING copy variable 'alarm' display "OFF" (DEFAULT)
						buttonPressed = 0;
						d->setCursor(145, 175); 			//Display clean slate (this is bad coding)
						d->printf("                ");//Print blank
					}
					//ELSE STATEMENT: password input = incorrect 
					else{
						d->setCursor(145, 175);				 //Error display location
						d->printf("ERROR: WRONG PIN"); //Print "ERROR: WRONG PIN"
					}
				}
			}	
			case RB_POT : { //Case 2 Ringbuffer Potentiameter
				//IF STATEMENT: Secure equals 0 potentiameter can increment decrement
				if (secure == 0){
//TODO:
				}	
				//ELSE STATEMENT: potentiameter value = FINAL
				else{
//TODO:
				}
			}					
				break;
			case RB_ACC : { //Case 3 Ringbuffer Accelerometer
				acc.read(accVal1, accVal2, accVal3); 
		//IF STATEMENT: While movement (bMoving = 1) is detected within the following bounds
		//							((y-coordinate > 20)  absolute
		//							 (x-coordinate > 20)  absolute
		//							 (y-coordinate < -20) absolute
		//							 (x-coordinate < -20))
    if ((accVal2 > 20) || (accVal1  > 20) || (accVal2 < -20) || (accVal1  < -20)){
			bMoving = 1; //Variable bMoving equals 1
			strcpy (moving , "MOVING    "); //STRING copy variable 'moving' displays "MOVING   "
	}
		//ELSE IF STATEMENT: While no movement (bMoving =0) is detected(not detected?) within the following bounds
		//								((y-coordinate < 20)  and
		//								 (x-coordinate < 20)  and
		//								 (y-coordinate > -20) and
		//								 (x-coordinate > -20))
		else if ((accVal2 < 20) && (accVal1  < 20) && (accVal2 > -20) && (accVal1  > -20)){
			strcpy (moving , "NOT MOVING"); //STRING copy variable 'moving' displays "NOT MOVING"
		}
		break;
		}
			case RB_TIME : { //Case 4 Ringbuffer Time
				//IF STATEMENT: Timer starts timer interval for alarm
				if((countTimer < interval) && (secure == 1) && (bMoving == 1)){
						countTimer++; //Increment countTimer
				}
				//ELSE IF STATEMENT: Timer sets Alarm to ON
				else if((countTimer == interval) && (secure == 1)){
					strcpy (alarm , "ON "); //STRING copy variable 'alarm' displays "ON "
					flashing[0] = !flashing[0]; //LED flashing
					flashing[1] = !flashing[1]; //LED flashing
				}
				break;
			}									
		}	
	}
}

/*
 * @brief buttonPressedAndReleased(button) tests to see if the button has
 *        been pressed then released.
 *        
 * @param button - the name of the button
 * @result - true if button pressed then released, otherwise false
 *
 * If the value of the button's pin is 0 then the button is being pressed,
 * just remember this in savedState.
 * If the value of the button's pin is 1 then the button is released, so
 * if the savedState of the button is 0, then the result is true, otherwise
 * the result is false.
 */

bool buttonPressedAndReleased(buttonId_t b) {
	bool result = false;
	uint32_t state;
	static uint32_t savedState[5] = {1,1,1,1,1};
	
	state = buttons[b].read();
  if ((savedState[b] == 0) && (state == 1)) {
		result = true;
	}
	savedState[b] = state;
	return result;
}

bool accInit(MMA7455& acc) {
  bool result = true;
  if (!acc.setMode(MMA7455::ModeMeasurement)) {
    // screen->printf("Unable to set mode for MMA7455!\n");
    result = false;
  }
  if (!acc.calibrate()) {
    // screen->printf("Failed to calibrate MMA7455!\n");
    result = false;
  }
  // screen->printf("MMA7455 initialised\n");
  return result;
}

void incDelay(void) {
	if (flashingDelay[0] + FLASH_DELAY_STEP > FLASH_MAX_DELAY) {
		flashingDelay[0] = FLASH_MAX_DELAY;
	}
	else {
		flashingDelay[0] += FLASH_DELAY_STEP;
	}
}

void decDelay(void) {
	if (flashingDelay[0] - FLASH_DELAY_STEP < FLASH_MIN_DELAY) {
		flashingDelay[0] = FLASH_MIN_DELAY;
	}
	else {
		flashingDelay[0] -= FLASH_DELAY_STEP;
	}
}
	
static void barChart(float value) {
	uint16_t const max = 100;
	uint16_t const left = 110;
	uint16_t const top = 13;
	uint16_t const width = int(value * max);
	uint16_t const height = 8; 
	
	d->fillRect(left, top, width, height, RED);
	d->fillRect(left + width, top, max - width, height, WHITE);
}