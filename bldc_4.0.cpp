
/**
 * BLDC MOTOR
 * 
 * 
 *    
 *   ***********************************************************************************
 *   This version is the start of the I2c control
 * 
 *   *********************************************************
 *      To Do
 * 
 *   auto swap hall poles to avoid wiring warning
 *     
 *   terrible at resisiting forcees while standing
 *  
 *   tigher current at steps 2-3 when old did it at 3-4 which is pin 3
 *  
 *  
 * 
 * 
 * 
 * Function we need
 * - Input Commands
 *      -target position
 *      -force to apply
 *      
 * - Output Commands  
 *      - position
 *      - opposing forces
 *      
 * 
 * PIN Chart   PWR  GND   Comp/BEMF
 *  
 * Phase A      2    5      8     
 * Phase B      3    6      9
 * Phase C      4    7      10
 * 
 * Phase Cycle Chart
 * 
 *   PWR GND BEMF     Step          Short-Step
 * 1) A   C   B       100 110 00    10111100 
 * 2) B   C   A       100 101 00    11011100
 * 3) B   A   C       001 101 00    01111100
 * 4) C   A   B       001 011 00    10111100
 * 5) C   B   A       010 011 00    11011100
 * 6) A   B   C       010 110 00    01111100
 * 
 *  Remember high side switching is pin low and low side switching is pin high
 *  Therefore this is an example of all phases off * 
 *   PORTD = B00011100;
 *            ||||||||_Nothing
 *            |||||||__Nothing
 *            ||||||___A PWR HIGH 
 *            |||||____B PWR HiGH 
 *            ||||_____C PWR HIGH
 *            |||______A PWR LOW
 *            ||_______B PWR LOW
 *            |________C PWR LOW
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * d12 is fan
 * d11 is led
 * 
 */

#include <Arduino.h>
#include <Wire.h>



/* #region  Timers    */
bool unDone = 1; //our loop shouldnt attempt to reset ports continuosly
byte step = 0;   //defines motor position
bool wait = 0;   //current chopping timer flag
bool flagOff = 1;

uint8_t changedbits;
volatile uint8_t portbHistory = 0xFF;

//timer 1 speed control factors
int slowest = 2500; //2500-2 normal   30k-28 for slow debug
int cutOff =  2000;  //must end with zero so can be divided evenly later
int fast = 200;
int bemfMin = 1800;

//timer 2 power chopping factors
unsigned int tgtCurr = 27;  //7025 22 - 28
const int pwrLengthMin = 20; //20 minimum
const int pwrLengthMax = 50; //80 warms fets quickly
unsigned int current = tgtCurr;   //if not reset anywhere here it is
int powerOff = 100;

/* #endregion */


/* #region  Pot Stuff */

const int potPos = A7;   //Control position of motor
const int potSpeed = A6; //speed motor should move
//speed
const int arrLength = 30; // does all average arrays
unsigned int averageRotateSpeed[arrLength];
unsigned int indexARS;
unsigned int tempARS;                 //hold initial reading
unsigned int holderARS;               //hold full sum
unsigned int finlARS;                 //below this value and it will start running
unsigned int rotation = slowest;      //this is the desired rotation speed
unsigned int speed;                   //this is the variable that holds actual calcs

//position 
boolean chase = false;
boolean newchase = 0; //motor jitter compensator
unsigned int currPos;
int timedOnlyValue = 1;
int hallMoveValue = 2;
int direction = timedOnlyValue; //this tells us which way to go. 1 clockwise and -1 counter
unsigned int averagePosition[arrLength];
unsigned int indexAP;
unsigned int tempAP;
unsigned int holderAP;
unsigned int finalAP;
byte wiggleFactor = 3; //how much wiggle room before new chase
byte stickLanding = 0;

int pauseCounter = 50;
int pauseLimit = 10;
/* #endregion */

/* #region  hall stuff */

//HallSens
boolean hallOnly = false;      //hall only
boolean timedWithHall = false; //timedhall
int currentHall = 1;
const int hall1 = A1;
const int hall2 = A2;
const int hall3 = A3;
int hall360; //temp holder for anlog read
int hall240;
int hall120;
int hallValue = 512; //analoug reading cutoff
byte stepCount = 0;
byte hallCount = 0;
int allowStepTime = 1;
bool allowStepSens = false;
boolean lost = true;
uint8_t stuckCtr = 0;
boolean freshStart = false;
int freshcount = 0;
int actualDirection = 0;
int wrongWayCount = 0;
int offset = 0;
/* #endregion */


/* #region  debug, i2c, future */

//cooling fan
boolean fanStatus = 0;
unsigned long fanT;

//I2c
const int sda = A4;
const int scl = A5;
byte buf1 = 0;
byte buf2;
uint16_t targetPosition;
uint16_t targetForce;
uint16_t actualPosition;
uint16_t actualForce;

//startup calibration
boolean noCalibrate = true;
byte wiringOffset = 0;


//debugging continuosly. . .
boolean ultraslowBool = false; //used for allowing step at startup
boolean slowrun = false;
boolean debugBool = 1;
long hallsdone = 0;
long stepsdone = 0;
int prevHall = 0;

// time between detections
long waitCyckes;
long aveWait;
long currWait;

//hall check between each step
int looparrcnt = 0;
long loops[30];
long loopcycles;
long loopAve;
long loopCurr;
//long prevloop;

int halldebounce = 0;
int headstart = 0;
int prevloop = 0;

int potdebounce = 0;

int stuckCount = 0;

unsigned long nextWrite;
int waitTIme = 1000;
boolean dumpdebug;
const int debSize = 20;
int debcnt = 0;
byte debarr[debSize];
int T1Debug = 0;
int starter;
boolean startUp = false;
/* #endregion */


/* #region ---> Set Up <----*/
void setup()
{
    DDRD = B11111100; //set 2-7 as output
    DDRB = B00011010; //8,9,10 comparator input    11 led to signal when entering pin intreupt mode
    PORTB |= B00000000;

    Serial.begin(9600);

    //ultraSlow(); //uncomment for slow clock like ticking

    setupTimerTwo(); // Current chopping always on
    setupTimerOne(); //if nothing is active above motor is on simple timer for accel deccel
    
    sei();
     
    
    //wireSetup();
   
    //hallSetup();    // 1) When hall active it overrides timed 100%
    timedHall();  // 2) when active it combines timing with hall confirmation
     //chaseSetup(); // 3) active it tracks motor position to match pot or I2c

   

    fanT = millis();
    nextWrite = millis();
}

/* #endregion */

/* #region  Main Loop */

void loop()
{

    hallSens();    
    readPots();  
    chasePot();

/*

    if (dumpdebug)
    {
        //doDump();        

    }

    loopcycles++;

*/

    
/*
    if (debugBool)
    {
        if (nextWrite < (millis() + waitTIme))
        {
            nextWrite += waitTIme;
            slowWork();
            debuging();
        }
    }
    */
    
}
/* #endregion */

// Steps------------------------

//Here is our stepping decision tree
/* #region  perform Steps */
void perform()
{

    
        if (flagOff && noCalibrate)
        {
            step0(); //disable all fets
            return;
        }

        switch (step)
        {
        case 1:
            if (wait)
            {
                step11();
            }
            else
            {
                step1();
            }
            break;

        case 2:
            if (wait)
            {
                step21();
            }
            else
            {
                step2();
            }
            break;

        case 3:
            if (wait)
            {
                step31();
            }
            else
            {
                step3();
            }
            break;

        case 4:
            if (wait)
            {
                step41();
            }
            else
            {
                step4();
            }
            break;
        case 5:
            if (wait)
            {
                step51();
            }
            else
            {
                step5();
            }
            break;
        case 6:
            if (wait)
            {
                step61();
            }
            else
            {
                step6();
            }
            break;

        default:
        //This will have random steps in here because the timer will interupt the loop in as its performing th calcs.
        //When this happens it should just do its prev step until instructions are complete
            break;
        }
    
}
/* #endregion */

//Here we have the power portion of each step
/* #region  Normal Steps */

void step0()
{
    PORTD = B00011100;

    //unDone = 0;
}

void step1()
{
    PORTD = B10011000;

   // unDone = 0;
}

void step2()
{
    PORTD = B10010100;

    //unDone = 0;
}

void step3()
{
    PORTD = B00110100;

    //unDone = 0;
}

void step4()
{
    // 4) C   A   B       00011100    11011100
    PORTD = B00101100;

   // unDone = 0;
}

void step5()
{
    //5) C   B   A       00110100    11110100
    PORTD = B01001100;

   // unDone = 0;
}

void step6()
{
    PORTD = B01011000;

    //unDone = 0;
}
/* #endregion */

//Here we have the current limiting portion of each step
//It does shorting to ground
/* #region  Shorted Steps */
void step11()
{  
    PORTD = B10111100;   
}

void step21()
{   
    PORTD = B11011100;   
}

void step31()
{
    PORTD = B01111100;     
}

void step41()
{   
    PORTD = B10111100; 
}

void step51()
{ 
    PORTD = B11011100;     
}

void step61()
{   
    PORTD = B01111100;     
}

/* #endregion */

/* #region  nextStep */
// Called by many method but must wait until several conditions ar met before stepping forward
void nextStep(int thisDir)
{

 

    if (allowStepTime == false)
    {
        return;
    }

    if(timedWithHall || hallOnly)
    {
        step = currentHall + wiringOffset + thisDir ;   
    }
    else
    {
        
         step += thisDir;   
        
    }
    

    if (step > 6 && step < 100)
    step = step - 6; // if reset needed
    else if (step == 0) step = 6;
    else if (step > 100)
    step = step- 250;

    if(currentHall == prevHall)
    {
            stuckCount++;
    }else
    {
        stuckCount = 0;
        prevHall == currentHall;
    }
       

    if (!hallOnly)
    {
        allowStepTime = false; 
    }
    
  

}




/* #endregion */

//Interupt Methods--------------------

// Calculates potSpeed input and takes next step
// x64 = 4 micros witha range of 200 thru 2000 = .8 mill to 10 milli
/* #region  Timer One */
void setupTimerOne()
{
    //if(timedWithHall == true) return; //temporary debugging

    TCCR1A = 0;                          //Set all to zero
    TCCR1B = 0;                          //Set all to zero
    TCNT1 = 0;                           //set count to zero
    if(ultraslowBool)
    {
        TCCR1B |= (1 << CS12) | (1 << CS10);   //set prescale to x1024 for debugging only
    }else{
        TCCR1B |= (1 << CS11) | (1 << CS10); //set prescale to x64
    }
    TCCR1B |= (1 << WGM12);              //CTC Mode Clea on timer comparison    
    TIMSK1 |= (1 << OCIE1A); //Set mask to enable macth compar for OCR1A

    //Initial Set Up
    OCR1A = rotation;
}

ISR(TIMER1_COMPA_vect)
{

    // non-potchase off switch
    if (rotation > cutOff) //off
    {
        if (flagOff == 0) //was on
        {
            flagOff = 1; // this allows the timer control to disable other control methods such as hall by preventing the perform() method            
            fanStatus = 0;
            dumpdebug = true;
        }

        return; //prevent it from changing step
    }
    else //on
    {
        if (flagOff == 1) //was off
        {
            fanStatus = 1;
            flagOff = 0;
            freshStart = true;
        }
    }

    if (hallOnly == false)
    {
        T1Debug++;
        OCR1A = rotation; //set rotation to speed by setting counter CTC value.
        allowStepTime++;
        nextStep(direction);        
       
        
    }
}

/* #endregion */

//Implements Current Chopping by switching between normal steps and shorted steps
// 500 nano prescaler with 20 and 100 match value 10 micros and 50 micros
//timer1 does not actually perform any steps. its done here in timer2
/* #region  Timer Two */
void setupTimerTwo()
{
    TCCR2A = 0;             //Set all to zero
    TCCR2B = 0;             //Set all to zero
    TCNT2 = 0;              //set count to zero
    OCR2B = 0;              //value to compare to  on/power time 10 micro
    OCR2A = powerOff;       //value to compare to  off/wait time 40 micro
    TCCR2A |= (1 << WGM21); // set to CTC on OCR2A
    TCCR2B |= (1 << CS21);  //set prescale to x8    
    TIMSK2 |= (1 << OCIE2A) | (1 << OCIE2B); //set mask to enable match compare for both registers
}

ISR(TIMER2_COMPB_vect)
{ //just got done working
    OCR2B = current;
    wait = 1;   
    perform();
}

ISR(TIMER2_COMPA_vect)
{ //just got done waiting
    wait = 0;    
    perform();
}

/* #endregion */

//Hall Sense Stuff--------------------------------------------------

/* #region  hall setups */
void hallSetup()
{
    pinMode(hall1, INPUT);
    pinMode(hall2, INPUT);
    pinMode(hall3, INPUT);
    direction = hallMoveValue;
    hallOnly = true;
    timedWithHall = false;
     alignMotor();       //This determines motor wiring on startup. cant determine direction yet
     
}

void timedHall()
{
    pinMode(hall1, INPUT);
    pinMode(hall2, INPUT);
    pinMode(hall3, INPUT);
    direction = hallMoveValue;
    timedWithHall = true;
    hallOnly = false;
     alignMotor();       //This determines motor wiring on startup. cant determine direction yet
}

/* #endregion */

/* #region  Hall Main */

//used by main loop to detect movement in either direction
void hallSens()
{
    


    if (flagOff ) 
    {
        return;
    }
    

    hall360 = analogRead(hall1);
    hall120 = analogRead(hall2);
    hall240 = analogRead(hall3);

    if (freshStart)
    {
        //let motor turn on
        allowStepSens = true;
        nextStep(direction-1);

        
        delay(50);

        //measure current position
        hallAlign(hall360, hall120, hall240);

        
         if (hallOnly || timedWithHall)
        {
              
            step = currentHall + wiringOffset; //wiring offset is calibrated on startup
        }

        
        if (step > 6)
        step = step - 6; // if reset needed
        if (step < 1)
        {
                 step = step + 6;
        Serial.println("731 criticalerror");
        }
             

        freshStart = false;
       
       
        return;
    }

    

    switch (currentHall)
    {

    case 1:
        //  x 0 0
        checkBOn(1);  //forward
        checkCOn(-1); //reverse
        break;

    case 2:
        //  x x 0
        checkAOff(1);  //forward
        checkBOff(-1); //reverse
        break;

    case 3:
        //  0 x 0
        checkCOn(1);  //forward
        checkAOn(-1); //reverse
        break;

    case 4:
        //  0 x x
        checkBOff(1);  //forward
        checkCOff(-1); //reverse
        break;

    case 5:
        //  0 0 x
        checkAOn(1);  //forward
        checkBOn(-1); //reverse
        break;

    case 6:
        //  x 0 x
        checkCOff(1);  //forward
        checkAOff(-1); //reverse
        break;
    }


 
    if (currentHall < 1)
        currentHall = 6;
    else if (currentHall > 6)
        currentHall = 1;

    //we can count step with above method but only need to move in 2 situatons
    if (hallOnly == false && timedWithHall == false)
        return;


    //calling next step no matter what.
    //timed hall can slow it down in the next step method       
    nextStep(direction); 

    //prevent from looping by bypassing zero and starting from 65k
    if(currPos > 10000)
    {
        Serial.println("oops");
        currPos = 0;
    }
        

}


/* #endregion */


/* #region  hallalign */


void hallAlign(int three, int one, int two)
{

    if (three < hallValue)
    {
        if (one < hallValue)
        {
            currentHall = 2;
        }
        else if (two < hallValue)
        {
            currentHall = 6;
        }
        else
        {
            currentHall = 1;
        }
        return;
    }

    else if (one < hallValue)
    {
        if (two < hallValue)
        {
            currentHall = 4;
        }
        else
        {
            currentHall = 3;
        }

        return;
    }

    else if (two < hallValue)
    {
        currentHall = 5;
        return;
    }
}

/* #endregion */


/* #region  Hall Checks */

void checkAOn(int i)
{

    if (hall360 < hallValue)
    {
        //getting turned on first time
        currentHall += i;        
        currPos += i;
        
    }
}

void checkAOff(int i)
{
    if (hall360 > hallValue)
    {
        //getting turned off
        currentHall += i;        
        currPos += i;
              
    }
}

void checkBOn(int i)
{
    if (hall120 < hallValue)
    {
        //getting turned on first time
        currentHall += i;        
        currPos += i;
              
    }
}

void checkBOff(int i)
{
    if (hall120 > hallValue)
    {
        //getting turned off
        currentHall += i;        
        currPos += i;
              
    }
}

void checkCOn(int i)
{
    if (hall240 < hallValue)
    {
        //getting turned on first time
        currentHall += i;        
        currPos += i;
               
    }
}
void checkCOff(int i)
{

    if (hall240 > hallValue)
    {
        //getting turned off
        currentHall += i;        
        currPos += i;
       
    }
}



/* #endregion */

//This method is called when the arduino starts. It makes hall and motor wiring agnostic but does not determine direction
/* #region  Startup Alignment */
void alignMotor()
{
    if(wiringOffset != 0) return;

    noCalibrate = false;
 
    byte prevspan;
    byte span = 0;
    byte prevhall = 0;
    byte thishall = 0;
    boolean passed = true;
    byte hallVv;
    byte hallsum = 0;
    

    step = 1; //get the first step taken
    

   for (size_t i = 0; i < 6; i++)
    {
        delay(100);

        Serial.print(" -  -  -  step - ");
        Serial.println(step);


        testHallSens();
        prevhall = thishall;
        thishall = currentHall;
        prevspan = span;

        //check if we can find a consistent distance between the hall and the step
            if(step < currentHall)
            {
                span = (step + 6) - currentHall;
            }
            else
            {
                span = step - currentHall;
            }

        



                Serial.print("currenthall = ");
                Serial.println(currentHall);            

                Serial.print("span = ");
                Serial.println(span);
      
      if(i > 0) {

            
            if( span == prevspan)
            {
                wiringOffset = span;
                Serial.print("- - align = ");
                Serial.println(span);               
                
            }else
            {
                 passed = false;
            }
                        
            
            //check if hall or steps are moving in opposite direction            
            if((prevhall > thishall) && (prevhall != 6) )
            {                  
               Serial.println("motor wiring reversed");
               passed = false;                
            }



        }
        
        //sum each hall 1 through 6 should equal 21 or hall sensor was skipped

            
            hallsum += currentHall;

            delay(1000);
            //take a step...remember T1 is returning due to off position and T2 is calling perform            
            step += (direction-1);
            if (step > 6)
            step = 1;
            if (step < 1)
            step = 6;

        

        
        
    }

    Serial.print("hallsum = ");
    Serial.println(hallsum);
    if(hallsum != 21)
    {
        passed = false;
    }

   

    if (passed)
    {
          Serial.println("  !!  passed align  !!  ");
        beep_High(300);delay(300);beep_High(300);delay(300);beep_High(300);
    }
    else
    {
        Serial.println("  !! Failed align  !! ");
        beep_low(300);delay(300);beep_low(300);
    }


    noCalibrate = true;

}

void testHallSens()
{
    
    hall360 = analogRead(hall1);
    hall120 = analogRead(hall2);
    hall240 = analogRead(hall3);       
        
    hallAlign(hall360, hall120, hall240);   
}


/* #endregion */


//Helper Methods--------------------------

/* #region   Chase Pot/Direction*/

void chaseSetup()
{

    for (size_t i = 0; i < 100; i++)
    {
        potdebounce = 26;
        readPots();
        delay(5);
    }
    
    currPos = finalAP; //currPOs is set within the hall check methods ++ and -- 
    chase = true;
}


void chasePot()
{
   
    //only chase if its set in setup and if we have just read the pots
    if (chase == false )
        return;

    //check if its outside of tolerable wiggle
    if (newchase == 0 && (currPos < (finalAP - wiggleFactor) || currPos > (finalAP + wiggleFactor)))
    {
        if (newchase == 0) // FIXME: not neccessary?
        {
            freshStart = true;
        }

        newchase = 1;
        
    }

//if so we are chasing. pausing prevents weird harmonic overshoots where reversing can actually speed it up
    if (newchase)
    {

        current = tgtCurr;

        if(currPos == finalAP && direction != 0)
        {
            pauseCounter = 0;
        }

        if (pauseCounter < pauseLimit)
        {
            pauseCounter++;
            stickLanding++;
            direction = 0;
            
            //we need to wait some number of loops before reducing power to motor otherwise motor loses power as it hits mark at high speed and will create wiggle
            if(stickLanding > 5)
            {
                 
                newchase = 0;
                current = pwrLengthMin;
                stickLanding = 0;
            }
            
           
        }

        else if (currPos < finalAP)
        {
            if (direction == (hallMoveValue * -1))
            {
                direction = 0;
                pauseCounter = 0;

            }
            else
            {
                direction = hallMoveValue;
            }           

            
        }
        else if (currPos > finalAP)
        {
             if (direction == (hallMoveValue ))
                {
                    direction = 0;
                    pauseCounter = 0;

                }
                else
                {
                    direction = (hallMoveValue * -1);
                }


            
        }
        

    }



}
/* #endregion */


/* #region  readPots */
void readPots()
{
    potdebounce++;
    if(potdebounce < 25) return;
    potdebounce = 0;

    //speed

    tempARS = analogRead(potSpeed); //Speed and delay inverse. x64 prescaler = 4 micro x? .8 mill through 20 milli
    holderARS -= averageRotateSpeed[indexARS];
    averageRotateSpeed[indexARS] = tempARS;
    holderARS += averageRotateSpeed[indexARS];
    indexARS++;
    if (indexARS == arrLength)
        indexARS = 0;

    finlARS = holderARS / arrLength;
    rotation = map(finlARS, 0, 1023, slowest, fast);

    //position

    tempAP = analogRead(potPos);
    holderAP -= averagePosition[indexAP];
    averagePosition[indexAP] = tempAP;
    holderAP += averagePosition[indexAP];
    indexAP++;
    if (indexAP == arrLength)
        indexAP = 0;
    finalAP = holderAP / arrLength;

//Serial.println(finalAP);

}

/* #endregion */


/* #region  ultraslow */

void ultraSlow()
{
    ultraslowBool = true; //used for allowing step at startup
    slowrun = true;
    current = 25;  //7025 22 - 28
    slowest = 32000; //16 bit max
    cutOff =  30000;    
    direction = timedOnlyValue;

    pinMode(hall1, INPUT);
    pinMode(hall2, INPUT);
    pinMode(hall3, INPUT);

}




/* #endregion */



/* #region  Debugging */

void debuging()
{
   /*
    Serial.print(step);
     Serial.print(" ");
    Serial.print(currentHall); 
       Serial.print(" ");
    Serial.print(hallCount);
       Serial.print(" ");
    Serial.print(stepCount);
    Serial.println("--");
    */

    Serial.println(currPos);
    Serial.println(finalAP);
    //Serial.println(currentHall);
    //Serial.println(step);
    Serial.println("--");

    //debugcount++;
}

/* #endregion */

/* #region  debugDump */
void doDump()
{
    Serial.println("-------"); 
    Serial.println(hallsdone); 
    Serial.println(stepsdone);
    Serial.println(loopcycles);
    Serial.println("--");
   
    



    hallsdone = 0;
    stepsdone = 0;
    loopcycles = 0;


    /*
    Serial.println("");  

    Serial.println("array hall to step conv: ");
    for (size_t i = 1; i < 7; i++)
    {
        Serial.print(i);
        Serial.print(" - ");
        Serial.println();
       
    }
    */

    dumpdebug = false;
}
/* #endregion */


/* #region  Slow Work */
void slowWork()
{
    //led indicator
    if (hallOnly)
        PORTB |= B00001000;
    else
        PORTB &= B11110111;

    //fan control
    if (fanStatus)
    {
        PORTB |= B00010000;
    }
    else
    {
        PORTB &= B11101111;
    }
}

/* #endregion */

/* #region  I2C/Wire */
void wireSetup()
{
    Wire.begin(2);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    //rotation = 1400;
}

void receiveEvent(int bytes)
{
    buf1 = Wire.read();
    buf2 = Wire.read();

    targetPosition = 0;
    targetPosition |= buf1 << 8;
    targetPosition |= buf2;

    buf1 = Wire.read();
    buf2 = Wire.read();

    //targetForce = ((uint16_t)buf1 << 8 + buf2);

    finalAP = targetPosition;
}

void requestEvent()
{
    
    Wire.write(actualPosition);
    Wire.write(actualForce);
}

/* #endregion */


/* #region  Beeps */

void beep_low(int millisReq)
{
          int x = 0;
  PORTD = B01010000;      //Set pahe 1 to high
  while (x < millisReq)
  { 
    PORTD = B01110100;   
    delayMicroseconds(50);
    PORTD = B01010000;      
    delayMicroseconds(950);    

    x = x + 1;
  }
   PORTD = B01010100;      //Set D2 (CL) to HIGH and the rest to LOW
      

}

void beep_High(int milliReq)
{
          int x = 0;
  PORTD = B01010000;      //Set pahe 1 to high
  while (x < milliReq)
  { 
    PORTD = B01110100;   
    delayMicroseconds(50);
    PORTD = B01010000;      
    delayMicroseconds(450);    

    x = x + 1;
  }
   PORTD = B01010100;      //Set D2 (CL) to HIGH and the rest to LOW
      

}


/* #endregion */
