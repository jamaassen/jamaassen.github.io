// Author: Jeffrey Maassen 2/01/09, last updated 05/22/09
// use whatever you want but please credit me in the comments if you do.
// PWM initialization code adapted from Lance Halstead's test2.c
//
// Steering output is PWM15 - pin 37 on H1.
// Speed output is PWM14 - pin 38 on H1
// r1sensor = ATDDR9H = pin 23-2
// r2sensor =  ATDDR10H = pin 24-2
// l1sensor = ATDDR13H = pin 28-2
// l2sensor = ATDDR14H = pin 27-2
// speedpot = ATDDR7H = pin 26
// straightaway speed pot = ATDDR4H = pin 29
// integral pot = ATDDR5H = pin 28
// steering DV pot = ATDDR6H = pin 27
// steering K pot = ATDDR3H = pin 24
// duty cycle 73 = wheels centered
// 55 = locked left; 91 = locked right;  

#include <stdio.h>
#define _SCI
#include <hcs12e128.h>
#define DUMMY_ENTRY (void(*)(void))0xFFFF
#define right_overshoot 1
#define left_overshoot 2
#define right_overshoot_ending 3
#define left_overshoot_ending 4
#define safetycycles 600
#define safetypin ((PORTA & 0x40))
#define NOISE 1
#define ZERO 40
#define SPEEDTHRESHOLD 46
#pragma nonpaged_function _start;
extern void _start(void);
char DvON=0;
int i=0, 
  dutycycle,position, 
  lastposition = 0, debug = 0,
  speedcalc = 0, lastpulse1count=0, currentpulse1count = 0,
  pulses, lastpulses=0, targetpulses = 0, 
  speedcorrection, 
  r1Zero= 81,
  r1Offset = 144,
  r2Zero = 35,
  r2Offset = 382,
  l1Zero= 80,
  l1Offset = 145,
  l2Zero = 80,
  l2Offset = 355,
  speeddutycycle=0, lastpulse2count=0, currentpulse2count = 0, pulses2, pulses1,
  rsensor, lsensor, overshoot=0, sensorcount=0,
  LEDcount =0,
  r1sensor, r2sensor, l1sensor, l2sensor,
  r1sensort, r2sensort, l1sensort, l2sensort,
  straightcount=0, overshootSpeed =0,
  steeringDv=0, steeringK=0, speedK =0, steeringDvK = 0,
  targetpulses, pulses, lastpulses,
  positioni, intK=0, integral = 0, movingTotal=0, arraySize=75,
  positionarray[75]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int putchar(char c)
  {
  if (c == '\n')
    putchar('\r');
  while ((SCI0SR1 & TDRE) == 0)
    ;
  SCI0DRL = c;
  return c;
  }
  
int getchar(void)
  {
  while ((SCI0SR1 & RDRF) == 0)
    ;
  return SCI0DRL;
  }

void Init(void){
  int i;
  COPCTL = 0x00;            // disable COP
  
  CLKSEL &= 0x7F;            // make sure PLLSEL bit=0 (default)
  SYNR = 5;             // set PLLCLK=48 MHz; bus clock=24 MHz
  REFDV = 1;           // 2*8MHz*(5+1)/(1+1)=48 MHz
  while ((CRGFLG & 0x08)==0x00) ;  // wait for PLL to LOCK
  CLKSEL |= 0x80;         // PLLSEL=1;         

  PWMCNT5 = 0;          // reset PWM counter 5
  PWMCNT4 = 0;          // reset PWM counter 4
  PWMCNT2 = 0;          // reset PWM counter 2
  PWMPOL = 0x34;        // PPOL5 = 1.......was 0x20
  PWMCLK = 0x34;        // clock SA = source for Ch. 5.. was 0x20
  PWMPRCLK = 0x02;        // A = E/4 = 24M/4 = 6M clock rate
  PWMCAE = 0;          // Ch. 5,4 Left Aligned Output mode
  PWMCTL = 0;          // 8-bit PWM register
  PWMSCLA = 60;          // SA = E/(2*PWMSCLA) = 6M/120=50K
  PWMPER5 = 167;        // Frequency = SA/167 =~ 300 Hz
  PWMPER4 = 50;         // Frequency = SA/50 = 1000 Hz
  PWMPER2 = 50;         // Frequency = SA/50 = 1000 Hz
  PWMDTY5 = 75;          // centered wheels duty cycle
  PWMDTY4 = 0;          // start with motor off. 50 is max on
  PWMDTY2 = 0;          // start with motor off. 50 is max on
  PWME = 0x34;          // enable Ch. 5 is 0x20, 4+5 is 0x30?
  ATD0CTL2 = 0xC0;        // enable and fast flag clear
  ATD0CTL3 = 0x00;        // set the ATD for multi channel conversion
  ATD0CTL4 = 0x05;        // set the ATD for 2 MHz,2 sample clks,10 bits
  ATDTEST1 = 0X00;        // do not use special acquisitions
  ATD0CTL5 = 0x30;        // right justified, continuous conversions, Multichannel
  INTR_ON();
  DDRP = 0x01;            // Enable LED port
  RTICTL = 0x20;         // 2^10 / 13 Set RTI divider for 300HZ 1C, 4 Hz time base 0x7F, 0x20 for 3906Hz
  CRGFLG |= 0x80;         // Clear the RTI Flag
  CRGINT |= 0x80;         // Enable the RTI
  P1ACTL = 0b01010000;       // enable pulse accumulator, event mode, rising edge, no clock, no interrupt
  P1AFLG = 0x03;
  P2ACTL = 0b01000000;       // enable pulse accumulator, event mode, falling edge, no clock, no interrupt
  P2AFLG = 0x03;
  DDRP = 0xFF;  //set led pin as output
 // DDRA = 0xFF;
 // DDRB = 0xFF;
  PTP = 0x00;  //set led off*/
  //for H-bridge circuit in unidirectional mode
  DDRA = 0xC0;
  PORTA=0x80;
  PWMDTY2 = 0;  


}

void main(void)
{ int i=0, duty, per;    
  int dutycycle, lsensor=0, rsensor;
  Init();                // initialize COP, Port P
  

  
  SCI0BD = 156;           // 9600 baud
  SCI0CR2 = 0x0C;         // enable transmitter and receiver
//  printf("targetpulses is %d, dvK is %f\n\n", targetpulses, dvK);
  
  puts("started!\n");

  
  while (1)    //to ensure program is still running to wait for interrupts
  {    
    //read constants from inputs
    targetpulses = ATDDR7H/2; // 0-128
    intK = ATDDR5H; // 0-4  
    speedK = ATDDR4H; // 0-2
    steeringDvK = ATDDR6H; // 0-8
    steeringK= ATDDR3H; // 0-.5
    //debug info for serial output
//  printf("intK is  %d, movingtotal is %d, integral is %d, position is %d\n",intK, movingTotal, integral, position);
    printf("l2 = %4d, l1 = %4d, r1 = %4d, r2 = %4d, pos = %d\n", l2sensor, l1sensor, r1sensor, r2sensor, position);

  }// end while    
}//end main

#pragma interrupt_handler rti_handler
void rti_handler(void){
  CRGFLG |= 0x80; // Clear the RTI Flag

  //average readings per cycle to reduce noise
  if (sensorcount == 0) // if new cycle, reset totals
  r1sensort = r2sensort = l1sensort = l2sensort = 0; 
  r1sensort += ATDDR9H; // accumulate
  r2sensort += ATDDR10H;
  l1sensort += ATDDR13H;
  l2sensort += ATDDR14H;
   
  if(++sensorcount == 13) //time divider
  {
    sensorcount = 0; //reset counter
    
    r1sensor = r1sensort / 13; // set averages
    r2sensor = r2sensort / 13;

    l1sensor = l1sensort / 13;
    l2sensor = l2sensort / 13;

    DvON = 1;
    if (r2sensor > r1Zero ) //4sensor version
    {
      rsensor = r2sensor + r1Offset; //if outer sensor is strong enough, use it for readings
      if (r1sensor < r2sensor  )
        rsensor = r1Offset + 250; //if wire has passed all sensors, assume max value
    }
    else
      rsensor = r1sensor;
    if (l2sensor > l1Zero ) //4s
      {
        lsensor = l2sensor + l1Offset;
        if (l1sensor < l2sensor )
          lsensor = l1Offset + 250;
      }
    else
      lsensor = l1sensor;

    //sharp cross detection, if both outer sensors are above threshold, must be at a cross, read inner sensors
    if (r2sensor > r1Zero && l2sensor > l1Zero) //4 sensor version
    {
      lsensor = l1sensor;
      rsensor = r1sensor;
    }
      
    rsensor = rsensor >> 2; // scale down sensors to avoid overflow during evaluation
    lsensor = lsensor >> 2;
    position = rsensor - lsensor; // set position according to right and left sensor values
    
    //overshoot section, check if lower than zero values
    if (l1sensor < 17 && l2sensor <= l1Zero && r1sensor < 17 && r2sensor <= r1Zero )
    // ^ 4s version
    {
      overshootSpeed = 15; // set speed to slow down
      if (lastposition > 0 )
        position = 100; // right overshoot, error = rightmost
      else
        position = -100; //left overshoot, error = leftmost
    }//end overshoot
    else
      overshootSpeed = 0; // not in overshoot, no slow down
    
    //integration
    positioni = positioni % arraySize; // keep circular array index in bounds

    movingTotal = movingTotal + position - positionarray[positioni]; //total = current total + current reading - oldest reading
    positionarray[positioni] = position; //add current reading to array, overwriting oldest
    positioni++; // increase array index
    integral = movingTotal/arraySize; // update average   

    steeringDv = position - lastposition; // DV = change in readings
    if ( ( steeringDv > NOISE || steeringDv < -NOISE )&& DvON == 1) // check if DV is above threshhold
    {// if DV, add center offset, proportional, derivative, and inetegral scaled by constants
      dutycycle = 73 + position*steeringK/512 + (steeringDvK*steeringDv)/32 + integral*intK/512;
      // PTP = 0x01; // turn led on when using Dv
    }
    else
    {// if no DV, add only center offset, proportional, and integral
      dutycycle = 73 + position*steeringK/512 + integral*intK/512;
      // PTP = 0x00; // turn led off when not using Dv
    }

    if ( dutycycle < 55) // safetey checks to prevent blown servo
      dutycycle = 55;
    if (dutycycle > 91)
      dutycycle = 91; 

    PWMDTY5 = dutycycle; //apply value to actual PWM channel

    lastposition = position; // update last position
  }// end set steering section


   // if (0) //debug line to disable speed section
  if (++speedcalc == 195) // do every 30 RTI cycles
  {
    speedcalc = 0;//reset time divider
    
    currentpulse1count = P1ACNT; //use both accumulators, 1 for rising edge
    currentpulse2count = P2ACNT; // 1 for falling edge, increases resolution entirely in software
    pulses1 = currentpulse1count - lastpulse1count;
    if (pulses1 < 0) // if overflowed
       pulses1 +=  0xFFFF; // add offset
    pulses2 = currentpulse2count - lastpulse2count;
    if (pulses2 < 0)
       pulses2 +=  0xFFFF;   
    pulses = pulses1 + pulses2; //total pulses

    if (ATDDR12H > SPEEDTHRESHOLD && position < 20 && position > -20) //if track is ahead and car is going straight
       straightcount++; // increase time known to be going straight
    else
        straightcount = 0; // else, reset time
    if (straightcount > 2) // if going straight long enough, add straightaway speed
       speedcorrection = (targetpulses+speedK - pulses) *2/5 - (pulses - lastpulses)*4/5 - overshootSpeed*2/5;
    else // else add normal speed
        speedcorrection = (targetpulses - pulses) *2/5 - (pulses - lastpulses)*4/5 - overshootSpeed*2/5;

    speeddutycycle += speedcorrection; // add correction to current value
    if (speeddutycycle > 50) //range check
         speeddutycycle = 50;
    if (speeddutycycle < 0)
       speeddutycycle = 0;

//  PWMDTY4 = ATDDR7H/5; //direct pot control, no feedback 
    if (targetpulses) // if speed is non-zero 
     PWMDTY4 = speeddutycycle;// apply proper feedback, porportional and derivative
    else
      PWMDTY4 = 0; //avoids noise when meant to be off.
    lastpulses = pulses; // move current pulse counts to last pulse counts for comparisons
    lastpulse1count = currentpulse1count; // update individual pulses counters
    lastpulse2count = currentpulse2count;

    
    if (++LEDcount == 5)// keeps LED flashing to show processor is still running
    {  
      LEDcount = 0;
      PTP ^= 0x01;
    }

  }//speedcalc
  PTP ^= 0x08; // oscillates channel PORTP channel3 (assuming 0-7 channels) to allow measuring evaluation times per cycle on a scope
}//end rti

#pragma abs_address: 0xFFF0

void (*interrupt_vectors[])(void) = 
{
 rti_handler,  /* Real Time Interrupt */ //rti_handler
 DUMMY_ENTRY,  /* IRQ */
 DUMMY_ENTRY,  /* XIRQ */
 DUMMY_ENTRY,  /* SWI */
 DUMMY_ENTRY,  /* Unimplement Instruction Trap */
 DUMMY_ENTRY,  /* COP failure reset */
 DUMMY_ENTRY,  /* Clock monitor fail reset */
 _start,    /* Reset */
};
#pragma end_abs_address 
