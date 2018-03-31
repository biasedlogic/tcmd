
// time constant for i2t
#define tau 120
// undervoltage lockout
#define umin 16000

//pre-square factor
#define i2t_scale (5)

//sample frequency
#define i2t_integrator_f 128

#define LED_OK  5
#define LED_ERR 4
#define LED_RUN 3

#define ISense 0
#define VSense 1
#define NSetpoint 2
#define RComp 4
#define ILim 3

#define STATUS_IDLE 0
#define STATUS_RUN  1
#define STATUS_OVERCURRENT 2
#define STATUS_OVERHEAT 3
#define STATUS_UNDERVOLTAGE 4
#define STATUS_WAIT_FOR_POWER 5
#define STATUS_VOLTAGE_DIP 6
#define STATUS_ERR 255

#define DIALPWMA OCR0A
#define DIALPWMB OCR0B
#define MAINPWMA OCR1A
#define MAINPWMB OCR1B

#define u_dc_scale 54 
// (uint16_t)(1000.0mV/V * 100.1k/9.1k * 5.0V/1024)
#define i_mot_scale 49
// 1000mA/A * 5V/1024 / 100mV/A

#define i_lim_scale 9.77
// 1000mA/A * 5V/1024 / 100mV/A

#define LED_OK_ON  PORTB&=~(1<<LED_OK)
#define LED_OK_OFF PORTB|=(1<<LED_OK)

#define LED_ERR_ON  PORTB&=~(1<<LED_ERR)
#define LED_ERR_OFF PORTB|=(1<<LED_ERR)

#define LED_RUN_ON  PORTB&=~(1<<LED_RUN)
#define LED_RUN_OFF PORTB|=(1<<LED_RUN)

#define DEBUG_PIN_HI PORTC|=0x20
#define DEBUG_PIN_LO PORTC&=~0x20

uint16_t i_ofs, u_dc, n_set, r_comp, i_lim, u_nom;
int16_t i_mot;
int32_t i_avg;
float i2t_int;
float comp_out;
char state=STATUS_IDLE;
char timeout = 0;

void setup() 
{
  // put your setup code here, to run once:

  DDRB  =   0b00111110;
  PORTB =   0b00000001;
  DDRC  =   0b00100000;
  PORTC =   0b01000000;
  DDRD  =   0b01101010;
  PORTD =   0b10010100;
  DIDR0 =   0b00011111;
  
  // master clock init sequence:
  CLKPR = 0x80;
  CLKPR = 0b00000000;
  
  //sync timers and reset their prescalers
  GTCCR = 0b10000011; 
  
  // timer mode initialization
  TCCR0A = 0b10100011; //976Hz two PWM outputs for analog dial
  TCCR0B = 0b00000011;
  TCCR1A = 0b10100011; //15625Hz main PWM power output + aux PWM power output
  TCCR1B = 0b11001001;
  TCCR2A = 0b00100011; //128Hz sync timer + one output 0..121
  TCCR2B = 0b00001111;
  
  // initial values for compare registers:
  OCR0A = 0;
  OCR0B = 0;
  
  OCR1A = 0;
  OCR1B = 0;
  ICR1 = 0;
  
  OCR2A = 121;
  OCR2B = 0;
  
  //release prescaler reset
  GTCCR = 0x00; 

  Serial.begin(115200);
  for (char i=0;i<16;i++)
    {
      i_ofs +=analogRead(ISense);
      delay(10);
    }

  i_ofs >>= 4; 
  u_dc =  analogRead(VSense);
  i_lim = analogRead(ILim);
  r_comp = analogRead(RComp);
  
  Serial.print(F("U DC: "));
  Serial.print(u_dc_scale*u_dc);
  Serial.print(F("mV; IN: "));
  Serial.print(i_lim*i_lim_scale);
  Serial.print(F("mA; R_Comp: "));
  Serial.print(r_comp*4);
  Serial.print(F("mOhm, I zero offset: "));
  Serial.print(i_ofs);
  Serial.println(F(" units (should be 512)"));

  if (u_dc<(umin/u_dc_scale))
  {
    state = STATUS_WAIT_FOR_POWER;
  }
  else
  {
    state = STATUS_IDLE;
  }
  LED_RUN_OFF;
  LED_OK_OFF;
  LED_ERR_OFF;
}

void loop() 
{
  //wait for time slice elapsing  
    while (!(TIFR2&0x02)) ;
    TIFR2|=0x02;
  
  //time the loop:
    DEBUG_PIN_HI;

  // timeout timer downcount
    if (timeout) timeout--;
    
  //get all analog values read
    i_mot = i_ofs-analogRead(ISense);
    u_dc =   analogRead(VSense);  
    n_set =  analogRead(NSetpoint);
    i_lim =  analogRead(ILim);
    r_comp = analogRead(RComp);

  // get some filtering here on motor immediate current value (1/16 ~ 125ms integrated moving average)
  // attention: i_avg is 16 times the motor current in absolute numbers!
    i_avg = i_avg-(i_avg>>4)+i_mot; 
    
  //from here on negative current is irrelevant and throws calculations off:
    if (i_mot<0) i_mot=0; 

  //check for immediate overcurrent
    if (((i_avg>>4)>i_lim)||(i_mot>480)) //this corresponds to 5x I_N or about 22A, whichever is lower
      {        
        state = STATUS_OVERCURRENT;
        timeout = 128; //wait 1s before going back to running
      }
      
  //compute the i2t integral
    i2t_int += ((float)i_mot*i_mot*i2t_scale*i2t_scale/((float)i_lim*i_lim)-i2t_int)/tau/i2t_integrator_f;
  
  //output the values
    if (i2t_int<0) DIALPWMA=0; else
      if (i2t_int>1.0) DIALPWMA = 255; else DIALPWMA=i2t_int*255;

    if (i_mot<128) DIALPWMB = i_mot<<1;
              else DIALPWMB = 255;
              
  //the main state machine
    switch (state)
    {
      case STATUS_IDLE: 
              LED_OK_ON;
              LED_ERR_OFF;
              LED_RUN_OFF;
            //check for speed command
              if (n_set>1) state = STATUS_RUN;
            //check for undervoltage
              if (u_dc<(umin/u_dc_scale))
                {
                    state = STATUS_UNDERVOLTAGE;
                    timeout = 128;
                }
              MAINPWMA = MAINPWMB = 0;              
            break;
    
      case STATUS_RUN :            
              LED_RUN_ON;    
              LED_ERR_OFF;        
              if (i2t_int>1) state = STATUS_OVERHEAT;
            //compute the PWM output with R_comp
              comp_out = (float)n_set + (float)r_comp/256*i_mot*i_mot_scale*1024/(u_dc*u_dc_scale) ; //+ (float)i_mot*i_mot_scale*r_comp*4/1000;
              if (comp_out>1023) {
                LED_ERR_ON;
                LED_OK_OFF;
                comp_out = 1023;
              } else {
                LED_ERR_OFF;
                LED_OK_ON;
              }                
              if (comp_out<0) comp_out = 0;
              MAINPWMA = MAINPWMB = comp_out;              
            //check for speed command
              if (n_set<=1) state = STATUS_IDLE;
            break;
      case STATUS_OVERCURRENT:
             // this state is overriden outside state machine, so just wait for timeout
             LED_ERR_ON;
             LED_OK_OFF;
             LED_RUN_OFF;
             MAINPWMA=MAINPWMB=0;             
             if (timeout==0) state = STATUS_IDLE;
            break;
      case STATUS_OVERHEAT: 
             LED_ERR_ON;
             LED_OK_OFF;
             LED_RUN_OFF;     
             MAINPWMA=MAINPWMB=0;          
             if (i2t_int<0.5) state = STATUS_IDLE;
            break;
      case STATUS_UNDERVOLTAGE: 
             if (u_dc<(umin/u_dc_scale))
                {
                    state = STATUS_UNDERVOLTAGE;
                    timeout = 128;
                } 
             if (timeout==0) state = STATUS_IDLE;
            break;
      case STATUS_WAIT_FOR_POWER: 
              LED_ERR_ON;
              LED_OK_ON;
              LED_RUN_OFF;
              if (u_dc>(umin/u_dc_scale)) state=STATUS_IDLE; 
            break;
      case STATUS_VOLTAGE_DIP: break;
      case STATUS_ERR: LED_OK_OFF; LED_RUN_OFF; LED_ERR_ON; DIALPWMA=0; DIALPWMB=0; MAINPWMA=0; MAINPWMB=0; break;
      default:  state = STATUS_ERR; break;
    }

    Serial.print(F("S: "));
    Serial.print((int8_t)state);
//    Serial.print(F("; I_lim: "));
//    Serial.print(i_lim*i_lim_scale);
    Serial.print(F(", I_sense [mA]: "));
    Serial.print(i_avg*i_mot_scale/16);
    Serial.print(F(", I2t: "));
    Serial.println(i2t_int);
    
//    if (TIFR0&0x02) LED_ERR_ON; //overrun 
//              else LED_ERR_OFF;
    DEBUG_PIN_LO;   
}
