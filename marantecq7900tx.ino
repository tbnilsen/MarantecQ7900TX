//***********************************************************************************************************************
// Remote Control via a Patricle Photon
//
// written by:  Terje B. Nilsen
// version:     1.0
// date:        04 OCT 2018
//
// This code sends a specified 24-bit code via a 315MHz transmitter card to a Marantec Q7900 Garage Door Opener.
// The correct keycode must be obtained first outside of this firmware.
// This has only been verified to work with a Q7900 opener.
//
// acknowledgment: Mark Terrill's PWM example https://gist.github.com/markterrill/b88759eaaee4995c2c68
//
// NOTE: I could have accomplished the Q7900 pulse train by bit-banging (delayMicroseconds()) but where's the fun in that?
//
//************************************************************************************************************************

//************************************************************************************************************************************
//Pin to manipulate. Changing this pin will require research into which Timers can be used for PWM and then related code changes below
//************************************************************************************************************************************
#define TXPIN D0 

//**********************************************************************************
//The TX bit rate for the Q7900 is 250Hz
//The transmission starts with a low idle of 12ms (3 x 250Hz periods)
//This idle period is also performed between repeated transmissions
//It's best (and normal - also done by normal remote controller) to repeat 2-3 times
//During IDLE the TXPIN is left floating (as an input) to avoid the transmitter 
//sending a constant active LOW that would interfere with other devices as well
//use unecessary power.
//**********************************************************************************
#define BITFREQ 250 //Hz
#define ONEBIT_DUTYCYCLE_PERCENT 0.375f //'0' bit is half this  ('1' is a 1500uSec pulse; '0' is a 750uSec pulse)
#define TXLENGTH 24 //number of bits to send to garage door
#define TXDELAY   3 //number of periods to delay after turning TX on and between each transmission
#define TXREPEAT  2 //After first key transmission, the number of times to repeat it

TIM_OCInitTypeDef   TIM_OCInitStructure;   //Output compare init structure for PWM

uint16_t bitDutycycle[2]; //Holds computed CCR reload values for '0' and '1'
int bitsTxed;   //number of bits transmitted
int delaysTxed; //number of initial delays transmitted
int keySent;    //number of times key sent
uint32_t txDoorkey; //key to transmit
volatile bool txDone;
volatile int txState;
volatile int txBusy;
volatile int txBoth;

uint16_t prescale, period;

//***********************************************
//Marantec Q7900 remote key codes are 24bits long
//NOTE: I chose to declare short TX pulses
//as '0's bit and long as the '1's
//The LSbit of a key is sent first
//***********************************************
const uint32_t hisDoorkey = 0xbbbbbb; //Use my MarantecQ7900RX code for the photon along with a 315MHz RF receiver to obtain these
const uint32_t herDoorkey = 0x555555;

//Function prototypes
void setupPWM(uint16_t, uint16_t);
void handle_Timer4(void);
void updateCCR(int , int);
void startTX(uint32_t);
void computeSetup(uint16_t *, uint16_t *);
int garageDoor(String);

//************************************************
//Local Interrupt Service Routine (ISR) for Timer4
//************************************************
void handle_Timer4(void)
{
    static uint32_t txKey;     //key being transmitted (i.e. shifted)
    static int led = 0;
    
    if (TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
    {
        //********************
        //Clear this interrupt
        //********************
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        switch(txState)
        {
        case 0:
            //*****************************
            //we are delaying before bit TX
            //*****************************
            if (++delaysTxed >= TXDELAY)
            {
                bitsTxed = 0;
                txKey = txDoorkey;
                updateCCR(TXPIN, bitDutycycle[(txKey & 1)]);
                txState++; //go to next TX state
            }
            break;
        case 1:
            //*******************
            //we are sending bits
            //*******************
            if (++bitsTxed < TXLENGTH)
            {
                txKey >>= 1; //shift down next bit
                updateCCR(TXPIN, bitDutycycle[(txKey & 1)]);
            }
            else
            {
                updateCCR(TXPIN, 0); //Drive TXPIN low
                txState++; //go to next TX state
            }
            break;
        case 2:
            //*********************************************
            //We have completed sending key - do it again?
            //*********************************************
            if (keySent++ < TXREPEAT)
            {
                //***********************
                //Start next transmission
                //***********************
                delaysTxed = 1; //start again but with one delay already out
                txState = 0;    //go back to start state
            }
            else
            { //we are done - stop tranmission
                digitalWrite(TXPIN, 0);
                TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
                TIM_Cmd(TIM4, DISABLE);
                led = 1; //turn off LED on the way out
                pinMode(TXPIN, INPUT);  //tri-state TXPIN
                if (!txBoth)
                {
                    txBusy = 0; //not busy anymore
                }
                txDone = 1; //Set complete flag
            }
            break;
        }
        //************************************
        //Blink the LED at the PWM period rate
        //************************************
        led ^= 1;
        digitalWrite(D7, led); //blink LED
    } 
}
//*******
//setup()
//*******
void setup() 
{
    //***********
    //Onboard LED
    //***********
    pinMode(D7, OUTPUT);
    //****************************
    //Make sure TX pin is floating
    //Don't want TX on during idle
    //****************************
    pinMode(TXPIN, INPUT);
    //***********************
    //Start serial debug port
    //***********************
    Serial.begin(9600);
    //*********************************************
    //Declare INET means to manipulate garage doors
    //*********************************************
    Particle.function("garage",garageDoor);
    //*********************************************
    //Delay for particle serial monitor start on PC
    //*********************************************
    delay(1000);
    Serial.println("MarantecQ7900 - TX Program Start");
    //****************************
    //Get everything ready for PWM
    //****************************
    computeSetup(&prescale, &period);
    setupPWM(prescale, period);
    Serial.println("Setup Complete");
}
//******
//loop()
//******
void loop() 
{
    delay(10);
    if (txDone)
    {
        txDone = 0;
        //*************************************
        //Are we sending both garage door keys?
        //*************************************
        if (txBoth)
        {
            txBoth = 0;
            Serial.printlnf("Starting Transmit of HIS Key: 0x%x",hisDoorkey);
            startTX(hisDoorkey);
        }
        else
        {
            Serial.println("Transmission Complete");
        }
    }
}
//****************************************************************
//Function that starts the transmission to the garage door opener. 
//The transmission is performed in the Timer4 interrupt
//****************************************************************
void startTX(uint32_t key)
{
    //*********
    //Get ready
    //*********
    txBusy      = 1;
    txDoorkey   = key;
    txDone      = 0;
    delaysTxed  = 0;
    keySent     = 0;
    txState     = 0;
    //****************************************************
    //Turn on transmitter pin and get ready for interrupts
    //****************************************************
    pinMode(TXPIN, OUTPUT);
    analogWrite(TXPIN, 0);
    delayMicroseconds(400); //Allow a short pulse to precede the inital delay for the first key pulse train
    setupPWM(prescale, period);
    //*****************************************************************
    //Enable the timer and interrupt and let it take over transmissions
    //*****************************************************************
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}
//**************************************************************************************
//Function setupPWM() is called once at startup to set up all PWM related chip functions
//**************************************************************************************
void setupPWM(uint16_t pre, uint16_t per)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //Time base structure for timer
    NVIC_InitTypeDef        NVIC_InitStructure;    //Nested Vector Interrupt Controller Initialisation Structure


    // GPIOB clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    // Initialize GPIO for PWM function
    GPIO_InitTypeDef    GPIO_InitStructure;//GPIO Initialization Structure
    // Initialize D0/PB7, Alternative Function, 100Mhz, Output, Push-pull*
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB, &GPIO_InitStructure); // Note the GPIOB.
    
    // Map the pin to the timer
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM4);
    // TIMER 4 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // Timer Base configuration
    TIM_TimeBaseStructure.TIM_Prescaler = pre; //set to value computed above during period calc
    //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3; // symmetrical PWM - no good for our application
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = per;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); // TIM4 = Timer 4

    // Timer 4 Channel 2 PWM output configuration
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = per;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

    updateCCR(TXPIN, 0);
    
    // Channel 2 init timer 4
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    //TIM_ARRPreloadConfig(TIM4, ENABLE); // don't seem to need this

    // ISR configuration
    attachSystemInterrupt(SysInterrupt_TIM4_Update, handle_Timer4); 
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
//*************************************************
//Function to change the dutycycle of the PWM pulse 
//*************************************************
void updateCCR(int pin, int dutyCycle)
{
    STM32_Pin_Info* PIN_MAP = HAL_Pin_Map();
    
    TIM_OCInitStructure.TIM_Pulse = dutyCycle;

    if (PIN_MAP[pin].timer_ch == TIM_Channel_1) 
    {
        PIN_MAP[pin].timer_peripheral-> CCR1 = TIM_OCInitStructure.TIM_Pulse;
    } 
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_2) 
    {
        PIN_MAP[pin].timer_peripheral-> CCR2 = TIM_OCInitStructure.TIM_Pulse;
    } 
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_3) 
    {
        PIN_MAP[pin].timer_peripheral-> CCR3 = TIM_OCInitStructure.TIM_Pulse;
    } 
    else if (PIN_MAP[pin].timer_ch == TIM_Channel_4) 
    {
        PIN_MAP[pin].timer_peripheral-> CCR4 = TIM_OCInitStructure.TIM_Pulse;
    }
}
//************************************
//Compute frequency related parameters
//************************************
void computeSetup(uint16_t *pre, uint16_t *per)
{
    float p;
    uint16_t zero_bit,one_bit;
    
    //***************************************************************
    //Compute largest period value not to exceed 65536
    //This yields finest resolution to our desired frequency
    //NOTE: this happens to give exact intergers for our bit rate
    //If you re-use this code for other frequencies
    //there is a chance that there is a period & prescale combination
    //the yields an exact number using a different method than below
    //***************************************************************
    *pre = 0;
    p = (float)SystemCoreClock / (2.0f * (float)BITFREQ);
    do
    {
        *pre += 1;
    }
    while((p / *pre) > 65536.0f);
    *per        = (uint16_t)(p / *pre + 0.5f); //Use computed period value that is closest to 65535 for our frequency
    one_bit     = (uint16_t)(p / *pre *  ONEBIT_DUTYCYCLE_PERCENT + 0.5f); //CCR reload value for a zero pulse
    zero_bit    = (uint16_t)(p / *pre * (ONEBIT_DUTYCYCLE_PERCENT/2.0f) + 0.5f); //CCR reload value for a one pulse
    zero_bit--;
    one_bit--;
    *pre -= 1;
    *per -= 1;
    Serial.printlnf("Period: \t%d, \tPrescaler: %d",*per,*pre);
    Serial.printlnf("Zero bit: \t%d, \tOne bit: %d",zero_bit,one_bit);

    //Remember reload values
    bitDutycycle[0] = zero_bit;
    bitDutycycle[1] = one_bit;
}
//*********************************
//Rotuine called via particle cloud
//*********************************
int garageDoor(String command) 
{
    if (txBusy)
    {
        return 0; //can't send now - we're busy sending already
    }
    else if (command=="his") 
    {
        Serial.printlnf("Starting Transmit of HIS Key: 0x%x",hisDoorkey);
        startTX(hisDoorkey);
        return 1;
    }
    else if (command=="hers") 
    {
        Serial.printlnf("Starting Transmit of HER Key: 0x%x",herDoorkey);
        startTX(herDoorkey);
        return 1;
    }
    else if (command=="both") 
    {
        txBoth = 1;
        //********************
        //Start her door first
        //********************
        Serial.printlnf("Starting Transmit of HER Key: 0x%x",herDoorkey);
        startTX(herDoorkey);
        return 1;
    }
    else 
    {
        //*********************
        //Comamnd not recogized
        //*********************
        return -1;
    }
}
