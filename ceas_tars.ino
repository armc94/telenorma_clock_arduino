
// CONNECTIONS:
// DS1302 CLK/SCLK --> D5
// DS1302 DAT/IO --> D4
// DS1302 RST/CE --> D2
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND

#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <avr/interrupt.h>

typedef signed char sint8;
typedef unsigned char uint8;
typedef signed int sint16;
typedef unsigned int uint16;
typedef signed long sint32;
typedef unsigned long uint32;


#define countof(a) (sizeof(a) / sizeof(a[0]))
#define SET_BIT(x, pos) (x |= (1U << pos))
#define CLEAR_BIT(x, pos) (x &= (~(1U<< pos)))
#define TOGGLE_BIT(x, pos) x ^= (1U<< pos)

#define NUM_TASKS 6
#define NOT_ACTUATED 0
#define ACTUATED 1
#define FALSE 0
#define TRUE 1

typedef enum task_state
{
	WAIT,
	READY,
	ACTIVE,
}task_state_e;

 typedef void(*function_pointer)(void);

 typedef struct task
 {
	 const uint16_t delay;
	 const uint16_t cycletime;
	 function_pointer task_pointer;
	 task_state_e state;
	 uint16_t current_time;
 }task_s;

ThreeWire myWire(A4,A5,A3); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

const int pin_RS = 8;
const int pin_EN = 9;
const int pin_d4 = 4;
const int pin_d5 = 5;
const int pin_d6 = 6;
const int pin_d7 = 7;
const int pin_BL = 10;
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);

uint32 cnt_one;
RtcDateTime now;

typedef struct	phisical_clock
{
	uint8 minute;
	uint8 hour;
	boolean tick_requested;
} phisical_clock_s;

phisical_clock pc;


void printDateTime(const RtcDateTime& dt);
void cyclic_rtc();
void print_out_time_to_lcd();
void print_in_time_to_lcd();
void lcd_cyclic();


void task_1ms(void);
void task_5ms(void);
void task_10ms(void);
void task_20ms(void);
void task_50ms(void);
void task_100ms(void);
task_s tablou_tasks[NUM_TASKS]=
{
	 {0,10,task_1ms,WAIT,0},
	 {1,50,task_5ms,WAIT,0},
	 {2,100,task_10ms,WAIT,0},
	 {3,200,task_20ms,WAIT,0},
	 {4,500,task_50ms,WAIT,0},
	 {5,1000,task_100ms,WAIT,0},
};

uint8_t i;
void init_OS(void)
{

	for(i=0;i<NUM_TASKS;i++)
	{
		tablou_tasks[i].current_time=tablou_tasks[i].delay;
	}

}
void idle(void)
{

	for(i=0;i<NUM_TASKS;i++)
	{
		if((tablou_tasks[i].state!=READY)&&(tablou_tasks[i].current_time>0))
		{
			tablou_tasks[i].current_time--;
		}
		if(tablou_tasks[i].current_time==0)
		{
			tablou_tasks[i].state=READY;
			tablou_tasks[i].current_time=tablou_tasks[i].cycletime;
		}
	}

}
void scheduler(void)
{
	while(1)
	{
		for(i=0;i<NUM_TASKS;i++)
		{
			if(tablou_tasks[i].state==READY)
			{
				tablou_tasks[i].state=ACTIVE;
				tablou_tasks[i].task_pointer();
				tablou_tasks[i].state=WAIT;
			}
		}
	}
}
void task_1ms(void)
{

}
void task_5ms(void)
{

}
void task_10ms(void)
{

}
void task_20ms(void)
{
	pc_cyclic();
	tick_cyclic();
	lcd_cyclic();
}
void task_50ms(void)
{
    cyclic_rtc();

}
void task_100ms(void)
{
	static uint8 i = 0;
	lcd.setCursor(0,0);
    lcd.println(i , 10);
	i++;
}
void pc_cyclic()
{
	if(now.Minute() != pc.minute || now.Hour() != pc.hour)
		pc.tick_requested = TRUE;
}

void cyclic_rtc()
{
	now = Rtc.GetDateTime();

	if (!now.IsValid())
	{
	    // Common Causes:
	    //    1) the battery on the device is low or even missing and
			//the power line was disconnected
	    Serial.println("RTC lost confidence in the DateTime!");
	}
}

void tick_init()

{

}

void tick_step()
{

}

void tick_cyclic()
{
	// function not called faster then the clock can work (50ms?)
	static uint8 state = NOT_ACTUATED;
	if(FALSE != pc.tick_requested)
	{
		if( NOT_ACTUATED == state)
		{
			if (pc.minute % 2 == 0)
			{
				CLEAR_BIT(PORTC, 1);
				SET_BIT(PORTC, 2);
				pc.minute++;
			}
			else
			{
				SET_BIT(PORTC, 1);
				CLEAR_BIT(PORTC, 2);
				pc.minute++;
				pc.hour += (pc.minute/60);
				pc.hour %= 24;
				pc.minute %= 60;
			}
			state = ACTUATED;
		}
		else
		{
			SET_BIT(PORTC, 1);
			SET_BIT(PORTC, 2);
			state = NOT_ACTUATED;

			pc.tick_requested = FALSE;
		}
	}

}

void lcd_cyclic()
{
	lcd.setCursor(7 ,0);
	print_in_time_to_lcd();

	lcd.setCursor(7 ,1);
	print_out_time_to_lcd();

	lcd.setCursor(3,0);
	lcd.print((PORTD >> PD3) & 0x01u, 10 );
}

void init_external_interrupts()
{
	cli();
    EICRA |= (1<<ISC01); // (1<<ISC00); // 11 is rising edge 10 is falling edge
    EIMSK |= (1<<INT0); // this is PD2
	sei();
}
void init_ports_for_ei()
{
	DDRD  &= ~(1<<PD2);
    PORTD |= (1<<PD2);

	// DDRC |= (1<<PC3);
	// PORTC |= (1<<PC3);

	// Serial.println(DDRD, 10);
	// Serial.println(PORTD, 10);
}
void init_ports_for_ticking()
{
	DDRC  |= (1<<PC1) | (1<<PC2);
	// CLEAR_BIT(DDRC, PC1);
	// CLEAR_BIT(DDRC, PC2);
	PORTC |= (1<<PC1) | (1<<PC2);
	// Serial.println(DDRC, 10);
}

void init_timer1(void)
{
	cli();
	TCCR2A|=0x00;
	TCCR2B|=(1<<WGM12)|(1<<CS10);
	TIMSK2|=(1<<OCIE2A);
	TIFR2|=(1<<OCF2A);
	OCR2A=1600-1;
	sei();
}

void init_rtc()
{
	Serial.print("compiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);

    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    // printDateTime(compiled);
    Serial.println();

    if (!Rtc.IsDateTimeValid())
    {
        // Common Causes:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(compiled);
    }

    if (Rtc.GetIsWriteProtected())
    {
        Serial.println("RTC was write protected, enabling writing now");
        Rtc.SetIsWriteProtected(false);
    }

    if (!Rtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    now = Rtc.GetDateTime();
    if (now < compiled)
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled)
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled)
    {
		Serial.println("RTCisthesameascompiletime!(notexpectedbutallisfine)");
    }

    // Serial.println("here");
}

void init_lcd()
{
	Serial.begin(9600);
	lcd.begin(16, 2);
}

void init_pc()
{
	pc.hour = EEPROM.read(0);
	pc.minute = EEPROM.read(1);
}

ISR(TIMER2_COMPA_vect)
{
	idle();
}

ISR(INT0_vect)        //External interrupt_one ISR
{
	cnt_one++;
	// Serial.println("catched here");

	// lcd.setCursor(0,1);
	// lcd.print(cnt_one, 10);

	EEPROM.update(0, pc.hour); // hour on address 0
	EEPROM.update(1, pc.minute); // minute on address 1
}


void setup ()
{
	init_lcd();
	init_rtc();
	init_OS();
	init_pc();
	init_timer1();
	init_ports_for_ei();
	init_external_interrupts();
	init_ports_for_ticking();

	scheduler();
}

void loop ()
{
    // int x;
    // RtcDateTime now = Rtc.GetDateTime();
    //
    // printDateTime(now);
    // Serial.println();
    //
    // x = analogRead (0);
    // lcd.setCursor(10,1);
    //
    // if (!now.IsValid())
    // {
    //     // Common Causes:
    //     //    1) the battery on the device is low or even missing and
	//		//the power line was disconnected
    //     Serial.println("RTC lost confidence in the DateTime!");
    // }
    //
    //
    //
    //  if (x < 60) {
    //    lcd.print ("Right ");
    //  }
    //  else if (x < 200) {
    //    lcd.print ("Up    ");
    //  }
    //  else if (x < 400){
    //    lcd.print ("Down  ");
    //  }
    //  else if (x < 600){
    //    lcd.print ("Left  ");
    //  }
    //  else if (x < 800){
    //    lcd.print ("Select");
    //    Serial.println("here");
    //
    // // delay(1000); // ten seconds
    // }
}



void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
    lcd.setCursor(0,0);
    lcd.print(datestring);
}
void print_in_time_to_lcd()
{
    char datestring[9];

    snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u:%02u:%02u"),
            now.Hour(),
            now.Minute(),
            now.Second() );
    // Serial.print(datestring);
    // lcd.setCursor(0,0);
    lcd.print(datestring);
}

void print_out_time_to_lcd()
{
    char datestring[6];

    snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u:%02u"),
            pc.hour,
            pc.minute );
    // Serial.print(datestring);
    // lcd.setCursor(0,0);
    lcd.print(datestring);
}
