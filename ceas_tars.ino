
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

RtcDateTime now;

typedef struct	phisical_clock
{
	uint8 minute;
	uint8 hour;
	boolean tick_requested;
} phisical_clock_s;

phisical_clock pc;
typedef enum fsm_state
{
	sci,
	scio,
	scim,
	scis,
	scii,
	acf,
	acfam,
	acfazm,
	acfao,
	acfasm,
	acfaszm,
	acfata,
	acfi,
	ice,
	iceo,
	icem,
	icess,
	icei,
	sd,
	sdz,
	sdl,
	sda,
	sdi,
	num_of_states
} e_fsm_state;
typedef enum buttons
{
	BT_RIGHT,
	BT_UP,
	BT_DOWN,
	BT_LEFT,
	BT_SELECT,
	NO_BT_PRESSED
} buttons_enum;

typedef struct disp
{
	char line1[16] = "                ";
	char line2[16] = "                ";
	uint8 blink[2] = {17, 17};
} disp;

disp display;

uint16 bt_rezistors[5] = {60, 200, 400, 600, 800};


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
buttons_enum but_pressed = NO_BT_PRESSED;

void debounce_D()
{
	uint16 x = analogRead (0);
	uint8 button = NO_BT_PRESSED;
	static uint8 buffers[5] = {0,0,0,0,0};

	if      (x < bt_rezistors[BT_RIGHT] ) button = BT_RIGHT;
	else if (x < bt_rezistors[BT_UP]    ) button = BT_UP;
	else if (x < bt_rezistors[BT_DOWN]  ) button = BT_DOWN;
	else if (x < bt_rezistors[BT_LEFT]  ) button = BT_LEFT;
	else if (x < bt_rezistors[BT_SELECT]) button = BT_SELECT;

	for(uint8 bt = BT_RIGHT; bt < NO_BT_PRESSED; bt++)
	{
		buffers[bt] <<= 1;
		if (button == bt) buffers[bt] |= 0x01u;
		if (buffers[bt] == 0x0f) but_pressed = bt;
	}

	lcd.setCursor(0,1);
	lcd.print(but_pressed, 10);
}



void init_OS(void)
{
	for(uint8 i=0;i<NUM_TASKS;i++)
	{
		tablou_tasks[i].current_time=tablou_tasks[i].delay;
	}

}
void idle(void)
{
	for(uint8 i=0;i<NUM_TASKS;i++)
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
		for(uint8 i=0;i<NUM_TASKS;i++)
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
	debounce_D();
}
void task_5ms(void)
{

}
void task_10ms(void)
{
	pc_cyclic();
	tick_cyclic();
	lcd_cyclic();
	cyclic_fsm();
}
void task_20ms(void)
{

}
void task_50ms(void)
{
    cyclic_rtc();
}
void task_100ms(void)
{
	static uint32 i = 0;
	lcd.setCursor(0,0);
    lcd.print(i , 10);
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

	// Good idea to throw an error...
	if (!now.IsValid())
	{
	    // Common Causes:
	    //    1) the battery on the device is low or even missing and
			//the power line was disconnected
	    Serial.println("RTC lost confidence in the DateTime!");
	}
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

	lcd.setCursor(10 ,1);
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
}
void init_ports_for_ticking()
{
	DDRC  |= (1<<PC1) | (1<<PC2);
	PORTC |= (1<<PC1) | (1<<PC2);
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

e_fsm_state sci()
{
	return SA;
}
e_fsm_state f_SB()
{
	return SB;
}
e_fsm_state f_SC()
{
	return SC;
}
e_fsm_state f_SD()
{
	return SD;
}

// typedef (*e_fsm_state)(void);
typedef e_fsm_state (*fsm_ptr)(void);
fsm_ptr transition_table[num_of_states][NO_BT_PRESSED] = {
	{ f_SB, NULL, NULL, NULL, NULL },
	{ NULL, NULL, f_SC, f_SA, NULL },
	{ f_SD, f_SB, NULL, NULL, NULL },
	{ NULL, NULL, NULL, f_SC, NULL },
};



void cyclic_fsm()
{
	static e_fsm_state prev_fsm_state, cur_fsm_state = SA;

	if (but_pressed != NO_BT_PRESSED && transition_table[cur_fsm_state][but_pressed] != NULL)
	{
		prev_fsm_state = cur_fsm_state;
		cur_fsm_state = transition_table[cur_fsm_state][but_pressed]();
		but_pressed = NO_BT_PRESSED;
	}
	// p_fsm das = f_SB;
	lcd.setCursor(3,1);
	lcd.print(cur_fsm_state, 10);
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
    lcd.print(datestring);
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

}

ISR(TIMER2_COMPA_vect)
{
	idle();
}

ISR(INT0_vect)        //External interrupt_one ISR
{
	EEPROM.update(0, pc.hour); // hour on address 0
	EEPROM.update(1, pc.minute); // minute on address 1
}
