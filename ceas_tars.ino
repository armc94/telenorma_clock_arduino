
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
#include <string.h>

typedef signed char sint8;
typedef unsigned char uint8;
typedef signed int sint16;
typedef unsigned int uint16;
typedef signed long sint32;
typedef unsigned long uint32;
typedef void (*function_pointer)(void);

#define countof(a) (sizeof(a) / sizeof(a[0]))
#define SET_BIT(x, pos) (x |= (1U << pos))
#define CLEAR_BIT(x, pos) (x &= (~(1U<< pos)))
#define TOGGLE_BIT(x, pos) x ^= (1U<< pos)

#define NUM_TASKS 6
#define NOT_ACTUATED 0
#define ACTUATED 1
#define FALSE 0
#define TRUE 1
#define pin_RS  8
#define pin_EN  9
#define pin_d4  4
#define pin_d5  5
#define pin_d6  6
#define pin_d7  7
#define pin_BL  10

typedef enum task_state
{
	WAIT,
	READY,
	ACTIVE,
}task_state_e;


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
	idl,
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
	inapoi,
	num_of_states
} e_fsm_state;

e_fsm_state fsm_state = idl;
typedef enum buttons
{
	BT_RIGHT,
	BT_UP,
	BT_DOWN,
	BT_LEFT,
	BT_SELECT,
	NO_BT_PRESSED
} buttons_enum;

const char* sci_string = "seteaza ceasul intern";

typedef struct disp
{
	char*  line1 = "                ";
	char*  line2 = "                ";
	uint8 blink1[2] = {17, 17};
	uint8 blink2[2] = {17, 17};
	uint8 scroll1[3] = {17,17, 17}; // first char, last char, length
	uint8 scroll2[3] = {17,17, 17}; // first char, last char, length
	uint8 len1 = 0;
	uint8 len2 = 0;

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

	// lcd.setCursor(0,1);
	// lcd.print(but_pressed, 10);
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
	fsm_cyclic();
}
void task_20ms(void)
{

}
void task_50ms(void)
{
	lcd_cyclic();
    cyclic_rtc();
}
void task_100ms(void)
{
	// static uint32 i = 0;
	// lcd.setCursor(0,0);
    // lcd.print(i , 10);
	// i++;
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
	    // Serial.println("RTC lost confidence in the DateTime!");
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

void idl_disp(disp* display)
{
	display->line1 = "da, am folosit scroll: asteapta!";
	display->line2 = "nu";
	display->blink1[0] = 17;
	display->blink1[1] = 17;
	display->scroll1[0] = 3;
	display->scroll1[1] = 23;
	display->scroll1[2] = 4;
	display->len1 = 32;
	display->len2 = 2;
}
void sci_disp(disp* display)
{
	idl_disp(display);
	display->line2[0] = 'r';
}
void scio_disp(disp* display)
{

}
void scim_disp(disp* display)
{

}
void scis_disp(disp* display)
{

}
void scii_disp(disp* display)
{

}
void acf_disp(disp* display)
{

}
void acfam_disp(disp* display)
{

}
void acfazm_disp(disp* display)
{

}
void acfao_disp(disp* display)
{

}
void acfasm_disp(disp* display)
{

}
void acfaszm_disp(disp* display)
{

}
void acfata_disp(disp* display)
{

}
void acfi_disp(disp* display)
{

}
void ice_disp(disp* display)
{

}
void iceo_disp(disp* display)
{

}
void icem_disp(disp* display)
{

}
void icess_disp(disp* display)
{

}
void icei_disp(disp* display)
{

}
void sd_disp(disp* display)
{

}
void sdz_disp(disp* display)
{

}
void sdl_disp(disp* display)
{

}
void sda_disp(disp* display)
{

}
void sdi_disp(disp* display)
{

}
void inapoi_disp(disp* display)
{

}

void update_lcd(char* line1, char* line2)
{
	static char old_line1[16], old_line2[16];
	for(uint8 i; i < 16; i++)
	{
		if(old_line1[i] != line1[i])
		{
			lcd.setCursor(i,0);
			lcd.print(line1[i]);
		}
		if(old_line2[i] != line2[i])
		{
			lcd.setCursor(i,1);
			lcd.print(line2[i]);
		}
	}
}


typedef void (*disp_func_ptr)(disp*);

disp_func_ptr disp_func[num_of_states] = {
	idl_disp,
	sci_disp,
	scio_disp,
	scim_disp,
	scis_disp,
	scii_disp,
	acf_disp,
	acfam_disp,
	acfazm_disp,
	acfao_disp,
	acfasm_disp,
	acfaszm_disp,
	acfata_disp,
	acfi_disp,
	ice_disp,
	iceo_disp,
	icem_disp,
	icess_disp,
	icei_disp,
	sd_disp,
	sdz_disp,
	sdl_disp,
	sda_disp,
	sdi_disp,
	inapoi_disp,
};

void lcd_cyclic()
{
	// lcd.setCursor(7 ,0);
	// print_in_time_to_lcd();
	//
	// lcd.setCursor(10 ,1);
	// print_out_time_to_lcd();

	static uint8 counter;
	char line1[16], line2[16];

	disp_func[fsm_state](&display);

	for(uint8 i = 0; i < 16; i++)
	{
		if(i < display.scroll1[0])
			line1[i] = display.line1[i];
		else if (i >= display.scroll1[0] && i < (display.scroll1[0] + display.scroll1[2]))
			line1[i] = display.line1[i + (counter % (display.scroll1[2] - display.scroll1[1]))];
		else if (i >= (display.scroll1[0] + display.scroll1[2]))
			line1[i] = display.line1[i + display.scroll1[1] - display.scroll1[0] - display.scroll1[2]];

		if( counter % 4 < 2 &&  i > display.blink1[0] && i < display.blink1[1])
		{
			line1[i] = ' ';
		}
		// else
		// {
		// 	line1[i] = display.line1[i];
		// }
		if( counter % 4 < 2 &&  i > display.blink2[0] && i < display.blink2[1])
		{
			line2[i] = ' ';
		}
		// else
		// {
		// 	line2[i] = display.line2[i];
		// }
	}

	counter++;

	update_lcd(line1, line2);

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
	// Serial.print("compiled: ");
    // Serial.print(__DATE__);
    // Serial.println(__TIME__);

    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    // printDateTime(compiled);
    // Serial.println();

    if (!Rtc.IsDateTimeValid())
    {
        // Common Causes:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        // Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(compiled);
    }

    if (Rtc.GetIsWriteProtected())
    {
        // Serial.println("RTC was write protected, enabling writing now");
        Rtc.SetIsWriteProtected(false);
    }

    if (!Rtc.GetIsRunning())
    {
        // Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    now = Rtc.GetDateTime();
    if (now < compiled)
    {
        // Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled)
    {
        // Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled)
    {
		// Serial.println("RTCisthesameascompiletime!(notexpectedbutallisfine)");
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

e_fsm_state idl_f(void)
{
	return idl;
}
e_fsm_state sci_f(void)
{
	return sci;
}
e_fsm_state scio_f(void)
{
	return scio;
}
e_fsm_state scim_f(void)
{
	return scim;
}
e_fsm_state scis_f(void)
{
	return scis;
}
e_fsm_state scii_f(void)
{
	return scii;
}
e_fsm_state acf_f(void)
{
	return acf;
}
e_fsm_state acfam_f(void)
{
	return acfam;
}
e_fsm_state acfazm_f(void)
{
	return acfazm;
}
e_fsm_state acfao_f(void)
{
	return acfao;
}
e_fsm_state acfasm_f(void)
{
	return acfasm;
}
e_fsm_state acfaszm_f(void)
{
	return acfaszm;
}
e_fsm_state acfata_f(void)
{
	return acfata;
}
e_fsm_state acfi_f(void)
{
	return acfi;
}
e_fsm_state ice_f(void)
{
	return ice;
}
e_fsm_state iceo_f(void)
{
	return iceo;
}
e_fsm_state icem_f(void)
{
	return icem;
}
e_fsm_state icess_f(void)
{
	return icess;
}
e_fsm_state icei_f(void)
{
	return icei;
}
e_fsm_state sd_f(void)
{
	return sd;
}
e_fsm_state sdz_f(void)
{
	return sdz;
}
e_fsm_state sdl_f(void)
{
	return sdl;
}
e_fsm_state sda_f(void)
{
	return sda;
}
e_fsm_state sdi_f(void)
{
	return sdi;
}
e_fsm_state inapoi_f(void)
{
	return inapoi;
}

// typedef (*e_fsm_state)(void);
typedef e_fsm_state (*fsm_ptr)(void);
fsm_ptr transition_table[num_of_states][NO_BT_PRESSED] =
{
/*             right      up         down       left       select   */
/* idl */     {sci_f,     sci_f,     sci_f,     sci_f,     sci_f    },
/* sci */  	  {acf_f,     NULL,      NULL,      inapoi_f,  scio_f   },
/* scio */    {NULL,      NULL,      NULL,      NULL,      NULL     },
/* scim */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* scis */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* scii */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* acf */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* acfam */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* acfazm */  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* acfao */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* acfasm */  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* acfaszm */ {NULL,      NULL,      NULL,      NULL,      NULL     },
/* acfata */  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* acfi */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* ice */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* iceo */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* icem */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* icess */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* icei */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* sd */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* sdz */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* sdl */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* sda */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* sdi */	  {NULL,      NULL,      NULL,      NULL,      NULL     },
/* inapoi */  {NULL,      NULL,      NULL,      NULL,      NULL     },

};



void fsm_cyclic()
{

	if (but_pressed != NO_BT_PRESSED && transition_table[fsm_state][but_pressed] != NULL)
	{
		fsm_state = transition_table[fsm_state][but_pressed]();
		but_pressed = NO_BT_PRESSED;
	}
	// p_fsm das = f_SB;
	// lcd.setCursor(3,1);
	// lcd.print(fsm_state, 10);
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
