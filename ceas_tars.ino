
// CONNECTIONS:
// DS1302 CLK/SCLK -. D5
// DS1302 DAT/IO -. D4
// DS1302 RST/CE -. D2
// DS1302 VCC -. 3.3v - 5v
// DS1302 GND -. GND

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

#define LEFT_ARROW 127
#define RIGHT_ARROW 126
#define SPACE 0x20u
#define V1 0x01
#define V2 0x02
#define X1 0x03
#define X2 0x04

#define HOUR    0x00u
#define MINUTES 0x01u
#define COLON   0x02u
#define ARROWS  0x03u
#define START   0x04u
#define CANCEL  0x05u
#define DAY     0x06u
#define MONTH   0x07u
#define YEAR    0x08u
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

uint16 minute_light_on; // the minute in which the night-light turns ON
uint16 minute_light_off; //the minute in which the night-light turns OFF

phisical_clock pc;
typedef enum fsm_state
{
	idl,
	sci,
	scio,
	scim,
	sciv,
	scix,
	ice,
	iceo,
	icem,
	icev,
	icex,
	sd,
	sdz,
	sdl,
	sda,
	sdv,
	sdx,
	sod,
	sodo,
	sodv,
	sodx,
	soi,
	soio,
	soiv,
	soix,
	mi,
	mim,
	miv,
	mix,
	ext,
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

const char* sci_string = "Set ceas int";
const char* ice_string = "Ind ceas ext";
const char* sd_string =  "Seteaza data";
const char* lum_ora_deschis_sting = "Lumina ON";
const char* lum_ora_inchis_string = "Lumina OFF";
const char* mod_iluminat_string = "Mod Iluminat";
const char* exit_string = "Inapoi->idle";
const char* start_string = "Start";
const char* il_deschis_string = "ilum. deschis";
const char* il_inchis_string =  "ilum. inchis";

const char month[12][3] = {"ian", "feb", "mar", "apr", "mai", "iun",
						 "iul", "aug", "sep", "oct", "noi", "dec" };

typedef struct disp
{
	char line1[16];
	char line2[16];
	uint8 blink1[2] = {17, 17};
	uint8 blink2[2] = {17, 17};
	uint8 scroll1[3] = {17,17, 17}; // first char, last char, length
	uint8 scroll2[3] = {17,17, 17}; // first char, last char, length
	uint8 len1 = 0;
	uint8 len2 = 0;

} disp;

disp display;

uint8 buffer;

uint16 bt_rezistors[5] = {60, 200, 400, 600, 800};

void printDateTime(const RtcDateTime& dt);
void cyclic_rtc(void);
void get_out_time(void);
void print_in_time_to_lcd(void);
void lcd_cyclic(void);
void update_lcd(void);

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

	lcd_cyclic();
	lcd.setCursor(7,1);
	lcd.print("  ");
	lcd.setCursor(1,0);
	lcd.print(fsm_state, 10);
}
void task_50ms(void)
{
}
void task_100ms(void)
{
	cyclic_rtc();
	// static uint32 i = 0;
	// lcd.setCursor(0,0);
    // lcd.print(i , 10);
	// i++;
}
void pc_cyclic(void)
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

uint8 get_hour()
{
	return FALSE;
}

uint8 get_minute()
{
	return FALSE;
}

uint8 get_second()
{
	return FALSE;
}

void cus_rtc()
{
	uint8 hour, minute, second;
	hour = get_hour();
	minute = get_minute();
	hour = get_second();

	return minute;

}

void blink_menu(uint8 menu)
{
	static uint8 i,j;

	if( i <= 2)
	{
		switch(menu)
		{
			case ARROWS:
				display.line1[ 0] = SPACE;
				display.line1[15] = SPACE;
				break;
			case HOUR:
				display.line2[11] = SPACE;
				display.line2[12] = SPACE;
				break;
			case MINUTES:
				display.line2[15] = SPACE;
				display.line2[14] = SPACE;
				break;
			case COLON:
				display.line2[13] = SPACE;
				break;
			case START:
				display.line2[ 1] = SPACE;
				display.line2[ 2] = SPACE;
				break;
			case CANCEL:
				display.line2[ 4] = SPACE;
				display.line2[ 5] = SPACE;
				break;
			case DAY:
				display.line2[ 7] = SPACE;
				display.line2[ 8] = SPACE;
				break;
			case MONTH:
				display.line2[10] = SPACE;
				display.line2[11] = SPACE;
				display.line2[12] = SPACE;
				break;
			case YEAR:
				display.line2[14] = SPACE;
				display.line2[15] = SPACE;
				break;
			default:
				break;
		}
	}
	else if( i >= 4)
	{
		i = 0;
	}
	i++;
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
void reset_display()
{
	display.scroll1[0] = 17;
	display.scroll1[1] = 17;
	display.scroll1[2] = 17;
	display.scroll2[0] = 17;
	display.scroll2[1] = 17;
	display.scroll2[2] = 17;
	display.blink1[0] = 17;
	display.blink1[1] = 17;
	display.blink2[0] = 17;
	display.blink2[1] = 17;
	display.len1 = 16;
	display.len2 = 16;
	memcpy(display.line1, "                ", 16 * sizeof(char));
	memcpy(display.line2, "                ", 16 * sizeof(char));
}
void apply_vx(void)
{
	display.line2[ 0] = SPACE;
	display.line2[ 1] = V1;
	display.line2[ 2] = V2;
	display.line2[ 3] = SPACE;
	display.line2[ 4] = X1;
	display.line2[ 5] = X2;
}
void apply_arrows(void)
{
	display.line1[ 0] = LEFT_ARROW;
	display.line1[15] = RIGHT_ARROW;
}
void idl_disp()
{
	reset_display();
	memcpy(display.line1, "     idle screen", 16 * sizeof(char));
	memcpy(display.line2, " merge          ", 16 * sizeof(char));
}
void sci_disp()
{
	char clock_string[9];
	reset_display();

	get_rtc_time(clock_string);

	memcpy(display.line1 + 2, sci_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_arrows();
	apply_vx();
	blink_menu(ARROWS);
	blink_menu(COLON);
}
void scio_disp()
{
	char clock_string[9];
	reset_display();

	get_rtc_time(clock_string);

	memcpy(display.line1 + 2, sci_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_vx();
	blink_menu(HOUR);
}
void scim_disp()
{
	char clock_string[9];
	reset_display();

	get_rtc_time(clock_string);

	memcpy(display.line1 + 2, sci_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_vx();
	blink_menu(MINUTES);
}
void sciv_disp()
{
	char clock_string[9];
	reset_display();

	get_rtc_time(clock_string);

	memcpy(display.line1 + 2, sci_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_vx();
	blink_menu(START);
}
void scix_disp()
{
	char clock_string[9];
	reset_display();

	get_rtc_time(clock_string);

	memcpy(display.line1 + 2, sci_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_vx();
	blink_menu(CANCEL);
}
void ice_disp()
{
	char clock_string[6];
	// reset_display();

	get_out_time(clock_string);

	memcpy(display.line1 + 2, ice_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_arrows();
	apply_vx();
	blink_menu(ARROWS);
}
void iceo_disp()
{
	char clock_string[6];
	reset_display();

	get_out_time(clock_string);

	memcpy(display.line1 + 2, ice_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_vx();

	blink_menu(HOUR);
}
void icem_disp()
{
	char clock_string[6];
	reset_display();

	get_out_time(clock_string);

	memcpy(display.line1 + 2, ice_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_vx();

	blink_menu(MINUTES);
}
void icev_disp()
{
	char clock_string[6];
	reset_display();

	get_out_time(clock_string);

	memcpy(display.line1 + 2, ice_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_vx();

	blink_menu(START);
}
void icex_disp()
{
	char clock_string[6];
	reset_display();

	get_out_time(clock_string);

	memcpy(display.line1 + 2, ice_string, 12 * sizeof(char));
	memcpy(display.line2 + 11, clock_string , 6 * sizeof(char));

	apply_vx();

	blink_menu(CANCEL);
}
void sd_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, sd_string, 12 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_arrows();
	apply_vx();
	blink_menu(ARROWS);
}
void sdz_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, sd_string, 12 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_vx();
	blink_menu(DAY);
}
void sdl_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, sd_string, 12 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_vx();
	blink_menu(MONTH);
}
void sda_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, sd_string, 12 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_vx();
	blink_menu(YEAR);
}
void sdv_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, sd_string, 12 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_vx();
	blink_menu(START);
}
void sdx_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, sd_string, 12 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_vx();
	blink_menu(CANCEL);
}
void sod_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, lum_ora_deschis_sting, 9 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_arrows();
	apply_vx();
	blink_menu(ARROWS);
}
void sodo_disp()
{

}
void sodv_disp()
{

}
void sodx_disp()
{

}
void soi_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, lum_ora_inchis_string, 10 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_arrows();
	apply_vx();
	blink_menu(ARROWS);
}
void soio_disp()
{

}
void soiv_disp()
{

}
void soix_disp()
{

}
void mi_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, mod_iluminat_string, 12 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_arrows();
	apply_vx();
	blink_menu(ARROWS);
}
void mim_disp()
{

}
void miv_disp()
{

}
void mix_disp()
{

}
void ext_disp()
{
	char date_string[11];
	reset_display();

	get_date(date_string);
	date_string[7] = date_string[9];
	date_string[8] = date_string[10];

	memcpy(display.line1 + 2, exit_string, 12 * sizeof(char));
	memcpy(display.line2 + 7, date_string , 11 * sizeof(char));

	apply_arrows();
	apply_vx();
	blink_menu(ARROWS);
}

void update_lcd()
{
	static char old_line1[16], old_line2[16];
	for(uint8 i=0; i < 16; i++)
	{
		if(old_line1[i] != display.line1[i])
		{
			lcd.setCursor(i,0);
			lcd.print(display.line1[i]);
			old_line1[i] = display.line1[i];
		}
		if(old_line2[i] != display.line2[i])
		{
			lcd.setCursor(i,1);
			lcd.print(display.line2[i]);
			old_line2[i] = display.line2[i];
		}
	}
}


typedef void (*disp_func_ptr)();

disp_func_ptr disp_func[num_of_states] = {
	idl_disp,
	sci_disp,
	scio_disp,
	scim_disp,
	sciv_disp,
	scix_disp,
	ice_disp,
	iceo_disp,
	icem_disp,
	icev_disp,
	icex_disp,
	sd_disp,
	sdz_disp,
	sdl_disp,
	sda_disp,
	sdv_disp,
	sdx_disp,
	sod_disp,
	sodo_disp,
	sodv_disp,
	sodx_disp,
	soi_disp,
	soio_disp,
	soiv_disp,
	soix_disp,
	mi_disp,
	mim_disp,
	miv_disp,
	mix_disp,
	ext_disp,
};

void lcd_cyclic()
{
	static uint8 counter;
	char line1[16], line2[16];

	disp_func[fsm_state]();

	for(uint8 i = 0; i < 16; i++)
	{
		// scrolling line 1
		if(i < display.scroll1[0])
		{
			line1[i] = display.line1[i];
		}
		else if (i >= display.scroll1[0] &&
			   	 i < (display.scroll1[0] + display.scroll1[2]))
		{
			line1[i] = display.line1[i + ((counter/2) %
						(display.scroll1[2] - display.scroll1[1]))];
		}
		else
		{
			line1[i] = display.line1[i + display.scroll1[1] -
									display.scroll1[0] - display.scroll1[2]];
		}

		// scrolling line 2}
		if(i < display.scroll2[0])
		{
			line2[i] = display.line2[i];
		}
		else if (i >= display.scroll2[0] &&
				i < (display.scroll2[0] + display.scroll2[2]))
		{
			line2[i] = display.line2[i + ((counter/2) %
					(display.scroll2[2] - display.scroll2[1]))];
		}
		else
		{
			line2[i] = display.line2[i + display.scroll2[1] -
				display.scroll2[0] - display.scroll2[2]];
		}

		// blinking
		if(counter % 4 < 2 &&  i >= display.blink1[0] && i < display.blink1[1])
		{
			line1[i] = ' ';
		}
		if(counter % 4 < 2 &&  i >= display.blink2[0] && i < display.blink2[1])
		{
			line2[i] = ' ';
		}
	}

	counter++;
	update_lcd();
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
	TCCR1A|=0x00;
	TCCR1B|=(1<<WGM12)|(1<<CS10);
	TIMSK1|=(1<<OCIE2A);
	TIFR1|=(1<<OCF2A);
	OCR1A=1600-1;
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
        // Serial.println("RTC is older than compile time!(Updating DateTime)");
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
	byte validate_1[8] = { B00000,
						   B00000,
						   B00000,
					       B10000,
					       B11000,
						   B01100,
					       B01101,
						   B00110 };
	byte validate_2[8] = { B00000,
						   B00011,
						   B00110,
					       B01100,
					       B11000,
						   B10000,
					       B10000,
						   B00000 };
	byte cancel_1[8] =   { B10000,
						   B01000,
						   B00100,
					       B00010,
					       B00001,
						   B00011,
					       B00110,
						   B01100 };
	byte cancel_2[8] =   { B00001,
						   B00111,
						   B00110,
					       B01100,
					       B11000,
						   B11000,
					       B00110,
						   B00011 };
	lcd.createChar(1, validate_1);
	lcd.createChar(2, validate_2);
	lcd.createChar(3, cancel_1);
	lcd.createChar(4, cancel_2);
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
e_fsm_state sciv_f(void)
{
	return sciv;
}
e_fsm_state scix_f(void)
{
	return scix;
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
e_fsm_state icev_f(void)
{
	return icev;
}
e_fsm_state icex_f(void)
{
	return icex;
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
e_fsm_state sdv_f(void)
{
	return sdv;
}
e_fsm_state sdx_f(void)
{
	return sdx;
}
e_fsm_state sod_f(void)
{
	return sod;
}
e_fsm_state sodo_f(void)
{
	return sodo;
}
e_fsm_state sodv_f(void)
{
	return sodv;
}
e_fsm_state sodx_f(void)
{
	return sodx;
}
e_fsm_state soi_f(void)
{
	return soi;
}
e_fsm_state soio_f(void)
{
	return soio;
}
e_fsm_state soiv_f(void)
{
	return soiv;
}
e_fsm_state soix_f(void)
{
	return soix;
}
e_fsm_state mi_f(void)
{
	return mi;
}
e_fsm_state mim_f(void)
{
	return mim;
}
e_fsm_state miv_f(void)
{
	return miv;
}
e_fsm_state mix_f(void)
{
	return mix;
}
e_fsm_state ext_f(void)
{
	return ext;
}

// typedef (*e_fsm_state)(void);
typedef e_fsm_state (*fsm_ptr)(void);
fsm_ptr transition_table[num_of_states][NO_BT_PRESSED] =
{
/*                 right      up         down       left       select   */
/* 0   idl */     {  sci_f,     sci_f,     sci_f,     sci_f,     sci_f    },

/* 1   sci */  	  {  ice_f,     NULL,      scix_f,    ext_f,     scio_f   },
/* 2  scio */     {  scim_f,    scio_f,    scio_f,    scix_f,    scim_f   },
/* 3  scim */	  {  NULL,      scim_f,    scim_f,    scio_f,    sciv_f   },
/* 4  sciv */	  {  scix_f,    NULL,      NULL,      NULL,      sci_f    },
/* 5  scix */	  {  scio_f,    NULL,      NULL,      sciv_f,    sci_f    },

/* 6   ice */	  {  sd_f,      NULL,      icex_f,    sci_f,     iceo_f   },
/* 7  iceo */	  {  icem_f,    iceo_f,    iceo_f,    icex_f,    icem_f   },
/* 8  icem */	  {  NULL,      icem_f,    icem_f,    iceo_f,    icev_f   },
/* 9  icev */	  {  icex_f,    NULL,      NULL,      NULL,      ice_f    },
/* 10 icex */	  {  iceo_f,    NULL,      NULL,      icev_f,    ice_f    },

/* 11   sd */	  {  sod_f,     NULL,      sdz_f,     ice_f,     sdz_f    },
/* 12  sdz */	  {  sdl_f,     sdz_f,     sdx_f,     sdx_f,     sdl_f    },
/* 13  sdl */	  {  sda_f,     sdl_f,     sdl_f,     sdz_f,     sda_f    },
/* 14  sda */	  {  NULL,      sda_f,     sda_f,     sdl_f,     sdv_f    },
/* 15  sdv */	  {  sdx_f,     NULL,      NULL,      NULL,      sd_f     },
/* 16  sdx */	  {  sdz_f,     NULL,      NULL,      sdv_f,     sd_f     },

/* 17   sod */	  {  soi_f,     NULL,      NULL,      sd_f,      sodo_f   },
/* 18  sodo */	  {  NULL,      sodo_f,    sodo_f,    sodx_f,    sodv_f   },
/* 19  sodv */	  {  sodx_f,    NULL,      NULL,      NULL,      sod_f    },
/* 20  sodx */	  {  sodo_f,    NULL,      NULL,      sodv_f,    sod_f    },

/* 21   soi */	  {  mi_f,      NULL,      NULL,      sod_f,     soio_f   },
/* 22  soio */	  {  NULL,      soio_f,    soio_f,    soix_f,    soiv_f   },
/* 23  soiv */	  {  soix_f,    NULL,      NULL,      NULL,      soi_f    },
/* 24  soix */	  {  soio_f,    NULL,      NULL,      soiv_f,    soi_f    },

/* 25    mi */	  {  ext_f,     NULL,      NULL,      soi_f,     mim_f    },
/* 26   mim */	  {  NULL,      mim_f,     mim_f,     soix_f,    miv_f    },
/* 27   miv */	  {  mix_f,     NULL,      NULL,      NULL,      mi_f     },
/* 28   mix */	  {  mim_f,     NULL,      NULL,      soiv_f,    mi_f     },

/* 29   ext */	  {  sci_f,     NULL,      NULL,      mi_f,      idl_f    },
};


void fsm_cyclic()
{

	if (but_pressed != NO_BT_PRESSED &&
			transition_table[fsm_state][but_pressed] != NULL)
	{
		fsm_state = transition_table[fsm_state][but_pressed]();
		but_pressed = NO_BT_PRESSED;
	}
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
            // countof(datestring)
			9,
            PSTR("%02u:%02u:%02u"),
            now.Hour(),
            now.Minute(),
            now.Second() );
    lcd.print(datestring);
}
void get_rtc_time(char string[9])
{
    snprintf_P(string, 9, PSTR("%02u:%02u:%02u"), now.Hour(), now.Minute(),
            now.Second() );
}

void get_date(char string[12])
{
    snprintf_P(string, 12, PSTR("%02u/   /%04u"), now.Day(), now.Year() );
	memcpy(string + 3, month[now.Month() - 1], 3);
}

void get_out_time(char string[6])
{
    snprintf_P(string, 6, PSTR("%02u:%02u"), pc.hour, pc.minute );
}

void print_night_time_on(char string[6])
{
	snprintf_P(string, 6, PSTR("%02u:%02u"),
		minute_light_on / 60,
	 	minute_light_on % 60);
}
void print_night_time_off(char string[6])
{
	snprintf_P(string, 6, PSTR("%02u:%02u"),
		minute_light_off / 60,
	 	minute_light_off % 60);
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

ISR(TIMER1_COMPA_vect)
{
	idle();
}

ISR(INT0_vect)        //External interrupt_one ISR
{
	EEPROM.update(0, pc.hour); // hour on address 0
	EEPROM.update(1, pc.minute); // minute on address 1
}
