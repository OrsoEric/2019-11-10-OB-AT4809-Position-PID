/****************************************************************
**	OrangeBot Project
*****************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************
**	OrangeBot AT4809 Firmware
*****************************************************************
**  This firmware is meant to run on the OrangeBot robot due to
**	participate in the PiWars 2020
**
**	Compiler flags: -std=c++11 -fno-threadsafe-statics -fkeep-inline-functions
**		-Os							|
**		-std=c++11					|
**		-fno-threadsafe-statics		| disable mutex around static local variables
**		-fkeep-inline-functions		| allow use of inline methods outside of .h for PidS16
****************************************************************/

/****************************************************************
**	DESCRIPTION
****************************************************************
**	OrangeBot MVP
**	UART interface with RPI
**	Uniparser V4
**	Set PWM from serial interface
****************************************************************/

/****************************************************************
**	HISTORY VERSION
****************************************************************
**		2019-11-01
**	Add UART3 uc <--> RPI
**	Add Uniparser V4
**	Add platform messages
**	Test OrangeBot Remote Control MVP
**		2019-11-05
**	Clone 2019-10-02 OrangeBot MVP Remote control
**	Add Encoder ISR
**		2019-11-06
**	
**		2019-11-09
**	Added Pid S16 class
****************************************************************/

/****************************************************************
**	USED PINS
****************************************************************
**	VNH7040
**				|	DRV0	|	DRV1	|	DRV2	|	DRV3	|	VNH7040
**	-------------------------------------------------------------------------
**	uC_SEN		|	PF0		|	PF0		|	PF0		|	PF0		|	SENSE ENABLE
**	uC_DIAG		|	PF1		|	PF1		|	PF1		|	PF1		|	SEL1
**	uC_PWM		|	PA2,B20	|	PA3,B21	|	PB4,B22	|	PB5,B23	|	PWM
**	uC_CTRLA	|	PA4		|	PA6		|	PB2		|	PD6		|	INA, SEL0
**	uC_CTRLB	|	PA5		|	PA7		|	PB3		|	PD7		|	INB
****************************************************************/

/****************************************************************
**	KNOWN BUGS
****************************************************************
**
****************************************************************/

/****************************************************************
**	DEFINES
****************************************************************/

#define EVER (;;)

/****************************************************************
**	INCLUDES
****************************************************************/


#include "global.h"
//Universal Parser V4
#include "uniparser.h"
//from number to string
#include "at_string.h"
//PID S16 class
#include "pid_s16.h"

/****************************************************************
** FUNCTION PROTOTYPES
****************************************************************/
	
	///----------------------------------------------------------------------
	///	PERIPHERALS
	///----------------------------------------------------------------------

//Initialize motors
extern void init_motors( void );
//! Initialize all PID controllers
extern bool init_pid( OrangeBot::Pid_s16 *pid_vector );

	///----------------------------------------------------------------------
	///	PID
	///----------------------------------------------------------------------

//TODO: make a class
extern Dc_motor_pwm convert_s16_to_pwm( int16_t input, bool f_dir );
//Set PWM of all motor channels applying slew rate limiting
extern void update_pwm( void );
//Compute speed. Unit of measure is Count/Tick.
extern bool compute_speed( int16_t *enc_speed );


//Generate double sided reference for all four motors
extern void generate_reference( Control_mode mode, uint16_t top );

/****************************************************************
** GLOBAL VARIABLES
****************************************************************/

volatile Isr_flags g_isr_flags;

	///----------------------------------------------------------------------
	///	BUFFERS
	///----------------------------------------------------------------------
	//	Buffers structure and data vectors

//Safe circular buffer for UART input data
volatile At_buf8_safe rpi_rx_buf;
//Safe circular buffer for uart tx data
At_buf8 rpi_tx_buf;
//allocate the working vector for the buffer
uint8_t v0[ RPI_RX_BUF_SIZE ];
//allocate the working vector for the buffer
uint8_t v1[ RPI_TX_BUF_SIZE ];

	///--------------------------------------------------------------------------
	///	CONTROL
	///--------------------------------------------------------------------------

//Motor control mode in use
Control_mode g_control_mode			= CONTROL_STOP;
//Target motor control mode
Control_mode g_control_mode_target	= CONTROL_STOP;
//Target for the position PID
int32_t g_pid_pos_target[ENC_NUM];

	///--------------------------------------------------------------------------
	///	MOTORS
	///--------------------------------------------------------------------------

//Desired setting for the DC motor channels
Dc_motor_pwm g_dc_motor[DC_MOTOR_NUM];
//Two DC Motor channels
Dc_motor_pwm g_dc_motor_target[DC_MOTOR_NUM];

	///--------------------------------------------------------------------------
	///	ENCODERS
	///--------------------------------------------------------------------------

//Global 32b encoder counters
volatile int32_t g_enc_cnt[ENC_NUM];
//Previous encoder reading
int32_t g_old_enc_cnt[ENC_NUM];
//Encoder speed
int16_t g_enc_spd[ENC_NUM];
//Encoder speed reference
int16_t g_pid_spd_target[ENC_NUM];

/****************************************************************************
**  Function
**  main |
****************************************************************************/
//! @return bool |
//! @brief dummy method to copy the code
//! @details test the declaration of a lambda method
/***************************************************************************/

int main(void)
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//system tick prescaler
	uint8_t pre = 0;
	//activity LED prescaler
	uint8_t pre_led = 0;
	
	//Blink speed of the LED. Start slow
	uint8_t blink_speed = 99;
	//Raspberry PI UART RX Parser
	Orangebot::Uniparser rpi_rx_parser = Orangebot::Uniparser();
	//Each encoder has an associated PID controller
	OrangeBot::Pid_s16 vnh7040_pid[ ENC_NUM ];
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

		///UART RX BUFFER INIT
	//I init the rx and tx buffers
	//attach vector to buffer
	AT_BUF_ATTACH( rpi_rx_buf, v0, RPI_RX_BUF_SIZE);
	//attach vector to buffer
	AT_BUF_ATTACH( rpi_tx_buf, v1, RPI_TX_BUF_SIZE);

	//! Initialize AT4809 internal peripherals
	init();
	//! Initialize external peripherals
	init_motors();
	//! Initialize the static vars of the encoder decoding ISR
	quad_encoder_decoder( PORTC.IN );
	//! Initialize all PID controllers
	init_pid( vnh7040_pid );

		//!	Initialize VNH7040
	//Enable sense output
	SET_BIT_VALUE( PORTF.OUT, 0, true );
	//Diagnostic mode OFF
	SET_BIT_VALUE( PORTF.OUT, 1, false );
	
		///----------------------------------------------------------------------
		///	REGISTER PARSER COMMANDS
		///----------------------------------------------------------------------

	//! Register commands and handler for the universal parser class. A masterpiece :')
	//Register ping command. It's used to reset the communication timeout
	rpi_rx_parser.add_cmd( "P", (void *)&ping_handler );
	//Register the Find command. Board answers with board signature
	rpi_rx_parser.add_cmd( "F", (void *)&signature_handler );
	//Set individual motor speed command.
	rpi_rx_parser.add_cmd( "M%SPWM%S", (void *)&set_speed_handler );
	//Set platform speed handler to be retro compatible with SoW-B
	rpi_rx_parser.add_cmd( "PWMR%SL%S", (void *)&set_platform_speed_handler );
	//Set individual motor speed command.
	rpi_rx_parser.add_cmd( "PID%SSPD%S", (void *)&set_pid_speed_handler );
	//Set platform speed handler using the on board PID controllers
	rpi_rx_parser.add_cmd( "SPDR%SL%S", (void *)&set_platform_pid_speed_handler );
	//Send encoder reading through UART
	rpi_rx_parser.add_cmd( "ENC", (void *)&get_encoder_cnt_handler );
	//Send encoder speed reading through UART
	rpi_rx_parser.add_cmd( "ENCSPD", (void *)&get_encoder_spd_handler );
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Main loop
	for EVER
	{
		//If: System Tick 500Hz
		if (g_isr_flags.system_tick == 1)
		{
			//Clear system tick
			g_isr_flags.system_tick = 0;
			//Execute speed PID
			g_isr_flags.ctrl_updt = true;
						
			//----------------------------------------------------------------
			//	SYSTEM PRESCALER SPEED CODE
			//----------------------------------------------------------------
				
			//If prescaler has reset
			if (pre == 0)
			{		
				//----------------------------------------------------------------
				//	LED BLINK
				//----------------------------------------------------------------
				//		Two speeds
				//	slow: not in timeout and commands can be executed
				//	fast: in timeout, motor stopped
				
				if (pre_led == 0)
				{
					//Toggle PF5.
					SET_BIT( PORTF.OUTTGL, 5 );	
					//DEBUG: Generate reference ramp and send encoder speed message
					//generate_reference( CONTROL_SPD, 15 );
					//get_encoder_spd_handler();
					get_encoder_cnt_handler();
				}
				//Increment with top
				pre_led = AT_TOP_INC( pre_led, blink_speed );
				
				//----------------------------------------------------------------
				//	PARSER TIMEOUT
				//----------------------------------------------------------------
				
				//Update communication timeout counter
				g_uart_timeout_cnt++;
				//
				if (g_uart_timeout_cnt >= RPI_COM_TIMEOUT)
				{
					//Clip timeout counter
					g_uart_timeout_cnt = RPI_COM_TIMEOUT;
					//If: it's the first time the communication timeout is detected
					if (g_f_timeout_detected == false)
					{						
						//LED is blinking faster
						blink_speed = 9;
					}
					//raise the timeout flag
					g_f_timeout_detected = true;
				}
				else
				{
					//This is the only code allowed to reset the timeout flag
					g_f_timeout_detected = false;
					//LED is blinking slower
					blink_speed = 99;
				}		
				
				//DEBUG: Generate Reference PWM ramp
				//generate_reference( CONTROL_PWM, 127 );
			}
			//Increment prescaler and reset if it exceeds the TOP.
			pre = AT_TOP_INC( pre, 4);
			
		}	//End If: System Tick
		
		//----------------------------------------------------------------
		//	MOTOR COTROL SYSTEM
		//----------------------------------------------------------------		
		
		//If: Authorized to execute one step the PID speed controllers
		if (g_isr_flags.ctrl_updt == 1)
		{
			//Clear flag
			g_isr_flags.ctrl_updt = 0;
				
				//! Control switch to new control system
			//If: communication timeout is detected
			if (g_f_timeout_detected == true)
			{
				//if: control system is not STOP
				if (g_control_mode != CONTROL_STOP)
				{
					//Set motors to full stop
					g_control_mode = CONTROL_STOP;
					//If, communication timeout was detected
					report_error( ERR_CODE_COMMUNICATION_TIMEOUT );
				}
				//if: control is already STOP
				else
				{
					//do nothing
				}
			}
			//if: PID saturation error has been issued
			else if (g_isr_flags.pid_sat_err == true)
			{
				//if: control system is not STOP
				if (g_control_mode != CONTROL_STOP)
				{
					//Set motors to full stop
					g_control_mode = CONTROL_STOP;
					//Report the error
					report_error( ERR_CODE_PID_UNLOCKED );
				}
				//if: control is already STOP
				else
				{
					//do nothing
				}
			}
			//otherwise
			else
			{
					//! switch control mode. Initialize target if necessary.
				//if: I'm switching between control modes
				if (g_control_mode != g_control_mode_target)
				{
					//If: Hybrid speed-position mode
					if (g_control_mode_target == CONTROL_SPD_POS)
					{
						//counter
						uint8_t t;
						//Currently accessing global encoder counters
						g_isr_flags.enc_sem = true;
						//For: Scan all encoders
						for (t = 0;t < ENC_NUM;t++)
						{
							//Initialize position target
							g_pid_pos_target[t] = g_enc_cnt[t];
						}
						//Unreserve
						g_isr_flags.enc_sem = false;
					}
				}
				
				//Control mode is the one desired by the user
				g_control_mode = g_control_mode_target;
			}
				//! Execute a step in the right control mode
			//If: control system is in STOP state
			if (g_control_mode == CONTROL_STOP)
			{
				//counter
				uint8_t t;
				//Scan all motors
				for (t = 0;t < DC_MOTOR_NUM;t++)
				{
					//Set Target PWM to zero
					g_dc_motor_target[ t ].f_dir	= false;
					g_dc_motor_target[ t ].pwm		= 0;
				}
				//Update PWM of the motors while applying the slew rate limiter
				update_pwm();
			}	//End If: control system is in STOP state
			//If: control system is open loop PWM
			else if (g_control_mode == CONTROL_PWM)
			{
				//Update PWM of the motors while applying the slew rate limiter
				update_pwm();
			}	//End If: control system is open loop PWM
			//If: control system is closed loop speed
			else if (g_control_mode == CONTROL_SPD)
			{
				//----------------------------------------------------------------
				//	COMPUTE SPEED
				//----------------------------------------------------------------
				//	Fetch encoder reading
				//	Compute derivative
				//	Update old encoder memory registers
				
				//temp flag
				bool f_ret;
				//temp speed
				int16_t enc_spd[ ENC_NUM ];
				//Compute speed
				f_ret = compute_speed( enc_spd );
				//if: failed to update
				if (f_ret == true)
				{
					//Signal error
					report_error( ERR_CODE_BAD_ENCODER_COUNTERS );
				}
				//if: update was success
				else
				{
					//counter
					uint8_t t = 0;
					//command
					int16_t cmd;
					//motor target pwm
					Dc_motor_pwm pwm;
					//Scan all PID
					for (t=0;t < ENC_NUM;t++)
					{
						//Process the speed and get the command
						cmd = vnh7040_pid[t].exe( g_pid_spd_target[t], enc_spd[t] );
						//Convert from S16 to PWM. Sign correction should not be applied here because it would change the sign of the feedback loop
						pwm = convert_s16_to_pwm( cmd, false );
						//Use the command as reference for the PWM
						g_dc_motor_target[t] = pwm;
					}
				}	//end if: update was success
				
				//Update PWM of the motors while applying the slew rate limiter
				update_pwm();
			}	//End If: control system is closed loop speed
			//If: control system is closed loop speed but with closing PID in position
			else if (g_control_mode == CONTROL_SPD_POS)
			{
				//----------------------------------------------------------------
				//	GET CURRENT POSITION
				//----------------------------------------------------------------
				
				//temp flag
				bool f_ret;
				//temp encoder counters. G-enc_cnt are volatile globals shared by ISR
				int32_t enc_cnt[ENC_NUM];
				//Force update of encoder counters and fetch their value
				f_ret = get_enc_cnt( enc_cnt );
				//if: update was success
				if (f_ret == false)
				{
					//Counter
					uint8_t t;
					int32_t enc_target;
					//temp errors
					int32_t err32;
					int16_t err16;
					//command
					int16_t cmd;
					//motor target pwm
					Dc_motor_pwm pwm;
					
					//----------------------------------------------------------------
					//	GENERATE POS REFERENCE | COMPUTE POSITION PID | GENERATE PWM REFERENCE
					//----------------------------------------------------------------
					//	This is an hybrid mode. User tell the speed, and the speed is used to update the position reference
					
					//Scan all encoders
					for (t=0;t < ENC_NUM;t++)
					{
						enc_target = g_pid_pos_target[t];
						//At each tick, integrate the user given speed to compute the target position
						enc_target += g_pid_spd_target[t];
						g_pid_pos_target[t] = enc_target;
						//Compute position error
						err32 = (int32_t)enc_target -(int32_t)enc_cnt[t];
						//Clip to 16b for use in the PID controller
						err16 = AT_SAT( err32, (int16_t)32767, (int16_t)-32767);
						//Compute PID and command feeding it directly the error
						cmd = vnh7040_pid[t].exe( err16 );
						//Convert from S16 to PWM. Sign correction should not be applied here because it would change the sign of the feedback loop
						pwm = convert_s16_to_pwm( cmd, false );
						//Use the command as reference for the PWM
						g_dc_motor_target[t] = pwm;
					}
				}
				//if: update failed
				else
				{
					//Signal error
					report_error( ERR_CODE_BAD_ENCODER_COUNTERS );
				}
				
				//Update PWM of the motors while applying the slew rate limiter
				update_pwm();
			}
			//if: undefined control system
			else
			{
				//Signal error
				report_error( ERR_CODE_UNDEFINED_CONTROL_SYSTEM );
				//Reset control mode
				g_control_mode = CONTROL_STOP;
			}	//End if: undefined control system
			
		}	//End If: Authorized to execute one step the PID speed controllers
		
		//----------------------------------------------------------------
		//	AT4809 --> RPI USART TX
		//----------------------------------------------------------------
		
		//if: RPI TX buffer is not empty and the RPI TX HW buffer is ready to transmit
		if ( (AT_BUF_NUMELEM( rpi_tx_buf ) > 0) && (IS_BIT_ONE(USART3.STATUS, USART_DREIF_bp)))
		{
			//temp var
			uint8_t tx_tmp;
			//Get the byte to be filtered out
			tx_tmp = AT_BUF_PEEK( rpi_tx_buf );
			AT_BUF_KICK( rpi_tx_buf );
			//Send data through the UART3
			USART3.TXDATAL = tx_tmp;
		}	//End If: RPI TX
		
		//----------------------------------------------------------------
		//	RPI --> AT4809 USART RX
		//----------------------------------------------------------------
		
		//if: RX buffer is not empty	
		if (AT_BUF_NUMELEM( rpi_rx_buf ) > 0)
		{
			//temp var
			uint8_t rx_tmp;
				
				///Get data
			//Get the byte from the RX buffer (ISR put it there)
			rx_tmp = AT_BUF_PEEK( rpi_rx_buf );
			AT_BUF_KICK_SAFER( rpi_rx_buf );

				///Loop back
			//Push into tx buffer
			//AT_BUF_PUSH( rpi_tx_buf, rx_tmp );

				///Command parser
			//feed the input RX byte to the parser
			rpi_rx_parser.exe( rx_tmp );
			
		} //endif: RPI RX buffer is not empty

	}	//End: Main loop

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return 0;
}	//end: main

/***************************************************************************/
//!	@brief function
//!	error | Error_code
/***************************************************************************/
//! @param err_code | (Error_code) caller report the error code experienced by the system
//! @return void |
//! @details
//!	Error code handler function
/***************************************************************************/

void report_error( Error_code err_code )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counter
	uint8_t t;
	//length of number string
	uint8_t ret_len;
	//numeric code
	uint8_t msg[MAX_DIGIT8+2];

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//error code
	uint8_t u8_err_code = (uint8_t)err_code;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Construct numeric string
	ret_len = u8_to_str( u8_err_code, msg );
	//Error command
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'R' );
	AT_BUF_PUSH( rpi_tx_buf, 'R' );
	//Send numeric string
	for (t = 0;t < ret_len;t++)
	{
		//Send number
		AT_BUF_PUSH( rpi_tx_buf, msg[t] );
	}
	//Send terminator
	AT_BUF_PUSH( rpi_tx_buf, '\0' );
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End Function: error | Error_code

/****************************************************************************
**  Function
**  init_motors
****************************************************************************/
//! @return void |
//! @brief Initialize motors
//! @details Initialize motors
//!
/***************************************************************************/

void init_motors( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counter
	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For: scan motors
	for (t = 0;t < DC_MOTOR_NUM;t++)
	{
		//Initialize motor
		g_dc_motor[t].f_dir = false;
		g_dc_motor[t].pwm = (uint8_t)0x00;
	}	//End For: scan motors
	
	//For: scan motors
	for (t = 0;t < DC_MOTOR_NUM;t++)
	{
		//Initialize motor
		g_dc_motor_target[t].f_dir = false;
		g_dc_motor_target[t].pwm = (uint8_t)0x00;
	}
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End: init_motors

/***************************************************************************/
//!	function
//!	function_template
/***************************************************************************/
//! @param x |
//! @return void |
//! @brief	Initialize all PID controllers
//! @details
/***************************************************************************/

bool init_pid( OrangeBot::Pid_s16 *pid_vector )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Scan all PID
	for (t = 0;t<ENC_NUM;t++)
	{
		//Register PID saturation handler
		pid_vector[t].register_error_handler( POS_PID_SAT_TH, (void *)&pid_saturation_error_handler );
		//Initialize PID limits
		pid_vector[t].limit_cmd_max() = +DC_MOTOR_MAX_PWM;
		pid_vector[t].limit_cmd_min() = -DC_MOTOR_MAX_PWM;
		//Initialize PID Gain
		pid_vector[t].gain_kp() = SPD_PID_KP;
		pid_vector[t].gain_ki() = SPD_PID_KI;
		pid_vector[t].gain_kd() = SPD_PID_KD;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}

/****************************************************************************
**  Function
**  set_vnh7040_speed
****************************************************************************/
//! @return void |
//! @param index	| index of the motor to be controlled. 0 to 3
//! @param f_dir	| direction of rotation of the motor
//! @param speed	| Speed of the motor
//! @brief Set direction and speed setting of the VNH7040 controlled motor
//! @details 
//!
/***************************************************************************/

void set_vnh7040_speed( uint8_t index, bool f_dir, uint8_t speed )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Driver 0
	if (index == 0)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTA.OUT, 0x30, 0x10);
		}
		else
		{
			SET_MASKED_BIT( PORTA.OUT, 0x30, 0x20);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB0.CCMPH = speed;	
	}
	//Driver 1
	else if (index == 1)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTA.OUT, 0xc0, 0x40);
		}
		else
		{
			SET_MASKED_BIT( PORTA.OUT, 0xc0, 0x80);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB1.CCMPH = speed;
	}
	//Driver 2
	else if (index == 2)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTB.OUT, 0x0c, 0x04);
		}
		else
		{
			SET_MASKED_BIT( PORTB.OUT, 0x0c, 0x08);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB2.CCMPH = speed;
	}
	//Driver 3
	else if (index == 3)
	{
		//Set INA, INB and SEL0 to select the direction of rotation of the motor
		if (f_dir == true)
		{
			SET_MASKED_BIT( PORTD.OUT, 0xc0, 0x40);
		}
		else
		{
			SET_MASKED_BIT( PORTD.OUT, 0xc0, 0x80);
		}
		//Set the PWM value of the right PWM channel of TCA0
		TCB3.CCMPH = speed;
	}
	//Default case
	else
	{
		//Driver index not installed.
		//Do nothing
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End: 

/***************************************************************************/
//!	@brief convert from a DC motor PWM structure to a speed number
//!	convert_pwm_to_s16 | Dc_motor_pwm
/***************************************************************************/
//! @param input | Dc_motor_pwm	input PWM structure
//! @param f_dir | bool			conversion between clockwise/counterclockwise to forward/backward
//! @return int16_t |			speed number
/***************************************************************************/

inline int16_t convert_pwm_to_s16( Dc_motor_pwm input, bool f_dir )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp return
	int16_t ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Extract PWM setting
	ret = input.pwm;
	//if: direction is backward
	if (input.f_dir != f_dir)
	{
		//Correct sign
		ret = -ret;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return ret;
}

/***************************************************************************/
//!	@brief convert from a speed number to a dc motor PWM structure
//!	convert_s16_to_pwm | int16_t, bool
/***************************************************************************/
//! @param input | int16_t 		Input speed number
//! @param f_dir | bool			conversion between clockwise/counterclockwise to forward/backward
//! @return Dc_motor_pwm		PWM structure
/***************************************************************************/

Dc_motor_pwm convert_s16_to_pwm( int16_t input, bool f_dir )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp return
	Dc_motor_pwm ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Clip to valid PWM values
	input = AT_SAT( input, DC_MOTOR_MAX_PWM, -DC_MOTOR_MAX_PWM);

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For each DC motor channel
	//If: right direction is backward
	if (input < 0)
	{
		//Assign the pwm to the right temp channel
		ret.pwm = -input;
		ret.f_dir = !f_dir;
	}
	//If: right direction is forward
	else
	{
		//Assign the pwm to the right temp channel
		ret.pwm = input;
		ret.f_dir = f_dir;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return ret;
}

/***************************************************************************/
//!	@brief update DC motor PWM
//!	update_pwm | void
/***************************************************************************/
//! @return void
//!	@details
//! Move PWM toward target PWM
//! Apply slew rate limiter
/***************************************************************************/

void update_pwm( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp counter
	uint8_t t;
	//true if speed has changed
	//bool f_change[DC_MOTOR_NUM];

	Dc_motor_pwm target_speed, actual_speed;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//If the communication failed
	if (g_f_timeout_detected == true)
	{
		//Counter
		uint8_t t;
		//For: scan motors
		for (t = 0;t < DC_MOTOR_NUM;t++)
		{
			//Stop the motors
			set_vnh7040_speed( (uint8_t)t, (uint8_t)false, (uint8_t)0x00 );
			//Update actual PWM so tat slew rate limiter will do sensible things when restarting
			g_dc_motor[t].pwm = (uint8_t)0x00;
		}
		//
		return;
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For: each DC motor channel
	for (t = 0;t < DC_MOTOR_NUM;t++)
	{
		//Fetch current settings
		target_speed = g_dc_motor_target[t];
		actual_speed = g_dc_motor[t];
		
		//If directions are different
		if (target_speed.f_dir != actual_speed.f_dir)
		{
			//if pwm is above slew rate
			if (actual_speed.pwm > DC_MOTOR_SLEW_RATE)
			{
				//Slow down by the slew rate
				actual_speed.pwm -= DC_MOTOR_SLEW_RATE;
			}
			//if pwm is below or at slew rate
			else
			{
				//Set neutral speed
				actual_speed.pwm = 0;
			}
			//if pwm is zero
			if (actual_speed.pwm == 0)
			{
				//I'm authorized to change direction
				actual_speed.f_dir = target_speed.f_dir;
			}
		}	//End If directions are different
		//if direction is the same
		else
		{
			//if pwm is above target
			if (actual_speed.pwm > target_speed.pwm)
			{
				//Decrease speed by PWM
				actual_speed.pwm = AT_SAT_SUM( actual_speed.pwm, -DC_MOTOR_SLEW_RATE, DC_MOTOR_MAX_PWM, 0 );
				//if: overshoot
				if (actual_speed.pwm < target_speed.pwm)
				{
					//I reached the target
					actual_speed.pwm = target_speed.pwm;
				}
			}
			//if pwm is below target
			else if (actual_speed.pwm < target_speed.pwm)
			{
				//Decrease speed by PWM
				actual_speed.pwm = AT_SAT_SUM( actual_speed.pwm, +DC_MOTOR_SLEW_RATE, DC_MOTOR_MAX_PWM, 0 );
				//if: overshoot
				if (actual_speed.pwm > target_speed.pwm)
				{
					//I reached the target
					actual_speed.pwm = target_speed.pwm;
				}
			}
			//if: I'm already at the right speed
			else
			{
				//do nothing
			}
		}
		
		//Apply setting
		set_vnh7040_speed( t, actual_speed.f_dir, actual_speed.pwm );
		
		//Write back setting
		g_dc_motor[t] = actual_speed;
	}	//End For: each DC motor channel

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: update_pwm | void

/***************************************************************************/
//!	function
//!	get_enc_cnt
/***************************************************************************/
//! @param enc_cnt | int32_t vector. Function returns in this vector the value of the global encoder counters
//! @return bool | false=OK | true=failed to update global counters
//! @brief Force an update and save the 32b encoder counters in an input vector
//! @details
//!		Algorithm:
//!	>disable interrupt
//!	>raise sync flag
//!	>manually execute encoder decoding routine
//! >Transfer register value to local vars
/***************************************************************************/

bool get_enc_cnt( int32_t *enc_cnt )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;
	//return flag
	bool f_ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Disable interrupts
	cli();
	//Force update of 32b global counters
	g_isr_flags.enc_updt = true;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Decode encoders and update 32b counters 
	quad_encoder_decoder( PORTC.IN );
	//Enable interrupts
	sei();
	//If encoder routine failed to update global counters
	f_ret = g_isr_flags.enc_updt;
	//For: all encoder channels
	for (t = 0;t < ENC_NUM;t++)
	{
		//Copy
		enc_cnt[t] = g_enc_cnt[t];
	}
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	//false = OK | true = fail (global counters failed to update)
	return f_ret;
}

/***************************************************************************/
//!	function
//!	function_template
/***************************************************************************/
//! @param enc_speed | (int16_t*) writeback vector that will hold the result
//! @return bool | false = success | true = fail
//! @brief Compute speed. Unit of measure is Count/Tick.
//! @details
/***************************************************************************/

bool compute_speed( int16_t *enc_speed )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp counter
	uint8_t t;
	//Temp return flag
	bool f_ret;
	//Local encoder counters
	int32_t enc_cnt[ ENC_NUM ];

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Force update of the global encoder counter registers and fetch value of the registers
	f_ret = get_enc_cnt( enc_cnt );
	//if: fail
	if (f_ret == true)
	{
		AT_BUF_PUSH(rpi_tx_buf, 'E');
		//fail
		return true;
	}
	//Scan encoders
	for (t = 0;t< ENC_NUM;t++)
	{
		//Compute speed saturating to limit of the var
		enc_speed[t] = AT_SAT_SUM( enc_cnt[t], -g_old_enc_cnt[t], (int16_t)32767, (int16_t)-32767 );
		//enc_speed[t] = enc_cnt[t] -g_old_enc_cnt[t];
		//Save memories
		g_old_enc_cnt[t] = enc_cnt[t];
		g_enc_spd[t] = enc_speed[t];
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}

/***************************************************************************/
//!	@brief function
//!	generate_reference
/***************************************************************************/
//! @param top | U8 maximum command
//! @return void |
//! @details
//! Generate double sided reference for all four motors
/***************************************************************************/

void generate_reference( Control_mode mode, uint16_t top )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//index of the motor under test
	static uint8_t motor_index = 0;
	//Speed ramp. false = accelerate | true = decelerate
	static bool f_ramp = false;
	//Direction. false =  clockwise | true = counterclockwise
	static bool f_dir = false;
	//Speed counter
	static uint16_t speed = 0;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
		
		//! Ramp generator
	//If increase speed
	if (f_ramp == false)
	{
		//increase speed
		speed++;
		//if: speed cap
		if (speed >= top)
		{
			//now decrease speed
			f_ramp = true;
		}
	}
	//If decrease speed
	else
	{
		//decrease speed
		speed--;
		//If: speed cap
		if (speed <= (uint8_t)0x00)
		{
			//Now increase speed
			f_ramp = false;
			//Invert motor direction
			f_dir = !f_dir;
			//If I'm going forward again
			if (f_dir == false)
			{
				//change motor under test
				motor_index = AT_TOP_INC( motor_index, 3);
			}
		}
	}
		//! Command generator
	//if: open loop PWM test ramp
	if (mode == CONTROL_PWM)
	{
		//Assign command
		g_dc_motor_target[ motor_index ].f_dir = f_dir;
		g_dc_motor_target[ motor_index ].pwm = speed;
		//Reset communication timeout
		g_uart_timeout_cnt = 0;
		//Select Open loop PWM control system
		g_control_mode_target = mode;
	}
	//if: speed PID control mode
	else if (mode == CONTROL_SPD)
	{
		//temp command
		int16_t command;
		
		if (f_dir == false)
		{
			command = speed;
		}
		else
		{
			command = -speed;
		}
		
		//Assign command
		g_pid_spd_target[ motor_index ] = command;
		//Reset communication timeout
		g_uart_timeout_cnt = 0;
		//Select control mode
		g_control_mode_target = mode;
		
		uint8_t t, len, msg[10];
		
		AT_BUF_PUSH( rpi_tx_buf, 'M' );
		len = u8_to_str( motor_index, msg );
		for (t = 0;t<len;t++)
		{
			AT_BUF_PUSH(rpi_tx_buf,msg[t]);
		}
		AT_BUF_PUSH( rpi_tx_buf, 'S' );
		AT_BUF_PUSH( rpi_tx_buf, 'P' );
		AT_BUF_PUSH( rpi_tx_buf, 'D' );
		len = s16_to_str( g_pid_spd_target[ motor_index ], msg );
		for (t = 0;t<len;t++)
		{
			AT_BUF_PUSH(rpi_tx_buf,msg[t]);
		}
		AT_BUF_PUSH( rpi_tx_buf, '\0' );
		
	}
	//if: bad reference control mode
	else
	{
		//
		g_control_mode_target = CONTROL_STOP;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: generate_reference

/***************************************************************************/
//!	@brief handler
//!	pid_saturation_error_handler
/***************************************************************************/
//! @return void |
//! @details
//! This function is automatically called by a PID when it cannot lock quickly enough to the reference
/***************************************************************************/

void pid_saturation_error_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//A PID wasn't able to lock quickly enough to the reference
	g_isr_flags.pid_sat_err = true;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: pid_saturation_error_handler

/***************************************************************************/
//!	@brief function
//!	function_template
/***************************************************************************/
//! @param x |
//! @return void |
//! @details
/***************************************************************************/

void function_template( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: 


