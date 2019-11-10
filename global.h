#ifndef GLOBAL_H
	//header environment variable, is used to detect multiple inclusion
	//of the same header, and can be used in the c file to detect the
	//included library
	#define GLOBAL_H

	/****************************************************************************
	**	ENVROIMENT VARIABILE
	****************************************************************************/

	#define F_CPU 20000000

	/****************************************************************************
	**	GLOBAL INCLUDE
	**	TIPS: you can put here the library common to all source file
	****************************************************************************/

	//type definition using the bit width and signedness
	#include <stdint.h>
	//define the ISR routune, ISR vector, and the sei() cli() function
	#include <avr/interrupt.h>
	//name all the register and bit
	#include <avr/io.h>
	//hard delay
	#include <util/delay.h>
	//General purpose macros
	#include "at_utils.h"
	//AT4809 PORT macros definitions
	#include "at4809_port.h"

	/****************************************************************************
	**	DEFINE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------

	#define RPI_RX_BUF_SIZE		16
	#define RPI_TX_BUF_SIZE		64
	
		///----------------------------------------------------------------------
		///	PARSER
		///----------------------------------------------------------------------
			
	#define RPI_COM_TIMEOUT		200
	
		///----------------------------------------------------------------------
		///	MOTORS
		///----------------------------------------------------------------------
	
	//Number of DC motors mounted on the platform
	#define DC_MOTOR_NUM		4
	//Maximum slew rate. PWM increment per tick
	#define DC_MOTOR_SLEW_RATE	1
	//Maximum PWM setting
	#define DC_MOTOR_MAX_PWM	255
	
		///----------------------------------------------------------------------
		///	ENCODERS
		///----------------------------------------------------------------------
	
	//Number of quadrature encoders
	#define ENC_NUM				4
	//Threshold upon which local counters are synced with global counters
	#define ENC_UPDATE_TH		100
	//1<<ENC_GAIN Gain of the encoder count.
	#define ENC_GAIN			4
	
		///----------------------------------------------------------------------
		///	PID
		///----------------------------------------------------------------------
		//	Define default gains of the speed PID
		//	Number are fixed point default position 8. means 32767=127.996 | 256=1.000 | 1=0.0039
		
	#define SPD_PID_KP			16
	#define SPD_PID_KI			0
	#define SPD_PID_KD			0
	
	//The PID is allowed this many tick with command saturated before going into emergency
	#define POS_PID_SAT_TH		200
	
	/****************************************************************************
	**	ENUM
	****************************************************************************/
		
	//Control modes
	typedef enum _Control_mode
	{
		CONTROL_STOP	= 0,	//STOP mode. PWM references are zero
		CONTROL_PWM		= 1,	//PWM mode. Feed open loop PWM signals directly
		CONTROL_POS		= 2,	//POS mode. Feed wheel positions directly
		CONTROL_SPD		= 3,	//SPD mode. Feed wheel speed directly
		CONTROL_SPD_POS	= 4		//Hybrid Speed mode. User feed speed reference but PID is closed in position
		
	} Control_mode;

	//Error codes that can be experienced by the program
	typedef enum _Error_code
	{
		ERR_CODE_UNDEFINED_CONTROL_SYSTEM,
		ERR_CODE_BAD_ENCODER_COUNTERS,
		ERR_CODE_COMMUNICATION_TIMEOUT,
		ERR_CODE_PID_UNLOCKED				//A PID has been unable to get a lock within the given number of ticks
	} Error_code;

	/****************************************************************************
	**	MACRO
	****************************************************************************/

		///----------------------------------------------------------------------
		///	LEDS
		///----------------------------------------------------------------------

	#define LED0_TOGGLE()	\
		TOGGLE_BIT( PORTB, PB6 )

	/****************************************************************************
	**	TYPEDEF
	****************************************************************************/

	//Global flags raised by ISR functions
	typedef struct _Isr_flags Isr_flags;
	
	//PWM and direction of a DC motor
	typedef struct _Dc_motor_pwm Dc_motor_pwm;

	/****************************************************************************
	**	STRUCTURE
	****************************************************************************/

	//Global flags raised by ISR functions
	struct _Isr_flags
	{
		//First byte
		U8 system_tick		: 1;	//System Tick
		U8 enc_double_event	: 1;	//true = At least one encoder double event detected
		U8 enc_sem			: 1;	//true = Encoder ISR is forbidden to write into the 32b encoder counters
		U8 enc_updt			: 1;	//true = Encoder ISR is forced to update the 32b counters and clear this flag if possible.
		U8 ctrl_updt		: 1;	//true = Execute the motor control system
		U8 pid_sat_err		: 1;	//true = A PID wasn't able to lock quickly enough to the reference
		U8 					: 2;	//unused bits
	};

	//PWM and direction of a DC motor
	struct _Dc_motor_pwm
	{
		uint8_t pwm;			//DC Motor PWM setting. 0x00 = stop | 0xff = maximum
		uint8_t f_dir;			//DC Motor direction. false=clockwise | true=counterclockwise
	};

	/****************************************************************************
	**	PROTOTYPE: INITIALISATION
	****************************************************************************/

	//port configuration and call the peripherals initialization
	extern void init( void );

	/****************************************************************************
	**	PROTOTYPE: FUNCTION
	****************************************************************************/
	
	//Error code handler function
	extern void report_error( Error_code err_code );
	
		///----------------------------------------------------------------------
		///	PARSER
		///----------------------------------------------------------------------
		//	Handlers are meant to be called automatically when a command is decoded
		//	User should not call them directly

	//Handler for the ping command. Keep alive connection
	extern void ping_handler( void );
	//Handler for the get board signature command. Send board signature via UART
	extern void signature_handler( void );
	//Handler for the motor speed set command
	extern void set_speed_handler( int16_t motor_index, int16_t pwm );
	//Handler for the platform speed. Firmware handles logical configuration of the motors. TODO: evolve to forward and turn
	extern void set_platform_speed_handler(int16_t right, int16_t left );
	//Handler for closed loop speed
	extern void set_pid_speed_handler( int16_t motor_index, int16_t spd );
	//Handler for the platform speed using the integrated speed controller. Firmware handles logical configuration of the motors.
	extern void set_platform_pid_speed_handler(int16_t right, int16_t left );
	//Handler for the get encoder count message
	extern void get_encoder_cnt_handler( void );
	//Handler for the get encoder speed message
	extern void get_encoder_spd_handler( void );
		
		///----------------------------------------------------------------------
		///	MOTORS
		///----------------------------------------------------------------------
		
	//Set direction and speed setting of the VNH7040 controlled motor
	extern void set_vnh7040_speed( uint8_t index, bool f_dir, uint8_t speed );
	
		///----------------------------------------------------------------------
		///	ENCODERS
		///----------------------------------------------------------------------
	
	//Decode four quadrature encoder channels
	extern void quad_encoder_decoder( uint8_t enc_in );
	//Force an update and save the 32b encoder counters in an input vector
	extern bool get_enc_cnt( int32_t *enc_cnt );
	
		///----------------------------------------------------------------------
		///	CONTROL
		///----------------------------------------------------------------------
		
	//This function is automatically called by a PID when it cannot lock quickly enough to the reference
	extern void pid_saturation_error_handler( void );
	
	/****************************************************************************
	**	PROTOTYPE: GLOBAL VARIABILE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	STATUS FLAGS
		///----------------------------------------------------------------------

	//Volatile flags used by ISRs
	extern volatile	Isr_flags g_isr_flags;
	
		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------
		//	Buffers structure and data vectors

	//Safe circular buffer for UART input data
	extern volatile At_buf8_safe rpi_rx_buf;
	//Safe circular buffer for uart tx data
	extern At_buf8 rpi_tx_buf;
	//allocate the working vector for the buffer
	extern uint8_t v0[ RPI_RX_BUF_SIZE ];
	//allocate the working vector for the buffer
	extern uint8_t v1[ RPI_TX_BUF_SIZE ];
	
		///--------------------------------------------------------------------------
		///	PARSER
		///--------------------------------------------------------------------------

	//Board Signature
	extern U8 *board_sign;
	//communication timeout counter
	extern U8 g_uart_timeout_cnt;
	//Communication timeout has been detected
	extern bool g_f_timeout_detected;
	
		///--------------------------------------------------------------------------
		///	CONTROL
		///--------------------------------------------------------------------------
	
	//Motor control mode in use
	extern Control_mode g_control_mode;
	//Target motor control mode
	extern Control_mode g_control_mode_target;
	
		///--------------------------------------------------------------------------
		///	MOTORS
		///--------------------------------------------------------------------------

	//Desired setting for the DC motor channels
	extern Dc_motor_pwm g_dc_motor_target[DC_MOTOR_NUM];
	//Two DC Motor channels current setting
	extern Dc_motor_pwm g_dc_motor[DC_MOTOR_NUM];
	
		///--------------------------------------------------------------------------
		///	ENCODERS
		///--------------------------------------------------------------------------

	//Global 32b encoder counters
	extern volatile int32_t g_enc_cnt[ENC_NUM];
	//Previous encoder reading
	extern int32_t g_old_enc_cnt[ENC_NUM];
	//Encoder speed
	extern int16_t g_enc_spd[ENC_NUM];
	//Encoder speed reference
	extern int16_t g_pid_spd_target[ENC_NUM];
	
#else
	#warning "multiple inclusion of the header file global.h"
#endif


