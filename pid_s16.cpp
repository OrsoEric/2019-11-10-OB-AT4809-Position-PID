/****************************************************************************
**	OrangeBot Project
*****************************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************************
**
*****************************************************************************
**	Author: 			Orso Eric
**	Creation Date:
**	Last Edit Date:
**	Revision:			1
**	Version:			0.1 ALFA
**	Compiler flags
**	-fkeep-inline-functions
****************************************************************************/

/****************************************************************************
**	HYSTORY VERSION
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	DESCRIPTION
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	KNOWN BUG
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	INCLUDES
****************************************************************************/

#include "global.h"
//Class Header
#include "Pid_s16.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

namespace OrangeBot
{

/****************************************************************************
**	GLOBAL VARIABILES
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	CONSTRUCTORS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Empty Constructor
//!	Pid_s16 | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Empty constructor
/***************************************************************************/

Pid_s16::Pid_s16( void )
{
	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Clear error handler
	this -> g_err_handler = nullptr;

	//Initialize PID memories
	this -> g_acc		= (int16_t)0;
	this -> g_old_err	= (int16_t)0;
	this -> g_sat_cnt	= (int16_t)0;

	//Initialize PID parameters
	this -> g_cmd_max	= (int16_t)32767;
	this -> g_cmd_min	= (int16_t)-32767;
	this -> g_sat_th	= (int16_t)0;

	//Initialize PID gains
	this -> g_kp		= (int16_t)0;
	this -> g_kd		= (int16_t)0;
	this -> g_ki		= (int16_t)0;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();

	return;	//OK
}	//end constructor:

/****************************************************************************
*****************************************************************************
**	DESTRUCTORS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Empty Destructor
//!	Pid_s16 | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Empty destructor
/***************************************************************************/

Pid_s16::~Pid_s16( void )
{
	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();

	return;	//OK
}	//end destructor:

/****************************************************************************
*****************************************************************************
**	OPERATORS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	SETTERS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Method
//!	register_error_handler | uint16_t, void *
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Set command saturation error and error handler
/***************************************************************************/

bool Pid_s16::register_error_handler( uint16_t sat_th, void *handler )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Save error threshold
	this -> g_sat_th = sat_th;
	//Register handler function
	this -> g_err_handler = handler;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();

	return false;	//OK
}	//end method: register_error_handler | uint16_t, void *

/****************************************************************************
*****************************************************************************
**	GETTERS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	REFERENCES
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Reference Operator
//!	limit_cmd_max | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

inline int16_t &Pid_s16::limit_cmd_max( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_cmd_max;
}	//end reference: limit_cmd_max | void

/***************************************************************************/
//!	@brief Reference Operator
//!	limit_cmd_min | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

inline int16_t &Pid_s16::limit_cmd_min( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_cmd_min;
}	//end reference: limit_cmd_min | void

/***************************************************************************/
//!	@brief Reference Operator
//!	limit_sat_th | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

inline uint16_t &Pid_s16::limit_sat_th( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_sat_th;
}	//end reference: limit_sat_th | void

/***************************************************************************/
//!	@brief Reference Operator
//!	gain_kp | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

inline int16_t &Pid_s16::gain_kp( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_kp;
}	//end reference: gain_kp | void

/***************************************************************************/
//!	@brief Reference Operator
//!	gain_kd | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

inline int16_t &Pid_s16::gain_kd( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_kd;
}	//end reference: gain_kd | void

/***************************************************************************/
//!	@brief Reference Operator
//!	gain_ki | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

inline int16_t &Pid_s16::gain_ki( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_ki;
}	//end reference: gain_ki | void

/****************************************************************************
*****************************************************************************
**	TESTERS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	PUBLIC METHODS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Method
//!	exe | int16_t, int16_t
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Execute a step of the PID controller
/***************************************************************************/

int16_t Pid_s16::exe( int16_t reference, int16_t feedback )
{
	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//error
	int16_t err;
	//derivative of the error
	int16_t err_d;
	//integral of error
	int16_t err_i;
	//partial results
	int32_t cmd_tmp;
	int16_t cmd_p, cmd_d, cmd_i;
	//command
	int16_t cmd;
	//Detect saturation of command
	bool f_sat;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Trace Enter
	DENTER_ARG("reference: %d, feedback: %d\n", reference, feedback);

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------
	//		ALGORITHM:
	//	Compute error


		///--------------------------------------------------------------------------
		///	COMPUTE ERROR
		///--------------------------------------------------------------------------

	//Error between reference and feedback
	err = AT_SAT_SUM( +reference, -feedback, (int16_t)32767, (int16_t)-32767 );

		///--------------------------------------------------------------------------
		///	COMPUTE PROPORTIONAL
		///--------------------------------------------------------------------------

	//Proportional contribution of the command. Consider gain as a fixed point number with a defined fixed point position
	cmd_tmp = err *this->g_kp;
	//Apply FP correction with a rounder toward odd to improve resolution
	cmd_tmp = AT_DIVIDE_RTO( cmd_tmp, PID_GAIN_FP);
	//Clip and save the command
	cmd_p = AT_SAT( cmd_tmp, (int16_t)32767, (int16_t)-32767);

		///--------------------------------------------------------------------------
		///	COMPUTE DERIVATIVE
		///--------------------------------------------------------------------------
		//This implementation use a simple two point derivative. I have a vastly superior four point derivative that minimize error to natural system response.

	//Compute error derivative.
	err_d = AT_SAT_3SUM( +reference, -feedback, -this->g_old_err, (int16_t)32767, (int16_t)-32767 );
	//Derivative contribution of the command. Consider gain as a fixed point number with a defined fixed point position
	cmd_tmp = err_d *this->g_kd;
	//Apply FP correction with a rounder toward odd to improve resolution
	cmd_tmp = AT_DIVIDE_RTO( cmd_tmp, PID_GAIN_FP);
	//Clip and save the command
	cmd_d = AT_SAT( cmd_tmp, (int16_t)32767, (int16_t)-32767);

		///--------------------------------------------------------------------------
		///	COMPUTE INTEGRATIVE
		///--------------------------------------------------------------------------

	//Compute integral of error
	err_i = AT_SAT_3SUM( +reference, -feedback, +this->g_acc, (int16_t)32767, (int16_t)-32767 );
	//Derivative contribution of the command. Consider gain as a fixed point number with a defined fixed point position
	cmd_tmp = err_i *this->g_ki;
	//Apply FP correction with a rounder toward odd to improve resolution
	cmd_tmp = AT_DIVIDE_RTO( cmd_tmp, PID_GAIN_FP);
	//Clip and save the command
	cmd_i = AT_SAT( cmd_tmp, (int16_t)32767, (int16_t)-32767);

		///--------------------------------------------------------------------------
		///	COMPUTE COMMAND
		///--------------------------------------------------------------------------

	DPRINT("proportional: %d | derivative: %d, integrative: %d\n", cmd_p, cmd_d, cmd_i );
	//Compute the command
	cmd = AT_SAT_3SUM( +cmd_p, +cmd_d, +cmd_i, this->g_cmd_max, this->g_cmd_min );
	//Detect command saturation
	f_sat = ((cmd == this -> g_cmd_max) || (cmd == this -> g_cmd_min));

		///--------------------------------------------------------------------------
		///	SATURATION DETECTION
		///--------------------------------------------------------------------------
		//	This feature is meant to detect when command is saturated for too long.
		//	This means that the PID is unable to keep up with the system and the PID is unlocked
		//	Es. A motor as encoder connected in reverse. The PID can never bring error to zero, so cut off to prevent damage.
		//	Es. Achieve a given speed against too great a resistive force. Command cannot be achieved with this level of power and PID cut off.

	//If feature is disabled
	if (this -> g_sat_th == 0)
	{
		//Saturation is disabled
		f_sat = false;
	}
	//If saturation detected
	else if (f_sat == true)
	{
		//Increment counter
		this -> g_sat_cnt++;
		//Detect error condition
		if (this -> g_sat_cnt > this -> g_sat_th)
		{
				//!PID UNLOCK DETECTED!!!
			//Create a pointer to function with no arguments and no return
			void (*my_function_ptr)(void);
			//promote the pointer to the right kind
			my_function_ptr = (void(*)(void))this -> g_err_handler;
			//Execute handler
			(*my_function_ptr)();
		}
	}
	//If saturation not detected
	else
	{
		//Clear the counter. Short saturations are permitted as the PID can still somehow keep up
		this -> g_sat_cnt = 0;
	}

		///--------------------------------------------------------------------------
		///	UPDATE REGISTERS
		///--------------------------------------------------------------------------

	//Update derivative register
	this -> g_old_err = err;
	//Authorize update of integrative register only if update would push a reduction of the command from saturation
	if ((f_sat == false) || (AT_ABS(err_i) < AT_ABS(this -> g_acc)))
	{
		//Update accumulator register
		this -> g_acc = err_i;
	}
	else
	{
		DPRINT("Saturation detected. Update of accumulator not authorized\n");
	}

	DPRINT("old: %5d, acc: %5d\n", this -> g_old_err, this -> g_acc);

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN_ARG("command: %d\n", cmd);

	return cmd;	//OK
}	//end method: exe | int16_t, int16_t

/***************************************************************************/
//!	@brief Public Method
//!	exe | int16_t
/***************************************************************************/
//! @param error (int16_t) error signal
//! @return no return
//!	@details
//! Execute a step of the PID controller
//! Give directly error. This is in case user does not have S16 reference and feedback
//! but for example S32 and S32 whose difference is still S16 bound
/***************************************************************************/

int16_t Pid_s16::exe( int16_t err )
{
	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//derivative of the error
	int16_t err_d;
	//integral of error
	int16_t err_i;
	//partial results
	int32_t cmd_tmp;
	int16_t cmd_p, cmd_d, cmd_i;
	//command
	int16_t cmd;
	//Detect saturation of command
	bool f_sat;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Trace Enter
	DENTER_ARG("error: %d\n", err);

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------
	//		ALGORITHM:
	//	Compute error

	///--------------------------------------------------------------------------
	///	COMPUTE PROPORTIONAL
	///--------------------------------------------------------------------------

	//Proportional contribution of the command. Consider gain as a fixed point number with a defined fixed point position
	cmd_tmp = err *this->g_kp;
	//Apply FP correction with a rounder toward odd to improve resolution
	cmd_tmp = AT_DIVIDE_RTO( cmd_tmp, PID_GAIN_FP);
	//Clip and save the command
	cmd_p = AT_SAT( cmd_tmp, (int16_t)32767, (int16_t)-32767);
	//cmd_p = AT_FP_SAT_MUL16( (int16_t)err, (int16_t), PID_GAIN_FP );

	///--------------------------------------------------------------------------
	///	COMPUTE DERIVATIVE
	///--------------------------------------------------------------------------
	//This implementation use a simple two point derivative. I have a vastly superior four point derivative that minimize error to natural system response.

	//Compute error derivative.
	err_d = AT_SAT_SUM( +err, -this->g_old_err, (int16_t)32767, (int16_t)-32767 );
	
	cmd_tmp = err_d *this->g_kd;
	//Apply FP correction with a rounder toward odd to improve resolution
	cmd_tmp = AT_DIVIDE_RTO( cmd_tmp, PID_GAIN_FP);
	//Clip and save the command
	cmd_d = AT_SAT( cmd_tmp, (int16_t)32767, (int16_t)-32767);
	//Derivative contribution of the command. Consider gain as a fixed point number with a defined fixed point position
	//cmd_d = AT_FP_SAT_MUL16( err_d, this->g_kd, PID_GAIN_FP );

	///--------------------------------------------------------------------------
	///	COMPUTE INTEGRATIVE
	///--------------------------------------------------------------------------

	//Compute integral of error
	err_i = AT_SAT_SUM( +err, +this->g_acc, (int16_t)32767, (int16_t)-32767 );
	//Derivative contribution of the command. Consider gain as a fixed point number with a defined fixed point position
	cmd_tmp = err_i *this->g_ki;
	//Apply FP correction with a rounder toward odd to improve resolution
	cmd_tmp = AT_DIVIDE_RTO( cmd_tmp, PID_GAIN_FP);
	//Clip and save the command
	cmd_i = AT_SAT( cmd_tmp, (int16_t)32767, (int16_t)-32767);
	//cmd_i = AT_FP_SAT_MUL16( err_i, this->g_ki, PID_GAIN_FP );

	///--------------------------------------------------------------------------
	///	COMPUTE COMMAND
	///--------------------------------------------------------------------------

	DPRINT("proportional: %d | derivative: %d, integrative: %d\n", cmd_p, cmd_d, cmd_i );
	//Compute the command
	cmd = AT_SAT_3SUM( +cmd_p, +cmd_d, +cmd_i, this->g_cmd_max, this->g_cmd_min );
	//Detect command saturation
	f_sat = ((cmd == this -> g_cmd_max) || (cmd == this -> g_cmd_min));

	///--------------------------------------------------------------------------
	///	SATURATION DETECTION
	///--------------------------------------------------------------------------
	//	This feature is meant to detect when command is saturated for too long.
	//	This means that the PID is unable to keep up with the system and the PID is unlocked
	//	Es. A motor as encoder connected in reverse. The PID can never bring error to zero, so cut off to prevent damage.
	//	Es. Achieve a given speed against too great a resistive force. Command cannot be achieved with this level of power and PID cut off.

	//If feature is disabled
	if (this -> g_sat_th == 0)
	{
		//Saturation is disabled
		f_sat = false;
	}
	//If saturation detected
	else if (f_sat == true)
	{
		//Increment counter
		this -> g_sat_cnt++;
		//Detect error condition
		if (this -> g_sat_cnt > this -> g_sat_th)
		{
			//!PID UNLOCK DETECTED!!!
			//Create a pointer to function with no arguments and no return
			void (*my_function_ptr)(void);
			//promote the pointer to the right kind
			my_function_ptr = (void(*)(void))this -> g_err_handler;
			//Execute handler
			(*my_function_ptr)();
		}
	}
	//If saturation not detected
	else
	{
		//Clear the counter. Short saturations are permitted as the PID can still somehow keep up
		this -> g_sat_cnt = 0;
	}

	///--------------------------------------------------------------------------
	///	UPDATE REGISTERS
	///--------------------------------------------------------------------------

	//Update derivative register
	this -> g_old_err = err;
	//Authorize update of integrative register only if update would push a reduction of the command from saturation
	if ((f_sat == false) || (AT_ABS(err_i) < AT_ABS(this -> g_acc)))
	{
		//Update accumulator register
		this -> g_acc = err_i;
	}
	else
	{
		DPRINT("Saturation detected. Update of accumulator not authorized\n");
	}

	DPRINT("old: %5d, acc: %5d\n", this -> g_old_err, this -> g_acc);

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN_ARG("command: %d\n", cmd);

	return cmd;	//OK
}	//end method: exe | int16_t

/****************************************************************************
*****************************************************************************
**	PUBLIC STATIC METHODS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	PRIVATE METHODS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Method
//!	Dummy | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Method
/***************************************************************************/

bool Pid_s16::dummy( void )
{
	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();

	return false;	//OK
}	//end method:

/****************************************************************************
**	INLINE FIX
****************************************************************************/

//Forces construction of inline methods
class Pid_s16;

/****************************************************************************
**	NAMESPACES
****************************************************************************/

} //End Namespace
