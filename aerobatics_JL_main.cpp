/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file aerobatics_main_JL.cpp
 * Implementation of a controller for fixed wing aerobatics.
 *
 * @author Josh Levin	<joshua.levin@mail.mcgill.ca>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <platforms/px4_defines.h>


#include <vector>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <uORB/topics/aerobatics_variables.h>
#include <uORB/topics/aerobatics_variables2.h>
#include <uORB/topics/debug.h>
#include <uORB/topics/debug_key_value.h>

//Custom (JL)
#include <px4_includes.h>
#include <px4_getopt.h>
#include <px4_log.h>

 //Custom (JL)
 #define PI 3.1415f
 #define rho 1.225f //Air Density (kg/m^3)
 #define g 9.81f //Acceleration due to Gravity (m/s^2)

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aerobatics_JL_main(int argc, char *argv[]);

class Control
{
public:
	/**
	 * Constructor
	 */
	Control();

	/**
	 * Destructor, also kills the main task.
	 */
	~Control();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	//custom
	float dt;
	//Maneuver Generator
	math::Quaternion q_ref;
    math::Vector<3> p_ref;
    math::Vector<3> p_0;
    int maneuver_type;
    bool pos_fb; //Custom(JL)
    bool thr_fb; //Custom(JL)
    bool in_hover; //Custom(JL)
    bool double_gains; //Custom(JL)
    int maneuver_type_old;
    bool maneuver_switch;
    float psi_ref;
    bool transition1;
    bool transition2;
    float start_time;
    //Position Control
	math::Vector<3> e_i_old;
	math::Vector<3> e_i_i;
	float delta_y_d_old;
	float delta_z_d_old;
	math::LowPassFilter2p lp_delta_y_d;
	math::LowPassFilter2p lp_delta_z_d;

	//Thrust Control
	float delta_h_old;
    float delta_h_d_old;
    float delta_h_i;
    float u_ref;
	math::LowPassFilter2p lp_delta_h_d;
	float omega_t_old;

	//Attitude Control
	float Ex_old;
	float Ex_d_old;
	float Ex_i;
	float Ey_old;
	float Ey_d_old;
	float Ey_i;
	float Ez_old;
	float Ez_d_old;
	float Ez_i;
	math::LowPassFilter2p lp_Ex_d;
	math::LowPassFilter2p lp_Ey_d;
	math::LowPassFilter2p lp_Ez_d;
	math::LowPassFilter2p lp_Vs;
	
	//Custom(JL)
	float cont_ff[4];



	int		_control_task;			/**< task handle */


	int		_ctrl_state_sub;	/**< control state subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */

#if 1
    int _counter;
#endif


	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 1 setpoint (Airframe) */
	//custom
	orb_advert_t	_aerobatics_variables_pub;
	orb_advert_t	_aerobatics_variables2_pub;
	orb_advert_t	_debug_pub;

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure
	orb_id_t _attitude_setpoint_id;

	struct control_state_s				_ctrl_state;	/**< control state */
	struct accel_report				_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s			_rates_sp;	/* attitude rates setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct vehicle_land_detected_s			_vehicle_land_detected;	/**< vehicle land detected */
	//custom	
	struct aerobatics_variables_s		_aerobatics_variables;
	struct aerobatics_variables2_s		_aerobatics_variables2;
	struct debug_s 						_debug;


	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	//bool		_debug;				/**< if set to true, print debug output */

	float _flaps_cmd_last;
	float _flaperons_cmd_last;




	struct {
		float p_tc;
		float p_p;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float r_tc;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_ff;
		float y_integrator_max;
		float y_coordinated_min_speed;
		int32_t y_coordinated_method;
		float y_rmax;
		float w_p;
		float w_i;
		float w_ff;
		float w_integrator_max;
		float w_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */
		float man_roll_scale;			/**< scale factor applied to roll actuator control in pure manual mode */
		float man_pitch_scale;			/**< scale factor applied to pitch actuator control in pure manual mode */
		float man_yaw_scale; 			/**< scale factor applied to yaw actuator control in pure manual mode */

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		int vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

		//Custom(JL)	
    	int man_kind;
    	int semi_auto;
    	float Kp_a;
    	float Kp_e;
    	float Kp_r;
    	float Kap;
    	float Kpp;
    	float Kad;
    	float Kpd;
    	float Kai;
    	float Kpi;
    	float Khp;
	   	float Khd;
    	float Khi;
    	float Ku;



	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t p_tc;
		param_t p_p;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t r_tc;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_d;
		param_t y_ff;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_coordinated_method;
		param_t y_rmax;
		param_t w_p;
		param_t w_i;
		param_t w_ff;
		param_t w_integrator_max;
		param_t w_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_roll_scale;
		param_t man_pitch_scale;
		param_t man_yaw_scale;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t vtol_type;

		//Custom(JL)
    	param_t man_kind;
	   	param_t semi_auto;
    	param_t Kp_a;
    	param_t Kp_e;
    	param_t Kp_r;
    	param_t Kap;
    	param_t Kpp;
    	param_t Kad;
    	param_t Kpd;
    	param_t Kai;
    	param_t Kpi;
    	param_t Khp;
	   	param_t Khd;
    	param_t Khi;
    	param_t Ku;



	}		_parameter_handles;		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;




	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle land detected updates.
	 */
	void		vehicle_land_detected_poll();

	/**
	 *
	 * @param q
	 */
	void		aerobatics_control();

	void 		maneuver_generator();




	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();
	
	//Custom (JL)
	struct Ref_traj{
        float ref_states[41][17];
        float ref_time[41][1];
    }; // Reference trajectory
    
    struct Level_trim{
		float V;
		float theta;
		float Elev;
		float Thrust;
	}; // Level trim

	char *my_strdup(const char *str);
	
	void read_warp_traj(float csv_array[41][17], const char* csv_file);
	
	void read_warp_traj_time(float csv_array[41][1], const char* csv_file);
	
	/*
    Ref_traj interp_traj(float state_mat_1[][17], float state_mat_2[][17],
                         float time_mat_1[][1],   float time_mat_2[][1],
                         float curr_V, const int num_rows);
	*/
	
    float* feedforward_traj(float state_mat[][17], float time_mat[][1], float curr_time, const int num_rows);
    
    Level_trim get_level_trim(float curr_V);

    float get_thrust_signal(float omega);
};

namespace _control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

Control	*g_control = nullptr;
}

Control::Control() :

	_task_should_exit(false),
	_task_running(false),
	//custom

    //Position Control    
    delta_y_d_old(0.0f),
    delta_z_d_old(0.0f),
    lp_delta_y_d(200.0f, 10.0f),
    lp_delta_z_d(200.0f, 10.0f),

    //Thrust Control
    delta_h_d_old(0.0f),
    lp_delta_h_d(200.0f, 3.0f),
    omega_t_old(4000.0f),

    //Attitude Control
    Ex_old(0.0f),
    Ex_d_old(0.0f),
    Ey_old(0.0f),
    Ey_d_old(0.0f),
    Ez_old(0.0f),
    Ez_d_old(0.0f),
    lp_Ex_d(200.0f, 10.0f),
    lp_Ey_d(200.0f, 10.0f),
    lp_Ez_d(200.0f, 10.0f),
    lp_Vs(200.0f, 2.0f),
		
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_accel_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_vehicle_status_sub(-1),
	_vehicle_land_detected_sub(-1),


	/* publications */
	_rate_sp_pub(nullptr),
	_attitude_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_2_pub(nullptr),
	//custom
	_aerobatics_variables_pub(nullptr),
	_aerobatics_variables2_pub(nullptr),
	_debug_pub(nullptr),		

	_rates_sp_id(0),
	_actuators_id(0),
	_attitude_setpoint_id(0),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),
#if 0
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fwa_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fwa_nano")),
#else
	_nonfinite_input_perf(nullptr),
	_nonfinite_output_perf(nullptr),
#endif
	/* states */
	_setpoint_valid(false),
	//_debug(false),
	_flaps_cmd_last(0),
	_flaperons_cmd_last(0)
{
	/* safely initialize structs */
	_ctrl_state = {};
	_accel = {};
	_att_sp = {};
	_rates_sp = {};
	_manual = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};
	_vehicle_status = {};
	_vehicle_land_detected = {};
	//custom
	memset(&_aerobatics_variables, 0, sizeof(_aerobatics_variables));
	memset(&_aerobatics_variables2, 0, sizeof(_aerobatics_variables2));
	memset(&_debug, 0, sizeof(_debug));

	_counter = 0;


	_parameter_handles.p_tc = param_find("FW_P_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");

	_parameter_handles.r_tc = param_find("FW_R_TC");
	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");

	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");
	_parameter_handles.y_coordinated_method = param_find("FW_YCO_METHOD");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");
	_parameter_handles.man_roll_scale = param_find("FW_MAN_R_SC");
	_parameter_handles.man_pitch_scale = param_find("FW_MAN_P_SC");
	_parameter_handles.man_yaw_scale = param_find("FW_MAN_Y_SC");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.vtol_type = param_find("VT_TYPE");

	//Custom
	_parameter_handles.man_kind = param_find("MAN_KIND_JL");
	_parameter_handles.semi_auto = param_find("SEMI_AUTO_JL");
	_parameter_handles.Kp_a     = param_find("KP_A");
	_parameter_handles.Kp_e     = param_find("KP_E");
	_parameter_handles.Kp_r     = param_find("KP_R");
	_parameter_handles.Kap      = param_find("KAP_JL");
	_parameter_handles.Kpp      = param_find("KPP_JL");
	_parameter_handles.Kad      = param_find("KAD_JL");
	_parameter_handles.Kpd      = param_find("KPD_JL");
	_parameter_handles.Kai      = param_find("KAI_JL");
	_parameter_handles.Kpi      = param_find("KPI_JL");
	_parameter_handles.Khp      = param_find("KHP_JL");
	_parameter_handles.Khd      = param_find("KHD_JL");
	_parameter_handles.Khi      = param_find("KHI_JL");
	_parameter_handles.Ku       = param_find("KU_JL");


	/* fetch initial parameter values */
	parameters_update();
}

Control::~Control()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	_control::g_control = nullptr;
}

int
Control::parameters_update()
{

	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_coordinated_method, &(_parameters.y_coordinated_method));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));

	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);
	param_get(_parameter_handles.man_roll_scale, &(_parameters.man_roll_scale));
	param_get(_parameter_handles.man_pitch_scale, &(_parameters.man_pitch_scale));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_parameters.w_p);
	_wheel_ctrl.set_k_i(_parameters.w_i);
	_wheel_ctrl.set_k_ff(_parameters.w_ff);
	_wheel_ctrl.set_integrator_max(_parameters.w_integrator_max);
	_wheel_ctrl.set_max_rate(math::radians(_parameters.w_rmax));

	//Custom(JL)
	param_get(_parameter_handles.man_kind, &_parameters.man_kind);
	param_get(_parameter_handles.semi_auto, &_parameters.semi_auto);
	param_get(_parameter_handles.Kp_a, &_parameters.Kp_a);
	param_get(_parameter_handles.Kp_e, &_parameters.Kp_e);
	param_get(_parameter_handles.Kp_r, &_parameters.Kp_r);
	param_get(_parameter_handles.Kap, &_parameters.Kap);
	param_get(_parameter_handles.Kpp, &_parameters.Kpp);
	param_get(_parameter_handles.Kad, &_parameters.Kad);
	param_get(_parameter_handles.Kpd, &_parameters.Kpd);
	param_get(_parameter_handles.Kai, &_parameters.Kai);
	param_get(_parameter_handles.Kpi, &_parameters.Kpi);
	param_get(_parameter_handles.Khp, &_parameters.Khp);
	param_get(_parameter_handles.Khd, &_parameters.Khd);
	param_get(_parameter_handles.Khi, &_parameters.Khi);
	param_get(_parameter_handles.Ku, &_parameters.Ku);

	return OK;
}

void
Control::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
Control::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
Control::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
Control::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
Control::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
Control::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
Control::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}


void
Control::aerobatics_control()
{
	//Get trajectory
	maneuver_generator();

    //Get feedback from state estimator
	math::Quaternion q(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]); //Attitude Quaternion
	math::Vector<3> Euler = q.to_euler(); //Euler Angles (rad)
	float theta = float(Euler(1)); //Pitch angle (rad)
	math::Vector<3> p; //NED Position (m)
	p(0) = _ctrl_state.x_pos;
	p(1) = _ctrl_state.y_pos;
	p(2) = _ctrl_state.z_pos;

	//Aircraft Properties
	const float A_prop = 0.041f; //Propeller Area (m^2)
	const float m = .45f; //Mass (kg)
	const float Ix = 0.003922f; //Moment of Inertia about x (kg m^2)
	const float Iy = 0.015940f; //Moment of Inertia about y (kg m^2)
	const float Iz = 0.019340f; //Moment of Inertia about z (kg m^2)
	const float S = 0.14274f; //Wing Area (m^2)
	const float b = 0.864f; //Wing Span (m)
	const float cbar = 0.21f; //Mean Aerodynamic Chord (m)

	const float Cl_delta_a = -0.0006777f; //Aileron Control Derivative Coefficient (/deg)
	const float Cm_delta_e = -0.0117747f; //Elevator Control Derivative Coefficient (/deg)
	const float Cn_delta_r = -0.0035663f; //Rudder Control Derivative Coefficient (/deg)
    //const float Cl_delta_r = 0.00093111f; //Other Aileron Control Derivative Coefficient  -- Custom (JL) -- Need to double check sign (+/-)

	const float AilDef_max = 52.0f; //52.0fMaximum Aileron Deflection (deg)
	const float ElevDef_max = 59.0f; //35.0fMaximum Elevator Deflection (deg)
	const float RudDef_max = 49.0f; //56.0fMaximum Rudder Deflection (deg)
	const float omega_t_min = 1716.0f; // Minimum Thrust(RPM)
	const float omega_t_max = 6710.0f; //Maximimum Thrust (RPM)

	//Position Controller-------------------------------------------------------------------------------------------------------------------------------

	math::Matrix<3, 3> C_ri = q_ref.to_dcm().transposed(); //Rotation Matrix from inertial to ref
	math::Vector<3> e_i = p_ref - p; //Inertial Position Error (m)
	math::Vector<3> e_i_d = (e_i - e_i_old) / dt; //Inertial Position Error Derivative (m/s)

	e_i_i += e_i * dt;
    e_i_old = e_i; //Last inertial position error

	math::Vector<3> e_r = C_ri*e_i; //Position Error expressed in Ref frame
	math::Vector<3> e_r_d = C_ri*e_i_d; //Position Error Derivative expressed in Ref frame
	math::Vector<3> e_r_i = C_ri*e_i_i; //Position Error Integral expressed in Ref frame

	float delta_y = e_r(1); //Ref Frame y error
	float delta_z = e_r(2); //Ref Frame z error

	float delta_y_d = e_r_d(1); //Ref Frame y error derivative
	float delta_z_d = e_r_d(2); //Ref Frame z error derivative
	float delta_y_i = e_r_i(1); //Ref Frame y error integral
	float delta_z_i = e_r_i(2); //Ref Frame z error integral

	if (maneuver_switch == true){
		delta_y_d = delta_y_d_old;
		delta_z_d = delta_z_d_old;
		e_i_i *= 0.0f;
		lp_delta_y_d.reset(delta_y_d);
		lp_delta_z_d.reset(delta_z_d);
	}

	if (abs(delta_y_d) > 50.0f or abs(delta_z_d) > 50.0f){
		delta_y_d = delta_y_d_old; 
		delta_z_d = delta_z_d_old;
	}

	delta_y_d_old = delta_y_d;
	delta_z_d_old = delta_z_d;

	float z_rot = (_parameters.Kpp * delta_y + _parameters.Kpd * lp_delta_y_d.apply(delta_y_d) + _parameters.Kpi * delta_y_i); //Angle of rotation of reference quaternion about bodyframe z (rad)
	float y_rot = (_parameters.Kpp * delta_z + _parameters.Kpd * lp_delta_z_d.apply(delta_z_d) + _parameters.Kpi * delta_z_i); //Angle of rotation of reference quaternion about bodyframe y (rad)

	/*Bound reference quaternion rotations to 45 degrees*/
	if (z_rot > PI/4.0f){z_rot = PI/4.0f;}
	if (z_rot < -PI/4.0f){z_rot = -PI/4.0f;}
	if (y_rot > PI/4.0f){y_rot = PI/4.0f;}
	if (y_rot < -PI/4.0f){y_rot = -PI/4.0f;}

	math::Quaternion q_z(cos(z_rot/2.0f),0.0f,0.0f,sin(z_rot/2.0f)); //z-axis rotation
	math::Quaternion q_y(cos(y_rot/2.0f),0.0f,-sin(y_rot/2.0f),0.0f); //y-axis rotation
	math::Quaternion q_zy = q_z*q_y; //total rotation
	math::Quaternion q_des = q_ref; //Desired Quaternion (not modified unless told to in next line)
	if(pos_fb){q_des = q_ref*q_zy;}

    //Thrust Controller----------------------------------------------------------------------------------------------------------------------------------

    float h_ref = -p_ref(2); // Get desired height from maneuver generator (m)    
    float delta_h = h_ref-(-p(2)); //Height error (m)
	float delta_h_d = (delta_h - delta_h_old) / dt; //Height error derivative (m/s)
    delta_h_i += delta_h * dt; //Height error integral (ms)

    if (maneuver_switch == true){
		delta_h_i = 0.0f; 
		delta_h_d = delta_h_d_old;
		lp_delta_h_d.reset(delta_h_d);
    }

    if (abs(delta_h_d) > 50.0f){
    	delta_h_d = delta_h_d_old;
    }

    delta_h_old = delta_h; //Last Height error
   	delta_h_d_old = delta_h_d;

	float delta_u = u_ref - _ctrl_state.x_vel; //Speed Error in body frame x [u] (m/s)

	float v = powf(powf(_ctrl_state.x_vel,2.0f) + powf(_ctrl_state.y_vel,2.0f) + powf(_ctrl_state.z_vel,2.0f),0.5f);//speed squared
	float J = v/(omega_t_old*.0042333f); //Advance ratio = V/(omega/60 * D)
	if (J < 0.0f){J = 0.0f;}
	if (J > 0.5f){J = 0.5f;}
	float kt = (-1.43909969f*powf(J,2.0f) - 2.21240323f*J + 2.24512051f) * powf(10.0,-7.0f);    
	//kt = 0.000000221f; //Custom (JL) old constant value

    //Apply PID control laws for a desired u dot
    float u_d = _parameters.Ku * delta_u + (_parameters.Khp * delta_h + _parameters.Khd * lp_delta_h_d.apply(delta_h_d) + _parameters.Khi * delta_h_i)*sinf(theta); //Commanded u dot (m/s^2)
    float T = kt*powf(cont_ff[3],2.0f) + m * u_d; //Commanded thrust force (N)
    // float T = kt*powf(cont_ff[3],2.0f); // Constant thrust


    //Do not allow negative thrust
	if (T < 0.0f) T = 0.0f;

    float omega_t = powf(T/kt,0.5f); //Commanded motor speed (RPM)

    //Custom(JL) - Hopefully I won't need this (for actual flight tests), but just in case I do...
    //cont_ff[3] = 0.0f + 1.1f*cont_ff[3];
    //cont_ff[3] = (float) cont_ff[3];


	//Attitude Controller------------------------------------------------------------------------------------------------------------------------------------
 	if ((q + q_des).length() < (q - q_des).length()){
 		q_des = -q_des;
 	}
 	 //This ensures delta_q results in error angles less than 180 deg
 	math::Quaternion delta_q = q.conjugated()*q_des; //Error Quaternion

 	if (delta_q(0)<-1){
 		delta_q(0)=-1.0f;
 	} //if slightly larger than |1| acos will give problems

 	if (delta_q(0)>1){
 		delta_q(0)=1.0f;
 	}

 	float Ex = 2.0f*acosf(delta_q(0))*delta_q(1)/delta_q.imag().length(); //Error along body frame x-axis (rad)
 	float Ey = 2.0f*acosf(delta_q(0))*delta_q(2)/delta_q.imag().length(); //Error along body frame y-axis (rad)
 	float Ez = 2.0f*acosf(delta_q(0))*delta_q(3)/delta_q.imag().length(); //Error along body frame z-axis (rad)
 	
	float Ex_d = (Ex - Ex_old) / dt; //Error Derivative along body frame x-axis (rad/s)
	float Ey_d = (Ey - Ey_old) / dt; //Error Derivative along body frame y-axis (rad/s)
	float Ez_d = (Ez - Ez_old) / dt; //Error Derivative along body frame z-axis (rad/s)

	Ex_i += Ex * dt;
	Ey_i += Ey * dt;
	Ez_i += Ez * dt;

	if (maneuver_switch == true){
		Ex_d = Ex_d_old; 
		Ey_d = Ey_d_old; 
		Ez_d = Ez_d_old; 
		Ex_i = 0.0f; 
		Ey_i = 0.0f; 
		Ez_i = 0.0f;
		lp_Ex_d.reset(Ex_d);
		lp_Ey_d.reset(Ey_d);
		lp_Ez_d.reset(Ez_d);
	}

	if (abs(Ex_d) > 20.0f or abs(Ey_d) > 20.0f or abs(Ez_d) > 20.0f){
		Ex_d = Ex_d_old; 
		Ey_d = Ey_d_old; 
		Ez_d = Ez_d_old;
	}

	Ex_d_old = Ex_d; //Last Ex
	Ey_d_old = Ey_d; //Last Ey
	Ez_d_old = Ez_d; //Last Ez

	Ex_old = Ex; //Last Ex
	Ey_old = Ey; //Last Ey
	Ez_old = Ez; //Last Ez

 	float Vs = powf(powf(_ctrl_state.x_vel,2.0f) + 2.0f * T / (rho * A_prop),0.5f);//Approximated Slipstream Velocity (m/s)
 	float Vs_min = powf(2.0f * m * g  / (rho * A_prop),0.5f);//Approximated minimum slipstream velocity-- Calculated from hover
 	if (Vs < Vs_min) Vs = Vs_min; //Ensure a minimum slipstream velocity
 	float Vs_filt = lp_Vs.apply(Vs);
 	if (Vs_filt < Vs_min) Vs_filt = Vs_min; 

 	//Custom(JL)
    float L = (_parameters.Kp_a*Ex + _parameters.Kad*lp_Ex_d.apply(Ex_d) + _parameters.Kai*Ex_i)*Ix; //Desired rolling moment (Nm)-- along the body frame x-axis 
    float M = (_parameters.Kp_e*Ey + _parameters.Kad*lp_Ey_d.apply(Ey_d) + _parameters.Kai*Ey_i)*Iy; //Desired pitching moment (Nm)-- along the body frame y-axis
    float N = (_parameters.Kp_r*Ez + _parameters.Kad*lp_Ez_d.apply(Ez_d) + _parameters.Kai*Ez_i)*Iz; //Desired yawing moment (Nm)-- along the body frame z-axis    

    if (double_gains)
    {
    	L = 2.0f*L;
    	M = 2.0f*M;
    	N = 2.0f*N;
    }

    float AilDef  = L/(.5f*rho*powf(Vs_filt,2.0f)*S*b*Cl_delta_a)       - cont_ff[0]; //Aileron Deflection (deg)
    float ElevDef = M/(.5f*rho*powf(Vs_filt,2.0f)*S*cbar*Cm_delta_e)    + cont_ff[1]; //Elevator Deflection (deg)
    float RudDef  = N/(.5f*rho*powf(Vs_filt,2.0f)*S*b*Cn_delta_r)       + cont_ff[2]; //Rudder Deflection (deg)

    /*
    //Indoor testing 
    AilDef  = -cont_ff[0];
    ElevDef =  cont_ff[1];
    RudDef  =  cont_ff[2];
    omega_t =  cont_ff[3];
    */

	
    //Thrust Control for extra slipstream---------------------------------------------------------------------------------------------------

    float Vs_des_A = 0.0f;
    float Vs_des_E = 0.0f;
    float Vs_des_R = 0.0f;

    if (abs(AilDef) > AilDef_max) Vs_des_A = powf(abs(L/(0.5f * rho * S * b * Cl_delta_a * AilDef_max)),0.5f);
    if (abs(ElevDef) > ElevDef_max) Vs_des_E = powf(abs(M/(0.5f * rho * S * cbar * Cm_delta_e * ElevDef_max)),0.5f);
    if (abs(RudDef) > RudDef_max) Vs_des_R = powf(abs(N/(0.5f * rho * S * b * Cn_delta_r * RudDef_max)),0.5f);

    float Vs_des = Vs_des_A;

    if (Vs_des < Vs_des_E) Vs_des = Vs_des_E;
    if (Vs_des < Vs_des_R) Vs_des = Vs_des_R; 

    if (Vs_des > _ctrl_state.x_vel && Vs_des > 0.0f)
    {
    	T += (rho*A_prop / 2.0f) * (powf(Vs_des,2.0f) - powf(_ctrl_state.x_vel,2.0f));

    	if(T < 0.0f) T = 0.0f;
    	omega_t = powf(T / kt,0.5f); //RPM command
    }
	

    //Saturate motor speed command
    if (omega_t < omega_t_min) omega_t = omega_t_min;
    else if (omega_t > omega_t_max) omega_t = omega_t_max;

    omega_t_old = omega_t;

    float thrust_PWM = powf(omega_t,2.0f) * powf(10.0f,-5.0f) * 4.78527f - 0.22539f * omega_t + 1345.53521f;
	float thrust_cmd = thrust_PWM / 1000.0f - 1.0f;

    //Send Signals------------------------------------------------------------------------------------------------------------------------
   	std::vector<float> outputs(4);
    outputs[0]= 0.0000016235f * powf(AilDef,3.0f) - 0.0000009861f * powf(AilDef,2.0f) + 0.0145866432f * AilDef;
    outputs[1]= 0.0000008317f * powf(ElevDef,3.0f) + 0.0000409759f * powf(ElevDef,2.0f) + 0.01396963f * ElevDef;
    outputs[2]= 0.0000007988f * powf(RudDef,3.0f) + 0.0000092020f * powf(RudDef,2.0f) + 0.0187045418f * RudDef;
	outputs[3]= thrust_cmd;

	//Saturate Outputs
    for (int i = 0; i < (outputs.size() - 1); ++i) {
        if (outputs[i] >= 1.0f) {
        	outputs[i] = 1.0f;
        } 
        else if (outputs[i] <= -1.0f) {
        	outputs[i] = -1.0f;
        }
    }

    //Send parameters to data logger
    _aerobatics_variables.Quaternion_Desired[0] = q_des(0);
    _aerobatics_variables.Quaternion_Desired[1] = q_des(1);
    _aerobatics_variables.Quaternion_Desired[2] = q_des(2);
    _aerobatics_variables.Quaternion_Desired[3] = q_des(3);
    _aerobatics_variables.Position_Reference[0] = p_ref(0);
    _aerobatics_variables.Position_Reference[1] = p_ref(1);
    _aerobatics_variables.Position_Reference[2] = p_ref(2);
    _aerobatics_variables.Slipstream = Vs;
    _aerobatics_variables.Speed_Reference = u_ref;
    _aerobatics_variables.Quaternion_Reference[0] = q_ref(0);
    _aerobatics_variables.Quaternion_Reference[1] = q_ref(1);
    _aerobatics_variables.Quaternion_Reference[2] = q_ref(2);
    _aerobatics_variables.Quaternion_Reference[3] = q_ref(3);


    //Send Debug values
    _debug.floats[0] = Ex;
    _debug.floats[1] = Ey;
    _debug.floats[2] = Ez;
    _debug.floats[3] = outputs[0];
    _debug.floats[4] = outputs[1];
    _debug.floats[5] = outputs[2];
    _debug.floats[6] = outputs[3];
    _debug.floats[7] = Vs_filt;
    _debug.floats[8] = dt;
    _debug.integer[0] = 0;
    _debug.integer[1] = 0;
    _debug.boolean[0] = false;
    _debug.boolean[1] = false;
    _debug.boolean[2] = false;
    //Send to view in real time
    struct debug_key_value_s dbg;
	orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
    dbg.value=Ey;
    orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);  



	//Send to actuators
    // Full Control
	if (true) {
		_actuators.control[actuator_controls_s::INDEX_ROLL] = outputs[0];
    	_actuators.control[actuator_controls_s::INDEX_PITCH] = -outputs[1];
		_actuators.control[actuator_controls_s::INDEX_YAW] = -outputs[2];
    	_actuators.control[actuator_controls_s::INDEX_THROTTLE] = outputs[3];
	} 
	// Aileron Control
	else if (_parameters.semi_auto == 2){
		_actuators.control[actuator_controls_s::INDEX_ROLL] = outputs[0];
		_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale + _parameters.trim_pitch;
		_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
	}
	// Aileron and Elevator Control
	else if (_parameters.semi_auto == 3){
		_actuators.control[actuator_controls_s::INDEX_ROLL] = outputs[0];
		_actuators.control[actuator_controls_s::INDEX_PITCH] = outputs[1];
		_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
	}
	// Aileron, Elevator, Rudder Control
	else if (_parameters.semi_auto == 4){
		_actuators.control[actuator_controls_s::INDEX_ROLL] = outputs[0];
		_actuators.control[actuator_controls_s::INDEX_PITCH] = outputs[1];
		_actuators.control[actuator_controls_s::INDEX_YAW] = outputs[2];
		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
	}
	// Pure Manual but recording control info
	else {
		_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _parameters.man_roll_scale + _parameters.trim_roll;
		_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale +
		_parameters.trim_pitch;
		_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
	}
}

//Custom(JL)
void 
Control::maneuver_generator()
{
	//Get Feedback from state estimator
	math::Quaternion q(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Vector<3> Euler = q.to_euler();
	float psi = float(Euler(2));
	math::Vector<3> p;
	p(0) = _ctrl_state.x_pos;
	p(1) = _ctrl_state.y_pos;
	p(2) = _ctrl_state.z_pos;

	// Level flight trims
	static Level_trim level_trim;
	
	//Agile maneuver variables
	static Ref_traj ref_traj;
	static float* ref_traj_i;
	float V_i;
	float curr_time;
	static float t0_maneuver;
	static float tf_maneuver;
	static float x_0;
	static float y_0;
	static float z_0;
	static float psi_0;
	static math::Quaternion q_0;
	static math::Matrix<3, 3> C_0;
	math::Vector<3> p_ff;
	static bool man_just_ended = true;
	pos_fb = false;
	thr_fb = false;
	in_hover = false;
	double_gains = false;
	static bool doing_H2C = false;
	
	//maneuver_switch == true --> first iteration of maneuver
	if (maneuver_type-maneuver_type_old!=0){maneuver_switch = true;}else{maneuver_switch = false;}
	if (maneuver_type_old == 0){doing_H2C = false;}

	//Maneuver CSV locations
	static const char* ATA_csv = PX4_ROOTFSDIR"/fs/microsd/ATA_pp.csv";
	static const char* ATA_time_csv = PX4_ROOTFSDIR"/fs/microsd/ATA_time_pp.csv";

	static const char* C2H_csv = PX4_ROOTFSDIR"/fs/microsd/C2H.csv";
	static const char* C2H_time_csv = PX4_ROOTFSDIR"/fs/microsd/C2H_time.csv";

	static const char* H2C_csv = PX4_ROOTFSDIR"/fs/microsd/H2C.csv";
	static const char* H2C_time_csv = PX4_ROOTFSDIR"/fs/microsd/H2C_time.csv";

	static const char* KE_csv = PX4_ROOTFSDIR"/fs/microsd/KE.csv";
	static const char* KE_time_csv = PX4_ROOTFSDIR"/fs/microsd/KE_time.csv";

	const int man_rows = 41;
	static float man[man_rows][17];
	static float man_time[man_rows][1];

	if (maneuver_type == 1 && (maneuver_type_old == 2 || doing_H2C)) //Hover to wings-level
	{

		if (maneuver_switch) //Maneuver just started
		{				
			read_warp_traj(man, H2C_csv);
			read_warp_traj_time(man_time, H2C_time_csv);

			doing_H2C = true;

			//No interpolation
			for(int i = 0; i < man_rows; ++i){
				ref_traj.ref_time[i][0] = man_time[i][0];
				for(int j = 0; j < 17; ++j){
					ref_traj.ref_states[i][j] = man[i][j];
				}
			}

			t0_maneuver = hrt_absolute_time();
			tf_maneuver = t0_maneuver + ref_traj.ref_time[man_rows-1][0]*1000000.0f;


			x_0 = p(0);
			y_0 = p(1);
			z_0 = p(2);
		}
		if (hrt_absolute_time() <= tf_maneuver) //During maneuver
		{

			thr_fb = true;
			//pos_fb = true;
			in_hover = true;
			
			curr_time = (hrt_absolute_time()-t0_maneuver)/1000000.0f;
			ref_traj_i = feedforward_traj(ref_traj.ref_states, ref_traj.ref_time, curr_time, man_rows);

			//Reference trajectory
			u_ref    = ref_traj_i[0];
			q_ref(0) = ref_traj_i[6];
			q_ref(1) = ref_traj_i[7];
			q_ref(2) = ref_traj_i[8];
			q_ref(3) = ref_traj_i[9];
			
			//Rotate about yaw angle at start
			q_ref = q_0*q_ref;
			
			p_ff(0) = ref_traj_i[10];
			p_ff(1) = ref_traj_i[11];
			p_ff(2) = ref_traj_i[12];
			p_ff = C_0*p_ff;
			
			p_ref(0) = x_0 + p_ff(0);
			p_ref(1) = y_0 + p_ff(1);
			p_ref(2) = z_0 + p_ff(2);

			//Feedforward control
			cont_ff[0] = ref_traj_i[13]*180.0f/PI;
			cont_ff[1] = ref_traj_i[14]*180.0f/PI;
			cont_ff[2] = ref_traj_i[15]*180.0f/PI;
			cont_ff[3] = ref_traj_i[16];		
		}
		else //Maneuver ended
		{		
			doing_H2C = false;
			maneuver_switch = true;
		}
	}

	if (maneuver_type == 1 && !doing_H2C) //Level flight
	{

		thr_fb = true;
		pos_fb = true;

		if (maneuver_switch) //Just entered level flight
		{
			//Get closest level trim conditions for speed
			V_i = powf((powf(_ctrl_state.x_vel, 2.0f) + powf(_ctrl_state.y_vel, 2.0f) + powf(_ctrl_state.z_vel, 2.0f)), 0.5f);
			level_trim = get_level_trim(V_i);
			
			p_0 = p;
			p_ref = p_0;
			if(maneuver_type_old == 1){psi_ref = psi_0;} else{psi_ref = psi;} //If recovering from hover, recover to heading before maneuver entry
			q_ref.from_euler(0.0f, level_trim.theta, psi_ref);
			u_ref = level_trim.V*cosf(level_trim.theta);
			
			cont_ff[0] = 0.0f;
			cont_ff[1] = level_trim.Elev;
			cont_ff[2] = 0.0f;
			cont_ff[3] = level_trim.Thrust;
		}
		p_ref(0)=float(pow(cos(psi_ref),2))*(p(0)-p_0(0)) + float(sin(psi_ref)*cos(psi_ref))*(p(1)-p_0(1)) + p_0(0);
		p_ref(1)=float(pow(sin(psi_ref),2))*(p(1)-p_0(1)) + float(sin(psi_ref)*cos(psi_ref))*(p(0)-p_0(0)) + p_0(1);
	}


	else if (maneuver_type == 2) //Wings-level to hover
	{	
		if (maneuver_switch) //Maneuver just started
		{		
			read_warp_traj(man, C2H_csv);
			read_warp_traj_time(man_time, C2H_time_csv);

			//V_i = powf((powf(_ctrl_state.x_vel, 2.0f) + powf(_ctrl_state.y_vel, 2.0f) + powf(_ctrl_state.z_vel, 2.0f)), 0.5f);
			//ref_traj = interp_traj(C2H_1, C2H_2, C2H_time_1, C2H_time_2, V_i, C2H_rows); //Interpolation
			
			//No interpolation
			for(int i = 0; i < man_rows; ++i){
				ref_traj.ref_time[i][0] = man_time[i][0];
				//PX4_INFO("C2H Time = %.4f", (double)C2H_time[i][0]);
				for(int j = 0; j < 17; ++j){
					ref_traj.ref_states[i][j] = man[i][j];
				}
			}

			t0_maneuver = hrt_absolute_time();
			tf_maneuver = t0_maneuver + ref_traj.ref_time[man_rows-1][0]*1000000.0f;
			
			psi_0 = psi;
			q_0.from_euler(0.0f, 0.0f, psi_0);
			C_0 = q_0.to_dcm(); 
			x_0 = p(0);
			y_0 = p(1);
			z_0 = p(2);
		}
		if (hrt_absolute_time() <= tf_maneuver) //During maneuver
		{
			thr_fb = true;
			//pos_fb = true;
			double_gains = true;
			in_hover = true;
			
			curr_time = (hrt_absolute_time()-t0_maneuver)/1000000.0f;
			ref_traj_i = feedforward_traj(ref_traj.ref_states, ref_traj.ref_time, curr_time, man_rows);

			//Reference trajectory
			u_ref    = ref_traj_i[0];
			q_ref(0) = ref_traj_i[6];
			q_ref(1) = ref_traj_i[7];
			q_ref(2) = ref_traj_i[8];
			q_ref(3) = ref_traj_i[9];
			
			//Rotate about yaw angle at start
			q_ref = q_0*q_ref;
			
			p_ff(0) = ref_traj_i[10];
			p_ff(1) = ref_traj_i[11];
			p_ff(2) = ref_traj_i[12];
			p_ff = C_0*p_ff;
			
			p_ref(0) = x_0 + p_ff(0);
			p_ref(1) = y_0 + p_ff(1);
			p_ref(2) = z_0 + p_ff(2);

			//Feedforward control
			cont_ff[0] = ref_traj_i[13]*180.0f/PI;
			cont_ff[1] = ref_traj_i[14]*180.0f/PI;
			cont_ff[2] = ref_traj_i[15]*180.0f/PI;
			cont_ff[3] = ref_traj_i[16];		
		}
		else //Maneuver ended
		{		
			if (man_just_ended)
			{
				p_ref = p;
				man_just_ended = false;
			}

			thr_fb = true;
			pos_fb = true;
			in_hover = true;
			//double_gains = true;

			//Reference trajectory
			u_ref    = 0.0f;
			q_ref(0) = ref_traj.ref_states[man_rows-1][6];
			q_ref(1) = ref_traj.ref_states[man_rows-1][7];
			q_ref(2) = ref_traj.ref_states[man_rows-1][8];
			q_ref(3) = ref_traj.ref_states[man_rows-1][9];
			
			//Rotate about yaw angle at start
			q_ref = q_0*q_ref;
			
			cont_ff[0] = 0.11f;
			cont_ff[1] = 0.0f;
			cont_ff[2] = 0.0f;
			cont_ff[3] = 4762.0f;
		}
	}
	
	else if (maneuver_type == 3)
	{
		thr_fb = true;
		pos_fb = true;
		in_hover = true;

		if(maneuver_switch)
		{
			psi_0 = psi;
			p_ref = p;
		}
		
		u_ref = 0.0f;
		q_ref.from_euler(0.0f, PI/2.0f, psi_0);
			
		cont_ff[0] = 0.11f;
		cont_ff[1] = 0.0f;
		cont_ff[2] = 0.0f;
		cont_ff[3] = 4762.0f;
		
	}
	
	else if (maneuver_type == 4) //ATA
	{
		if (maneuver_switch) //Maneuver just started
		{		
			read_warp_traj(man, ATA_csv);
			read_warp_traj_time(man_time, ATA_time_csv);
			//V_i = powf((powf(_ctrl_state.x_vel, 2.0f) + powf(_ctrl_state.y_vel, 2.0f) + powf(_ctrl_state.z_vel, 2.0f)), 0.5f);
			//ref_traj = interp_traj(ATA_1, ATA_2, ATA_time_1, ATA_time_2, V_i, ATA_rows); //Interpolation

			//No interpolation
			for(int i = 0; i < man_rows; ++i){
				ref_traj.ref_time[i][0] = man_time[i][0];
				for(int j = 0; j < 17; ++j){
					ref_traj.ref_states[i][j] = man[i][j];
				}
			}

			t0_maneuver = hrt_absolute_time();
			tf_maneuver = t0_maneuver + ref_traj.ref_time[man_rows-1][0]*1000000.0f;
			
			psi_0 = psi;
			q_0.from_euler(0.0f, 0.0f, psi_0);
			C_0 = q_0.to_dcm(); 
			x_0 = p(0);
			y_0 = p(1);
			z_0 = p(2);
		}
		if (hrt_absolute_time() <= tf_maneuver) //During maneuver
		{
			thr_fb = true;
			//pos_fb= true;
			//double_gains = true;
			
			curr_time = (hrt_absolute_time()-t0_maneuver)/1000000.0f;
			ref_traj_i = feedforward_traj(ref_traj.ref_states, ref_traj.ref_time, curr_time, man_rows);

			//Reference trajectory
			u_ref    = ref_traj_i[0];
			q_ref(0) = ref_traj_i[6];
			q_ref(1) = ref_traj_i[7];
			q_ref(2) = ref_traj_i[8];
			q_ref(3) = ref_traj_i[9];
			
			//Rotate about yaw angle at start
			q_ref = q_0*q_ref;
			
			p_ff(0) = ref_traj_i[10];
			p_ff(1) = ref_traj_i[11];
			p_ff(2) = ref_traj_i[12];
			p_ff = C_0*p_ff;
			
			p_ref(0) = x_0 + p_ff(0);
			p_ref(1) = y_0 + p_ff(1);
			p_ref(2) = z_0 + p_ff(2);

			//Feedforward control
			cont_ff[0] = ref_traj_i[13]*180.0f/PI;
			cont_ff[1] = ref_traj_i[14]*180.0f/PI;
			cont_ff[2] = ref_traj_i[15]*180.0f/PI;
			cont_ff[3] = ref_traj_i[16];	
		}
		else //Maneuver ended
		{
			thr_fb = true;
			pos_fb = true;

			if (man_just_ended) //Just entered level flight
			{
				//Get closest level trim conditions for speed
				V_i = powf((powf(_ctrl_state.x_vel, 2.0f) + powf(_ctrl_state.y_vel, 2.0f) + powf(_ctrl_state.z_vel, 2.0f)), 0.5f);
				level_trim = get_level_trim(V_i);
				
				p_0 = p;
				p_ref = p_0;
				psi_ref = psi_0 + PI; 
				q_ref.from_euler(0.0f, level_trim.theta, psi_ref);
				u_ref = level_trim.V*cosf(level_trim.theta);
				
				cont_ff[0] = 0.0f;
				cont_ff[1] = level_trim.Elev;
				cont_ff[2] = 0.0f;
				cont_ff[3] = level_trim.Thrust;
				
				man_just_ended = false;
			}
			p_ref(0)=float(pow(cos(psi_ref),2))*(p(0)-p_0(0)) + float(sin(psi_ref)*cos(psi_ref))*(p(1)-p_0(1)) + p_0(0);
			p_ref(1)=float(pow(sin(psi_ref),2))*(p(1)-p_0(1)) + float(sin(psi_ref)*cos(psi_ref))*(p(0)-p_0(0)) + p_0(1);
		}
	}

	else if (maneuver_type == 5) //Knife-edge
	{
		if (maneuver_switch) //Maneuver just started
		{		
			read_warp_traj(man, KE_csv);
			read_warp_traj_time(man_time, KE_time_csv);
			//V_i = powf((powf(_ctrl_state.x_vel, 2.0f) + powf(_ctrl_state.y_vel, 2.0f) + powf(_ctrl_state.z_vel, 2.0f)), 0.5f);
			//ref_traj = interp_traj(ATA_1, ATA_2, ATA_time_1, ATA_time_2, V_i, ATA_rows); //Interpolation

			//No interpolation
			for(int i = 0; i < man_rows; ++i){
				ref_traj.ref_time[i][0] = man_time[i][0];
				for(int j = 0; j < 17; ++j){
					ref_traj.ref_states[i][j] = man[i][j];
				}
			}

			t0_maneuver = hrt_absolute_time();
			tf_maneuver = t0_maneuver + ref_traj.ref_time[man_rows-1][0]*1000000.0f;
			
			psi_0 = psi;
			q_0.from_euler(0.0f, 0.0f, psi_0);
			C_0 = q_0.to_dcm(); 
			x_0 = p(0);
			y_0 = p(1);
			z_0 = p(2);
		}
		if (false) //(hrt_absolute_time() <= tf_maneuver) //During maneuver
		{
			thr_fb = true;
			//pos_fb= true;
			//double_gains = true;
			
			curr_time = (hrt_absolute_time()-t0_maneuver)/1000000.0f;
			ref_traj_i = feedforward_traj(ref_traj.ref_states, ref_traj.ref_time, curr_time, man_rows);

			//Reference trajectory
			u_ref    = ref_traj_i[0];
			q_ref(0) = ref_traj_i[6];
			q_ref(1) = ref_traj_i[7];
			q_ref(2) = ref_traj_i[8];
			q_ref(3) = ref_traj_i[9];
			
			//Rotate about yaw angle at start
			q_ref = q_0*q_ref;
			
			p_ff(0) = ref_traj_i[10];
			p_ff(1) = ref_traj_i[11];
			p_ff(2) = ref_traj_i[12];
			p_ff = C_0*p_ff;
			
			p_ref(0) = x_0 + p_ff(0);
			p_ref(1) = y_0 + p_ff(1);
			p_ref(2) = z_0 + p_ff(2);

			//Feedforward control
			cont_ff[0] = ref_traj_i[13]*180.0f/PI;
			cont_ff[1] = ref_traj_i[14]*180.0f/PI;
			cont_ff[2] = ref_traj_i[15]*180.0f/PI*0.25f;
			cont_ff[3] = ref_traj_i[16];	
		}
		else //Maneuver ended
		{
			in_hover = true;
			pos_fb = true;
			//double_gains = true;

			//Reference trajectory
			u_ref    = ref_traj.ref_states[man_rows-1][0];
			q_ref(0) = ref_traj.ref_states[man_rows-1][6];
			q_ref(1) = ref_traj.ref_states[man_rows-1][7];
			q_ref(2) = ref_traj.ref_states[man_rows-1][8];
			q_ref(3) = ref_traj.ref_states[man_rows-1][9];
			
			//Rotate about yaw angle at start
			q_ref = q_0*q_ref;
			
			cont_ff[0] = ref_traj.ref_states[man_rows-1][13]*180.0f/PI;
			cont_ff[1] = ref_traj.ref_states[man_rows-1][14]*180.0f/PI;
			cont_ff[2] = ref_traj.ref_states[man_rows-1][15]*180.0f/PI*0.25f;
			cont_ff[3] = ref_traj.ref_states[man_rows-1][16];

			if (man_just_ended) //Just entered level flight
			{			
				p_0 = p;
				p_ref = p_0;
				
				man_just_ended = false;
			}
			p_ref(0)=float(pow(cos(psi_ref),2))*(p(0)-p_0(0)) + float(sin(psi_ref)*cos(psi_ref))*(p(1)-p_0(1)) + p_0(0);
			p_ref(1)=float(pow(sin(psi_ref),2))*(p(1)-p_0(1)) + float(sin(psi_ref)*cos(psi_ref))*(p(0)-p_0(0)) + p_0(1);
		}
	}

	if(maneuver_type == 1){man_just_ended = true;} //Reset
	
}
//Custom(JL)
char*
Control::my_strdup(const char *str) {
    size_t len = strlen(str);
    char *x = (char *)malloc(len+1);
    if(!x) return NULL; 
    memcpy(x,str,len+1); 
    return x;
}


void
Control::read_warp_traj(float csv_array[41][17], const char* csv_file)
{
   int i=0,j=0;
   FILE* stream = fopen(csv_file,"r");

    char line[200];
    while(fgets(line, 200, stream))
    {
        char* tmp = my_strdup(line);
        char* split = strtok(tmp, ",");
        while(split != NULL)
        {
            csv_array[i][j++] = atof(split);
            split = strtok(NULL,",");
        }
        ++i;
        j = 0;
        free(tmp);
    }
    fclose(stream);
}

void
Control::read_warp_traj_time(float csv_array_time[41][1], const char* csv_file_time)
{
   int i=0,j=0;
   FILE* stream = fopen(csv_file_time,"r");

    char line[200];
    while(fgets(line, 200, stream))
    {
        char* tmp = my_strdup(line);
        char* split = strtok(tmp, ",");
        while(split != NULL)
        {
            csv_array_time[i][j++] = atof(split);
            split = strtok(NULL,",");
        }
        ++i;
        j = 0;
        free(tmp);
    }
    fclose(stream);
}

/*
Control::Ref_traj
Control::interp_traj(float state_mat_1[][17], float state_mat_2[][17],
                     float time_mat_1[][1],   float time_mat_2[][1],
                     float curr_V, const int num_rows)
{
	float V01 = sqrt(pow(state_mat_1[0][0],2) + pow(state_mat_1[0][1],2) + pow(state_mat_1[0][2],2));
    float V02 = sqrt(pow(state_mat_2[0][0],2) + pow(state_mat_2[0][1],2) + pow(state_mat_2[0][2],2));
    float sigma = (curr_V - V02)/(V01 - V02);
    int num_cols = 17;

    Ref_traj ref_traj_return;

    for(int i = 0; i < num_rows; ++i){
        ref_traj_return.ref_time[i][0] = sigma*time_mat_1[i][0] + (1 - sigma)*time_mat_2[i][0];
        for(int j = 0; j < num_cols; ++j){
            ref_traj_return.ref_states[i][j] = sigma*state_mat_1[i][j] + (1 - sigma)*state_mat_2[i][j];
        }
    }

    return ref_traj_return;
}
*/

//Custom(JL)
float*
Control::feedforward_traj(float state_mat[][17], float time_mat[][1], float curr_time, const int num_rows)
{
	int index_low = -1;
    static float inst_ref_traj[17];

    for(int i = 0; i < num_rows; ++i)
    {
        if (curr_time < time_mat[i][0]) {break;}
        ++index_low;
    }

    for(int j = 0 ; j < 17 ; ++j){
        inst_ref_traj[j] = state_mat[index_low][j] + (curr_time - time_mat[index_low][0])*(state_mat[index_low+1][j] - state_mat[index_low][j])/(time_mat[index_low+1][0] - time_mat[index_low][0]);
    }

    return inst_ref_traj;
}

Control::Level_trim
Control::get_level_trim(float curr_V)
{
	Level_trim level_trim_return;
	
	/*
	float V_3 = abs(curr_V - 3.0f);
	float V_5 = abs(curr_V - 5.0f);
	float V_7 = abs(curr_V - 7.0f);
	
	if (V_3 < V_5 && V_3 < V_7)
	{
		PX4_INFO("Current speed, %.4f, is closest to 3", (double)curr_V);
		level_trim_return.V      = 3.0f;
		level_trim_return.theta  = 0.7837f;
		level_trim_return.Elev   = -0.3668f;
		level_trim_return.Thrust = 4243.0f;
	}
	else if (V_7 < V_3 && V_7 < V_5)
	{
		PX4_INFO("Current speed, %.4f, is closest to 7", (double)curr_V);
		level_trim_return.V      = 7.0f;
		level_trim_return.theta  = 0.2085f;
		level_trim_return.Elev   = -0.1169f;
		level_trim_return.Thrust = 3732.0f;
	}
	else
	{
		PX4_INFO("Current speed, %.4f, is closest to 5", (double)curr_V);
		level_trim_return.V      = 5.0f;
		level_trim_return.theta  = 0.4137f;
		level_trim_return.Elev   = -0.2648f;
		level_trim_return.Thrust = 3802.0f;
	}
	*/
	
	//Just fly at 7 m/s
	level_trim_return.V      = 7.0f;
	level_trim_return.theta  = 0.2085f;
	level_trim_return.Elev   = -0.1169f;
	level_trim_return.Thrust = 3732.0f;
	
	return level_trim_return;
}

float
Control::get_thrust_signal(float omega)
{
	const float RPM_pre[20] = {1716, 2418, 3107, 3651, 4163, 4439, 4677, 4967, 5190, 5512, 5668, 5781, 5904, 5982, 6112, 6207, 6280, 6407, 6507, 6710};
	const float PWM_pre[20] = {1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950, 2000};
	float thrust_PWM;
	float thrust_cmd;

	int index_low = -1;

    for(int i = 0; i < 20; ++i)
    {
        if (omega < RPM_pre[i]) {break;}
        ++index_low;
    }

    thrust_PWM = PWM_pre[index_low] + (omega - RPM_pre[index_low])*(PWM_pre[index_low+1] - PWM_pre[index_low])/(RPM_pre[index_low+1] - RPM_pre[index_low]);

    thrust_cmd = thrust_PWM/1000.0f - 1.0f; 

    return thrust_cmd;
}

void
Control::task_main_trampoline(int argc, char *argv[])
{
	_control::g_control->task_main();
}

void
Control::task_main()
{
	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	// Run Control Loop at 200Hz
	orb_set_interval(_ctrl_state_sub, 5);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	vehicle_land_detected_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;


	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			/*if (dt > 1.0f) {
				dt = 0.01f;
			}

			if (dt < .001f){
				dt = .001f;
			}*/

			/* load local copies */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);


			/* get current rotation matrix and euler angles from control state quaternions */
			/*math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			_R = q_att.to_dcm();

			float _u = _ctrl_state.x_vel;*/

#if 0

			math::Vector<3> euler_angles;
			euler_angles = _R.to_euler();
			_roll    = euler_angles(0);
			_pitch   = euler_angles(1);
			_yaw     = euler_angles(2);

			if (_vehicle_status.is_vtol && _parameters.vtol_type == 0) {
				/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
				 *
				 * Since the VTOL airframe is initialized as a multicopter we need to
				 * modify the estimated attitude for the fixed wing operation.
				 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
				 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
				 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
				 * Additionally, in order to get the correct sign of the pitch, we need to multiply
				 * the new x axis of the rotation matrix with -1
				 *
				 * original:			modified:
				 *
				 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
				 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
				 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
				 * */
				math::Matrix<3, 3> R_adapted = _R;		//modified rotation matrix

				/* move z to x */
				R_adapted(0, 0) = _R(0, 2);
				R_adapted(1, 0) = _R(1, 2);
				R_adapted(2, 0) = _R(2, 2);

				/* move x to z */
				R_adapted(0, 2) = _R(0, 0);
				R_adapted(1, 2) = _R(1, 0);
				R_adapted(2, 2) = _R(2, 0);

				/* change direction of pitch (convert to right handed system) */
				R_adapted(0, 0) = -R_adapted(0, 0);
				R_adapted(1, 0) = -R_adapted(1, 0);
				R_adapted(2, 0) = -R_adapted(2, 0);
				euler_angles = R_adapted.to_euler();  //adapted euler angles for fixed wing operation

				/* fill in new attitude data */
				_R = R_adapted;
				_roll    = euler_angles(0);
				_pitch   = euler_angles(1);
				_yaw     = euler_angles(2);

				/* lastly, roll- and yawspeed have to be swaped */
				float helper = _ctrl_state.roll_rate;
				_ctrl_state.roll_rate = -_ctrl_state.yaw_rate;
				_ctrl_state.yaw_rate = helper;
			}

			vehicle_setpoint_poll();

			vehicle_accel_poll();

			vehicle_control_mode_poll();

			vehicle_manual_poll();

			global_pos_poll();

			vehicle_status_poll();

			vehicle_land_detected_poll();

			// the position controller will not emit attitude setpoints in some modes
			// we need to make sure that this flag is reset
			_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

			/* lock integrator until control is started */
			bool lock_integrator;

			if (_vcontrol_mode.flag_control_attitude_enabled && !_vehicle_status.is_rotary_wing) {
				lock_integrator = false;

			} else {
				lock_integrator = true;
			}

			/* Simple handling of failsafe: deploy parachute if failsafe is on */
			if (_vcontrol_mode.flag_control_termination_enabled) {
				_actuators_airframe.control[7] = 1.0f;
				//warnx("_actuators_airframe.control[1] = 1.0f;");

			} else {
				_actuators_airframe.control[7] = 0.0f;
				//warnx("_actuators_airframe.control[1] = -1.0f;");
			}

			/* if we are in rotary wing mode, do nothing */
			if (_vehicle_status.is_rotary_wing && !_vehicle_status.is_vtol) {
				continue;
			}

			/* default flaps to center */
			float flaps_control = 0.0f;

			static float delta_flaps = 0;

			/* map flaps by default to manual if valid */
			if (PX4_ISFINITE(_manual.flaps) && _vcontrol_mode.flag_control_manual_enabled) {
				flaps_control = 0.5f * (_manual.flaps + 1.0f) * _parameters.flaps_scale;

			} else if (_vcontrol_mode.flag_control_auto_enabled) {
				flaps_control = _att_sp.apply_flaps ? 1.0f * _parameters.flaps_scale : 0.0f;
			}

			// move the actual control value continuous with time
			static hrt_abstime t_flaps_changed = 0;

			if (fabsf(flaps_control - _flaps_cmd_last) > 0.01f) {
				t_flaps_changed = hrt_absolute_time();
				delta_flaps = flaps_control - _flaps_cmd_last;
				_flaps_cmd_last = flaps_control;
			}

			static float flaps_applied = 0.0f;

			if (fabsf(flaps_applied - flaps_control) > 0.01f) {
				flaps_applied = (flaps_control - delta_flaps) + (float)hrt_elapsed_time(&t_flaps_changed) * (delta_flaps) / 1000000;
			}

			/* default flaperon to center */
			float flaperon = 0.0f;

			static float delta_flaperon = 0.0f;

			/* map flaperons by default to manual if valid */
			if (PX4_ISFINITE(_manual.aux2) && _vcontrol_mode.flag_control_manual_enabled) {
				flaperon = 0.5f * (_manual.aux2 + 1.0f) * _parameters.flaperon_scale;

			} else if (_vcontrol_mode.flag_control_auto_enabled) {
				flaperon = _att_sp.apply_flaps ? 1.0f * _parameters.flaperon_scale : 0.0f;
			}

			// move the actual control value continuous with time
			static hrt_abstime t_flaperons_changed = 0;

			if (fabsf(flaperon - _flaperons_cmd_last) > 0.01f) {
				t_flaperons_changed = hrt_absolute_time();
				delta_flaperon = flaperon - _flaperons_cmd_last;
				_flaperons_cmd_last = flaperon;
			}

			static float flaperon_applied = 0.0f;

			if (fabsf(flaperon_applied - flaperon) > 0.01f) {
				flaperon_applied = (flaperon - delta_flaperon) + (float)hrt_elapsed_time(&t_flaperons_changed) *
						   (delta_flaperon) / 1000000;
			}

			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				/* scale around tuning airspeed */
				float airspeed;

				/* if airspeed is not updating, we assume the normal average speed */
				if (bool nonfinite = !PX4_ISFINITE(_ctrl_state.airspeed) || !_ctrl_state.airspeed_valid) {
					airspeed = _parameters.airspeed_trim;

					if (nonfinite) {
						perf_count(_nonfinite_input_perf);
					}

				} else {
					/* prevent numerical drama by requiring 0.5 m/s minimal speed */
					airspeed = math::max(0.5f, _ctrl_state.airspeed);
				}

				/*
				 * For scaling our actuators using anything less than the min (close to stall)
				 * speed doesn't make any sense - its the strongest reasonable deflection we
				 * want to do in flight and its the baseline a human pilot would choose.
				 *
				 * Forcing the scaling to this value allows reasonable handheld tests.
				 */
				float airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min :
							 airspeed);

				/* Use min airspeed to calculate ground speed scaling region.
				 * Don't scale below gspd_scaling_trim
				 */
				float groundspeed = sqrtf(_global_pos.vel_n * _global_pos.vel_n +
							  _global_pos.vel_e * _global_pos.vel_e);
				float gspd_scaling_trim = (_parameters.airspeed_min * 0.6f);
				float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

				float roll_sp = _parameters.rollsp_offset_rad;
				float pitch_sp = _parameters.pitchsp_offset_rad;
				float yaw_sp = 0.0f;
				float yaw_manual = 0.0f;
				float throttle_sp = 0.0f;

				// in STABILIZED mode we need to generate the attitude setpoint
				// from manual user inputs
				if (!_vcontrol_mode.flag_control_climb_rate_enabled) {
					_att_sp.roll_body = _manual.y * _parameters.man_roll_max + _parameters.rollsp_offset_rad;
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, -_parameters.man_roll_max, _parameters.man_roll_max);
					_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max + _parameters.pitchsp_offset_rad;
					_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max, _parameters.man_pitch_max);
					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust = _manual.z;
					int instance;
					orb_publish_auto(_attitude_setpoint_id, &_attitude_sp_pub, &_att_sp, &instance, ORB_PRIO_DEFAULT);
				}

				roll_sp = _att_sp.roll_body;
				pitch_sp = _att_sp.pitch_body;
				yaw_sp = _att_sp.yaw_body;
				throttle_sp = _att_sp.thrust;

				/* allow manual yaw in manual modes */
				if (_vcontrol_mode.flag_control_manual_enabled) {
					yaw_manual = _manual.r;
				}

				/* reset integrals where needed */
				if (_att_sp.roll_reset_integral) {
					_roll_ctrl.reset_integrator();
				}

				if (_att_sp.pitch_reset_integral) {
					_pitch_ctrl.reset_integrator();
				}

				if (_att_sp.yaw_reset_integral) {
					_yaw_ctrl.reset_integrator();
					_wheel_ctrl.reset_integrator();
				}

				/* If the aircraft is on ground reset the integrators */
				if (_vehicle_land_detected.landed || _vehicle_status.is_rotary_wing) {
					_roll_ctrl.reset_integrator();
					_pitch_ctrl.reset_integrator();
					_yaw_ctrl.reset_integrator();
					_wheel_ctrl.reset_integrator();
				}

				/* Prepare speed_body_u and speed_body_w */
				float speed_body_u = _R(0, 0) * _global_pos.vel_n + _R(1, 0) * _global_pos.vel_e + _R(2, 0) * _global_pos.vel_d;
				float speed_body_v = _R(0, 1) * _global_pos.vel_n + _R(1, 1) * _global_pos.vel_e + _R(2, 1) * _global_pos.vel_d;
				float speed_body_w = _R(0, 2) * _global_pos.vel_n + _R(1, 2) * _global_pos.vel_e + _R(2, 2) * _global_pos.vel_d;

				/* Prepare data for attitude controllers */
				struct ECL_ControlData control_input = {};
				control_input.roll = _roll;
				control_input.pitch = _pitch;
				control_input.yaw = _yaw;
				control_input.roll_rate = _ctrl_state.roll_rate;
				control_input.pitch_rate = _ctrl_state.pitch_rate;
				control_input.yaw_rate = _ctrl_state.yaw_rate;
				control_input.speed_body_u = speed_body_u;
				control_input.speed_body_v = speed_body_v;
				control_input.speed_body_w = speed_body_w;
				control_input.acc_body_x = _accel.x;
				control_input.acc_body_y = _accel.y;
				control_input.acc_body_z = _accel.z;
				control_input.roll_setpoint = roll_sp;
				control_input.pitch_setpoint = pitch_sp;
				control_input.yaw_setpoint = yaw_sp;
				control_input.airspeed_min = _parameters.airspeed_min;
				control_input.airspeed_max = _parameters.airspeed_max;
				control_input.airspeed = airspeed;
				control_input.scaler = airspeed_scaling;
				control_input.lock_integrator = lock_integrator;
				control_input.groundspeed = groundspeed;
				control_input.groundspeed_scaler = groundspeed_scaler;

				_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);

				/* Run attitude controllers */
				if (PX4_ISFINITE(roll_sp) && PX4_ISFINITE(pitch_sp)) {
					_roll_ctrl.control_attitude(control_input);
					_pitch_ctrl.control_attitude(control_input);
					_yaw_ctrl.control_attitude(control_input); //runs last, because is depending on output of roll and pitch attitude
					_wheel_ctrl.control_attitude(control_input);

					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_bodyrate(control_input);
					_actuators.control[0] = (PX4_ISFINITE(roll_u)) ? roll_u + _parameters.trim_roll : _parameters.trim_roll;

					if (!PX4_ISFINITE(roll_u)) {
						_roll_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("roll_u %.4f", (double)roll_u);
						}
					}

					float pitch_u = _pitch_ctrl.control_bodyrate(control_input);
					_actuators.control[1] = (PX4_ISFINITE(pitch_u)) ? pitch_u + _parameters.trim_pitch : _parameters.trim_pitch;

					if (!PX4_ISFINITE(pitch_u)) {
						_pitch_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("pitch_u %.4f, _yaw_ctrl.get_desired_rate() %.4f,"
							      " airspeed %.4f, airspeed_scaling %.4f,"
							      " roll_sp %.4f, pitch_sp %.4f,"
							      " _roll_ctrl.get_desired_rate() %.4f,"
							      " _pitch_ctrl.get_desired_rate() %.4f"
							      " att_sp.roll_body %.4f",
							      (double)pitch_u, (double)_yaw_ctrl.get_desired_rate(),
							      (double)airspeed, (double)airspeed_scaling,
							      (double)roll_sp, (double)pitch_sp,
							      (double)_roll_ctrl.get_desired_rate(),
							      (double)_pitch_ctrl.get_desired_rate(),
							      (double)_att_sp.roll_body);
						}
					}

					float yaw_u = 0.0f;

					if (_att_sp.fw_control_yaw == true) {
						yaw_u = _wheel_ctrl.control_bodyrate(control_input);
					}

					else {
						yaw_u = _yaw_ctrl.control_bodyrate(control_input);
					}

					_actuators.control[2] = (PX4_ISFINITE(yaw_u)) ? yaw_u + _parameters.trim_yaw : _parameters.trim_yaw;

					/* add in manual rudder control */
					_actuators.control[2] += yaw_manual;

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						_wheel_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("yaw_u %.4f", (double)yaw_u);
						}
					}

					/* throttle passed through if it is finite and if no engine failure was
					 * detected */
					_actuators.control[3] = (PX4_ISFINITE(throttle_sp) &&
								 !(_vehicle_status.engine_failure ||
								   _vehicle_status.engine_failure_cmd)) ?
								throttle_sp : 0.0f;

					if (!PX4_ISFINITE(throttle_sp)) {
						if (_debug && loop_counter % 10 == 0) {
							warnx("throttle_sp %.4f", (double)throttle_sp);
						}
					}

				} else {
					perf_count(_nonfinite_input_perf);

					if (_debug && loop_counter % 10 == 0) {
						warnx("Non-finite setpoint roll_sp: %.4f, pitch_sp %.4f", (double)roll_sp, (double)pitch_sp);
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_rate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_rate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_rate();

				_rates_sp.timestamp = hrt_absolute_time();

				if (_rate_sp_pub != nullptr) {
					/* publish the attitude rates setpoint */
					orb_publish(_rates_sp_id, _rate_sp_pub, &_rates_sp);

				} else if (_rates_sp_id) {
					/* advertise the attitude rates setpoint */
					_rate_sp_pub = orb_advertise(_rates_sp_id, &_rates_sp);
				}

			} else {
				/* manual/direct control */
				_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _parameters.man_roll_scale + _parameters.trim_roll;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale +
						_parameters.trim_pitch;
				_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}

			_actuators.control[actuator_controls_s::INDEX_FLAPS] = flaps_applied;
			_actuators.control[5] = _manual.aux1;
			_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = flaperon_applied;
			_actuators.control[7] = _manual.aux3;

#endif
			//custom	
			vehicle_status_poll();
			vehicle_accel_poll();
			vehicle_manual_poll();
			vehicle_control_mode_poll();

			if (_vehicle_status.rc_signal_lost == false){
				if ((int)_manual.aux1 == -1) {
					maneuver_type = 0;
					_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _parameters.man_roll_scale + _parameters.trim_roll;
					_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale + _parameters.trim_pitch;
					_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
				}
				else if((int)_manual.aux1 == 0) {
					maneuver_type = 1;
					aerobatics_control();
				}
				else if((int)_manual.aux1 == 1){
					//maneuver_type = _parameters.man_kind; /old way to test one maneuver at a time
					if(_manual.aux2 > 0.99f){maneuver_type = 2;} //Far clockwise: Hover
					else if(_manual.aux2 < -0.99f){maneuver_type = 4;} //Far counterclockwise: ATA
					else{maneuver_type = 5;} //Mid: Knife-edge
					aerobatics_control();
				}
			} else {
				_actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = 0.0f;
				_actuators.control[actuator_controls_s::INDEX_YAW] = 0.0f;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
			}

			_aerobatics_variables2.kpp = _parameters.Kpp;
		    _aerobatics_variables2.kpd = _parameters.Kpd;
		    _aerobatics_variables2.kpi = _parameters.Kpi;
		    _aerobatics_variables2.kap = _parameters.Kap;
		    _aerobatics_variables2.kad = _parameters.Kad;
		    _aerobatics_variables2.kai = _parameters.Kai;
		    _aerobatics_variables2.ku = _parameters.Ku;
		    _aerobatics_variables2.khp = _parameters.Khp;
		    _aerobatics_variables2.khd = _parameters.Khd;
		    _aerobatics_variables2.khi = _parameters.Khi;
		    _aerobatics_variables2.manuever_type = maneuver_type;
		    _aerobatics_variables2.semi_auto = _parameters.semi_auto;
		    _aerobatics_variables2.timestep = dt;






			if (_debug_pub != nullptr) {		
				orb_publish(ORB_ID(debug), _debug_pub, &_debug);		
			}		
			else{		
				_debug_pub = orb_advertise(ORB_ID(debug), &_debug);
			}

			if (_aerobatics_variables_pub != nullptr) {		
				orb_publish(ORB_ID(aerobatics_variables), _aerobatics_variables_pub, &_aerobatics_variables);		
			}		
			else{		
				_aerobatics_variables_pub = orb_advertise(ORB_ID(aerobatics_variables), &_aerobatics_variables);
			}
			if (_aerobatics_variables2_pub != nullptr) {		
				orb_publish(ORB_ID(aerobatics_variables2), _aerobatics_variables2_pub, &_aerobatics_variables2);		
			}		
			else{		
				_aerobatics_variables2_pub = orb_advertise(ORB_ID(aerobatics_variables2), &_aerobatics_variables2);
			}



			/* Vérifie que l'autopilot est armee avant d'envoyer la commande au thrust */
			if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _actuators.control[actuator_controls_s::INDEX_THROTTLE];
			} else {
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
			}

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _ctrl_state.timestamp;


			/* publish the actuator controls */
			if (_actuators_0_pub != nullptr) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}

			if (_actuators_2_pub != nullptr) {
				/* publish the actuator controls*/
				orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_airframe);

			} else {
				/* advertise and publish */
				_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_airframe);
			}
			maneuver_type_old = maneuver_type;
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_counter = 0;
}

int
Control::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("aerobatics_JL",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   10000,
					   (px4_main_t)&Control::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int aerobatics_JL_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: aerobatics_JL {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		_control::g_control = new Control;

		if (_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != _control::g_control->start()) {
			delete _control::g_control;
			_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (_control::g_control == nullptr || !_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (_control::g_control == nullptr || !_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete _control::g_control;
		_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");

	return 1;
}
