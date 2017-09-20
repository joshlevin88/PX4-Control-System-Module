/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file aerobatics_JL_params.c
 *
 * Parameters defined by control task
 *
 * @author Josh Levin <joshua.levin@mail.mcgill.ca>
 */

#include <systemlib/param/param.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Proportionnal gain for aileron
 *
 * @min 0.0
 * @max 3000.0
 * @decimal 2
 * @increment 100.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KP_A, 130.0f);

/**
 * Proportionnal gain for elevator
 *
 * @min 0.0
 * @max 3000.0
 * @decimal 2
 * @increment 20.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KP_E, 130.0f);

/**
 * Proportionnal gain for rudder
 *
 * @min 0.0
 * @max 3000.0
 * @decimal 2
 * @increment 50.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KP_R, 130.0f);

/**
 * Maneuver Kind
 *
 * Corresponds to the type of maneuver
 *
 * @min 0
 * @max 5
 * @value 0 manual control
 * @value 1 level flight
 * @value 2 ATA
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_INT32(MAN_KIND_JL, 5);

/**
 * Semi-Autonomous
 *
 * Semi-autonomous control modes
 *
 * @min 0
 * @max 15
 * @value 0 all manual
 * @value 1 auto ail
 * @value 2 auto elev
 * @value 3 auto rud
 * @value 4 auto thrust
 * @value 5 auto ail + elev
 * @value 6 auto ail + rud
 * @value 7 auto ail + thrust
 * @value 8 auto elev + rud
 * @value 9 auto elev + thrust
 * @value 10 auto rud + thrust 
 * @value 11 auto ail + elev + rud
 * @value 12 auto ail + elev + thrust
 * @value 13 auto ail + rud + thrust
 * @value 14 auto elev + rud + thrust
 * @value 15 auto ail + elev + rud + thrust
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_INT32(SEMI_AUTO_JL, 1);

/**
 * Proportional Attitude Gain
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 2
 * @increment 10.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KAP_JL, 130.0f);

/**
 * Derivative Attitude Gain
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 2
 * @increment 1.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KAD_JL, 8.0f);

/**
 * Integral Attitude Gain
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 2
 * @increment 1.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KAI_JL, 0.0f);

/**
 * Proportional Position Gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KPP_JL, 0.07f);

/**
 * Derivative Position Gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KPD_JL, 0.12f);

/**
 * Integral Position Gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KPI_JL, 0.00f);

/**
 * Proportional Height Gain
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 1.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KHP_JL, 15.0f);

/**
 * Derivative Height Gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 1.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KHD_JL, 3.0f);

/**
 * Integral Height Gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KHI_JL, 1.0f);

/**
 * Proportional Speed Gain
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 1.0
 * @reboot_required true
 * @group Aerobatics
 */
PARAM_DEFINE_FLOAT(KU_JL, 5.0f);

