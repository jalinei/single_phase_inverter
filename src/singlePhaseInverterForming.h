/*
 *
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#pragma once

#include "sogi.h"
#include "transform.h"
#include "filters.h"
#include "inverterTypes.h"

/**
 * @brief Grid-forming VSI with a reference oscillator and cascaded dq voltage/current loops.
 * Call init() before calculateDuty(), once per sampling period. Hardware PWM and
 * duty limiting remain the caller's responsibility. Faults latch until clearFault().
 */
class singlePhaseInverterForming {
public:
    /** @brief Initialize with positive finite bus voltage (V), angular frequency (rad/s), and period (s).
     * @return 0 on success, -EINVAL for invalid setup, or a PI initialization error.
     * Requires grid_w0 * Ts < pi. Failed setup latches INVALID_CONFIGURATION
     * and requires a successful init() before control or fault clearing.
     */
    int8_t init(float32_t V_bus, float32_t grid_w0, float32_t Ts);
    /** @brief Return unclamped leg duty: alpha output voltage / (2 * V_bus) + 0.5. */
    float32_t calculateDuty(float32_t vgrid_meas, float32_t igrid_meas);
    dqo_t getVdq();
    dqo_t getVdqOut();
    dqo_t getIdq();
    /** @brief Voltage-loop correction added to the supplied current reference (A). */
    dqo_t getIdqRefDelta();
    /** @brief Latest active power (d, W) and reactive power (q, var). */
    dqo_t getPower();
    clarke_t getIab();
    clarke_t getVab();
    clarke_t getVabOutput();
    float32_t getTheta();
    float32_t getw();
    void setVBus(float32_t V_bus);
    /** @brief Set finite dq current reference (A); nonfinite values latch INVALID_REFERENCE. */
    void setIdqRef(dqo_t Idq_ref);
    /** @brief Set finite dq voltage (V); nonpositive d resets loops on the next control step.
     * Nonfinite d/q values latch INVALID_REFERENCE. Valid references are not clamped.
     */
    void setVdqRef(dqo_t Vdq_ref);
    /** @brief Set positive finite frequency (rad/s) with w_ref * Ts < pi.
     * Invalid values latch INVALID_REFERENCE and retain the previous reference.
     */
    void setWRef(float32_t w_ref);
    bool isPwmRequested() const;
    vsi_fault_reason_t getFaultReason() const;
    void tripFault(vsi_fault_reason_t reason = VSI_FAULT_EXTERNAL);
    /** @brief Clear the fault latch; the voltage reference is retained. */
    bool clearFault();

private:
    bool _initialized = false; ///< True only after all controller initialization succeeds.
    dqo_t _power;

    dqo_t _Vdq;
    dqo_t _Vdq_ref;
    dqo_t _Vdq_output;

    dqo_t _Idq;
    dqo_t _Idq_ref;
    dqo_t _Idq_ref_delta;

    clarke_t _Vab;
    clarke_t _Vab_output;
    clarke_t _Iab;
    float32_t _Vond;

    float32_t _Ts;

    Sogi _sogi_i;
    Sogi _sogi_v;

    PidParams _current_pi_params;    ///< PI controller parameters for the current loop.
    Pid _current_d_pi;                 ///< PI controller for the current loop.
    Pid _current_q_pi;                 ///< PI controller for the current loop.

    PidParams _voltage_pi_params;    ///< PI controller parameters for the voltage loop.
    Pid _voltage_d_pi;                 ///< PI controller for the voltage loop.
    Pid _voltage_q_pi;                 ///< PI controller for the voltage loop.

    float32_t _theta;        ///< Current phase angle.
    float32_t _w;            ///< Angular frequency.
    float32_t _w_ref;        ///< Reference angular frequency.

    float32_t _duty_cycle;            ///< Internal duty cycle.

    float32_t _V_bus;   ///< Internal bus voltage.

    vsi_fault_reason_t _fault_reason = VSI_FAULT_NONE;
};
