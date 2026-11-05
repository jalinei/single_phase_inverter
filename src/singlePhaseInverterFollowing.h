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
 * @class singlePhaseInverterFollowing
 * @brief Control a single-phase VSI using SOGI quadrature signals and dq PI loops.
 *
 * Uses a PLL to track grid phase, then applies current control with
 * measured-voltage feedforward through an explicit operating state machine.
 *
 * @pre Call init() before using the controller. Call calculateDuty() once per
 * sampling period, then read its cached measurements and commands as needed.
 * @note This class calculates references only; the caller configures and enables
 * hardware PWM, selects modulation, applies duty limits, and supplies valid measurements.
 * Only d and q components of dqo_t quantities are used or computed here; the
 * zero-sequence component is not maintained.
 */
class singlePhaseInverterFollowing {
public:
    /**
     * @brief Construct a controller with angular frequency initialized to zero.
     * @note Call init() before using any other operation.
     */
    singlePhaseInverterFollowing();

    /**
     * @brief Configure filters and PI loops and initialize control references.
     *
     * Resets phase, synchronization, PI states, and current references.
     * Power injection is initially disabled.
     *
     * @param V_bus Positive DC-bus voltage in volts, used for duty scaling.
     * @param grid_Vpk Positive nominal grid peak voltage in volts, used for PLL tuning.
     * @param grid_w0 Positive nominal grid angular frequency in radians per second.
     * @param Ts Positive control sampling period in seconds.
     * @return 0 on success, -EINVAL for invalid parameters, or a PI initialization error.
     * Invalid setup latches INVALID_CONFIGURATION and requires a successful init().
     * Frequency must be below the sampling Nyquist limit (grid_w0 * Ts < pi).
     * @note Starts in VSI_OFF with connection feedback and all requests cleared.
     */
    int8_t init(float32_t V_bus, float32_t grid_Vpk, float32_t grid_w0, float32_t Ts);

    /**
     * @brief Advance phase estimation and control by one sampling period.
     *
     * Updates SOGI signals, dq measurements, current control, and power.
     * FOLLOWING qualifies positive d voltage within 10% of the nominal peak,
     * frequency within 5% of its reference, and |Vq| < 0.1 V for 1000 consecutive
     * calls before allowing a requested connection. requestSync() holds this
     * stage without requesting PWM or relay closure. Connection/opening feedback has a
     * one-second timeout. These are control defaults, not grid protection limits.
     *
     * @param vgrid_meas Instantaneous measured grid voltage in volts.
     * @param igrid_meas Instantaneous measured grid current in amperes.
     * @return Leg-1 duty ratio: output alpha voltage / (2 * V_bus) + 0.5.
     * The result is not clamped to [0, 1]. Pass it to the hardware API that
     * updates both bridge legs according to the selected modulation scheme.
     * @pre init() has been called and the stored bus voltage is positive.
     * @note VSI_RUN uses the supplied current reference without a ramp or clamp.
     * VSI_GRID_CONNECTED and VSI_STOPPING regulate both axes to zero.
     * PLL estimation continues in OFF; hardware actions remain with the caller.
     */
    float32_t calculateDuty(float32_t vgrid_meas, float32_t igrid_meas);

    /** @brief Read the latest measured dq grid voltage. @return d/q voltage in volts. */
    dqo_t getVdq();

    /** @brief Read the latest dq voltage command. @return d/q voltage in volts. */
    dqo_t getVdqOut();

    /** @brief Read the SOGI current signals. @return Alpha/beta current in amperes. */
    clarke_t getIab();

    /** @brief Read the SOGI voltage signals. @return Alpha/beta voltage in volts. */
    clarke_t getVab();

    /** @brief Read the bridge output voltage command. @return Alpha/beta voltage in volts. */
    clarke_t getVabOutput();

    /** @brief Read the latest measured dq grid current. @return d/q current in amperes. */
    dqo_t getIdq();

    /**
     * @brief Read power computed during the latest control step.
     * @return d = active power P in watts; q = reactive power Q in vars.
     * P = 0.5 * (Vd * Id + Vq * Iq); Q = 0.5 * (Id * Vq - Iq * Vd).
     */
    dqo_t getPower();

    /**
     * @brief Read the angular frequency used for the latest phase update.
     * @return PLL frequency, in rad/s.
     */
    float32_t getw();

    /** @brief Read the latest electrical phase. @return Wrapped phase in radians. */
    float32_t getTheta();

    /**
     * @brief Check whether FOLLOWING has qualified synchronization.
     * @return True after consecutive qualification in VOLTAGE_SYNC, including
     * sync-only operation, or while aligned in GRID_CONNECT, GRID_CONNECTED, or RUN.
     * False in OFF, STOPPING, DISCONNECT, FAULT, or when current alignment is lost.
     */
    bool getSync();

    /** @brief Read the operating mode. @return Current mode. */
    vsi_mode_t getMode() const;

    /** @brief Read the first latched fault cause. @return NONE when no fault is latched. */
    vsi_fault_reason_t getFaultReason() const;

    /**
     * @brief Supply the quality verdict of an external grid observer.
     * @param good True for acceptable grid quality; false for unacceptable quality.
     * @note In FOLLOWING, false blocks progression from synchronization to connection.
     * In CONNECT, CONNECTED, RUN, STOPPING, or DISCONNECT it latches BAD_GRID_QUALITY
     * and drops PWM/relay requests, without waiting for current to decrease.
     * True does not clear a latched fault.
     * Defaults to true after init() for operation without an observer; the application
     * must supply an initial verdict and refresh it when using external monitoring.
     * Call from the control task; this method does not synchronize concurrent access.
     */
    void setGridQuality(bool good);

    /**
     * @brief Start or hold voltage synchronization without requesting PWM or connection.
     * @note Cancels RUN. If already connecting or connected, disconnects before
     * synchronizing again. Call requestConnect() to allow connection afterwards.
     * Consumed by calculateDuty(); ignored while faulted.
     */
    void requestSync();

    /**
     * @brief Request synchronization and grid connection in FOLLOWING mode.
     * @note Once connected, target zero current unless requestRun() was also issued.
     * Consumed by calculateDuty(); ignored while faulted.
     */
    void requestConnect();

    /**
     * @brief Request tracking of the supplied dq current reference in FOLLOWING.
     * @note Call requestConnect() separately to initiate connection. RUN begins
     * only after synchronization and confirmed connection. The reference is
     * applied directly, without an internal ramp or reference clamp.
     * Ignored while faulted.
     */
    void requestRun();

    /**
     * @brief Request zero current on both axes while keeping the grid connection.
     * @note Cancels any pending RUN request; does not disconnect or disable PWM.
     * Consumed by calculateDuty(); ignored while faulted.
     */
    void requestStop();

    /**
     * @brief Request orderly grid disconnection and return to OFF in FOLLOWING.
     * @note Enters STOPPING to target zero current with PWM and relay enabled, then
     * DISCONNECT to stop PWM, request relay opening, and wait for open feedback.
     * The separate RUN request is retained; use requestStop() to cancel it.
     * Consumed by calculateDuty(); ignored while faulted.
     */
    void requestDisconnect();

    /**
     * @brief Supply application-confirmed connection state.
     * @param connected True when the grid connection is established, false when open.
     * @note Does not command hardware. With no relay, the application may use PWM
     * enable/disable acknowledgement as its logical connection feedback.
     */
    void setGridConnected(bool connected);

    /**
     * @brief Latch FAULT immediately and drop PWM/relay requests without waiting for zero current.
     * @param reason Fault cause; defaults to EXTERNAL. NONE is treated as EXTERNAL.
     * @note Preserves the first fault cause until cleared. Hardware actions remain
     * with the application; no relay opening or PWM stopping is performed here.
     */
    void tripFault(vsi_fault_reason_t reason = VSI_FAULT_EXTERNAL);

    /**
     * @brief Clear FAULT only after the application confirms disconnection.
     * @return True if no fault remains; false if connection feedback is still true
     * or the external grid-quality verdict is bad in FOLLOWING.
     * @note Invalid initialization cannot be cleared: call init() successfully.
     * A cleared controller stays OFF until explicitly enabled again.
     */
    bool clearFault();

    /**
     * @brief Query the grid connection request for the application.
     * @return True in CONNECT, CONNECTED, RUN, or STOPPING; false in DISCONNECT.
     */
    bool isGridConnectionRequested() const;

    /**
     * @brief Query the PWM enable request for the application.
     * @return The grid connection request.
     */
    bool isPwmRequested() const;

    /**
     * @brief Update the voltage used to normalize the bridge duty command.
     * @param V_bus Positive DC-bus voltage in volts; no validation is performed.
     */
    void setVBus(float32_t V_bus);

    /**
     * @brief Set the current reference without clamping.
     * @param Idq_ref Finite d/q current reference in amperes, used directly in RUN.
     * Nonfinite d/q values latch INVALID_REFERENCE; valid references are not clamped.
     */
    void setIdqRef(dqo_t Idq_ref);

    /**
     * @brief Restart synchronization, disconnecting first if connection is active.
     * @note Retains enable/run requests. Does not clear a fault.
     */
    void setSyncOff();

    /**
     * @brief Update the frequency reference used by phase control and both SOGIs.
     * @param w_ref Positive finite reference frequency in radians per second.
     * Must satisfy w_ref * Ts < pi; invalid values latch INVALID_REFERENCE
     * and leave the previous reference unchanged.
     * @note Does not retune the PLL gains or limits established by init().
     */
    void setWRef(float32_t w_ref);

private:
    bool synchronizationAligned() const;

    // Internal state variables

    bool _initialized = false; ///< True only after all controller initialization succeeds.
    dqo_t _power;

    dqo_t _Vdq;
    dqo_t _Vdq_output;

    dqo_t _Idq;
    dqo_t _Idq_ref;

    clarke_t _Vab;
    clarke_t _Vab_output;
    clarke_t _Iab;
    float32_t _Vond;

    float32_t _grid_Vpk;
    float32_t _Ts;

    Sogi _sogi_i;
    Sogi _sogi_v;

    PidParams _pll_pi_params;    ///< PI controller parameters.
    Pid _pll_pi;                 ///< PI controller.

    PidParams _current_pi_params;    ///< PI controller parameters for the current loop.
    Pid _current_d_pi;                 ///< PI controller for the current loop.
    Pid _current_q_pi;                 ///< PI controller for the current loop.

    float32_t _theta;        ///< Current phase angle.
    float32_t _w;            ///< Angular frequency.
    float32_t _w_ref;        ///< Reference angular frequency.

    float32_t _duty_cycle;            ///< Internal duty cycle.

    float32_t _V_bus;   ///< Internal bus voltage.

    void updateMode();
    void enterMode(vsi_mode_t mode);

    vsi_mode_t _mode = VSI_OFF;
    vsi_fault_reason_t _fault_reason = VSI_FAULT_NONE;
    bool _grid_quality_good = true;
    bool _enabled = false;
    bool _connect_requested = false;
    bool _run_requested = false;
    bool _grid_connected = false;
    bool _resync_requested = false;
    float32_t _state_elapsed = 0.0F;
    const float32_t _connection_timeout = 1.0F; ///< Connection/disconnection timeout (s).
    const float32_t _sync_voltage_tolerance = 0.1F; ///< q-axis voltage tolerance (V).
    const float32_t _disconnect_current_tolerance = 0.1F; ///< Both dq axes (A).
    uint16_t _sync_delay_counter = 0;
    const uint16_t _sync_min_delay = 1000;
};
