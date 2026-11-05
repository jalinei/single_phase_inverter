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

#include "singlePhaseInverterForming.h"
#include <math.h>
#include <errno.h>
#include "inverterValidation.h"

int8_t singlePhaseInverterForming::init(float32_t V_bus, float32_t grid_w0, float32_t Ts) {
    _initialized = false;
    _fault_reason = VSI_FAULT_NONE;
    if (!inverter_validation::positive(V_bus) ||
        !inverter_validation::timing(grid_w0, Ts)) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return -EINVAL;
    }

    _w = grid_w0;
    _w_ref = grid_w0;
    _Ts = Ts;
    _theta = 0;
    _V_bus = V_bus;

    _sogi_v.init(500.0F, _Ts);
    _sogi_i.init(500.0F, _Ts);

    _current_pi_params.Ts = _Ts;
    _current_pi_params.Kp = 0.001;
    _current_pi_params.Ti = 0.001/3000;
    _current_pi_params.Td = 0.0;
    _current_pi_params.N = 1.0;
    _current_pi_params.upper_bound = 30;
    _current_pi_params.lower_bound = -30;
    const int8_t current_d_result = _current_d_pi.init(_current_pi_params);
    if (current_d_result != 0) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return current_d_result;
    }
    const int8_t current_q_result = _current_q_pi.init(_current_pi_params);
    if (current_q_result != 0) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return current_q_result;
    }
    _current_d_pi.reset();
    _current_q_pi.reset();
    _voltage_pi_params.Ts = _Ts;
    _voltage_pi_params.Kp = 0.01;
    _voltage_pi_params.Ti = 0.003;
    _voltage_pi_params.Td = 0.0;
    _voltage_pi_params.N = 1.0;
    _voltage_pi_params.upper_bound = 30;
    _voltage_pi_params.lower_bound = -30;

    const int8_t voltage_d_result = _voltage_d_pi.init(_voltage_pi_params);
    if (voltage_d_result != 0) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return voltage_d_result;
    }
    const int8_t voltage_q_result = _voltage_q_pi.init(_voltage_pi_params);
    if (voltage_q_result != 0) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return voltage_q_result;
    }
    _voltage_d_pi.reset();
    _voltage_q_pi.reset();

    _Vdq = {};
    _Idq = {};
    _Vdq_output = {};
    _Vab = {};
    _Iab = {};
    _Vab_output = {};
    _power = {};
    _Vond = 0.0F;
    _duty_cycle = 0.5F;
    _Idq_ref = {};
    _Vdq_ref = {};
    _Idq_ref_delta = {};
    _fault_reason = VSI_FAULT_NONE;
    _initialized = true;
    return 0;
}

float32_t singlePhaseInverterForming::calculateDuty(float32_t vgrid_meas, float32_t igrid_meas) {
    if (!_initialized) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return _duty_cycle;
    }

    if (!isfinite(vgrid_meas) || !isfinite(igrid_meas)) {
        tripFault(VSI_FAULT_INVALID_MEASUREMENT);
        return _duty_cycle;
    }
    if (!isfinite(_V_bus) || _V_bus <= 0.0F) {
        tripFault(VSI_FAULT_INVALID_BUS_VOLTAGE);
        return _duty_cycle;
    }

    _w = _w_ref;
    const float32_t next_theta = _theta + _w * _Ts;
    if (!isfinite(_w) || !isfinite(next_theta)) {
        tripFault(VSI_FAULT_INVALID_CONTROL_STATE);
        return _duty_cycle;
    }
    // Floating-point wrapping avoids integer conversion overflow for large angles.
    _theta = fmodf(next_theta, 2.0F * PI);
    if (_theta < 0.0F) _theta += 2.0F * PI;

    _Vab = _sogi_v.calc(vgrid_meas, _w_ref);
    _Iab = _sogi_i.calc(igrid_meas, _w_ref);

    _Vdq = Transform::rotation_to_dqo(_Vab, _theta);
    _Idq = Transform::rotation_to_dqo(_Iab, _theta);
    if (!isfinite(_w) || !isfinite(_Vdq.d) || !isfinite(_Vdq.q) ||
        !isfinite(_Idq.d) || !isfinite(_Idq.q)) {
        tripFault(VSI_FAULT_INVALID_CONTROL_STATE);
        return _duty_cycle;
    }

    if (_fault_reason == VSI_FAULT_NONE) {

        if (_Vdq_ref.d <= 0.0F) {
            _voltage_d_pi.reset();
            _voltage_q_pi.reset();
            _current_d_pi.reset();
            _current_q_pi.reset();
            _Idq_ref_delta.d = 0.0F;
            _Idq_ref_delta.q = 0.0F;
            _Vdq_output.d = 0.0F;
            _Vdq_output.q = 0.0F;
        } else {
            _Idq_ref_delta.d = _voltage_d_pi.calculateWithReturn(_Vdq_ref.d, _Vdq.d);
            _Idq_ref_delta.q = _voltage_q_pi.calculateWithReturn(_Vdq_ref.q, _Vdq.q);

            _Vdq_output.d = _current_d_pi.calculateWithReturn(_Idq_ref.d + _Idq_ref_delta.d, _Idq.d);
            _Vdq_output.q = _current_q_pi.calculateWithReturn(_Idq_ref.q + _Idq_ref_delta.q, _Idq.q);

            _Vdq_output.d = _Vdq_output.d + _Vdq_ref.d;
            _Vdq_output.q = _Vdq_output.q + _Vdq_ref.q;
        }

    }
    if (_fault_reason != VSI_FAULT_NONE) {
        _Vdq_output = {};
    }

    _Vab_output = Transform::rotation_to_clarke(_Vdq_output, _theta);

    _Vond = _Vab_output.alpha;
    _duty_cycle = _Vond / (2.0F * _V_bus) + 0.5F;

    // Calculate active and reactive power
    _power.d = 0.5F * (_Vdq.d * _Idq.d + _Vdq.q * _Idq.q);
    _power.q = 0.5F * (_Idq.d * _Vdq.q - _Idq.q * _Vdq.d);

    if (!isfinite(_Vdq_output.d) || !isfinite(_Vdq_output.q) ||
        !isfinite(_Vab_output.alpha) || !isfinite(_Vab_output.beta) ||
        !isfinite(_duty_cycle) || !isfinite(_power.d) || !isfinite(_power.q)) {
        tripFault(VSI_FAULT_INVALID_CONTROL_STATE);
    }
    return _duty_cycle;

}
dqo_t singlePhaseInverterForming::getVdq() {
    return _Vdq;
}

dqo_t singlePhaseInverterForming::getVdqOut() {
    return _Vdq_output;
}

dqo_t singlePhaseInverterForming::getIdq() {
    return _Idq;
}

dqo_t singlePhaseInverterForming::getIdqRefDelta() {
    return _Idq_ref_delta;
}

dqo_t singlePhaseInverterForming::getPower() {
    return _power;
}

clarke_t singlePhaseInverterForming::getIab() {
    return _Iab;
}

clarke_t singlePhaseInverterForming::getVab() {
    return _Vab;
}

clarke_t singlePhaseInverterForming::getVabOutput() {
    return _Vab_output;
}

float32_t singlePhaseInverterForming::getTheta() {
    return _theta;
}

float32_t singlePhaseInverterForming::getw() {
    return _w;
}

void singlePhaseInverterForming::setVBus(float32_t V_bus) {
    _V_bus = V_bus;
}

void singlePhaseInverterForming::setIdqRef(dqo_t Idq_ref) {
    if (!isfinite(Idq_ref.d) || !isfinite(Idq_ref.q)) {
        tripFault(VSI_FAULT_INVALID_REFERENCE);
        return;
    }
    _Idq_ref = {Idq_ref.d, Idq_ref.q, 0.0F};
}

void singlePhaseInverterForming::setVdqRef(dqo_t Vdq_ref) {
    if (!isfinite(Vdq_ref.d) || !isfinite(Vdq_ref.q)) {
        tripFault(VSI_FAULT_INVALID_REFERENCE);
        return;
    }
    _Vdq_ref = {Vdq_ref.d, Vdq_ref.q, 0.0F};
}

void singlePhaseInverterForming::setWRef(float32_t w_ref) {
    if (!_initialized || !inverter_validation::timing(w_ref, _Ts)) {
        tripFault(VSI_FAULT_INVALID_REFERENCE);
        return;
    }
    _w_ref = w_ref;
}

bool singlePhaseInverterForming::isPwmRequested() const {
    return _fault_reason == VSI_FAULT_NONE && _Vdq_ref.d > 0.0F;
}

vsi_fault_reason_t singlePhaseInverterForming::getFaultReason() const { return _fault_reason; }

void singlePhaseInverterForming::tripFault(vsi_fault_reason_t reason) {
    if (_fault_reason != VSI_FAULT_NONE) return;
    _fault_reason = reason == VSI_FAULT_NONE ? VSI_FAULT_EXTERNAL : reason;
    if (_initialized) {
        _voltage_d_pi.reset();
        _voltage_q_pi.reset();
        _current_d_pi.reset();
        _current_q_pi.reset();
        if (reason == VSI_FAULT_INVALID_CONTROL_STATE) {
            // Discard poisoned filter/PLL history so recovery can observe a healthy grid.
            _sogi_v.init(500.0F, _Ts);
            _sogi_i.init(500.0F, _Ts);
            _theta = 0.0F;
            _Vdq = {};
        }
    }
    _Vdq_output = {};
    _Vab_output = {};
    _duty_cycle = 0.5F;
}

bool singlePhaseInverterForming::clearFault() {
    if (!_initialized || !inverter_validation::positive(_V_bus)) return false;

    _fault_reason = VSI_FAULT_NONE;
    return true;
}
