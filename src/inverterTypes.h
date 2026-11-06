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

/** @brief Operating modes of the grid-following controller. */
typedef enum {
    VSI_OFF,            ///< PWM and grid connection are not requested.
    VSI_VOLTAGE_SYNC,   ///< Qualify grid voltage, frequency, and PLL alignment.
    VSI_GRID_CONNECT,   ///< Request connection and wait for application feedback.
    VSI_GRID_CONNECTED, ///< Regulate both current axes to zero.
    VSI_RUN,            ///< Apply the supplied dq current reference directly.
    VSI_STOPPING,       ///< Keep PWM/relay enabled while regulating current to zero.
    VSI_DISCONNECT,     ///< PWM off; request relay opening and wait for open feedback.
    VSI_FAULT,         ///< Latched shutdown; requires explicit clearFault().
    VSI_PWM_SETTLE     ///< Match grid with PWM on for ten periods; relay remains open.
} vsi_mode_t;

/** @brief First cause of a latched VSI_FAULT; cleared by clearFault() or init(). */
typedef enum {
    VSI_FAULT_NONE,
    VSI_FAULT_BAD_GRID_QUALITY,       ///< External grid observer reported unacceptable quality.
    VSI_FAULT_CONNECTION_TIMEOUT,   ///< Relay did not confirm connection in time.
    VSI_FAULT_DISCONNECTION_TIMEOUT, ///< Disconnect sequence did not finish in time.
    VSI_FAULT_GRID_CONNECTION_LOST,  ///< Established relay connection feedback was lost.
    VSI_FAULT_INVALID_MEASUREMENT,   ///< Nonfinite input voltage or current measurement.
    VSI_FAULT_INVALID_BUS_VOLTAGE,   ///< Nonfinite or nonpositive DC-bus voltage.
    VSI_FAULT_INVALID_CONTROL_STATE, ///< Nonfinite estimated frequency or dq measurement.
    VSI_FAULT_PWM_UPDATE_FAILED,     ///< Application could not apply the PWM command.
    VSI_FAULT_EXTERNAL,             ///< Application-reported fault without a specific cause.
    VSI_FAULT_INVALID_CONFIGURATION, ///< Invalid initialization or failed PI setup.
    VSI_FAULT_INVALID_REFERENCE,     ///< Nonfinite dq reference or invalid frequency reference.
    VSI_FAULT_SYNC_LOST              ///< PLL alignment lost during PWM settling or while awaiting relay closure.
} vsi_fault_reason_t;
