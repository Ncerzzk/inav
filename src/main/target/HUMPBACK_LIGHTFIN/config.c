/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "fc/config.h"
#include "io/serial.h"

static void updateUartTimerUsage(serialPortIdentifier_e portIdentifier, int firstTimerIndex)
{
    const int serialPortIndex = findSerialPortIndexByIdentifier(portIdentifier);
    const bool useAsDshot = serialPortIndex >= 0 && serialConfig()->portConfigs[serialPortIndex].functionMask == FUNCTION_NONE;

    timerHardware[firstTimerIndex].usageFlags = useAsDshot ? TIM_USE_OUTPUT_AUTO : TIM_USE_ANY;
    timerHardware[firstTimerIndex + 1].usageFlags = useAsDshot ? TIM_USE_OUTPUT_AUTO : TIM_USE_ANY;
}

void validateAndFixTargetConfig(void)
{
    /*
     * Apply PWM output inversion policy for this target.
     *
     * This board requires inverted PWM polarity for servo outputs on
     * certain timers. TMR1 and TMR2 (all channels except CH3) are
     * excluded because they need normal polarity regardless of use.
     *
     * For each timer channel:
     *  - Motor output → normal polarity (no inversion)
     *  - Servo / other output → inverted polarity
     *
     * Setting TIMER_OUTPUT_INVERTED directly in timerHardware[].output
     * avoids modifying shared pwm_output.c / timer driver code. The flag
     * is read by impl_timerPWMConfigChannel() when configuring the timer
     * output channel.
     */
    for (int i = 0; i < timerHardwareCount; i++) {
        // TMR1 and TMR2 (except CH3) always use normal polarity
        if (timerHardware[i].tim == TMR1) continue;
        if (timerHardware[i].tim == TMR2 && timerHardware[i].channelIndex != 3) continue;

        if(TIM_IS_MOTOR(timerHardware[i].usageFlags) && motorConfig()->motorPwmProtocol == PWM_TYPE_BRUSHED) continue;
        
        // Servo / non-brushed-motor outputs: inverted
        timerHardware[i].output |= TIMER_OUTPUT_INVERTED;

    }

    updateUartTimerUsage(SERIAL_PORT_USART1, 4);
    updateUartTimerUsage(SERIAL_PORT_USART5, 6);
}
