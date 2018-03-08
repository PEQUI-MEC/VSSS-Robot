/**
 * Copyright (c) 2015 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */

#ifndef __IO_H_
#define __IO_H_

#define DR_PWM_MAX_VAL      0x3FF

namespace XBeeLib {

/**
 * @defgroup IoMode
 * @{
 */
/**
 * IoMode
 */
enum IoMode {
    Disabled         = 0,  /**< Disabled */
    SpecialFunc      = 1,  /**< Special Function */
    Adc              = 2,  /**< Adc */
    Pwm              = 2,  /**< Pwm */
    DigitalInput     = 3,  /**< Digital Input */
    DigitalOutLow    = 4,  /**< Digital Out Low */
    DigitalOutHigh   = 5,  /**< Digital Out High */
};
/**
 * @}
 */

/**
 * @defgroup DioVal
 * @{
 */
/**
 * DioVal
 */
enum DioVal {
    Low     = 0,      /**< Low Value */
    High    = 1,      /**< High Value */
};
/**
 * @}
 */

}   /* namespace XBeeLib */


#endif /* __IO_H_ */
