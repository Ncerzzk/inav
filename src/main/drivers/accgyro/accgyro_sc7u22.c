/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_IMU_SC7U22)

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_sc7u22.h"

#define SC7U22_CHIP_ID                         0x6A

#define SC7U22_REG_WHO_AM_I                    0x01
#define SC7U22_REG_COM_CONF                    0x04
#define SC7U22_REG_ACC_XH                      0x0C
#define SC7U22_REG_SOFT_RST                    0x4A
#define SC7U22_REG_PWR_CTRL                    0x7D
#define SC7U22_REG_SEG_SEL                     0x7F
#define SC7U22_REG_ACC_CONF                    0x40
#define SC7U22_REG_ACC_RANGE                   0x41
#define SC7U22_REG_GYR_CONF                    0x42
#define SC7U22_REG_GYR_RANGE                   0x43

#define SC7U22_COM_CONF_BDU                    BIT(6)
#define SC7U22_COM_CONF_ADDR_AUTO              BIT(4)

#define SC7U22_PWR_CTRL_TEMP_EN                BIT(3)
#define SC7U22_PWR_CTRL_ACC_EN                 BIT(2)
#define SC7U22_PWR_CTRL_GYR_EN                 BIT(1)

#define SC7U22_ACC_FILTER_PERF                 BIT(7)
#define SC7U22_ACC_BWP_OSR4_AVG1               (0x00 << 4)
#define SC7U22_ACC_ODR_1600                    0x0C
#define SC7U22_ACC_RANGE_16G                   0x03

#define SC7U22_GYR_FILTER_PERF                 BIT(7)
#define SC7U22_GYR_BWP_OSR4_AVG1               (0x00 << 4)
#define SC7U22_GYR_BWP_OSR2_AVG2               (0x01 << 4)
#define SC7U22_GYR_BWP_NORM_AVG4               (0x02 << 4)
#define SC7U22_GYR_RANGE_2000DPS               0x00

#define SC7U22_SOFT_RESET_VALUE                0xA5
#define SC7U22_RESET_DELAY_MS                  200
#define SC7U22_SENSOR_START_DELAY_MS           60
#define SC7U22_CONFIG_SETTLE_DELAY_MS          2

#define SC7U22_DATA_LENGTH                     12
#define SC7U22_CONTEXT_MAGIC                   0x7A22

typedef struct __attribute__ ((__packed__)) sc7u22ContextData_s {
    uint16_t chipMagicNumber;
    uint8_t lastReadStatus;
    uint8_t __padding_dummy;
    uint8_t raw[SC7U22_DATA_LENGTH];
} sc7u22ContextData_t;

STATIC_ASSERT(sizeof(sc7u22ContextData_t) < BUS_SCRATCHPAD_MEMORY_SIZE, busDevice_scratchpad_memory_too_small);

static const gyroFilterAndRateConfig_t sc7u22GyroConfigs[] = {
    { GYRO_LPF_256HZ, 3200, { SC7U22_GYR_FILTER_PERF | SC7U22_GYR_BWP_OSR4_AVG1 | 0x0D } },
    { GYRO_LPF_256HZ, 1600, { SC7U22_GYR_FILTER_PERF | SC7U22_GYR_BWP_OSR4_AVG1 | 0x0C } },
    { GYRO_LPF_188HZ, 3200, { SC7U22_GYR_FILTER_PERF | SC7U22_GYR_BWP_OSR2_AVG2 | 0x0D } },
    { GYRO_LPF_98HZ,  3200, { SC7U22_GYR_FILTER_PERF | SC7U22_GYR_BWP_NORM_AVG4 | 0x0D } },
};

static int16_t sc7u22Int16BigEndian(const uint8_t *data, int index)
{
    return (int16_t)((data[index * 2] << 8) | data[index * 2 + 1]);
}

static void sc7u22WriteRegister(const busDevice_t *dev, uint8_t reg, uint8_t value, unsigned delayMs)
{
    busWrite(dev, reg, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static bool sc7u22DetectDevice(busDevice_t *dev)
{
    uint8_t chipId = 0;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);
    sc7u22WriteRegister(dev, SC7U22_REG_SEG_SEL, 0x00, 1);

    for (int attempts = 0; attempts < 5; attempts++) {
        if (busRead(dev, SC7U22_REG_WHO_AM_I, &chipId) && chipId == SC7U22_CHIP_ID) {
            return true;
        }
        delay(10);
    }

    return false;
}

static void sc7u22AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t *dev = gyro->busDev;
    const gyroFilterAndRateConfig_t *config = chooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs,
                                                               sc7u22GyroConfigs, ARRAYLEN(sc7u22GyroConfigs));
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    sc7u22WriteRegister(dev, SC7U22_REG_SEG_SEL, 0x00, 1);
    sc7u22WriteRegister(dev, SC7U22_REG_COM_CONF, SC7U22_COM_CONF_BDU | SC7U22_COM_CONF_ADDR_AUTO, 1);
    sc7u22WriteRegister(dev, SC7U22_REG_SOFT_RST, SC7U22_SOFT_RESET_VALUE, 1);
    sc7u22WriteRegister(dev, SC7U22_REG_SOFT_RST, SC7U22_SOFT_RESET_VALUE, SC7U22_RESET_DELAY_MS);

    sc7u22WriteRegister(dev, SC7U22_REG_SEG_SEL, 0x00, 1);
    sc7u22WriteRegister(dev, SC7U22_REG_COM_CONF, SC7U22_COM_CONF_BDU | SC7U22_COM_CONF_ADDR_AUTO, 1);

    sc7u22WriteRegister(dev, SC7U22_REG_PWR_CTRL, 0x00, 1);

    sc7u22WriteRegister(dev, SC7U22_REG_ACC_RANGE, SC7U22_ACC_RANGE_16G, 1);
    sc7u22WriteRegister(dev, SC7U22_REG_GYR_RANGE, SC7U22_GYR_RANGE_2000DPS, 1);
    sc7u22WriteRegister(dev, SC7U22_REG_ACC_CONF, SC7U22_ACC_FILTER_PERF | SC7U22_ACC_BWP_OSR4_AVG1 | SC7U22_ACC_ODR_1600, 1);
    sc7u22WriteRegister(dev, SC7U22_REG_GYR_CONF, config->gyroConfigValues[0], SC7U22_CONFIG_SETTLE_DELAY_MS);

    sc7u22WriteRegister(dev, SC7U22_REG_PWR_CTRL, SC7U22_PWR_CTRL_TEMP_EN | SC7U22_PWR_CTRL_ACC_EN | SC7U22_PWR_CTRL_GYR_EN,
                        SC7U22_SENSOR_START_DELAY_MS);

    busSetSpeed(dev, BUS_SPEED_STANDARD);
}

static bool sc7u22GyroReadScratchpad(gyroDev_t *gyro)
{
    sc7u22ContextData_t *ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->lastReadStatus = busReadBuf(gyro->busDev, SC7U22_REG_ACC_XH, ctx->raw, SC7U22_DATA_LENGTH);

    if (ctx->lastReadStatus) {
        gyro->gyroADCRaw[X] = (float)sc7u22Int16BigEndian(&ctx->raw[6], 0);
        gyro->gyroADCRaw[Y] = (float)sc7u22Int16BigEndian(&ctx->raw[6], 1);
        gyro->gyroADCRaw[Z] = (float)sc7u22Int16BigEndian(&ctx->raw[6], 2);
        return true;
    }

    return false;
}

static bool sc7u22AccReadScratchpad(accDev_t *acc)
{
    sc7u22ContextData_t *ctx = busDeviceGetScratchpadMemory(acc->busDev);

    if (ctx->lastReadStatus) {
        acc->ADCRaw[X] = (float)sc7u22Int16BigEndian(ctx->raw, 0);
        acc->ADCRaw[Y] = (float)sc7u22Int16BigEndian(ctx->raw, 1);
        acc->ADCRaw[Z] = (float)sc7u22Int16BigEndian(ctx->raw, 2);
        return true;
    }

    return false;
}

static void sc7u22GyroInit(gyroDev_t *gyro)
{
    sc7u22AccAndGyroInit(gyro);
}

static void sc7u22AccInit(accDev_t *acc)
{
    acc->acc_1G = 2048;
}

bool sc7u22GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_SC7U22, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!sc7u22DetectDevice(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    sc7u22ContextData_t *ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = SC7U22_CONTEXT_MAGIC;

    gyro->initFn = sc7u22GyroInit;
    gyro->readFn = sc7u22GyroReadScratchpad;
    gyro->intStatusFn = gyroCheckDataReady;
    gyro->scale = 1.0f / 16.384f; // 2000 dps, 61.035 mdps/LSB
    gyro->gyroAlign = gyro->busDev->param;
    return true;
}

bool sc7u22AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_SC7U22, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    sc7u22ContextData_t *ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != SC7U22_CONTEXT_MAGIC) {
        return false;
    }

    acc->initFn = sc7u22AccInit;
    acc->readFn = sc7u22AccReadScratchpad;
    acc->accAlign = acc->busDev->param;
    return true;
}

#endif // USE_IMU_SC7U22
