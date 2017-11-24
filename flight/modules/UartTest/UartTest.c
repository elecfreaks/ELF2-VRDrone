/**
 ******************************************************************************
 * @brief Uart test
 *
 * @file       UartTest.c
 * @author     Richile.
 * @brief      Uart test
 *
 *****************************************************************************/

#include <openpilot.h>

#include "hwsettings.h"
#include "taskinfo.h"

#include <stdbool.h>

// ****************
// Private functions

static void uartTestTask(void *parameters);
static void updateSettings(UAVObjEvent *ev);

// ****************
// Private constants

#define UARTTEST_STACK_SIZE_BYTES		64
#define TASK_PRIORITY        			(tskIDLE_PRIORITY + 1)
#define BRIDGE_BUF_LEN       			20

static xTaskHandle uartTestTaskHandle;

static uint32_t usart_port;
static uint8_t *uarttest_buf;

static bool bridge_enabled = false;

/**
 * Initialise the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */

static int32_t uartTestStart(void)
{
    if (bridge_enabled) {
        // Start tasks
        xTaskCreate(uartTestTask, "UartTest", UARTTEST_STACK_SIZE_BYTES, NULL, TASK_PRIORITY, &uartTestTaskHandle);
        return 0;
    }

    return -1;
}
/**
 * Initialise the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
static int32_t uartTestInitialize(void)
{
    usart_port     = PIOS_COM_BRIDGE;

#ifdef MODULE_COMUSBBRIDGE_BUILTIN
    bridge_enabled = true;
#else
    HwSettingsInitialize();
    HwSettingsOptionalModulesData optionalModules;

    HwSettingsOptionalModulesGet(&optionalModules);

    if (usart_port) {
        bridge_enabled = true;
    } else {
        bridge_enabled = false;
    }
#endif

    if (bridge_enabled) {
		uarttest_buf = pios_malloc(BRIDGE_BUF_LEN);
        HwSettingsConnectCallback(&updateSettings);
        updateSettings(0);
    }

    return 0;
}
MODULE_INITCALL(uartTestInitialize, uartTestStart);

/**
 * Main task. It does not return.
 */

static void uartTestTask(__attribute__((unused)) void *parameters)
{
    /* Handle usart -> vcp direction */
    volatile uint32_t tx_errors = 0;

    while (1) {
        uint32_t rx_bytes;

        rx_bytes = PIOS_COM_ReceiveBuffer(usart_port, uarttest_buf, BRIDGE_BUF_LEN, 500);
        if (rx_bytes > 0) {
            /* Bytes available to transfer */
            if (PIOS_COM_SendBuffer(usart_port, uarttest_buf, rx_bytes) != (int32_t)rx_bytes) {
                /* Error on transmit */
                tx_errors++;
            }
        }
    }
}

static void updateSettings(__attribute__((unused)) UAVObjEvent *ev)
{
    if (usart_port) {
        // Retrieve settings
        uint8_t speed;
        HwSettingsComUsbBridgeSpeedGet(&speed);

        // Set port speed
        switch (speed) {
        case HWSETTINGS_COMUSBBRIDGESPEED_2400:
            PIOS_COM_ChangeBaud(usart_port, 2400);
            break;
        case HWSETTINGS_COMUSBBRIDGESPEED_4800:
            PIOS_COM_ChangeBaud(usart_port, 4800);
            break;
        case HWSETTINGS_COMUSBBRIDGESPEED_9600:
            PIOS_COM_ChangeBaud(usart_port, 9600);
            break;
        case HWSETTINGS_COMUSBBRIDGESPEED_19200:
            PIOS_COM_ChangeBaud(usart_port, 19200);
            break;
        case HWSETTINGS_COMUSBBRIDGESPEED_38400:
            PIOS_COM_ChangeBaud(usart_port, 38400);
            break;
        case HWSETTINGS_COMUSBBRIDGESPEED_57600:
            PIOS_COM_ChangeBaud(usart_port, 57600);
            break;
        case HWSETTINGS_COMUSBBRIDGESPEED_115200:
            PIOS_COM_ChangeBaud(usart_port, 115200);
            break;
        }
    }
}

