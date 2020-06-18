/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
/******************************************************************************/
/*                                Includes                                    */
/******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "transport_ble.h"
#include "cy_retarget_io.h"
#include <stdio.h>


/******************************************************************************/
/*                                 Defines                                    */
/******************************************************************************/


/******************************************************************************/
/*                           Function Prototypes                              */
/******************************************************************************/
void AppCallBack(uint32_t event, void* eventParam);


/******************************************************************************/
/*                       Interrupt Service Routines                           */
/******************************************************************************/


/******************************************************************************/
/*                            Global Variables                                */
/******************************************************************************/


/******************************************************************************/
/*                          Function Definitions                              */
/******************************************************************************/
/*******************************************************************************
* Function:     main
* Author:
* Description:    The entry point of the program.
* Date:         03-23-20
*******************************************************************************/
int main(void)
{
	cy_rslt_t           result;
	uint32_t            count;               // Used to count seconds
	cy_en_dfu_status_t  status;              // Status codes for DFU API
	uint32_t            state;               // DFU state
	const uint32_t      paramsTimeout = 20u; // Cy_DFU_Continue() timeout (ms)

	/* Buffer to store DFU commands */
	CY_ALIGN(4) static uint8_t buffer[CY_DFU_SIZEOF_DATA_BUFFER];

	/* Buffer for DFU data packets for transport API */
	CY_ALIGN(4) static uint8_t packet[CY_DFU_SIZEOF_CMD_BUFFER ];

	/* DFU params, used to configure DFU */
	cy_stc_dfu_params_t dfuParams;

	/* Initialize dfuParams structure */
	dfuParams.timeout          = paramsTimeout;
	dfuParams.dataBuffer       = &buffer[0];
	dfuParams.packetBuffer     = &packet[0];
	status = Cy_DFU_Init(&state, &dfuParams);

	/* Initialize the device and board peripherals */
	result = cybsp_init() ;
	if (result != CY_RSLT_SUCCESS)
	{
	    CY_ASSERT(0);
	}

	/* Initialize retarget-io to use the debug UART port */
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	if (result != CY_RSLT_SUCCESS)
	{
	    CY_ASSERT(0);
	}


	__enable_irq();

	/* Initialize DFU communication */
	Cy_DFU_TransportStart();

	count = 0;
	for(;;)
	{
		status = Cy_DFU_Continue(&state, &dfuParams);
		++count;
		if (state == CY_DFU_STATE_FINISHED)
		{
		   /* Finished loading the application image */
		   /* Validate DFU application, if it is valid then switch to it */
		   status = Cy_DFU_ValidateApp(1u, &dfuParams);
		   if (status == CY_DFU_SUCCESS)
		   {
		       Cy_DFU_TransportStop();
		       Cy_DFU_ExecuteApp(1u);
		   }
		   else if (status == CY_DFU_ERROR_VERIFY)
		   {
		       /*
		       * Restarts loading, an alternatives are to Halt MCU here
		       * or switch to the other app if it is valid.
		       * Error code may be handled here, i.e. print to debug UART.
		       */
		       status = Cy_DFU_Init(&state, &dfuParams);
		       Cy_DFU_TransportReset();
		   }
		}
		else if (state == CY_DFU_STATE_FAILED)
		{
		   /* An error has happened during the loading process */
		   /* Handle it here */
		   /* In this Code Example just restart loading process */
		   status = Cy_DFU_Init(&state, &dfuParams);
		   Cy_DFU_TransportReset();
		}
		else if (state == CY_DFU_STATE_UPDATING)
		{
		   uint32_t passed5seconds = (count >= (5000ul/paramsTimeout)) ? 1 : 0;
		   /*
		   * if no command has been received during 5 seconds when the loading
		   * has started then restart loading.
		   */
		   if (status == CY_DFU_SUCCESS)
		   {
		       count = 0u;
		   }
		   else if (status == CY_DFU_ERROR_TIMEOUT)
		   {
		       if (passed5seconds != 0u)
		       {
		           count = 0u;
		           Cy_DFU_Init(&state, &dfuParams);
		           Cy_DFU_TransportReset();
		       }
		   }
		   else
		   {
		       count = 0u;
		       /* Delay because Transport still may be sending error response to
		        * a host
		        */
		       Cy_SysLib_Delay(paramsTimeout);
		       Cy_DFU_Init(&state, &dfuParams);
		       Cy_DFU_TransportReset();
		   }
		}

		/* If SW2 pressed, switch to App1 if it is valid */
		if (Cy_GPIO_Read(PIN_SW2_PORT, PIN_SW2_PIN) == 0u)
		{
		   /* 50 ms delay for button debounce on button press */
		   Cy_SysLib_Delay(50u);
		   if (Cy_GPIO_Read(PIN_SW2_PORT, PIN_SW2_PIN) == 0u)
		   {
		       while (Cy_GPIO_Read(PIN_SW2_PORT, PIN_SW2_PIN) == 0u)
		       {   /* 50 ms delay for button debounce on button release */
		           Cy_SysLib_Delay(50u);
		       }
		       /* Validate and switch to App1 */
		       status = Cy_DFU_ValidateApp(1u, &dfuParams);
		       if (status == CY_DFU_SUCCESS)
		       {
		           Cy_DFU_TransportStop();
		           Cy_DFU_ExecuteApp(1u);
		       }
		   }
		}
	}
}


/*******************************************************************************
* Function:     Cy_OnResetUser
* Author:       Cypress Semiconductor
* Description:	  This function is called at the start of Reset_Handler(). DFU
*               requires it to call Cy_DFU_OnResetApp0() in app#0.
* Date:         05-20-19
*******************************************************************************/
void Cy_OnResetUser(void)
{
    Cy_DFU_OnResetApp0();
}


/*******************************************************************************
* Function:     AppCallBack
* Author:       Cypress Semiconductor (modified by Matt Mielke)
* Description:    This is an event callback function to receive events from the
*               BLE Component. Used in Cy_DFU_TransportStart()
* Date:         03-23-20
*******************************************************************************/
void AppCallBack(uint32_t event, void* eventParam)
{
    cy_en_ble_api_result_t apiResult;

    static cy_stc_ble_gap_sec_key_info_t keyInfo =
    {
        .localKeysFlag    = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST |
                            CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST |
                            CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST,
        .exchangeKeysFlag = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST |
                            CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST |
                            CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_ENC_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_IRK_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_CSRK_KEY_DIST,
    };

    switch(event)
    {
        /**********************************************************************
         * General events
         *********************************************************************/

        /* This event is received when the BLE stack is started */
        case CY_BLE_EVT_STACK_ON:
        {
            printf("[INFO] : BLE stack started\r\n");
            /* Enter into discoverable mode so that remote can search it. */
            apiResult = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, 0u);
            if(CY_BLE_SUCCESS != apiResult)
            {
                printf("[ERROR] : Failed to start advertisement 0x%X\r\n", apiResult);
            }

            apiResult = Cy_BLE_GAP_GenerateKeys(&keyInfo);
            if(apiResult != CY_BLE_SUCCESS)
            {
                printf("[Error] : Generate Keys API 0x%X\r\n", apiResult);
            }
            break;
        }

        /* This event indicates that some internal HW error has occurred. */
        case CY_BLE_EVT_HARDWARE_ERROR:
        {
            printf("[INFO] : Hardware Error\r\n");
            break;
        }

         /* This event is received when there is a timeout */
        case CY_BLE_EVT_TIMEOUT:
        {
            /* Reason for Timeout */
            cy_en_ble_to_reason_code_t reason_code = ((cy_stc_ble_timeout_param_t*)eventParam)->reasonCode;

            switch(reason_code)
            {
                case CY_BLE_GAP_ADV_TO:
                {
                    printf("[INFO] : Advertisement timeout event\r\n");
                    break;
                }
                case CY_BLE_GATT_RSP_TO:
                {
                    printf("[INFO] : GATT response timeout\r\n");
                    break;
                }
                default:
                {
                    printf("[INFO] : BLE timeout event\r\n");
                    break;
                }
            }
            break;
        }

        /* This event indicates completion of Set LE event mask */
        case CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE:
        {
            printf("[INFO] : Set LE mask event mask command completed\r\n");
            break;
        }

        /* This event indicates set device address command completed */
        case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
        {
            printf("[INFO] : Set device address command has completed\r\n");
            break;
        }

        /* This event indicates set Tx Power command completed */
        case CY_BLE_EVT_SET_TX_PWR_COMPLETE:
        {
            printf("[INFO] : Set Tx power command completed\r\n");
            break;
        }

        /**********************************************************************
         * GAP events
         *********************************************************************/

        case CY_BLE_EVT_GAP_AUTH_REQ:
        {
            printf("[INFO] : GAP authentication request\r\n");
            if (cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].security
                == (CY_BLE_GAP_SEC_MODE_1 | CY_BLE_GAP_SEC_LEVEL_1))
            {
               cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].authErr =
                   CY_BLE_GAP_AUTH_ERROR_PAIRING_NOT_SUPPORTED;
            }

            cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].bdHandle =
               ((cy_stc_ble_gap_auth_info_t *)eventParam)->bdHandle;

            apiResult = Cy_BLE_GAPP_AuthReqReply(&cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX]);
            if (apiResult != CY_BLE_SUCCESS)
            {
                printf("[ERROR] : Authentication Request Reply API 0x%X\r\n", apiResult);
            }
            break;
        }

        /* This event is triggered instead of 'CY_BLE_EVT_GAP_DEVICE_CONNECTED',
        * if Link Layer Privacy is enabled in component customizer
        */
        case CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE:
        {
            printf("[INFO] : GAP enhanced connection complete\r\n");
            /* sets the security keys that are to be exchanged with a peer
             * device during key exchange stage of the authentication procedure
             */
            keyInfo.SecKeyParam.bdHandle =
                (*(cy_stc_ble_gap_enhance_conn_complete_param_t *)eventParam).bdHandle;

            apiResult = Cy_BLE_GAP_SetSecurityKeys(&keyInfo);
            if (apiResult != CY_BLE_SUCCESS)
            {
                printf("[ERROR] : Set Security Keys API 0x%X\r\n", apiResult);
            }
            break;
        }

        /* This event indicates security key generation complete */
        case CY_BLE_EVT_GAP_KEYS_GEN_COMPLETE:
        {
            printf("[INFO] : GAP key generation complete\r\n");
            keyInfo.SecKeyParam = (*(cy_stc_ble_gap_sec_key_param_t *)eventParam);
            Cy_BLE_GAP_SetIdAddress(&cy_ble_deviceAddress);
            break;
        }

        /* This event indicates SMP has completed pairing feature exchange */
        case CY_BLE_EVT_GAP_SMP_NEGOTIATED_AUTH_INFO:
        {
            printf("[INFO] : Pairing feature exchange complete\r\n");
            break;
        }

        /* This event is generated when disconnected from remote device or
         * failed to establish connection
         */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
            printf("[INFO] : GAP device disconnected\r\n");
            /* Enter into discoverable mode so that remote can search it. */
            apiResult = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, 0u);
            if(CY_BLE_SUCCESS != apiResult)
            {
                printf("[ERROR] : Failed to start advertisement 0x%X\r\n", apiResult);
            }

            break;
        }

        case CY_BLE_EVT_GAP_KEYINFO_EXCHNGE_CMPLT:
        {
            printf("[INFO] : GAP Key Info Exchange complete\r\n");
            break;
        }

        /* This event indicates GAP authentication complete */
        case CY_BLE_EVT_GAP_AUTH_COMPLETE:
        {
            printf("[INFO] : GAP Authentication complete\r\n");
            break;
        }

        /* This event indicates authentication process between two devices has
         * failed */
        case CY_BLE_EVT_GAP_AUTH_FAILED:
        {
            printf("[INFO] : GAP authentication failed 0x%X\r\n",
                    ((cy_stc_ble_gap_auth_info_t*)eventParam)->authErr);
            break;
        }

        /* This event indicates encryption is changed for an active connection */
        case CY_BLE_EVT_GAP_ENCRYPT_CHANGE:
        {
            printf("[INFO] : GAP encryption change complete\r\n");
            break;
        }

        /* This event is generated after connection parameter update is
         * requested from the host to the controller
         */
        case CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE:
        {
            printf("[INFO] : GAP connection update complete\r\n");
            break;
        }

        /* This event indicates peripheral device has started/stopped
         *  advertising
         */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        {
            if(CY_BLE_ADV_STATE_ADVERTISING == Cy_BLE_GetAdvertisementState())
            {
                printf("[INFO] : BLE advertisement started\r\n");
            }
            else if (CY_BLE_ADV_STATE_STOPPED == Cy_BLE_GetAdvertisementState())
            {
                printf("[INFO] : BLE advertisement stopped\r\n");
            }
            break;
        }

        /**********************************************************************
         * GATT events
         *********************************************************************/

        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device
         */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            appConnHandle = *(cy_stc_ble_conn_handle_t *)eventParam;
            printf("[INFO] : GATT device connected\r\n");
            break;
        }

        /* This event is generated at the GAP Peripheral end after disconnection */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            printf("[INFO] : GATT device disconnected\r\n");
            break;
        }

        case CY_BLE_EVT_GATTS_WRITE_CMD_REQ:
        {
            printf("[INFO] : GATT write command request\r\n");
            break;
        }

        /* This event indicates that the 'GATT MTU Exchange Request' is received */
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
        {
            printf("[INFO] : GATT MTU Exchange Request received\r\n");
            break;
        }

        /* This event received when GATT read characteristic request received */
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
        {
            printf("[INFO] : GATT read characteristic request received for handle 0x%X\r\n",
                    (*(cy_stc_ble_gatts_char_val_read_req_t*)eventParam).attrHandle);
            break;
        }

        /***********************************************************************
        *                           Other Events                               *
        ***********************************************************************/
        default:
        {
            printf("[INFO] : BLE Event 0x%lX\r\n", (unsigned long) event);
        }
    }
}

/* [] END OF FILE */
