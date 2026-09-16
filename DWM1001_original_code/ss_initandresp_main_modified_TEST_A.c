/*! ----------------------------------------------------------------------------

*  @file    ss_init_main.c

*  @brief   Peer-to-Peer UWB Ranging Node (Combined Initiator/Responder)

*/

#include <stdio.h>

#include <string.h>

#include "FreeRTOS.h"

#include "task.h"

#include "deca_device_api.h"

#include "deca_regs.h"

#include "port_platform.h"

#include "UART.h"
 
#define APP_NAME "SS TWR P2P v1.1"
 
/* ===================================================================== */

/* PEER-TO-PEER NETWORK CONFIGURATION                                    */

/* ===================================================================== */

extern volatile uint16_t heading;
 
// 1. THIS IS MODULE A

#define MY_ADDRESS 0x1111
 
// 2. Target list: Look for Module B

#define NUM_TARGETS 1

static uint16 target_nodes[NUM_TARGETS] = {0x2222};

static uint8 current_target_idx = 0;
 
// 3. Scheduling Timer

static uint16 scan_timer = 0;

#define SCAN_INTERVAL_TICKS 5 // 5 ticks * 50ms RX timeout = ~4 pings a second
 
/* ===================================================================== */
 
/* Delay between receiving a poll and sending a response (in UWB microseconds) */

#define POLL_RX_TO_RESP_TX_DLY_UUS 3300
 
/* Frames used in the ranging process */

static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE0, 0, 0};

static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 
#define ALL_MSG_COMMON_LEN 10

#define ALL_MSG_SN_IDX 2

#define RESP_MSG_POLL_RX_TS_IDX 10

#define RESP_MSG_RESP_TX_TS_IDX 14

#define RESP_MSG_TS_LEN 4

/****************************************** 
 Heading
*******************************************/
 
#define RESP_MSG_HEADING_IDX 18

static uint8 frame_seq_nb = 0;

#define RX_BUF_LEN 32

static uint8 rx_buffer[RX_BUF_LEN];
 
static uint32 status_reg = 0;

#define UUS_TO_DWT_TIME 65536

#define SPEED_OF_LIGHT 299702547
 
static double tof;

static double distance;
 
/*Transactions Counters */

static volatile int tx_count = 0 ; 

static volatile int rx_count = 0 ;
 
/* Helper Functions */

static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

static void resp_msg_set_ts(uint8 *ts_field, const uint32 ts);

static uint64 get_rx_timestamp_u64(void);
 
/*! ------------------------------------------------------------------------------------------------------------------

* @fn ss_init_run()

*/

int ss_init_run(void)

{

    // Set basic RX timeout to ~50ms

    dwt_setrxtimeout(50000);
 
    // Listen for incoming Polls (Responder Mode)

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
 
    // Wait for frame, timeout, or error

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };
 
    if (status_reg & SYS_STATUS_RXFCG) 

    {

        /* --- WE RECEIVED A MESSAGE (Act as Responder) --- */

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG); 

        uint32 frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

        if (frame_len <= RX_BUF_LEN) {

            dwt_readrxdata(rx_buffer, frame_len, 0);

        }
 
        uint16 dest_addr = rx_buffer[5] | (rx_buffer[6] << 8);

        if (dest_addr == MY_ADDRESS && rx_buffer[9] == 0xE0) // It is a POLL

        {

            uint16 src_addr = rx_buffer[7] | (rx_buffer[8] << 8);

            uint64 poll_rx_ts_64;

            uint32 poll_rx_ts_32, resp_tx_time, resp_tx_ts;

            // Read 40-bit RX timestamp to prevent 32-bit overflow

            poll_rx_ts_64 = get_rx_timestamp_u64();

            poll_rx_ts_32 = (uint32)poll_rx_ts_64;

            // Calculate precise delayed TX time for the hardware

            resp_tx_time = (poll_rx_ts_64 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

            dwt_setdelayedtrxtime(resp_tx_time);

            // Calculate actual 32-bit payload timestamp, accounting for Antenna Delay

            resp_tx_ts = (((uint32)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
 
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts_32);

            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

            /********************************************************************
             Put heading data into tx_resp_msg
            ********************************************************************/

            tx_resp_msg[RESP_MSG_HEADING_IDX] = heading & 0xFF;

            tx_resp_msg[RESP_MSG_HEADING_IDX + 1] = (heading >> 8) & 0xFF;
 


            tx_resp_msg[5] = src_addr & 0xFF;

            tx_resp_msg[6] = (src_addr >> 8) & 0xFF;

            tx_resp_msg[7] = MY_ADDRESS & 0xFF;

            tx_resp_msg[8] = (MY_ADDRESS >> 8) & 0xFF;

            tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;
 
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);





            //printf("TX len=%d heading=%.2f data=%u\n",
            //(int)sizeof(tx_resp_msg),
            //heading,
            //heading_data);

            //printf("TX[18]=%02X TX[19]=%02X\n",
            //tx_resp_msg[18],
            //tx_resp_msg[19]);





            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {

                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) { };

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

            }

        }

    }

    else 

    {

        /* --- TIMEOUT: SCANNER LOGIC (Act as Initiator) --- */

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        scan_timer++;

        if (scan_timer >= SCAN_INTERVAL_TICKS) 

        { 

            scan_timer = 0;
 
            tx_poll_msg[5] = target_nodes[current_target_idx] & 0xFF;

            tx_poll_msg[6] = (target_nodes[current_target_idx] >> 8) & 0xFF;

            tx_poll_msg[7] = MY_ADDRESS & 0xFF;

            tx_poll_msg[8] = (MY_ADDRESS >> 8) & 0xFF;

            tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;
 
            dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);

            dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

            // Start TX and automatically turn on RX right after

            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

            // Wait for the transmission to physically leave the antenna

            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) { };

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

            // Radio is now automatically in RX mode waiting for the response. 

            uint32_t resp_status_reg;

            while (!((resp_status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };
 
            if (resp_status_reg & SYS_STATUS_RXFCG) 

            {

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG); 

                uint32 frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

                if (frame_len <= RX_BUF_LEN) {

                    dwt_readrxdata(rx_buffer, frame_len, 0);












                    //printf("RX frame_len=%lu\n", (unsigned long)frame_len);

                    //for (uint32_t i = 0; i < frame_len; i++)
                    //{
                    //  printf("%02X ", rx_buffer[i]);
                    //}
                    //printf("\n");

                    //printf("RX[18]=%02X RX[19]=%02X\n",
                    //  rx_buffer[18],
                    //  rx_buffer[19]);











                }
 
                uint16 dest_addr = rx_buffer[5] | (rx_buffer[6] << 8);

                if (dest_addr == MY_ADDRESS && rx_buffer[9] == 0xE1) // It's a Response!

                {

                    uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;

                    int32 rtd_init, rtd_resp;

                    float clockOffsetRatio;
 
                    poll_tx_ts = dwt_readtxtimestamplo32();

                    resp_rx_ts = dwt_readrxtimestamplo32();

                    clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);
 
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);

                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);
 
                    rtd_init = resp_rx_ts - poll_tx_ts;

                    rtd_resp = resp_tx_ts - poll_rx_ts;
 
                    tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS;

                    distance = tof * SPEED_OF_LIGHT;


                    /***********************************************************
                    Get heading data from the rx_buffer
                    ************************************************************/   

                    uint16_t heading_data;

                    heading_data = rx_buffer[RESP_MSG_HEADING_IDX] | ((uint16_t)rx_buffer[RESP_MSG_HEADING_IDX + 1] << 8);

                    float received_heading = heading_data / 100.0f;

                    printf("%.2f,%.2f\n", distance, received_heading);

                }

            } 

            else 

            {

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            }
 
            current_target_idx++;

            if (current_target_idx >= NUM_TARGETS) {

                current_target_idx = 0;

            }

        }

    }

    return 1;

}
 
static uint64 get_rx_timestamp_u64(void)

{

    uint8 ts_tab[5];

    uint64 ts = 0;

    int i;

    dwt_readrxtimestamp(ts_tab);

    for (i = 4; i >= 0; i--)

    {

        ts <<= 8;

        ts |= ts_tab[i];

    }

    return ts;

}
 
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)

{

    int i;

    *ts = 0;

    for (i = 0; i < RESP_MSG_TS_LEN; i++)

    {

        *ts += ts_field[i] << (i * 8);

    }

}
 
static void resp_msg_set_ts(uint8 *ts_field, const uint32 ts)

{

    int i;

    for (i = 0; i < RESP_MSG_TS_LEN; i++)

    {

        ts_field[i] = (uint8)(ts >> (i * 8));

    }

}
 
void ss_initiator_task_function (void * pvParameter)

{

    UNUSED_PARAMETER(pvParameter);

    dwt_setleds(DWT_LEDS_ENABLE);
 
    while (true)

    {

        uart_receive_update();

        ss_init_run();

    }

}
 