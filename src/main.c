// https://www.scaler.com/topics/static-variables-in-c/

/* ha egy meglévő fájlt nyit meg és nem seekel a végére akkor a felülírás
közben rengeteget ít/olvas, a fájl vége felé egyre kevesebbet, végül 
'normál' mennyiséget, ha már új adatot ír */

// C:\ncs\v2.1.2\modules\hal\nordic\nrfx\drivers\nrfx_errors.h

/* 
 *  by setting up endpoints of the channel m_gppi_channels [ gppi_channels_purpose_t::SAADC_SAMPLING ]
 *  to trigger SAADC sample task ( nrf_saadc_task_t::NRF_SAADC_TASK_SAMPLE ) on the timer compare event.
 *  - Hardware start-on-end must be provided. It is done by setting up endpoints of the channel
 *  m_gppi_channels [ gppi_channels_purpose_t::SAADC_START_ON_END ] to trigger SAADC task start
 *  ( nrf_saadc_task_t::NRF_SAADC_TASK_START ) on the SAADC event end ( nrf_saadc_event_t::NRF_SAADC_EVENT_END ).
 *
 *  Calibration in a non-blocking manner is triggered by nrfx_saadc_offset_calibrate. Then at NRFX_SAADC_EVT_CALIBRATEDONE
 *  event in adc_eventHandler() sampling is initiated by calling nrfx_saadc_mode_trigger() function.
 *  Consecutive sample tasks are triggered by the external timer at the sample rate specified in SAADC_SAMPLE_FREQUENCY symbol.
 */

// https://devzone.nordicsemi.com/guides/nrf-connect-sdk-guides/b/software/posts/building-a-bluetooth-application-on-nrf-connect-sdk-part-3-optimizing-the-connection#mcetoc_1fsockao80


//#include <zephyr/logging/log.h>
//#include <stdio.h>
//#include <zephyr/device.h>

#include <zephyr/kernel.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/scan.h>
#include <hal/nrf_power.h>
#include <hal/nrf_radio.h>

LOG_MODULE_REGISTER();

#define VERSION "0.8.1"

#define BT_ADDR_LE_STR_LEN 30

#define STANDALONE 1
#define BT_MASTER 2
#define BT_SLAVE 3

#define ADC_NUM_OF_CHANNELS		5

#define ADC_INPUT_0_CHANNEL_INDEX	0
#define ADC_INPUT_1_CHANNEL_INDEX	1
#define ADC_INPUT_2_CHANNEL_INDEX	2
#define ADC_INPUT_3_CHANNEL_INDEX	3
#define ADC_INPUT_4_CHANNEL_INDEX	4

#define ADC_INPUT_0_PIN		NRF_SAADC_INPUT_AIN0	// AIN0 = P0.02
#define ADC_INPUT_1_PIN		NRF_SAADC_INPUT_AIN1	// AIN1 = P0.03
#define ADC_INPUT_2_PIN		NRF_SAADC_INPUT_AIN2	// AIN2 = P0.04
#define ADC_INPUT_3_PIN		NRF_SAADC_INPUT_AIN3	// AIN3 = P0.05
#define ADC_INPUT_4_PIN		NRF_SAADC_INPUT_AIN4 	// AIN4 = P0.28

#define KILL 0 //P0.00
#define INT 1 // P0.01
// #define MOSI 6
// #define SCK 7
// #define MISO 8
// #define CS 9 
#define CARD_DETECT 10 //P0.10
#define PWM 20
#define LED 21 //P0.21

#define SIGNAL_THREAD_STACKSIZE 512
#define SIGNAL_THREAD_PRIORITY 7	// kisebb szám -> magasabb prioritás

#define TICK_TIME_MS 10

#define BUFFER_COUNT 2U
#define BUFFER_SIZE ADC_NUM_OF_CHANNELS //5UL

#define SAMPLING_ITERATIONS 360000UL // 4h @ 25Hz
#define SAADC_SAMPLE_FREQUENCY 25U
#define TIME_TO_WAIT_US (uint32_t)(1000000UL / SAADC_SAMPLE_FREQUENCY)

#define	RED 0
#define	GREEN 1

#define	OFF 0
#define RED_ON 1
#define GREEN_ON 2
#define	RED_BLINK 3
#define	GREEN_BLINK 4
#define RED_GREEN 5
#define	STARTUP 6     
#define	FS_INIT_PROGRESS 7		
#define G_O_GG_O 8
#define R_O_RR_O 9
#define X 10
#define ERROR 11

// BT ---------------------
static struct bt_conn *central_conn;
static struct bt_conn *peripheral_conn;
static struct bt_nus_client nus_client; 

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};
// BT ----------------------

// FS --------------------
static struct fs_file_t master_file;
static struct fs_file_t slave_file;
static struct fs_littlefs lfsfs;
static struct fs_mount_t __mp = {
	.type = FS_LITTLEFS,
	.fs_data = &lfsfs,
	.flags = FS_MOUNT_FLAG_USE_DISK_ACCESS,
};
static struct fs_mount_t *mp = &__mp;

static char fname_master[37];
static char fname_slave[32];
// FS -----------------------------

static const struct pwm_dt_spec pwm_led0 = {
	.dev = DEVICE_DT_GET(DT_NODELABEL(pwm0)),
	.channel = 0,
    .period = 100,
    .flags = PWM_POLARITY_NORMAL,
};


//Samples buffer to store values from a SAADC channel. 
static nrf_saadc_value_t adc_rawDataBuffer[BUFFER_COUNT][BUFFER_SIZE];

// Array of the GPPI channels. 
static uint8_t m_gppi_channels[2];

static nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(1);

static K_MUTEX_DEFINE(power_mutex);

static K_SEM_DEFINE(fs_init_ok, 0, 1);
static K_SEM_DEFINE(ble_peripheral_NUS_connected, 0, 1);
static K_SEM_DEFINE(ble_central_NUS_connected, 0, 1);
static K_SEM_DEFINE(gatt_discovery_complete, 0, 1);

K_MSGQ_DEFINE(adc_data_queue, 10, 560, 2);			// 560, 800
K_MSGQ_DEFINE(bt_received_data_queue, 244, 7, 2); 	// 7, 10
K_MSGQ_DEFINE(signaling_mode_queue, 2, 4, 2);



// Enum with intended uses of GPPI channels defined as m_gppi_channels. 
typedef enum
{
    SAADC_SAMPLING,     ///< Triggers SAADC sampling task on external timer event.
    SAADC_START_ON_END, ///< Triggers SAADC start task on SAADC end event.
} gppi_channels_purpose_t;



/**
 * @brief Function for handling TIMER driver events.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of the timer.
 *                       This parameter can be used to pass additional information to the handler
 *                       function for example the timer ID.
 */
static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    (void)event_type;
    (void)p_context;    
}

void error(void) {
	
	uint16_t msg;
	bt_disable();

	LOG_INF("in error");
	msg = ERROR;
	k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);
	
	while (1) {
		k_sleep(K_MSEC(10000));	
	}
}

static int littlefs_mount(struct fs_mount_t *mp)
{
	static const char *disk_mount_pt = "/"CONFIG_SDMMC_VOLUME_NAME":";
	static const char *disk_pdrv = CONFIG_SDMMC_VOLUME_NAME;

	mp->storage_dev = (void *)disk_pdrv;
	mp->mnt_point = disk_mount_pt;

	return fs_mount(mp);
}

static void adc_eventHandler(nrfx_saadc_evt_t const * p_event)
{
	nrfx_err_t err;
    
    static uint16_t buffer_index = 1;
    static uint32_t buf_req_evt_counter;

	switch (p_event->type) {
		case NRFX_SAADC_EVT_DONE:						

			err = k_msgq_put(&adc_data_queue, p_event->data.done.p_buffer, K_NO_WAIT);
			if (err) {
				LOG_WRN("q");
			} 
			break;
			
		case NRFX_SAADC_EVT_LIMIT:
			break;

		case NRFX_SAADC_EVT_CALIBRATEDONE:

            err = nrfx_saadc_mode_trigger();
            if(err != NRFX_SUCCESS) {LOG_INF("Error %d in nrfx_saadc_mode_trigger()", err);
			}
                       
			break;

		case NRFX_SAADC_EVT_BUF_REQ:
			if (++buf_req_evt_counter < SAMPLING_ITERATIONS) {
                // Next available buffer must be set on the NRFX_SAADC_EVT_BUF_REQ event to achieve the continuous conversion. 
                err = nrfx_saadc_buffer_set(adc_rawDataBuffer[buffer_index++], BUFFER_SIZE);
                if(err != NRFX_SUCCESS) {LOG_INF("Error %d in nrfx_saadc_buffer_set()", err);
				}
                
                buffer_index = buffer_index % BUFFER_COUNT;
            }
            else {
                nrfx_gppi_channels_disable(NRFX_BIT(m_gppi_channels[SAADC_START_ON_END]));
            }
			break;
		
		case NRFX_SAADC_EVT_READY:
			nrfx_gppi_channels_enable(NRFX_BIT(m_gppi_channels[SAADC_SAMPLING]));
			break;

		case NRFX_SAADC_EVT_FINISHED:
			nrfx_gppi_channels_disable(NRFX_BIT(m_gppi_channels[SAADC_SAMPLING]));
			break;

		default:
			break;
	}
}

// timer configuration -----------------------------------------------------------
uint8_t timer_initialise()
{
	nrfx_err_t err;
    	
	// Initialise Timer	
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_config.p_context = &timer_inst;

    err = nrfx_timer_init(&timer_inst, &timer_config, timer_handler);
    if(err != NRFX_SUCCESS) {
		LOG_INF("Error %d in nrfx_timer_init()", err);
	}
	
    nrfx_timer_clear(&timer_inst);

    // calculate ticks
    uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer_inst, TIME_TO_WAIT_US);

    // Setting the timer channel NRF_TIMER_CC_CHANNEL1 in the extended compare mode to clear
    // the timer and to not trigger an interrupt if the internal counter register is equal to
    // desired_ticks.
    nrfx_timer_extended_compare(&timer_inst,
                                NRF_TIMER_CC_CHANNEL1,
                                desired_ticks,
                                NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK,
                                false);

    return 0;
    
}

// ADC configuration ---------------------------------------------------------------
uint8_t adc_initialise()
{
    nrfx_err_t err;

	err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
	if(err != NRFX_SUCCESS)
		LOG_INF("Error %d in nrfx_saadc_init()", err);

	IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0);

	nrfx_saadc_channel_t adc_input1Config = 
	{
		.channel_config ={
			.reference = NRF_SAADC_REFERENCE_INTERNAL,		
			.acq_time = NRF_SAADC_ACQTIME_40US,			// Acquisition time 40uS
			.mode = NRF_SAADC_MODE_SINGLE_ENDED,		// Single Ended
			.burst = NRF_SAADC_BURST_DISABLED, 			// Normal Operation (Burst disabled)
			.gain = NRF_SAADC_GAIN1_6,
		},
		.channel_index = ADC_INPUT_0_CHANNEL_INDEX,		// ADC0
		.pin_p = ADC_INPUT_0_PIN						// Input 0 = AIN0/P0.02
	};

	nrfx_saadc_channel_t adc_input2Config = 
	{
		.channel_config = adc_input1Config.channel_config,
		.channel_index = ADC_INPUT_1_CHANNEL_INDEX,		// ADC1
		.pin_p = ADC_INPUT_1_PIN						// Input 1 = AIN1/P0.03
	};

    nrfx_saadc_channel_t adc_input3Config = 
	{
		.channel_config = adc_input1Config.channel_config,
		.channel_index = ADC_INPUT_2_CHANNEL_INDEX,		// ADC2
		.pin_p = ADC_INPUT_2_PIN						// Input 2 = AIN2/P0.04
	};

    nrfx_saadc_channel_t adc_input4Config = 
	{
		.channel_config = adc_input1Config.channel_config,
		.channel_index = ADC_INPUT_3_CHANNEL_INDEX,		// ADC3
		.pin_p = ADC_INPUT_3_PIN						// Input 3 = AIN3/P0.05
	};

// Vbat=3.2V -> ADCin=1.5V -> value=1713   (107) ???
	nrfx_saadc_channel_t adc_input5Config = 
	{
		.channel_config = adc_input1Config.channel_config,
		.channel_index = ADC_INPUT_4_CHANNEL_INDEX,		// ADC4
		.pin_p = ADC_INPUT_4_PIN						// Input 4 = AIN4/P0.28
	};

	// Configuration includes the two inputs described above
	nrfx_saadc_channel_t adc_configArray[ADC_NUM_OF_CHANNELS] = {
		adc_input1Config, 
		adc_input2Config,
        adc_input3Config,
		adc_input4Config, 
		adc_input5Config
	};

	// Apply configuration above
	err = nrfx_saadc_channels_config(adc_configArray, ADC_NUM_OF_CHANNELS);
	if(err != NRFX_SUCCESS) {
		LOG_INF("Error %d in nrfx_saadc_channels_config()", err);    
	}
	/* ADC advanced configuration */
	nrfx_saadc_adv_config_t adc_advancedConfig =
	{
		.start_on_end = false,
		.burst = NRF_SAADC_BURST_DISABLED,
		.internal_timer_cc = 0,
	};

	// For non-blocking provide adc event handler function
    err = nrfx_saadc_advanced_mode_set(
		BIT(ADC_INPUT_0_CHANNEL_INDEX)|BIT(ADC_INPUT_1_CHANNEL_INDEX)|BIT(ADC_INPUT_2_CHANNEL_INDEX)|
		BIT(ADC_INPUT_3_CHANNEL_INDEX)|BIT(ADC_INPUT_4_CHANNEL_INDEX),
        NRF_SAADC_RESOLUTION_12BIT,
        &adc_advancedConfig,
        adc_eventHandler);
    if(err != NRFX_SUCCESS) {
		LOG_INF("Error %d in nrfx_saadc_advanced_mode_set()", err);
	}
	// Set-up buffer
    err = nrfx_saadc_buffer_set(adc_rawDataBuffer[0], BUFFER_SIZE);
    if(err != NRFX_SUCCESS) {
		LOG_INF("Error %d in nrfx_saadc_buffer_set()", err);
	}
    return 0;
}

uint8_t ppi_initialise()
{
    nrfx_err_t err;
    
    // Allocate a dedicated channel and configure endpoints of that channel so that the timer compare event
    // is connected with the SAADC sample task. This means that each time the timer interrupt occurs,
    // the SAADC sampling will be triggered.
    err = nrfx_gppi_channel_alloc(&m_gppi_channels[SAADC_SAMPLING]);
    if(err != NRFX_SUCCESS){
		LOG_INF("Error %d in nrfx_gppi_channel_alloc SAADC_SAMPLING", err);
	}
    nrfx_gppi_channel_endpoints_setup(m_gppi_channels[SAADC_SAMPLING],
        nrfx_timer_compare_event_address_get(&timer_inst, NRF_TIMER_CC_CHANNEL1),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    
    // Allocate a dedicated channel and configure endpoints of that so that the SAADC event end is connected
    // with the SAADC task start. This means that each time the SAADC fills up the result buffer,
    // the SAADC will be restarted and the result buffer will be prepared in RAM.
    err = nrfx_gppi_channel_alloc(&m_gppi_channels[SAADC_START_ON_END]);
    if(err != NRFX_SUCCESS) {
		LOG_INF("Error %d in nrfx_gppi_channel_alloc SAADC_START_ON_END", err);
	}
    nrfx_gppi_channel_endpoints_setup(m_gppi_channels[SAADC_START_ON_END],
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

	return 0;
}

static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
	struct bt_nus_client *nus = context;
	LOG_INF("Service discovery completed");

	//bt_gatt_dm_data_print(dm);
	bt_nus_handles_assign(dm, nus);
	bt_nus_subscribe_receive(nus);
	bt_gatt_dm_data_release(dm);

	k_sem_give(&gatt_discovery_complete);
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
	LOG_WRN("Service not found");
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
	LOG_ERR("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
	nrfx_err_t err;

	if (conn != central_conn) {
		return;
	}

	err = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb, &nus_client);
	if (err) {
		LOG_ERR("could not start the discovery procedure, error code: %d", err);
	}
}

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done");
	} else {
		LOG_ERR("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	// char addr[BT_ADDR_LE_STR_LEN];
	// bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	// LOG_INF("scan filter match: %s", addr);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_ERR("scan connecting central_conn failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	LOG_INF("scan connecting central_conn");
	central_conn = bt_conn_ref(conn);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	nrfx_err_t err;
	struct bt_scan_init_param scan_init = {
		.connect_if_match = true
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, true);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	return err;
}

static uint8_t central_ble_data_received(struct bt_nus_client *nus, const uint8_t *data, uint16_t len)
{
	// max bt data len: max PDU-4-3 = 251-4-3 = 244
	nrfx_err_t err;
	ARG_UNUSED(nus);

	nrf_gpio_pin_toggle(LED);

	if (len == 244){
		err = k_msgq_put(&bt_received_data_queue, data, K_NO_WAIT);
		if (err) {
			LOG_WRN("bt_q");
		} 				
	}
	else {
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}
	return BT_GATT_ITER_CONTINUE;
}

static int nus_client_init(void)  // central NUS
{
	nrfx_err_t err;
	struct bt_nus_client_init_param init = {
		.cb = {
			.received = central_ble_data_received,
			//.sent = central_ble_data_sent,
		}
	};

	err = bt_nus_client_init(&nus_client, &init);
	if (err) {
		LOG_ERR("NUS Client initialization failed (err %d)", err);
		return err;
	}

	LOG_INF("NUS Client module initialized");
	return err;
}
// central role related end --------------

static void get_master_fname(char mac_address[37]) {	
	unsigned int device_addr_0 = NRF_FICR->DEVICEADDR[0];
	unsigned int device_addr_1 = NRF_FICR->DEVICEADDR[1];
	const uint8_t* part_0 = &device_addr_0;
	const uint8_t* part_1 = &device_addr_1;

// https://devzone.nordicsemi.com/f/nordic-q-a/76344/retrieve-ble-mac-address-in-zephyr-environment
	snprintf(mac_address, 33, "/SDMMC:/%02X_%02X_%02X_%02X_%02X_%02X_master",
		part_1[1] | 0xC0, part_1[0], part_0[3],part_0[2], part_0[1], part_0[0]);

}

static void get_standalone_fname(char mac_address[37]) {	
	unsigned int device_addr_0 = NRF_FICR->DEVICEADDR[0];
	unsigned int device_addr_1 = NRF_FICR->DEVICEADDR[1];
	const uint8_t* part_0 = &device_addr_0;
	const uint8_t* part_1 = &device_addr_1;

// https://devzone.nordicsemi.com/f/nordic-q-a/76344/retrieve-ble-mac-address-in-zephyr-environment
	snprintf(mac_address, 37, "/SDMMC:/%02X_%02X_%02X_%02X_%02X_%02X_standalone",
		part_1[1] | 0xC0, part_1[0], part_0[3],part_0[2], part_0[1], part_0[0]);

}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	nrfx_err_t err;
	struct bt_conn_info info;		
		
	if (conn_err) {
		LOG_ERR("Failed to connect to %s (%d)", fname_slave, conn_err);

		if (conn == central_conn) {
			bt_conn_unref(central_conn);
			central_conn = NULL;
			err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			if (err) {
				LOG_ERR("Scanning failed to start (err %d)", err);
				error();
			}
			
			}
		else if (conn == peripheral_conn) {
			bt_conn_unref(peripheral_conn);
			peripheral_conn = NULL;

			err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
			if (err) {
				LOG_ERR("Advertising failed to start (err %d)", err);
				error();
			}
		}
		return;
	}

	bt_conn_get_info(conn, &info);
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);

	LOG_INF("connected to peer");

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		LOG_INF("connected role = central");

		err = bt_scan_stop();
		if ((!err) && (err != -EALREADY)) {
			LOG_ERR("Stop LE scan failed (err %d)", err);
		} 

		static struct bt_gatt_exchange_params exchange_params;

		exchange_params.func = exchange_func;
		err = bt_gatt_exchange_mtu(conn, &exchange_params);
		if (err) {
			LOG_ERR("MTU exchange request failed (err %d)", err);
		} else {
			LOG_INF("MTU exchange request done");
		}

		gatt_discover(conn);
		
		int tout = 100;

		while (!k_sem_take(&gatt_discovery_complete, K_NO_WAIT)) {
			k_sleep(K_MSEC(10));
			tout --;
			if (tout < 1) {
				LOG_ERR("GATT discover timeout");
				error();
			}
		}

		k_sem_give(&ble_central_NUS_connected);

	} else {

		err = bt_le_adv_stop();
		if (err) {
			LOG_ERR("adv stop error (err %d)", err);
		}

		LOG_INF("connected role = peripheral");

		peripheral_conn = bt_conn_ref(conn);		

		uint16_t msg = R_O_RR_O;
		k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);

		k_sem_give(&ble_peripheral_NUS_connected);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	nrfx_err_t err;
	
	const bt_addr_le_t *disconnected_mac_address = bt_conn_get_dst(conn);

	bt_addr_le_to_str(disconnected_mac_address, addr, sizeof(addr));	

	if (conn == central_conn) {
		LOG_WRN("Disconnected: %s (reason %u), role: central", addr, reason);

		bt_conn_unref(central_conn);
		central_conn = NULL;

		err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, disconnected_mac_address);
		if (err) {
			LOG_ERR("address filter cannot be set (err %d)", err);	
				
		}
		else {
			err = bt_scan_filter_enable(BT_SCAN_FILTER_TYPE_ADDR, true);
			if (err) {
				LOG_ERR("Filters cannot be turned on (err %d)", err);
				error();
			}
		}

		LOG_WRN("scan restart");

		uint16_t msg = GREEN_BLINK;
		k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);

		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err) {
			LOG_ERR("Scanning failed to start (err %d)", err);
			error();
		}
		
	} else if (conn == peripheral_conn) {
		LOG_WRN("Disconnected: %s (reason %u), role: peripheral", addr, reason);
		bt_conn_unref(peripheral_conn);
		peripheral_conn = NULL;		

		uint16_t msg = RED_BLINK;
		k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);

		err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			LOG_ERR("Advertising failed to start (err %d)", err);
			error();
		}
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

uint8_t peripheral_nus_init() {
	
	nrfx_err_t err;
	int tout_cnt = 150;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Failed to enable BT (err: %d)", err);
		error();
	}

	LOG_INF("Bluetooth initialized");

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize NUS service (err: %d)", err);
		error();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		error();
	}
	
	while (tout_cnt) {
		
		err = k_sem_take(&ble_peripheral_NUS_connected, K_NO_WAIT);
		if (err == 0){
			LOG_INF("NUS connected!");
			return 0;
		}
		k_sleep(K_MSEC(200));
		tout_cnt --;
	}

	LOG_INF("not connected");
	return 1;
}

uint8_t central_nus_init() {
	
	nrfx_err_t err;
	int tout_cnt = 150;

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("BT enable error");		
		return 1;
	}

	LOG_INF("Bluetooth initialized");

	LOG_INF("NUS client (central) init");
	err = nus_client_init();
	if (err) {
		return;
	}

	LOG_INF("scan init");
	scan_init();

	LOG_INF("scan start");
	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		error();
	}
	
	while (tout_cnt) {		
		err = k_sem_take(&ble_central_NUS_connected, K_NO_WAIT);
		if (err == 0){
			LOG_INF("peripheral NUS connected!");
			return 0;
		}
		k_sleep(K_MSEC(200));
		tout_cnt --;
	}

	LOG_INF("bt disable");
	bt_disable();
	return 1;
}

static void daq_system_init(void) {

	if (adc_initialise() != 0) {
		error();
	}
	LOG_INF("ADC initialised");

	if (timer_initialise() != 0) {
		error();
	}
	LOG_INF("timer initialised");

	if (ppi_initialise() != 0) {
		error();
	}
	LOG_INF("ppi initialised");
    
	nrfx_timer_enable(&timer_inst); 

	if (!device_is_ready(pwm_led0.dev)) {
	 	LOG_INF("Error: PWM device %s is not ready", pwm_led0.dev->name);
	 	error();
	}

	pwm_set_dt(&pwm_led0, 5000, 2000);
	// 	LOG_INF("Error %d: failed to set pulse width", err);
	// 	error();
	// }
	LOG_INF("PWM started");

    nrfx_gppi_channels_enable(NRFX_BIT(m_gppi_channels[SAADC_START_ON_END]));
}

void main(void)
{
	static uint16_t bt_data_array_idx = 0;
	static uint8_t bt_pkt_cnt_buffer[6];
	static uint8_t bt_data_array[512];	// buffer for data from BT to write to uSD
	static uint8_t bt_data_rcvd[244];	// buffer for data received through BT
	static uint8_t data_array[512]; 	// buffer for data from ADC to write to uSD or to send through BT
	static uint8_t data_idx = 0;	
	static uint8_t data_limit;			// amount of data to be put in data_array, depends on mode
	static uint8_t cnt = 0;
	static uint8_t mode;
	uint8_t data_rcvd[BUFFER_SIZE*2];
	nrfx_err_t err;
	uint16_t msg;

	nrf_gpio_cfg_output(KILL);
	nrf_gpio_pin_set(KILL);		// keep power on

	nrf_gpio_cfg_output(LED);
	nrf_gpio_cfg_input(CARD_DETECT, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(INT,NRF_GPIO_PIN_PULLUP);
	
	nrf_gpio_pin_write(LED, RED);

    LOG_INF("ADC test, board: %s, ver.: %s", CONFIG_BOARD, VERSION);
	//printk("k ADC test\nBoard: %s", CONFIG_BOARD);
		
	//k_sleep(K_MSEC(2000));

	daq_system_init();

	// check if CARD_DETECT input is low continously for 0.5 sec
	int tout_cnt = 50;
	while (tout_cnt) {		
		if (nrf_gpio_pin_read(CARD_DETECT) != 0) {
			break;
		}
		k_sleep(K_MSEC(10));
		tout_cnt --;
	}

	if  (tout_cnt == 0) {	// and (nrf_gpio_pin_read(CARD_DETECT) == 0)  // uSD card present 

		msg = GREEN_BLINK;
		k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);

		LOG_INF("uSD card present");

		err = littlefs_mount(mp);
		if (err) {
			LOG_ERR("fs mount failed %d", err);
			error();
		}

		LOG_INF("master file init");
		fs_file_t_init(&master_file);		
		
		data_limit = 503;

		LOG_INF("init central NUS");
		err = central_nus_init();

		if (err) {
			LOG_INF("BT peripheral NOT connected, mode: STANDALONE");
			get_standalone_fname(fname_master);
			LOG_INF("fname, standalone: %s", fname_master);

			mode = STANDALONE;
			msg = G_O_GG_O;
			k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);

		}
		else{		
			LOG_INF("BT peripheral connected, mode: BT_MASTER");
			get_master_fname(fname_master);
			LOG_INF("fname, master: %s", fname_master);

			mode = BT_MASTER;

			const bt_addr_le_t *addr = bt_conn_get_dst(central_conn);
			snprintf(fname_slave, 32, "/SDMMC:/%02X_%02X_%02X_%02X_%02X_%02X_slave",
				addr->a.val[5], addr->a.val[4], addr->a.val[3],
				addr->a.val[2], addr->a.val[1], addr->a.val[0]);
					
			LOG_INF("fname, slave: %s", fname_slave);

			LOG_INF("slave file init");
			fs_file_t_init(&slave_file);
			
			msg = R_O_RR_O;
			k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);
		}

	}
	else {	// uSD card not present
		LOG_INF("No uSD card present, peripheral NUS init start");
		
		msg = RED_BLINK;
		k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);
				
		err = peripheral_nus_init();	
		if (err) {
			error();
		}

		LOG_INF("mode: BT slave");
		mode = BT_SLAVE;
		data_limit = 239;

		msg = GREEN_ON;
		k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);		
	}

	if (mode != STANDALONE) {
		nrf_radio_txpower_set(NRF_RADIO, (nrf_radio_txpower_t)NRF_RADIO_TXPOWER_POS4DBM);
		int txp = nrf_radio_txpower_get(NRF_RADIO);

		k_sleep(K_MSEC(1000));

		LOG_INF("tx power: %d", txp);
	}

	
	LOG_INF("starting data acquisition");
	
	k_sleep(K_MSEC(2000));

    err = nrfx_saadc_offset_calibrate(adc_eventHandler);
    if(err != NRFX_SUCCESS) { 
		LOG_INF("Error %d in nrfx_saadc_offset_calibrate", err);
	}
	LOG_INF("while loop starts");

    while (1) {
		
		err = k_msgq_get(&adc_data_queue, &data_rcvd, K_NO_WAIT);  // csak K_NO_WAIT lehet, mert ISR-ből jön!!!!!

		if (err == 0 && nrf_gpio_pin_read(INT) == 1) { // got data from queue			

			data_array[data_idx] = data_rcvd[0];
			data_idx ++;
			data_array[data_idx] = data_rcvd[2];
			data_idx ++;			
			data_array[data_idx] = data_rcvd[1] & 0x0f;		    
			data_array[data_idx] |= (data_rcvd[3] << 4) & 0xf0;
			data_idx ++;			
			data_array[data_idx] = data_rcvd[4];
			data_idx ++;			
			data_array[data_idx] = data_rcvd[6];
			data_idx ++;			
			data_array[data_idx] = data_rcvd[5] & 0x0f;	    
			data_array[data_idx] |= (data_rcvd[7] << 4) & 0xf0;
			data_idx ++;

			if (data_idx > data_limit){ // packet full, add counter and Vbat to the end 
				
				cnt += 1;	
				
				data_array[data_idx] = data_rcvd[8];	// Vbat
				data_idx ++;
				data_array[data_idx] = data_rcvd[9];	// Vbat
				data_idx ++;
				data_array[data_idx] = cnt & 0xff;
				data_idx ++;
				data_array[data_idx] = (cnt >> 8) & 0xff;

				if (mode != BT_SLAVE) {  

					data_idx ++;
					data_array[data_idx] = 0; 
					data_idx ++;
					data_array[data_idx] = 0; 
					data_idx ++;
					data_array[data_idx] = 0; 
					data_idx ++;
					data_array[data_idx] = 0; 

					err = fs_open(&master_file, fname_master, FS_O_CREATE | FS_O_RDWR);
					if (err) {
						LOG_ERR("master open fail, %d", err);
						error();
					}
										
					err = fs_seek(&master_file, 0, FS_SEEK_END);
					if (err) {
						LOG_ERR("master seek fail, %d", err);
						error();
						}

					k_mutex_lock(&power_mutex, K_FOREVER);

					err = fs_write(&master_file, data_array, sizeof(data_array));
					if (err != 512) {
						LOG_ERR("master write fail, %d", err);
						error();
					}

					if (fs_close(&master_file) != 0) error();

					k_mutex_unlock(&power_mutex);

					//nrf_gpio_pin_toggle(LED);
				}
				else { 	// send packet on bluetooth
					msg = RED_ON;
					k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);	

					while (bt_nus_send(NULL, data_array, data_limit + 4 + 1)) {	// 4-gyel több, plusz nullás indexelés miat még egy
						k_sleep(K_MSEC(10));
					}
					msg = GREEN_ON;
					k_msgq_put(&signaling_mode_queue, &msg, K_NO_WAIT);	
				}
				data_idx = 0;
			}
		}

		if (mode == BT_MASTER && nrf_gpio_pin_read(INT) == 1) {
			err = k_msgq_get(&bt_received_data_queue, &bt_data_rcvd, K_NO_WAIT);  // csak K_NO_WAIT lehet, mert ISR-ből jön!!!!!

			if (err == 0) {								

				bt_pkt_cnt_buffer[5] = bt_pkt_cnt_buffer[3];
				bt_pkt_cnt_buffer[4] = bt_pkt_cnt_buffer[2];	
				bt_pkt_cnt_buffer[3] = bt_pkt_cnt_buffer[1];
				bt_pkt_cnt_buffer[2] = bt_pkt_cnt_buffer[0];	

				bt_pkt_cnt_buffer[0] = bt_data_rcvd[242];
				bt_pkt_cnt_buffer[1] = bt_data_rcvd[243];
				
				for (int i=0; i<240; i++) {			

					bt_data_array[bt_data_array_idx] = bt_data_rcvd[i];
					bt_data_array_idx ++;			

					if (bt_data_array_idx > 503) {
						bt_data_array[bt_data_array_idx] = bt_data_rcvd[240];	// Vbat
						bt_data_array_idx ++;
						bt_data_array[bt_data_array_idx] = bt_data_rcvd[241];	// Vbat
						bt_data_array_idx ++;		
						bt_data_array[bt_data_array_idx] = bt_pkt_cnt_buffer[0];
						bt_data_array_idx ++;
						bt_data_array[bt_data_array_idx] = bt_pkt_cnt_buffer[1];
						bt_data_array_idx ++;
						bt_data_array[bt_data_array_idx] = bt_pkt_cnt_buffer[2];
						bt_data_array_idx ++;
						bt_data_array[bt_data_array_idx] = bt_pkt_cnt_buffer[3];
						bt_data_array_idx ++;
						bt_data_array[bt_data_array_idx] = bt_pkt_cnt_buffer[4];
						bt_data_array_idx ++;
						bt_data_array[bt_data_array_idx] = bt_pkt_cnt_buffer[5];

						bt_data_array_idx = 0;

						//nrf_gpio_pin_toggle(LED);				
						err = fs_open(&slave_file, fname_slave, FS_O_CREATE | FS_O_RDWR);
						if (err) {
							LOG_ERR("slave open fail, %d", err);
							error();
						}
											
						err = fs_seek(&slave_file, 0, FS_SEEK_END);
						if (err) {
							LOG_ERR("slave seek fail, %d", err);
							error();
						}

						k_mutex_lock(&power_mutex, K_FOREVER);

						err = fs_write(&slave_file, bt_data_array, 512);
						if (err != 512) {
							LOG_ERR("slave write fail!, %d", err);
							error();
						}

						if (fs_close(&slave_file) != 0) error();

						k_mutex_unlock(&power_mutex);
					}						
				}					
			}						
		}

		if (nrf_gpio_pin_read(INT) != 1) {
			nrf_gpio_pin_clear(KILL);
		}
		
		k_sleep(K_MSEC(10));	
	}
}

void signal_thread(void)
{
	nrfx_err_t err;
	static uint16_t tick = 0;
	static uint16_t signaling_mode = OFF;
	uint16_t sm = 0;

	LOG_INF("signal thread starting");

	for (;;) {

		if (nrf_gpio_pin_read(INT) == 0) {	// power off
			k_mutex_lock(&power_mutex, K_FOREVER);
			nrf_gpio_pin_clear(KILL);	
			while(1);
		}

		err = k_msgq_get(&signaling_mode_queue, &sm, K_NO_WAIT);  // csak K_NO_WAIT lehet, mert ISR-ből jön!!!!!
		if (err == 0){
			signaling_mode = sm; 
			LOG_INF("sm: %d", sm);
		}
					
		switch (signaling_mode) {
			
			case X:
				tick = 0;
				break;

			case OFF:
				nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL);
				tick = 0;
				break;

			case RED_ON:
				nrf_gpio_cfg_output(LED);
				nrf_gpio_pin_write(LED, RED);
				tick = 0;
				break;
				
			case GREEN_ON:
				nrf_gpio_cfg_output(LED);
				nrf_gpio_pin_write(LED, GREEN);
				tick = 0;
				break;
				
			case ERROR:
				switch (tick) {
					case 1:
						nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL);
						break;

					case 13:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, RED);
						break;

					case 25:
						tick = 0;
						break;
					
					default:
						break;
				}
				break;

			case RED_BLINK:
				switch (tick) {
					
					case 1:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, RED);
						break;
					
					case 11:
						nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL);
						break;
					
					default:
						break;
				}
				break;
			
			case GREEN_BLINK:
				switch (tick) {
					case 1:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, GREEN);
						break;
						
					case 11:
						nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL);
						break;

					default:
						break;
				}
				break;
			
			case G_O_GG_O:
				switch (tick) {
					case 1:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, GREEN);
						break;

					case 20:
						nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL);
						break;

					case 40:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, GREEN);
						break;
					
					case 80:
						nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL);
						break;
					
					default:
						break;
				}
				break;

			case R_O_RR_O:
				switch (tick) {
					case 1:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, RED);
						break;

					case 20:
						nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL);
						break;

					case 40:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, RED);
						break;
					
					case 80:
						nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL);
						break;
					
					default:
						break;
				}
				break;

			case RED_GREEN:
				switch (tick) {
					case 1:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, RED);
						break;

					case 51:
						nrf_gpio_cfg_output(LED);
						nrf_gpio_pin_write(LED, GREEN);
						break;
				
					default:
						break;
				}
				break;

			default:
				break;
		}
	
		k_sleep(K_MSEC(TICK_TIME_MS));
		tick ++;
		if (tick > 98) tick = 0;
	}
} 

K_THREAD_DEFINE(signal_thread_id, SIGNAL_THREAD_STACKSIZE, signal_thread, NULL, NULL,
  		NULL, SIGNAL_THREAD_PRIORITY, 0, 0); 
