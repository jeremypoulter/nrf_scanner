#include <Arduino.h>

// https://github.com/NordicSemiconductor/pc-nrfconnect-rssi/blob/master/fw/src/rssi_uart/main.c

#include <string.h>

#define FREQ_ADV_CHANNEL_37         2      /**<Radio channel number which corresponds with 37-th BLE channel **/
#define FREQ_ADV_CHANNEL_38         26     /**<Radio channel number which corresponds with 38-th BLE channel **/
#define FREQ_ADV_CHANNEL_39         80     /**<Radio channel number which corresponds with 39-th BLE channel **/
#define UART_TX_BUF_SIZE            512    /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE            2048   /**< UART RX buffer size. */
#define NEW_PACKET_BYTE             255
#define RSSI_NO_SIGNAL              127    /**< Minimum value of RSSISAMPLE */

#if defined ( __CC_ARM )
static const char version_str[16] __attribute__((at(0x2000))) = "rssi-fw-1.0.0\0\0\0";
#endif


static uint8_t min_channel        = 0;     /**< Lowest scanned channel if adv_channels_en = true */
static uint8_t max_channel        = 80;    /**< highest scanned channel if adv_channels_en = true */
static uint32_t sweep_delay       = 1000;
static uint32_t scan_repeat_times = 1;

static bool uart_error = false;
static bool uart_send = false;
static bool scan_ble_adv = false;

#define uart_put(c) if(uart_send) { Serial.write(c); }

void set_uart_send_enable(bool enable)
{
	uart_send = enable;
//	if (uart_send) {
//		bsp_board_led_on(1);
//	} else {
//		bsp_board_led_off(1);
//	}
}

void uart_send_packet(uint8_t channel_number, uint8_t rssi)
{
	uart_put(0xff);
	uart_put(channel_number);
	uart_put(rssi);
}

void set_scan_ble_adv(bool enable) {
	scan_ble_adv = enable;
	for (uint8_t i = min_channel; i <= max_channel; ++i)
	{
		uart_send_packet(i, RSSI_NO_SIGNAL);
	}
}

void rssi_measurer_configure_radio(void)
{
	NRF_RADIO->POWER  = 1;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
	NVIC_EnableIRQ(RADIO_IRQn);

	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

#define WAIT_FOR( m ) do { while (!m); m = 0; } while(0)

uint8_t rssi_measurer_scan_channel(uint8_t channel_number)
{
	uint8_t sample;

	NRF_RADIO->FREQUENCY  = channel_number;
	NRF_RADIO->TASKS_RXEN = 1;

	WAIT_FOR(NRF_RADIO->EVENTS_READY);
	NRF_RADIO->TASKS_RSSISTART = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_RSSIEND);

	sample = 127 & NRF_RADIO->RSSISAMPLE;

	NRF_RADIO->TASKS_DISABLE = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_DISABLED);

	return sample;
}

uint8_t rssi_measurer_scan_channel_repeat(uint8_t channel_number)
{
	uint8_t sample;
	uint8_t max = RSSI_NO_SIGNAL;
	for (int i = 0; i <= scan_repeat_times; ++i) {
		sample = rssi_measurer_scan_channel(channel_number);
		// taking minimum since sample = -dBm.
		max = min(sample, max);
	}
	return max;
}

void uart_get_line()
{
	static const int bufsize = 64;
	uint8_t buf[bufsize];

	memset(buf+1, bufsize-1, 0);

  Serial.readBytesUntil('\r', buf, bufsize);

	char* q = (char*)&buf[0];
	if (strncmp(q, "set ", 4) == 0) {
		q += 4;
		if (strncmp(q, "delay ", 6) == 0) {
			q += 6;
			int d = atoi(q);
			sweep_delay = max(5, min(d, 1000));
			return;
		}
		if (strncmp(q, "repeat ", 7) == 0) {
			q += 7;
			int d = atoi(q);
			scan_repeat_times = max(1, min(d, 100));
			return;
		}
		if (strncmp(q, "channel min ", 12) == 0) {
			q += 12;
			int d = atoi(q);
			min_channel = max(1, min(d, max_channel));
			return;
		}
		if (strncmp(q, "channel max ", 12) == 0) {
			q += 12;
			int d = atoi(q);
			max_channel = max(min_channel, min(d, 100));
			return;
		}
		return;
	}
	if (strncmp(q, "start", 5) == 0) {
		set_uart_send_enable(true);
		return;
	}
	if (strncmp(q, "stop", 4) == 0) {
		set_uart_send_enable(false);
		return;
	}
	if (strncmp(q, "scan adv ", 9) == 0) {
		q += 9;
		if (strncmp(q, "true", 4) == 0) {
			set_scan_ble_adv(true);
			return;
		}
		if (strncmp(q, "false", 5) == 0) {
			set_scan_ble_adv(false);
		}
		return;
	}
	if (strncmp(q, "led", 3) == 0) {
		return;
	}
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() 
{
	uint8_t sample;
	if (scan_ble_adv) {
		sample = rssi_measurer_scan_channel_repeat(FREQ_ADV_CHANNEL_37);
		uart_send_packet(FREQ_ADV_CHANNEL_37, sample);
		sample = rssi_measurer_scan_channel_repeat(FREQ_ADV_CHANNEL_38);
		uart_send_packet(FREQ_ADV_CHANNEL_38, sample);
		sample = rssi_measurer_scan_channel_repeat(FREQ_ADV_CHANNEL_39);
		uart_send_packet(FREQ_ADV_CHANNEL_39, sample);
	} else {
		for (uint8_t i = min_channel; i <= max_channel; ++i)
		{
			sample = rssi_measurer_scan_channel_repeat(i);
			uart_send_packet(i, sample);
		}
	}

	uart_get_line();

	if (uart_error) {
		delay(max(sweep_delay, 500));
		uart_error = false;
		set_uart_send_enable(uart_send);
	}

	delay(sweep_delay);
}