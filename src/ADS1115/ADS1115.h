#ifndef ADS1115_H
#define ADS1115_H

#include <stdint.h>

#define ADS1115_COMP_QUEUE_SHIFT 0
#define ADS1115_COMP_LATCH_SHIFT 2
#define ADS1115_COMP_POLARITY_SHIFT 3
#define ADS1115_COMP_MODE_SHIFT 4
#define ADS1115_DATA_RATE_SHIFT 5
#define ADS1115_MODE_SHIFT 8
#define ADS1115_PGA_SHIFT 9
#define ADS1115_MUX_SHIFT 12

enum ads1115_comp_queue {
	ADS1115_COMP_QUEUE_AFTER_ONE = 0,
	ADS1115_COMP_QUEUE_AFTER_TWO = 0x1 << ADS1115_COMP_QUEUE_SHIFT,
	ADS1115_COMP_QUEUE_AFTER_FOUR = 0x2 << ADS1115_COMP_QUEUE_SHIFT,
	ADS1115_COMP_QUEUE_DISABLE = 0x3 << ADS1115_COMP_QUEUE_SHIFT,
	ADS1115_COMP_QUEUE_MASK = 0x3 << ADS1115_COMP_QUEUE_SHIFT,
};

enum ads1115_comp_latch {
	ADS1115_COMP_LATCH_NO = 0,
	ADS1115_COMP_LATCH_YES = 1 << ADS1115_COMP_LATCH_SHIFT,
	ADS1115_COMP_LATCH_MASK = 1 << ADS1115_COMP_LATCH_SHIFT,
};

enum ads1115_comp_polarity {
	ADS1115_COMP_POLARITY_ACTIVE_LOW = 0,
	ADS1115_COMP_POLARITY_ACTIVE_HIGH = 1 << ADS1115_COMP_POLARITY_SHIFT,
	ADS1115_COMP_POLARITY_MASK = 1 << ADS1115_COMP_POLARITY_SHIFT,
};

enum ads1115_comp_mode {
	ADS1115_COMP_MODE_WINDOW = 0,
	ADS1115_COMP_MODE_HYSTERESIS = 1 << ADS1115_COMP_MODE_SHIFT,
	ADS1115_COMP_MODE_MASK = 1 << ADS1115_COMP_MODE_SHIFT,
};

enum ads1115_data_rate {
	ADS1115_DATA_RATE_8_SPS = 0,
	ADS1115_DATA_RATE_16_SPS = 0x1 << ADS1115_DATA_RATE_SHIFT,
	ADS1115_DATA_RATE_32_SPS = 0x2 << ADS1115_DATA_RATE_SHIFT,
	ADS1115_DATA_RATE_64_SPS = 0x3 << ADS1115_DATA_RATE_SHIFT,
	ADS1115_DATA_RATE_128_SPS = 0x4 << ADS1115_DATA_RATE_SHIFT,
	ADS1115_DATA_RATE_250_SPS = 0x5 << ADS1115_DATA_RATE_SHIFT,
	ADS1115_DATA_RATE_475_SPS = 0x6 << ADS1115_DATA_RATE_SHIFT,
	ADS1115_DATA_RATE_860_SPS = 0x7 << ADS1115_DATA_RATE_SHIFT,
	ADS1115_DATA_RATE_MASK = 0x7 << ADS1115_DATA_RATE_SHIFT,
};

enum ads1115_mode {
	ADS1115_MODE_CONTINUOUS = 0,
	ADS1115_MODE_SINGLE_SHOT = 1 << ADS1115_MODE_SHIFT,
	ADS1115_MODE_MASK = 1 << ADS1115_MODE_SHIFT,
};

enum ads1115_pga {
	ADS1115_PGA_TWO_THIRDS = 0,
	ADS1115_PGA_ONE = 0x1 << ADS1115_PGA_SHIFT,
	ADS1115_PGA_TWO = 0x2 << ADS1115_PGA_SHIFT,
	ADS1115_PGA_FOUR = 0x3 << ADS1115_PGA_SHIFT,
	ADS1115_PGA_EIGHT = 0x4 << ADS1115_PGA_SHIFT,
	ADS1115_PGA_SIXTEEN = 0x5 << ADS1115_PGA_SHIFT,
	ADS1115_PGA_MASK = 0x7 << ADS1115_PGA_SHIFT,
};

enum ads1115_mux {
	ADS1115_MUX_DIFF_AIN0_AIN1 = 0,
	ADS1115_MUX_DIFF_AIN0_AIN3 = 0x1 << ADS1115_MUX_SHIFT,
	ADS1115_MUX_DIFF_AIN1_AIN3 = 0x2 << ADS1115_MUX_SHIFT,
	ADS1115_MUX_DIFF_AIN2_AIN3 = 0x3 << ADS1115_MUX_SHIFT,
	ADS1115_MUX_GND_AIN0 = 0x4 << ADS1115_MUX_SHIFT,
	ADS1115_MUX_GND_AIN1 = 0x5 << ADS1115_MUX_SHIFT,
	ADS1115_MUX_GND_AIN2 = 0x6 << ADS1115_MUX_SHIFT,
	ADS1115_MUX_GND_AIN3 = 0x7 << ADS1115_MUX_SHIFT,
	ADS1115_MUX_MASK = 0x7 << ADS1115_MUX_SHIFT,
};

class ADS1115 {
public:
	ADS1115(uint8_t address = 0x48);

	void begin();
	uint8_t trigger_sample();
	uint8_t reset();
	bool is_sample_in_progress();
	int16_t read_sample();
	float sample_to_float(int16_t val);
	float read_sample_float();

	void set_comp_queue(enum ads1115_comp_queue val) { set_config(val, ADS1115_COMP_QUEUE_MASK); }
	void set_comp_latching(enum ads1115_comp_latch val) { set_config(val, ADS1115_COMP_LATCH_MASK); }
	void set_comp_polarity(enum ads1115_comp_polarity val) { set_config(val, ADS1115_COMP_POLARITY_MASK); }
	void set_comp_mode(enum ads1115_comp_mode val) { set_config(val, ADS1115_COMP_MODE_MASK); }
	void set_data_rate(enum ads1115_data_rate val) { set_config(val, ADS1115_DATA_RATE_MASK); }
	void set_mode(enum ads1115_mode val) { set_config(val, ADS1115_MODE_MASK); }
	void set_pga(enum ads1115_pga val) { set_config(val, ADS1115_PGA_MASK); m_voltage_range = val >> ADS1115_PGA_SHIFT; }
	void set_mux(enum ads1115_mux val) { set_config(val, ADS1115_MUX_MASK); }

private:
	void set_config(uint16_t val, uint16_t mask) {
		m_config = (m_config & ~mask) | val;
	}

	uint8_t write_register(uint8_t reg, uint16_t val);
	uint16_t read_register(uint8_t reg);

	uint8_t m_address;
	uint16_t m_config;
	int m_voltage_range;
};

#endif
