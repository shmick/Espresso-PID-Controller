# ADS1115

This is a library to allow an Arduino environment (also esp8266, etc.) to
effectively utilize the ADS1115. It can also be used for other ADS111X devices
but was never tested with any of the simpler brethern.

The main goal here was to make it efficient and easily support asynchronous
reading of variables instead of the more common blocking read.

An example usage would be:

    ADS1115 adc;

    void setup() {
        adc.begin();
        adc.set_data_rate(ADS1115_DATA_RATE_860_SPS);
        adc.set_mode(ADS1115_MODE_SINGLE_SHOT);
        adc.set_mux(ADS1115_MUX_GND_AIN1);
        adc.set_pga(ADS1115_PGA_TWO);
    }

    void loop() {
        static int read_triggered = 0;

        if (!read_triggered) {
                if (adc.trigger_sample() == 0)
                        read_triggered = 1;
                else
                        Serial.println("adc read trigger failed (ads1115 not connected?)");
         } else {
                if (!adc.is_sample_in_progress()) {
                        float val = adc.read_sample_float();
                        Serial.print("Value read is ");
                        Serial.println(val);
                        read_triggered = 0;
                }
         }
    }


You can also use it with continuous sampling as:

    ADS1115 adc;

    void setup() {
        adc.begin();
        adc.set_data_rate(ADS1115_DATA_RATE_860_SPS);
        adc.set_mode(ADS1115_MODE_CONTINUOUS);
        adc.set_mux(ADS1115_MUX_GND_AIN1);
        adc.set_pga(ADS1115_PGA_TWO);

        if (adc.trigger_sample() != 0)
                Serial.println("adc read trigger failed (ads1115 not connected?)");
    }

    void loop() {
            /* You will be oversampling if the loop takes too short a time */
            float val = adc.read_sample_float();
            Serial.print("Value read is ");
            Serial.println(val);
    }

