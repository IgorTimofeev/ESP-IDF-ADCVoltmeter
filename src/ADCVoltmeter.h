#pragma once

#include <algorithm>

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

namespace YOBA {
	class ADCVoltmeter {
		public:
			ADCVoltmeter(
				const adc_unit_t ADCUnit,
				const adc_oneshot_unit_handle_t* ADCOneshotUnit,
				const adc_channel_t ADCChannel,

				const uint32_t inputVoltageMinMV,
				const uint32_t inputVoltageMaxMV,

				const uint32_t dividerResistanceR1,
				const uint32_t dividerResistanceR2,

				const uint8_t multisamplingThreshold = 8
			);

			virtual ~ADCVoltmeter() = default;

			virtual void setup();

			uint32_t getInputVoltageMinMV() const;

			uint32_t getInputVoltageMaxMV() const;

			uint32_t getDividerResistanceR1() const;

			uint32_t getDividerResistanceR2() const;

			uint8_t getMultisamplingThreshold() const;

			uint16_t getVoltageMV() const;

			uint8_t getCharge() const;

			float getChargeF() const;

			virtual void tick();

		private:
			adc_unit_t _ADCUnit;
			const adc_oneshot_unit_handle_t* _ADCOneshotUnit;
			adc_channel_t _ADCChannel;
			uint32_t _inputVoltageMinMV;
			uint32_t _inputVoltageMaxMV;
			uint32_t _dividerResistanceR1;
			uint32_t _dividerResistanceR2;
			uint8_t _multisamplingThreshold;

			adc_cali_handle_t _ADCCaliHandle {};
			uint32_t _ADCSampleSum = 0;
			uint8_t _ADSSampleIndex = 0;
			uint16_t _voltage = 0;
	};

	class TransistorControlledADCVoltmeter : public ADCVoltmeter {
		public:
			TransistorControlledADCVoltmeter(
				const gpio_num_t transistorPin,

				const adc_unit_t ADCUnit,
				const adc_oneshot_unit_handle_t* ADCOneshotUnit,
				const adc_channel_t ADCChannel,

				const uint32_t inputVoltageMinMV,
				const uint32_t inputVoltageMaxMV,

				const uint32_t dividerResistanceR1,
				const uint32_t dividerResistanceR2,

				const uint8_t multisamplingThreshold = 8
			);

			void setup() override;

			void tick() override;

		private:
			gpio_num_t _transistorPin;
	};
}