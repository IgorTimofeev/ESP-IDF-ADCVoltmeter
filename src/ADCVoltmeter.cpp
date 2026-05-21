#include "ADCVoltmeter.h"

namespace YOBA {
	ADCVoltmeter::ADCVoltmeter(
		const adc_unit_t ADCUnit,
		const adc_oneshot_unit_handle_t* ADCOneshotUnit,
		const adc_channel_t ADCChannel,

		const uint32_t sourceVoltageMinMV,
		const uint32_t sourceVoltageMaxMV,

		const uint32_t dividerResistanceR1,
		const uint32_t dividerResistanceR2,

		const uint8_t multisamplingThreshold
	) :
		_ADCUnit(ADCUnit),
		_ADCOneshotUnit(ADCOneshotUnit),
		_ADCChannel(ADCChannel),

		_sourceVoltageMinMV(sourceVoltageMinMV),
		_sourceVoltageMaxMV(sourceVoltageMaxMV),

		_dividerResistanceR1(dividerResistanceR1),
		_dividerResistanceR2(dividerResistanceR2),

		_multisamplingThreshold(multisamplingThreshold)
	{

	}

	void ADCVoltmeter::setup() {
		// Setting up ADC
		adc_oneshot_chan_cfg_t channelConfig {};
		channelConfig.atten = ADC_ATTEN_DB_12;
		channelConfig.bitwidth = ADC_BITWIDTH_12;
		ESP_ERROR_CHECK(adc_oneshot_config_channel(*_ADCOneshotUnit, _ADCChannel, &channelConfig));

		#ifdef ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
		adc_cali_curve_fitting_config_t fittingConfig {};
		fittingConfig.unit_id = _ADCUnit;
		fittingConfig.atten = ADC_ATTEN_DB_12;
		fittingConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
		ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&fittingConfig, &_ADCCaliHandle));

		#else
		adc_cali_line_fitting_config_t fittingConfig {};
		fittingConfig.unit_id = _ADCUnit;
		fittingConfig.atten = ADC_ATTEN_DB_12;
		fittingConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
		fittingConfig.default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
		ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&fittingConfig, &_ADCCaliHandle));

		#endif

		// Reading first sample & computing initial voltage without averaging
		computeVoltage(readSample());
	}

	uint16_t ADCVoltmeter::readSample() {
		int sample;
		ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_get_calibrated_result(*_ADCOneshotUnit, _ADCCaliHandle, _ADCChannel, &sample));

		return sample;
	}

	void ADCVoltmeter::computeVoltage(const uint16_t ADCValue) {
		const uint16_t GPIOVoltageMaxMV = _sourceVoltageMaxMV * _dividerResistanceR2 / (_dividerResistanceR1 + _dividerResistanceR2);
		const auto voltage = _sourceVoltageMinMV + (_sourceVoltageMaxMV - _sourceVoltageMinMV) * ADCValue / GPIOVoltageMaxMV;

		// ESP_LOGI("bat", "voltage after: %d", voltage);

		_voltage = static_cast<uint16_t>(voltage);
	}

	void ADCVoltmeter::tick() {
		_ADCSampleSum += readSample();
		_ADSSampleIndex++;

		if (_ADSSampleIndex < _multisamplingThreshold)
			return;

		computeVoltage(_ADCSampleSum / _multisamplingThreshold);

		_ADCSampleSum = 0;
		_ADSSampleIndex = 0;
	}

	uint32_t ADCVoltmeter::getsourceVoltageMinMV() const {
		return _sourceVoltageMinMV;
	}

	uint32_t ADCVoltmeter::getsourceVoltageMaxMV() const {
		return _sourceVoltageMaxMV;
	}

	uint32_t ADCVoltmeter::getDividerResistanceR1() const {
		return _dividerResistanceR1;
	}

	uint32_t ADCVoltmeter::getDividerResistanceR2() const {
		return _dividerResistanceR2;
	}

	uint8_t ADCVoltmeter::getMultisamplingThreshold() const {
		return _multisamplingThreshold;
	}

	uint16_t ADCVoltmeter::getVoltageMV() const {
		return _voltage;
	}

	uint8_t ADCVoltmeter::getCharge() const {
		if (_voltage <= _sourceVoltageMinMV) {
			return 0;
		}

		if (_voltage >= _sourceVoltageMaxMV) {
			return 0xFF;
		}

		return static_cast<uint8_t>((_voltage - _sourceVoltageMinMV) * 0xFF / (_sourceVoltageMaxMV - _sourceVoltageMinMV));
	}

	float ADCVoltmeter::getChargeF() const {
		return static_cast<float>(getCharge()) /  static_cast<float>(0xFF);
	}

	TransistorControlledADCVoltmeter::TransistorControlledADCVoltmeter(
		const gpio_num_t transistorPin,

		const adc_unit_t ADCUnit,
		const adc_oneshot_unit_handle_t* ADCOneshotUnit,
		const adc_channel_t ADCChannel,

		const uint32_t sourceVoltageMinMV,
		const uint32_t sourceVoltageMaxMV,

		const uint32_t dividerResistanceR1,
		const uint32_t dividerResistanceR2,

		const uint8_t multisamplingThreshold
	) :
		ADCVoltmeter(
			ADCUnit,
			ADCOneshotUnit,
			ADCChannel,

			sourceVoltageMinMV,
			sourceVoltageMaxMV,

			dividerResistanceR1,
			dividerResistanceR2,

			multisamplingThreshold
		),
		_transistorPin(transistorPin)
	{

	}

	void TransistorControlledADCVoltmeter::setup() {
		// Transistor pin
		{
			gpio_config_t g = {};
			g.pin_bit_mask = 1ULL << _transistorPin;
			g.mode = GPIO_MODE_INPUT;
			g.pull_up_en = GPIO_PULLUP_ENABLE;
			g.pull_down_en = GPIO_PULLDOWN_DISABLE;
			g.intr_type = GPIO_INTR_LOW_LEVEL;
			gpio_config(&g);

			gpio_set_level(_transistorPin, 0);
		}

		// ADC
		ADCVoltmeter::setup();
	}

	uint16_t TransistorControlledADCVoltmeter::readSample() {
		gpio_set_level(_transistorPin, true);

		const auto sample = ADCVoltmeter::readSample();

		gpio_set_level(_transistorPin, false);

		return sample;
	}
}
