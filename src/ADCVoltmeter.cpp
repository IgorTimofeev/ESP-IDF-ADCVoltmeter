#include "ADCVoltmeter.h"

namespace YOBA {
	ADCVoltmeter::ADCVoltmeter(
		const adc_unit_t ADCUnit,
		const adc_oneshot_unit_handle_t* ADCOneshotUnit,
		const adc_channel_t ADCChannel,

		const uint32_t inputVoltageMinMV,
		const uint32_t inputVoltageMaxMV,

		const uint32_t dividerResistanceR1,
		const uint32_t dividerResistanceR2,

		const uint8_t multisamplingThreshold = 8
	) :
		_ADCUnit(ADCUnit),
		_ADCOneshotUnit(ADCOneshotUnit),
		_ADCChannel(ADCChannel),

		_inputVoltageMinMV(inputVoltageMinMV),
		_inputVoltageMaxMV(inputVoltageMaxMV),

		_dividerResistanceR1(dividerResistanceR1),
		_dividerResistanceR2(dividerResistanceR2),

		_multisamplingThreshold(multisamplingThreshold)
	{

	}

	void ADCVoltmeter::setup() {
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
	}

	uint32_t ADCVoltmeter::getInputVoltageMinMV() const {
		return _inputVoltageMinMV;
	}

	uint32_t ADCVoltmeter::getInputVoltageMaxMV() const {
		return _inputVoltageMaxMV;
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
		if (_voltage <= _inputVoltageMinMV) {
			return 0;
		}

		if (_voltage >= _inputVoltageMaxMV) {
			return 0xFF;
		}

		return static_cast<uint8_t>((_voltage - _inputVoltageMinMV) * 0xFF / (_inputVoltageMaxMV - _inputVoltageMinMV));
	}

	float ADCVoltmeter::getChargeF() const {
		return static_cast<float>(getCharge()) /  static_cast<float>(0xFF);
	}

	void ADCVoltmeter::tick() {
		int ADCSample;
		const auto error = adc_oneshot_get_calibrated_result(*_ADCOneshotUnit, _ADCCaliHandle, _ADCChannel, &ADCSample);

		// Timeout on same oneshot unit?
		if (error != ESP_OK) {
			ESP_ERROR_CHECK_WITHOUT_ABORT(error);
			return;
		}

		_ADCSampleSum += ADCSample;
		_ADSSampleIndex++;

		if (_ADSSampleIndex < _multisamplingThreshold)
			return;

		const auto ADCValue = _ADCSampleSum / _multisamplingThreshold;

		_ADCSampleSum = 0;
		_ADSSampleIndex = 0;

		// ESP_LOGI("bat", "voltage before: %d", voltage);

		// Restoring real voltage
		const uint16_t voltageOnDividerMaxMV = _inputVoltageMaxMV * _dividerResistanceR2 / (_dividerResistanceR1 + _dividerResistanceR2);
		const auto voltage = _inputVoltageMinMV + (_inputVoltageMaxMV - _inputVoltageMinMV) * ADCValue / voltageOnDividerMaxMV;

		// ESP_LOGI("bat", "voltage after: %d", voltage);

		_voltage = static_cast<uint16_t>(voltage);
	}

	TransistorControlledADCVoltmeter::TransistorControlledADCVoltmeter(
		const gpio_num_t transistorPin,

		const adc_unit_t ADCUnit,
		const adc_oneshot_unit_handle_t* ADCOneshotUnit,
		const adc_channel_t ADCChannel,

		const uint32_t inputVoltageMinMV,
		const uint32_t inputVoltageMaxMV,

		const uint32_t dividerResistanceR1,
		const uint32_t dividerResistanceR2,

		const uint8_t multisamplingThreshold
	) :
		ADCVoltmeter(
			ADCUnit,
			ADCOneshotUnit,
			ADCChannel,

			inputVoltageMinMV,
			inputVoltageMaxMV,

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

	void TransistorControlledADCVoltmeter::tick() {
		gpio_set_level(_transistorPin, true);

		ADCVoltmeter::tick();

		gpio_set_level(_transistorPin, false);
	}
}
