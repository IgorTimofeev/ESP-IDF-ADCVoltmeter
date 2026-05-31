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
		constexpr static adc_atten_t ADCAttenuation = ADC_ATTEN_DB_12;
		constexpr static adc_bitwidth_t ADCBitwidth = ADC_BITWIDTH_12;

		// Setting up ADC
		adc_oneshot_chan_cfg_t channelConfig {};
		channelConfig.atten = ADCAttenuation;
		channelConfig.bitwidth = ADCBitwidth;
		ESP_ERROR_CHECK(adc_oneshot_config_channel(*_ADCOneshotUnit, _ADCChannel, &channelConfig));

		#ifdef ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
			adc_cali_curve_fitting_config_t fittingConfig {};
			fittingConfig.unit_id = _ADCUnit;
			fittingConfig.atten = ADCAttenuation;
			fittingConfig.bitwidth = ADCBitwidth;
			ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&fittingConfig, &_ADCCaliHandle));

		#else
			adc_cali_line_fitting_config_t fittingConfig {};
			fittingConfig.unit_id = _ADCUnit;
			fittingConfig.atten = ADCAttenuation;
			fittingConfig.bitwidth = ADCBitwidth;
			fittingConfig.default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
			ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&fittingConfig, &_ADCCaliHandle));

		#endif

		// Reading & computing initial divider voltage without averaging
		computeDividerVoltage(readADCVoltage());
	}

	uint16_t ADCVoltmeter::readADCVoltage() {
		int ADCVoltageMV;
		ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_get_calibrated_result(*_ADCOneshotUnit, _ADCCaliHandle, _ADCChannel, &ADCVoltageMV));

		return ADCVoltageMV;
	}

	void ADCVoltmeter::computeDividerVoltage(const uint16_t ADCVoltageMV) {
		const auto dividerVoltageMaxMV = _sourceVoltageMaxMV * _dividerResistanceR2 / (_dividerResistanceR1 + _dividerResistanceR2);

		_sourceVoltageMV = _sourceVoltageMinMV + (_sourceVoltageMaxMV - _sourceVoltageMinMV) * ADCVoltageMV / dividerVoltageMaxMV;
	}

	void ADCVoltmeter::tick() {
		_sampleSum += readADCVoltage();
		_sampleIndex++;

		if (_sampleIndex < _multisamplingThreshold)
			return;

		computeDividerVoltage(_sampleSum / _multisamplingThreshold);

		_sampleSum = 0;
		_sampleIndex = 0;
	}

	uint32_t ADCVoltmeter::getSourceVoltageMinMV() const {
		return _sourceVoltageMinMV;
	}

	uint32_t ADCVoltmeter::getSourceVoltageMaxMV() const {
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

	uint32_t ADCVoltmeter::getVoltageMV() const {
		return _sourceVoltageMV;
	}

	uint16_t ADCVoltmeter::getCharge16() const {
		if (_sourceVoltageMV <= _sourceVoltageMinMV)
			return 0;

		if (_sourceVoltageMV >= _sourceVoltageMaxMV)
			return 0xFFFF;

		return static_cast<uint16_t>((_sourceVoltageMV - _sourceVoltageMinMV) * 0xFFFF / (_sourceVoltageMaxMV - _sourceVoltageMinMV));
	}

	float ADCVoltmeter::getChargeF() const {
		return static_cast<float>(getCharge16()) / static_cast<float>(0xFFFF);
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

	uint16_t TransistorControlledADCVoltmeter::readADCVoltage() {
		gpio_set_level(_transistorPin, true);

		const auto sample = ADCVoltmeter::readADCVoltage();

		gpio_set_level(_transistorPin, false);

		return sample;
	}
}
