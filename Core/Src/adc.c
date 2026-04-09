#include "main.h"
#include "app.h"

uint16_t MCU_KR_ADC_VAL[MCU_KR_NUM_ADC_CHANNEL];

static uint16_t adc_sma_buf[MCU_KR_FILTERSIZE];
static uint32_t adc_sma_sum = 0;
static uint16_t adc_sma_fill = 0;
static uint16_t adc_sma_idx = 0;
static volatile uint16_t u24_adc_filtered = 0;

uint16_t ADC_GetU24Filtered(void) { return u24_adc_filtered; }

static uint16_t SmaProcess(uint16_t val)
{
    if (adc_sma_fill == MCU_KR_FILTERSIZE) {
        adc_sma_sum -= adc_sma_buf[adc_sma_idx];
    } else {
        adc_sma_fill++;
    }

    adc_sma_buf[adc_sma_idx] = val;
    adc_sma_sum += val;

    adc_sma_idx++;
    if (adc_sma_idx >= MCU_KR_FILTERSIZE) {
        adc_sma_idx = 0;
    }

    return (uint16_t)(adc_sma_sum / adc_sma_fill);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) {
        return;
    }
    u24_adc_filtered = SmaProcess(MCU_KR_ADC_VAL[0]);
}

