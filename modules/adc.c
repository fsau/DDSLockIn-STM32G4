#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>

#define ADC_BUF_LEN 1024
volatile uint32_t adc_buffer[ADC_BUF_LEN];
volatile uint8_t adc_capture_complete = 0;

void adc_dual_dma_init(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2 | GPIO6);
    
    rcc_periph_clock_enable(RCC_ADC12);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_DMAMUX1);
    
    adc_disable_deeppwd(ADC1);
    adc_disable_deeppwd(ADC2);

    for (volatile int i = 0; i < 10000; i++) __asm__("nop");

    adc_enable_regulator(ADC1);
    adc_enable_regulator(ADC2);

    for (volatile int i = 0; i < 10000; i++) __asm__("nop");

    adc_power_off(ADC1);
    adc_power_off(ADC2);

    ADC_CR(ADC1) &= ~ADC_CR_ADCALDIF;
    ADC_CR(ADC2) &= ~ADC_CR_ADCALDIF;
    
    adc_set_clk_source(ADC1,ADC_CCR_CKMODE_DIV4);
    adc_set_resolution(ADC1,ADC_CFGR1_RES_12_BIT);
    adc_set_resolution(ADC2,ADC_CFGR1_RES_12_BIT);

    for (volatile int i = 0; i < 1000000; i++) __asm__("nop");

    adc_calibrate(ADC1);
    adc_calibrate(ADC2);

    for (volatile int i = 0; i < 10000; i++) __asm__("nop");

    /*  Configure ADC1 (Master) */
    // Use continuous conversion mode for free-running
    adc_set_continuous_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time(ADC1, 0, ADC_SMPR_SMP_2DOT5CYC);  // Fastest sampling
    
    uint8_t adc1_channels[] = {3};  // Channel 3 (PA2)
    adc_set_regular_sequence(ADC1, 1, adc1_channels);

    /*  Configure ADC2 (Slave) */
    adc_set_continuous_conversion_mode(ADC2);
    adc_set_right_aligned(ADC2);
    adc_set_sample_time(ADC2, 1, ADC_SMPR_SMP_2DOT5CYC);  // Fastest sampling
    
    uint8_t adc2_channels[] = {3};  // Channel 3 (PA6)
    adc_set_regular_sequence(ADC2, 1, adc2_channels);

    ADC_CFGR1(ADC1) |= (1<<31); // JQDIS: Injected Queue disable
    ADC_CFGR1(ADC1) |= ADC_CFGR1_OVRMOD;
    ADC_CFGR1(ADC2) |= (1<<31); // JQDIS: Injected Queue disable
    ADC_CFGR1(ADC2) |= ADC_CFGR1_OVRMOD;

    /*  Configure Dual Mode - Regular Simultaneous */
    adc_set_multi_mode(ADC1, ADC_CCR_DUAL_REGULAR_SIMUL);
    ADC_CCR(ADC1) |= ADC_CCR_MDMA_12_10_BIT;
    
    adc_power_on(ADC1);
    adc_power_on(ADC2);
    
    /*  Configure DMA */
    dma_channel_reset(DMA1, DMA_CHANNEL1);
    
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_CDR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUF_LEN);
    
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    
    // 32-bit transfers: [ADC2:high 16 bits][ADC1:low 16 bits]
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_32BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_32BIT);
    
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);
    
    // Single buffer transfer (not circular)
    DMA_CCR(DMA1, DMA_CHANNEL1) &= ~DMA_CCR_CIRC;

    /* Route ADC1 DMA request to DMA1 Channel 1 */
    dmamux_set_dma_channel_request(
        DMAMUX1,
        DMA_CHANNEL1,
        DMAMUX_CxCR_DMAREQ_ID_ADC1
    );

    dmamux_enable_request_generator(DMAMUX1, DMA_CHANNEL1);
    
    /* No synchronization, no request generator */
    // dmamux_disable_dma_request_sync(DMAMUX1, DMA_CHANNEL1);
    // dmamux_disable_dma_request_event_generation(DMAMUX1, DMA_CHANNEL1);

    /*  Enable DMA for ADC1 */
    adc_enable_dma(ADC1);

    // Enable transfer complete interrupt
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    
    /*  Configure NVIC for DMA */
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    
    /*  Enable DMA channel */
    // dma_enable_channel(DMA1, DMA_CHANNEL1);
}

void adc_capture_buffer(uint16_t *adc1_data, uint16_t *adc2_data) {
    /* Reset completion flag */
    adc_capture_complete = 0;
    
    /* Reset DMA configuration */
    dma_disable_channel(DMA1, DMA_CHANNEL1);
    
    DMA_CCR(DMA1, DMA_CHANNEL1) &= ~DMA_CCR_CIRC;
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUF_LEN);
    
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
    dma_enable_channel(DMA1, DMA_CHANNEL1);
    
    // adc_set_single_conversion_mode(ADC1);
    // adc_set_single_conversion_mode(ADC2);
    adc_set_continuous_conversion_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC2);
    adc_set_right_aligned(ADC1);
    adc_set_right_aligned(ADC2);
    
    uint8_t adc1_channels[] = {3};  // Channel 3 (PA2)
    uint8_t adc2_channels[] = {3};  // Channel 3 (PA6)
    adc_set_regular_sequence(ADC1, 1, adc1_channels);
    adc_set_regular_sequence(ADC2, 1, adc2_channels);
    
    // Set sample time (fastest)
    adc_set_sample_time(ADC1, 0, ADC_SMPR_SMP_2DOT5CYC);
    adc_set_sample_time(ADC2, 1, ADC_SMPR_SMP_2DOT5CYC);
    
    // Configure dual simultaneous mode
    ADC_CCR(ADC1) &= ~ADC_CCR_DUAL_MASK;
    ADC_CCR(ADC1) |= ADC_CCR_DUAL_REGULAR_SIMUL;
    
    /* Start conversions - ADC2 will start automatically in dual mode */
    adc_start_conversion_regular(ADC1);
    
    /* Wait for DMA transfer to complete */
    while (!adc_capture_complete) {
        __asm__("nop");
        // Busy wait - you could add WFI here if interrupts are enabled
    }
    
    /* De-interleave the packed 32-bit data */
    for (int i = 0; i < ADC_BUF_LEN; i++) {
        uint32_t packed = adc_buffer[i];
        adc1_data[i] = (uint16_t)(packed & 0xFFFF);
        adc2_data[i] = (uint16_t)(packed >> 16);   
    }
}

void adc_timer_trigger_init(void)
{
    /* Enable clocks */
    rcc_periph_clock_enable(RCC_TIM6);
    rcc_periph_clock_enable(RCC_ADC12);

    /* Timer setup */
    uint32_t timer_clk = 80000000; // APB2 timer clock
    uint32_t adc_rate = 1000000;   // 1 MSa/s
    uint32_t prescaler = 0;        // no prescaler
    uint32_t arr = (timer_clk / (adc_rate * (prescaler + 1))) - 1;

    timer_set_prescaler(TIM6, prescaler);
    timer_set_period(TIM6, arr);

    /* TRGO on update event */
    timer_set_master_mode(TIM6, TIM_CR2_MMS_UPDATE);

    /* Enable counter */
    timer_enable_counter(TIM6);

    /* ADC setup */
    ADC_CFGR1(ADC1) &= ~ADC_CFGR1_CONT; // disable continuous mode

    adc_enable_external_trigger_regular(ADC1,
        ADC12_CFGR1_EXTSEL_TIM6_TRGO, ADC_CFGR1_EXTEN_RISING_EDGE);
}

void adc_capture_buffer_no_dma(uint16_t *adc1_data, uint16_t *adc2_data, uint32_t num_samples) {
    /* Capture buffer using polling mode (no DMA) */
    
    /*  Configure ADC1 and ADC2 for dual simultaneous mode */
    // Make sure ADCs are enabled and configured
    adc_set_single_conversion_mode(ADC1);
    adc_set_single_conversion_mode(ADC2);
    adc_set_right_aligned(ADC1);
    adc_set_right_aligned(ADC2);
    
    // Configure channels (assuming PA0 and PA1)
    uint8_t adc1_channels[] = {3};  // Channel 3 (PA2)
    uint8_t adc2_channels[] = {3};  // Channel 3 (PA6)
    adc_set_regular_sequence(ADC1, 1, adc1_channels);
    adc_set_regular_sequence(ADC2, 1, adc2_channels);
    
    // Set sample time (fastest)
    adc_set_sample_time(ADC1, 0, ADC_SMPR_SMP_2DOT5CYC);
    adc_set_sample_time(ADC2, 1, ADC_SMPR_SMP_2DOT5CYC);
    
    // Configure dual simultaneous mode
    ADC_CCR(ADC1) &= ~ADC_CCR_DUAL_MASK;
    ADC_CCR(ADC1) |= ADC_CCR_DUAL_REGULAR_SIMUL;
    
    /*  Capture samples */
    for (uint32_t i = 0; i < num_samples; i++) {
        /* Start conversion on both ADCs */
        // In dual simultaneous mode, starting ADC1 also starts ADC2
        adc_start_conversion_regular(ADC1);
        
        /* Wait for conversion complete */
        // Wait for EOC (End of Conversion) flag on ADC1
        while (!(ADC_ISR(ADC1) & ADC_ISR_EOC));
        
        /* Read ADC1 data */
        adc1_data[i] = adc_read_regular(ADC1);
        
        /* Clear EOC flag */
        ADC_ISR(ADC1) |= ADC_ISR_EOC;
        
        /* Note: In dual simultaneous mode, ADC2 data is available at the same time
         * but we need to read it from the common data register (ADC_CDR)
         */
        uint32_t combined_data = ADC_CDR(ADC1);
        
        /* Extract ADC2 data from high 16 bits */
        adc2_data[i] = (uint16_t)(combined_data >> 16);
        
        /* Optional: Add small delay between conversions if needed */
        // for (volatile int j = 0; j < 10; j++);
    }
    
    ADC_CR(ADC1) &= ~ADC_CR_ADSTART;
}

void adc_start_continuous_mode(void) {
    /* Configure DMA for circular mode */
    DMA_CCR(DMA1, DMA_CHANNEL1) |= DMA_CCR_CIRC;
    
    /* Start conversions */
    adc_start_conversion_regular(ADC1);
}

void adc_stop_continuous_mode(void) {
    /* Stop conversions */
    ADC_CR(ADC1) &= ~ADC_CR_ADSTART;
    
    /* Disable circular mode */
    DMA_CCR(DMA1, DMA_CHANNEL1) &= ~DMA_CCR_CIRC;
}

void dma1_channel1_isr(void) {
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        adc_capture_complete = 1;
    }
}