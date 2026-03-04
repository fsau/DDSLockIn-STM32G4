#pragma once

#include "ddsli.h"

typedef enum
{
    PLL_MAX_LIM = 0,
    PLL_MAX_WRAP,
    PLL_MAX_RESET
} pll_max_act;

typedef struct 
{
    uint8_t source;
    float phase_sp;
    ddsli_phase_inc_t f_min;
    ddsli_phase_inc_t f_max;
    ddsli_phase_inc_t f_init;
    float tau;
    pll_max_act max_act; 
} pll_config;

typedef struct
{
    pll_config conf;
    // uint64_t curr_f;
} pll_instance;

void update_pll(pll_instance *inst, ddsli_output_t out, ddsli_phase_ctrl_t *f_ctrl);
