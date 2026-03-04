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
    uint32_t phase_sp;
    uint64_t f_min;
    uint64_t f_max;
    uint32_t tau;
    pll_max_act max_act; 
} pll_config;

typedef struct
{
    pll_config conf;
    uint64_t curr_f;
} pll_instance;

void update_pll(ddsli_output_t out, ddsli_phase_ctrl_t *f_ctrl);