#ifndef UVX_APP_BATTERY_H
#define UVX_APP_BATTERY_H

#include <stdint.h>

/* Watch fc_prepare_timing in the debugger; retained after the ON ACK. */
typedef struct
{
    uint32_t elapsed_ms;
    uint32_t first_sweep_ms;
    uint32_t ready_ms;
    uint32_t max_service_gap_ms;
    uint32_t bus_wait_calls;
    uint32_t status_sweeps;
    uint32_t read_errors;
} UVX_FC_PREPARE_TIMING;

#ifdef __cplusplus
extern "C" {
#endif

void UVX_APP_Batt(void);
extern volatile UVX_FC_PREPARE_TIMING fc_prepare_timing;

#ifdef __cplusplus
}
#endif

#endif
