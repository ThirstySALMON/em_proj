#ifndef COMPLETION_DETECTOR_H
#define COMPLETION_DETECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t open_threshold_mm;
    uint16_t confirm_ovfs;
    uint8_t  no_reading_is_open;
} completion_detector_config_t;

typedef struct {
    completion_detector_config_t config;
    uint8_t  seeing_all_open;
    uint16_t all_open_start_ovf;
} completion_detector_t;

void completion_detector_init(completion_detector_t *det,
                              const completion_detector_config_t *config);
void completion_detector_reset(completion_detector_t *det);
uint8_t completion_detector_update(completion_detector_t *det,
                                   uint16_t front_mm,
                                   uint16_t left_mm,
                                   uint16_t right_mm,
                                   uint16_t now_ovf);

#ifdef __cplusplus
}
#endif

#endif /* COMPLETION_DETECTOR_H */
