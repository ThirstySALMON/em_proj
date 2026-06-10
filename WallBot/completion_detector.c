#include "completion_detector.h"
#include "ultrasonic.h"

static uint8_t sensor_is_open(const completion_detector_t *det, uint16_t mm) {
    if (mm == US_NO_READING) {
        return det->config.no_reading_is_open;
    }
    return mm > det->config.open_threshold_mm;
}

void completion_detector_init(completion_detector_t *det,
                              const completion_detector_config_t *config) {
    det->config = *config;
    completion_detector_reset(det);
}

void completion_detector_reset(completion_detector_t *det) {
    det->seeing_all_open = 0;
    det->all_open_start_ovf = 0;
}

uint8_t completion_detector_update(completion_detector_t *det,
                                   uint16_t front_mm,
                                   uint16_t left_mm,
                                   uint16_t right_mm,
                                   uint16_t now_ovf) {
    uint8_t all_open = sensor_is_open(det, front_mm)
                    && sensor_is_open(det, left_mm)
                    && sensor_is_open(det, right_mm);

    if (!all_open) {
        completion_detector_reset(det);
        return 0;
    }

    if (!det->seeing_all_open) {
        det->seeing_all_open = 1;
        det->all_open_start_ovf = now_ovf;
        return 0;
    }

    return ((uint16_t)(now_ovf - det->all_open_start_ovf)
            >= det->config.confirm_ovfs);
}
