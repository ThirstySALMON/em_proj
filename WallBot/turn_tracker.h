#ifndef TURN_TRACKER_H_
#define TURN_TRACKER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void turns_init(void);
void turn_left(void);
void turn_right(void);
void sendReport(void);

#ifdef __cplusplus
}
#endif

#endif /* TURN_TRACKER_H */
