#ifndef __DISPLAY_H
#define __DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Message level type */
typedef enum {
  ALERT,
  WARNING,
  INFO,
} MessageLevel_t;

/* Message type */
typedef struct {
  uint8_t         id;
  MessageLevel_t  level;
  const char*     text;
  uint32_t        keep_time;
  uint32_t        color;
  bool            blink;
} Message_t;

void display_init(void);
void message_init(Message_t* message, MessageLevel_t level, const char* text);
bool display_message(const Message_t* message);
bool clear_message(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif
