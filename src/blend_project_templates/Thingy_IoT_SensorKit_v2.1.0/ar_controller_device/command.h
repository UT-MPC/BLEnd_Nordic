#ifndef _UTMPCARCOMMAND_H
#define _UTMPCARCOMMAND_H


typedef struct {
  uint8_t cmd_id; /*!< Identifer of the command. */
  uint8_t data_len; /* Length of the data */
} command_t;

#define CMD_LIGHT 0
#define CMD_SOUND 1

#endif
