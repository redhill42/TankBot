#include "main.h"
#include "cmsis_os.h"
#include "ps2controller.h"
#include "delay.h"

#define CS_H    PS2_CS_GPIO_Port->BSRR=PS2_CS_Pin
#define CS_L    PS2_CS_GPIO_Port->BSRR=PS2_CS_Pin<<16

#define CMD_H   PS2_CMD_GPIO_Port->BSRR=PS2_CMD_Pin
#define CMD_L   PS2_CMD_GPIO_Port->BSRR=PS2_CMD_Pin<<16

#define CLK_H   PS2_CLK_GPIO_Port->BSRR=PS2_CLK_Pin
#define CLK_L   PS2_CLK_GPIO_Port->BSRR=PS2_CLK_Pin<<16

#define DATA    ((PS2_DATA_GPIO_Port->IDR&PS2_DATA_Pin)!=(uint32_t)GPIO_PIN_RESET)

// Half a clock cycle; 10us seems to be about right
#define CTRL_CLK 10

extern osTimerId ps2_timerHandle;
extern osMutexId ps2_mutexHandle;
static uint8_t ps2_data[9];

/**
 * The actual serial transfer. Handles clock. The PS2 controler is full duplex,
 * so this will send a byte as well as receive one.
 */
static uint8_t ps2_send_receive(uint8_t cmd) {
  uint8_t data = 0;
  uint16_t bit;
  
  for (bit=1; bit<0x100; bit<<=1) {
    // drop the clock
    CLK_L;
    
    // set the command (outgoing) pin
    if (cmd & bit) {
      CMD_H;
    } else {
      CMD_L;
    }
    
    // wait half the clock cycle
    HAL_Delay_us(CTRL_CLK);
    
    // raise the clock to HIGH
    CLK_H;
    
    // at which point read the data
    if (DATA) {
      data |= bit;
    }
    
    // and wait the other half of the clock cycle
    HAL_Delay_us(CTRL_CLK);
  }
  
  // Clock should already be high at this point, but just to be sure.
  CLK_H;
  
  return data;
}

/**
 * Send a command using the send receive method.
 */
static void ps2_send_command(uint8_t cmd[], uint8_t out[], uint8_t size) {
  // Before submit each command packet, you must set attention low; once
  // you are done each packet, return it high. You have to toggle the line
  // before submit another command.
  CS_L; 
  for (uint8_t i=0; i<size; i++) {
    uint8_t val = ps2_send_receive(cmd[i]);
    if (out != NULL) {
      out[i] = val;
    }
  }
  CS_H;
}

static void ps2_analog_mode(void) {
  // Enter config mode
  static uint8_t enter_config_command[] = 
    {0x01, 0x43, 0x00, 0x01, 0x00};
  ps2_send_command(enter_config_command, NULL, sizeof(enter_config_command));
  
  // Lock to Analog Mode on stick
  static uint8_t analog_mode_command[] = 
    {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
  ps2_send_command(analog_mode_command, NULL, sizeof(analog_mode_command));
  
  // Exit config mode
  static uint8_t exit_config_command[] = 
    {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
  ps2_send_command(exit_config_command, NULL, sizeof(exit_config_command));
}

/**
 * Read raw ps2 data.
 */
void ps2_raw_read(const void* args) {
  static uint8_t read_command[] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  osMutexWait(ps2_mutexHandle, 10000);
  ps2_send_command(read_command, ps2_data, sizeof(ps2_data));
  osMutexRelease(ps2_mutexHandle);
}

/**
 * Initialize the pins, and set the controller up to the correct mode. This must
 * be called before any other functions are called.
 */
void ps2_init(void) {
  // Initialize pins
  CS_H; CLK_H; CMD_H;
  
  // Init by polling once
  ps2_raw_read(NULL);
  
  do {
    // Lock to Analog Mode on stick
    ps2_analog_mode();
    
    // Poll to ensure mode set
    ps2_raw_read(NULL);
  } while ((ps2_data[1] & 0xF0) != 0x70);
  
  // Initialize timer to read gamepad periodically
  osTimerStart(ps2_timerHandle, 20);
}

/**
 * Read the gamepad. You need to call this whenever you want update state.
 */
uint16_t ps2_get_key(uint16_t keep) {
  osMutexWait(ps2_mutexHandle, 10000);
  
  // Retrieve key value from raw ps2 data
  uint16_t* pkey = (uint16_t*)(ps2_data+3);
  uint16_t key = *pkey;
  
  // Reset key value unless the key is kept
  *pkey |= ~keep;
  
  osMutexRelease(ps2_mutexHandle);
  
  // Returns the user key value
  return ~key;
}

uint8_t ps2_get_stick(uint8_t stick) {
  if (ps2_data[1] == 0x73)
    return ps2_data[stick];
  return 128;
}
