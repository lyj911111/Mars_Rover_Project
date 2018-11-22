/*
 * robotics_arm.c
 *
 *  Created on: 2018. 11. 21.
 *      Author: JuYeong
 */
#include "robotics_arm.h"


extern UART_HandleTypeDef huart2;

uint16_t update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

const uint8_t robotic_reg_eeprom[]={
          MODEL_NUMBER_REG,
          MODEL_INFORMATION_REG,
          FIRMWARE_VERSION_REG,
          TRANSMISSION_ID_REG,
          BAUD_RATE_REG,
          RETURN_DELAY_REG,
          DRIVE_MODE_REG,
          OPERATING_MODE_REG,
          SECONDARY_ID_REG,
          PROTOCOL_VERSION_REG,
          HOMING_OFFSET_REG,
          MOVING_THRESHOLD_REG,
          TEMPERATURE_LIMIT_REG,
          MAX_VOLTAGE_LIMIT_REG,
          MIN_VOLTAGE_LIMIT_REG,
          PWM_LIMIT_REG,
          ACCELERATION_LIMIT_REG,
          VELOCITY_LIMIT_REG,
          MAX_POSITION_LIMIT_REG,
          MIN_POSITION_LIMIT_REG,
          SHUTDOWN_REG,
        };
const uint8_t robotic_reg_eeprom_size[]={
        MODEL_NUMBER_REG_SIZE,
        MODEL_INFORMATION_REG_SIZE,
        FIRMWARE_VERSION_REG_SIZE,
        TRANSMISSION_ID_REG_SIZE,
        BAUD_RATE_REG_SIZE,
        RETURN_DELAY_REG_SIZE,
        DRIVE_MODE_REG_SIZE,
        OPERATING_MODE_REG_SIZE,
        SECONDARY_ID_REG_SIZE,
        PROTOCOL_VERSION_REG_SIZE,
        HOMING_OFFSET_REG_SIZE,
        MOVING_THRESHOLD_REG_SIZE,
        TEMPERATURE_LIMIT_REG_SIZE,
        MAX_VOLTAGE_LIMIT_REG_SIZE,
        MIN_VOLTAGE_LIMIT_REG_SIZE,
        PWM_LIMIT_REG_SIZE,
        ACCELERATION_LIMIT_REG_SIZE,
        VELOCITY_LIMIT_REG_SIZE,
        MAX_POSITION_LIMIT_REG_SIZE,
        MIN_POSITION_LIMIT_REG_SIZE,
        SHUTDOWN_REG_SIZE,
      };

const uint8_t robotic_reg_ram[]={
          TORQUE_ENABLE_REG,
          LED_ON_OFF_REG,
          STATUS_RETURN_LEVEL_REG,
          REGISTERED_INSTRUCTION_REG,
          HARDWARE_ERROR_STATUS_REG,
          VELOCITY_I_GAIN_REG,
          VELOCITY_P_GAIN_REG,
          POSITION_D_GAIN_REG,
          POSITION_I_GAIN_REG,
          POSITION_P_GAIN_REG,
          FEEDFORWARD_2ND_GAIN_REG,
          FEEDFORWARD_1ST_GAIN_REG,
          BUS_WATCHDOG_REG,
          GOAL_PWM_REG,
          GOAL_VELOCITY_REG,
          PROFILE_ACCELERATION_REG,
          PROFILE_VELOCITY_REG,
          GOAL_POSTION_REG,
        };
const uint8_t robotic_reg_ram_size[]={
          TORQUE_ENABLE_REG_SIZE,
          LED_ON_OFF_REG_SIZE,
          STATUS_RETURN_LEVEL_REG_SIZE,
          REGISTERED_INSTRUCTION_REG_SIZE,
          HARDWARE_ERROR_STATUS_REG_SIZE,
          VELOCITY_I_GAIN_REG_SIZE,
          VELOCITY_P_GAIN_REG_SIZE,
          POSITION_D_GAIN_REG_SIZE,
          POSITION_I_GAIN_REG_SIZE,
          POSITION_P_GAIN_REG_SIZE,
          FEEDFORWARD_2ND_GAIN_REG_SIZE,
          FEEDFORWARD_1ST_GAIN_REG_SIZE,
          BUS_WATCHDOG_REG_SIZE,
          GOAL_PWM_REG_SIZE,
          GOAL_VELOCITY_REG_SIZE,
          PROFILE_ACCELERATION_REG_SIZE,
          PROFILE_VELOCITY_REG_SIZE,
          GOAL_POSTION_REG_SIZE,
        };


enum{
  HEAD1,HEAD2,HEAD3,RESERVED,PACKET_ID,LENGTH_LOW,LENGTH_HIGH,COMMAND,REG_LOW,REG_HIGH
};
void arm_write_ram(uint8_t id,uint8_t reg,uint32_t data)
{
  uint8_t str[20]={0};
  uint8_t str_para_num=robotic_reg_ram_size[reg];
  uint8_t str_size;
  uint8_t str_pos = 0;
  uint16_t crc;
  str[str_pos++] = 0xFF;      //HEAD1
  str[str_pos++] = 0xFF;      //HEAD2
  str[str_pos++] = 0xFD;      //HEAD3
  str[str_pos++] = 0x00;      //RESERVED
  str[str_pos++] = id;        //PACKET_ID
  str[str_pos++] = 5+str_para_num;  //LENGTH_LOW
  str[str_pos++] = 0;               //LENGTH_HIGH
  str[str_pos++] = WRITE;     //COMMAND
  str[str_pos++] = robotic_reg_ram[reg];  //REG_LOW
  str[str_pos++] = 0;                     //REG_HIGH
  for(int i=0 ; i<str_para_num ; i++){
    str[str_pos++] = (uint8_t)((data>>(i*8)) & 0xFF);
  }
  str_size = str_pos;
  crc = update_crc(0, str, str_size);
  str[str_pos++] = crc & 0x00FF;
  str[str_pos++] = (crc>>8) & 0x00FF;
  str_size = str_pos;

  HAL_UART_Transmit(&huart2, str, str_size,10);
}
void arm_write_eeprom(uint8_t id,uint8_t reg,uint32_t data)
{
  uint8_t str[20]={0};
  uint8_t str_para_num=robotic_reg_eeprom_size[reg];
  uint8_t str_size;
  uint8_t str_pos=0;
  uint16_t crc;
  str[str_pos++] = 0xFF;      //HEAD1
  str[str_pos++] = 0xFF;      //HEAD2
  str[str_pos++] = 0xFD;      //HEAD3
  str[str_pos++] = 0x00;      //RESERVED
  str[str_pos++] = id;        //PACKET_ID
  str[str_pos++] = 5+str_para_num;  //LENGTH_LOW
  str[str_pos++] = 0;               //LENGTH_HIGH
  str[str_pos++] = WRITE;     //COMMAND
  str[str_pos++] = robotic_reg_eeprom[reg];   //REG_LOW
  str[str_pos++] = 0;                         //REG_HIGH
  for(int i=0 ; i<str_para_num ; i++){
    str[str_pos++] = (uint8_t)((data>>(i*8)) & 0xFF);
  }
  str_size = str_pos;
  crc = update_crc(0, str, str_size);
  str[str_pos++] = (uint8_t)(crc & 0x00FF);
  str[str_pos++] = (uint8_t)((crc>>8) & 0x00FF);
  str_size = str_pos;

  HAL_UART_Transmit(&huart2, str, str_size,10);
}
void arm_factory_reset(uint8_t id)
{
  uint8_t str[20]={0};
  uint8_t str_size;
  uint8_t str_pos=0;
  uint16_t crc;
  str[str_pos++] = 0xFF;      //HEAD1
  str[str_pos++] = 0xFF;      //HEAD2
  str[str_pos++] = 0xFD;      //HEAD3
  str[str_pos++] = 0x00;      //RESERVED
  str[str_pos++] = id;        //PACKET_ID
  str[str_pos++] = 4;         //LENGTH_LOW
  str[str_pos++] = 0;         //LENGTH_HIGH
  str[str_pos++] = FACTORY_RESET;   //COMMAND
  str[str_pos++] = 1;               //PARAMETER1
  str_size = str_pos;
  crc = update_crc(0, str, str_size);
  str[str_pos++] = (uint8_t)(crc & 0x00FF);
  str[str_pos++] = (uint8_t)((crc>>8) & 0x00FF);
  str_size = str_pos;
  HAL_UART_Transmit(&huart2, str, str_size,10);
}
void arm_reboot(uint8_t id)
{
  uint8_t str[20]={0};
  uint8_t str_size;
  uint8_t str_pos=0;
  uint16_t crc;
  str[str_pos++] = 0xFF;      //HEAD1
  str[str_pos++] = 0xFF;      //HEAD2
  str[str_pos++] = 0xFD;      //HEAD3
  str[str_pos++] = 0x00;      //RESERVED
  str[str_pos++] = id;        //PACKET_ID
  str[str_pos++] = 3;         //LENGTH_LOW
  str[str_pos++] = 0;         //LENGTH_HIGH
  str[str_pos++] = REBOOT;    //COMMAND
  str_size = str_pos;
  crc = update_crc(0, str, str_size);
  str[str_pos++] = (uint8_t)(crc & 0x00FF);
  str[str_pos++] = (uint8_t)((crc>>8) & 0x00FF);
  str_size = str_pos;
  HAL_UART_Transmit(&huart2, str, str_size,10);
}
uint16_t update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    static const unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
























