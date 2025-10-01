/*
 * functions.h
 *
 *  Created on: Sep 26, 2025
 *      Author: Tocqueville
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_


#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include "cmsis_os.h"
#include "timers.h"



#define MESS_BUFFER_SIZE 500
#define MESS_LG_MAX	100

// Niveaux de verbosité
#define LOG_LEVEL_ERROR    1
#define LOG_LEVEL_WARNING  2
#define LOG_LEVEL_INFO     3
#define LOG_LEVEL_DEBUG    4
#define LOG_LEVEL_VERBOSE  5

#define TIMER_PERIOD_MS  7000   // 50s

// Niveau de verbosité global (modifiable)
#define CURRENT_LOG_LEVEL  LOG_LEVEL_DEBUG

extern uint8_t code_erreur, comptage_erreur;
extern uint8_t err_donnee1, err_donnee2;


// Code erreur

     // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
     // Erreurs 20 a 7F : 4 fois        (Appli >0x70)
     // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

#define code_erreur_envoi 0x50
#define code_erreur_dequeue 0x51

#define erreur_RX_full         0x20
#define erreur_RX_buff_vide    0x21
#define erreur_RX_pas_fin      0x22
#define erreur_RX_framing      0x23
#define timeout_RX             0x24
#define erreur_TX_empty        0x25
#define erreur_TX_full         0x26
#define erreur_TX_pas_fin      0x27
#define erreur_TX_DMA          0x28
#define erreur_mess_tab        0x29 // +2
#define erreur_mess_decod_hexa 0x2C
#define timeout_TX             0x2D
#define erreur_rtos            0x2E
#define erreur_mess            0x2F
#define erreur_uart            0x30
#define erreur_messageIn       0x31

#define erreur_analog               0x42  // 40 et 41:periph
#define erreur_demar                0x43
#define erreur_util_rtos            0x43
#define erreur_tab                  0x44



#define dest_erreur	'1'
#define dest_log '1'

#define NB_UART 1
#define car_fin_trame 13

typedef struct
{
  uint8_t           num_Uart;
  TimerHandle_t     h_timeout_RX;
  TimerHandle_t     h_timeout_TX;
} UartStruct;

extern UartStruct UartSt[NB_UART];
void reception_message_Uart2(void);
uint8_t envoie_routage(const uint8_t *mess, uint8_t len);


// Fonction principale de logging
void print_log(uint8_t level, const char* format, ...);
void init_communication(void);
void raz_Uart(uint8_t num_uart);  // raz car en reception (suite timeout)

// Macros pour faciliter l'utilisation
#define LOG_ERROR(...)   print_log(LOG_LEVEL_ERROR,   __VA_ARGS__)
#define LOG_WARNING(...) print_log(LOG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_INFO(...)    print_log(LOG_LEVEL_INFO,    __VA_ARGS__)
#define LOG_DEBUG(...)   print_log(LOG_LEVEL_DEBUG,   __VA_ARGS__)
#define LOG_VERBOSE(...) print_log(LOG_LEVEL_VERBOSE, __VA_ARGS__)

extern osThreadId_t Uart_TX_TaskHandle;
extern UART_HandleTypeDef huart2;
extern SUBGHZ_HandleTypeDef hsubghz;
extern HAL_StatusTypeDef send_lora_message(const char* message, uint8_t message_length, uint8_t dest);

// Fonction pour changer le niveau de verbosité à l'exécution
void set_log_level(uint8_t level);

// Fonction pour obtenir le niveau actuel
uint8_t get_log_level(void);
void init_functions(void);
uint8_t envoie_mess_ASC(const char* format, ...);
uint8_t envoie_mess_bin(const uint8_t *buf);
uint8_t deci (uint8_t val);
void envoi_code_erreur(void);


#endif /* INC_COMMUNICATION_H_ */
