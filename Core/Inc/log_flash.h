#ifndef INC_LOG_FLASH_H_
#define INC_LOG_FLASH_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// avec 8 pages de LOG_SIZE=16 => 1024 log possibles

// Configuration des logs flash
#define LOG_ENTRY_SIZE          16          // Taille d'une entrée de log (16, 32 ou 64)
#define LOG_PAGE_SIZE           2048        // Taille d'une page flash (STM32WLE5)
#define LOG_PAGE_COUNT          8           // Nombre de pages utilisées pour les logs
#define LOG_MAX_ENTRIES_PER_PAGE (LOG_PAGE_SIZE / LOG_ENTRY_SIZE)

// Adresse de base des pages flash pour les logs
#define LOG_PAGE_BASE_ADDR      0x0803A000  // Adresse de la première page (Page 126)

// Macro pour calculer l'adresse d'une page
#define LOG_PAGE_ADDR(page_num) (LOG_PAGE_BASE_ADDR + ((page_num) * LOG_PAGE_SIZE))

// États des pages de logs
typedef enum {
    LOG_PAGE_ERASED = 0xFF,
    LOG_PAGE_ACTIVE = 0xF0,
    LOG_PAGE_FULL   = 0x80,
    LOG_PAGE_INVALID= 0x00
} log_page_status_t;

// Structure d'en-tête de page de logs

typedef struct __attribute__((packed, aligned(8))) {
    uint8_t page_status;
    uint8_t page_number;
    uint16_t reserved;
    uint32_t magic_number;
} log_header_flash_t;

typedef struct  {
    uint8_t page_status;
    uint8_t entry_count;
    uint8_t write_index;
} log_page_header_t;

// Structure d'une entrée de log
typedef struct __attribute__((packed, aligned(8))) {
    uint32_t timestamp;          // Timestamp (4 octets)
    uint8_t code;               // Code de log (1 octet)
    uint8_t c1;                 // Paramètre 1 (1 octet)
    uint8_t c2;                 // Paramètre 2 (1 octet)
    uint8_t c3;                 // Paramètre 3 (1 octet)
    char message[LOG_ENTRY_SIZE-8];  // Message (8 octets)
} LogEntry;

// Fonctions publiques
HAL_StatusTypeDef log_init(void);
uint8_t log_write(uint8_t code, uint8_t c1, uint8_t c2, uint8_t c3, const char* message);
uint8_t log_write_entry(LogEntry* entry);
uint16_t log_read(uint16_t debut_entry, uint16_t max_entries, uint8_t dest, uint8_t type);
//HAL_StatusTypeDef log_clear(void);
HAL_StatusTypeDef log_get_stats(uint32_t* total_entries, uint32_t* free_space);
HAL_StatusTypeDef LOG_Format(void);


#endif /* INC_LOG_FLASH_H_ */
