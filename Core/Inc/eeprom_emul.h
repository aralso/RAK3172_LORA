/*
 * eeprom_emul.h
 *
 *  Created on: Dec 2024
 *      Author: Tocqueville
 */

#ifndef INC_EEPROM_EMUL_H_
#define INC_EEPROM_EMUL_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Configuration de l'émulation EEPROM
#define EEPROM_PAGE_SIZE          2048        // Taille d'une page flash (STM32WLE5)
#define EEPROM_PAGE_COUNT         4           // Nombre de pages utilisées
#define EEPROM_DATA_SIZE          1016        // Taille des données utilisateur (1016 variables de 16 bits ou la moitie en 32 bits)
#define EEPROM_HEADER_SIZE        8          // Taille de l'en-tête
#define EEPROM_RECORD_SIZE        8           // Taille d'un enregistrement 8/16 bits
#define EEPROM_RECORD_SIZE_32     8           // Taille d'un enregistrement 32 bits
#define EEPROM_MAX_RECORDS        ((EEPROM_PAGE_SIZE - EEPROM_HEADER_SIZE) / EEPROM_RECORD_SIZE)
#define EEPROM_MAX_RECORDS_32     ((EEPROM_PAGE_SIZE - EEPROM_HEADER_SIZE) / EEPROM_RECORD_SIZE_32)

// Adresses des pages flash
#define EEPROM_PAGE_0_ADDR        0x08038000  // Page  (début)   Max: 803FFFF
//#define EEPROM_PAGE_1_ADDR        0x08038800  // Page
//#define EEPROM_PAGE_2_ADDR        0x08039000  // Page
//#define EEPROM_PAGE_3_ADDR        0x08039800  // Page

// États des pages
typedef enum {
    EEPROM_PAGE_ERASED = 0xFFFF,
    EEPROM_PAGE_VALID_PAGE = 0xEEEE,
    EEPROM_PAGE_RECEIVE_DATA = 0x0088,
    EEPROM_PAGE_INVALID = 0x0000
} eeprom_page_status_t;

// Structure d'en-tête de page
typedef struct {
    uint16_t page_status;         // Statut de la page
    uint16_t page_number;         // Numéro de page
    uint16_t write_counter;       // Compteur d'écritures
    uint16_t reserved;           // Réservé
} eeprom_page_header_t;

// Structure d'enregistrement pour 8/16 bits
/*typedef struct {
    uint16_t address;            // Adresse des données
    uint16_t data;               // Données (8 ou 16 bits)
    uint32_t crc;                // CRC de l'enregistrement
} eeprom_record_t;*/

// Structure d'enregistrement pour 32 bits
typedef struct {
    uint32_t data;               // Données (32 bits)
    uint16_t address;            // Adresse des données
    uint16_t crc;                // CRC réduit
} eeprom_record_32_t;

// Fonctions publiques
HAL_StatusTypeDef EEPROM_Init(void);
HAL_StatusTypeDef EEPROM_Read8(uint16_t address, uint8_t *data);
HAL_StatusTypeDef EEPROM_Read16(uint16_t address, uint16_t *data);
HAL_StatusTypeDef EEPROM_Read32(uint16_t address, uint32_t *data);
HAL_StatusTypeDef EEPROM_Write8(uint16_t address, uint8_t data);
HAL_StatusTypeDef EEPROM_Write16(uint16_t address, uint16_t data);
HAL_StatusTypeDef EEPROM_Write32(uint16_t address, uint32_t data);
HAL_StatusTypeDef EEPROM_Format(void);
HAL_StatusTypeDef EEPROM_GetStats(uint32_t *write_count, uint32_t *free_space);

void test_eeprom_simple(void);

#endif /* INC_EEPROM_EMUL_H_ */
