/*
 * eeprom_emul.c
 *
 *  Created on: Dec 2024
 *      Author: Tocqueville
 */

#include "eeprom_emul.h"
#include "stm32wlxx_hal_flash.h"
#include <string.h>
#include <communication.h>

// Variables globales
static uint8_t current_page = 0;
static uint16_t current_write_address = 0;
static eeprom_page_header_t page_headers[EEPROM_PAGE_COUNT];

// Fonctions internes
static HAL_StatusTypeDef EEPROM_ReadPageHeader(uint32_t page_addr, eeprom_page_header_t *header);
static HAL_StatusTypeDef EEPROM_WritePageHeader(uint32_t page_addr, eeprom_page_header_t *header);
static HAL_StatusTypeDef EEPROM_FindValidPage(void);
//static HAL_StatusTypeDef EEPROM_ReadRecord(uint32_t page_addr, uint16_t record_index, eeprom_record_t *record);
static HAL_StatusTypeDef EEPROM_WriteRecord(uint32_t page_addr, uint16_t record_index, eeprom_record_32_t *record);
static HAL_StatusTypeDef EEPROM_ReadRecord32(uint32_t page_addr, uint16_t record_index, eeprom_record_32_t *record);
//static HAL_StatusTypeDef EEPROM_TransferPage(uint32_t from_page, uint32_t to_page);
//static HAL_StatusTypeDef EEPROM_TransferPage32(uint32_t from_page, uint32_t to_page);
static HAL_StatusTypeDef EEPROM_ErasePage(uint32_t page_addr);
static uint32_t EEPROM_CalculateCRC(const uint8_t *data, uint32_t length);
//static HAL_StatusTypeDef EEPROM_VerifyRecord(const eeprom_record_t *record);
static HAL_StatusTypeDef EEPROM_VerifyRecord32(const eeprom_record_32_t *record);
static HAL_StatusTypeDef move_to_next_page(void);
uint16_t find_next_write_index(uint8_t page);
HAL_StatusTypeDef search_in_page(uint8_t page, uint16_t address, uint32_t *data);

/**
 * @brief Initialise l'émulation EEPROM
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_Init(void)
{
    LOG_INFO("=== INITIALISATION EEPROM ===");
    osDelay(200);

    //LOG_INFO("Recherche de la page valide...");
    
    HAL_StatusTypeDef status = HAL_OK;
    
    // Lire les en-têtes des pages
    for (int i = 0; i < EEPROM_PAGE_COUNT; i++) {
        uint32_t page_addr = EEPROM_PAGE_0_ADDR + (i * EEPROM_PAGE_SIZE);
        //LOG_INFO("Lecture page %d à l'adresse 0x%08lX", i, page_addr);
        
        status = EEPROM_ReadPageHeader(page_addr, &page_headers[i]);
        if (status != HAL_OK) {
            LOG_ERROR("Erreur lecture page %d: %d", i, status);
            return status;
        }
        
        //LOG_INFO("Page %d - Status: 0x%08lX, Write count: %lu",
        //         i, page_headers[i].page_status, page_headers[i].write_counter);
    }
    
    // Trouver la page valide
    //LOG_INFO("Recherche de la page valide...");
    status = EEPROM_FindValidPage();
    if (status != HAL_OK) {
        LOG_WARNING("Aucune page valide trouvee, format...");
        status = EEPROM_Format();
        if (status != HAL_OK) {
            LOG_ERROR("Erreur formatage EEPROM: %d", status);
            return status;
        }
        //LOG_INFO("EEPROM formatee avec succes");
    } else {
        LOG_INFO("Page valide trouvee: %d", current_page);
    }
    
    //current_write_address = find_next_write_index(current_page);

    LOG_INFO("=== EEPROM INITIALISEE ===");
    return status;
}

/**
 * @brief Lit une donnée 8 bits
 * @param address Adresse de la donnée
 * @param data Pointeur vers la donnée lue
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_Read8(uint16_t address, uint8_t *data)
{
    if (data == NULL || address >= EEPROM_DATA_SIZE) {
        return HAL_ERROR;
    }
    
    uint32_t data32;
    HAL_StatusTypeDef status = EEPROM_Read32(address, &data32);
    if (status == HAL_OK) {
        *data = (uint8_t)(data32 & 0xFF);
    }
    
    return status;
}

/**
 * @brief Lit une donnée 16 bits
 * @param address Adresse de la donnée
 * @param data Pointeur vers la donnée lue
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_Read16(uint16_t address, uint16_t *data)
{
    if (data == NULL || address >= EEPROM_DATA_SIZE) {
        return HAL_ERROR;
    }
    
    uint32_t data32;
    HAL_StatusTypeDef status = EEPROM_Read32(address, &data32);
    if (status == HAL_OK) {
        *data = (uint16_t)(data32 & 0xFFFF);
    }
    return status;
}

  /*  uint32_t page_addr = EEPROM_PAGE_0_ADDR + (current_page * EEPROM_PAGE_SIZE);
    LOG_DEBUG("Read16: addr=0x%04X, page=0x%08lX", address, page_addr);
    
    // Chercher l'enregistrement le plus récent pour cette adresse
    uint16_t latest_record = 0xFFFF;
    uint32_t latest_write_counter = 0;
    
    for (uint16_t i = 0; i < EEPROM_MAX_RECORDS; i++) {
        eeprom_record_t record;
        if (EEPROM_ReadRecord(page_addr, i, &record) == HAL_OK) {
            LOG_DEBUG("R[%d]: a=0x%04X, d=0x%04X, c=0x%08lX", 
                     i, record.address, record.data, record.crc);
            if (record.address == address && record.crc != 0xFFFFFFFF) {
                // Vérifier le CRC
                if (EEPROM_VerifyRecord(&record)) {
                    LOG_DEBUG("Valid R[%d] wc=%lu", i, page_headers[current_page].write_counter);
                    if (page_headers[current_page].write_counter > latest_write_counter) {
                        latest_record = i;
                        latest_write_counter = page_headers[current_page].write_counter;
                    }
                } else {
                    LOG_DEBUG("CRC invalide R[%d]", i);
                }
            }
        }
    }
    
    if (latest_record != 0xFFFF) {
        eeprom_record_t record;
        if (EEPROM_ReadRecord(page_addr, latest_record, &record) == HAL_OK) {
            *data = record.data;
            LOG_DEBUG("Data read: 0x%04X", *data);
            return HAL_OK;
        }
    }
    
    // Donnée non trouvée, retourner 0
    LOG_DEBUG("No valid record for addr=0x%04X", address);
    *data = 0;
    return HAL_ERROR;
}*/

/**
 * @brief Écrit une donnée 8 bits
 * @param address Adresse de la donnée
 * @param data Donnée à écrire
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_Write8(uint16_t address, uint8_t data)
{
    if (address >= EEPROM_DATA_SIZE) {
        return HAL_ERROR;
    }
    
    return EEPROM_Write32(address, (uint32_t)data);
}

/**
 * @brief Écrit une donnée 16 bits
 * @param address Adresse de la donnée
 * @param data Donnée à écrire
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_Write16(uint16_t address, uint16_t data)
{
    if (address >= EEPROM_DATA_SIZE) {
        return HAL_ERROR;
    }

    return EEPROM_Write32(address, (uint32_t)data);

}

/*    LOG_DEBUG("Write16: a=0x%04X, d=0x%04X, p=%lu, w=%lu",
              address, data, current_page, current_write_address);
    
    // Vérifier si la page actuelle est pleine
    if (current_write_address >= EEPROM_MAX_RECORDS) {
        // Transférer vers l'autre page
        uint32_t other_page = (current_page + 1) % EEPROM_PAGE_COUNT;
        if (EEPROM_TransferPage(current_page, other_page) != HAL_OK) {
            return HAL_ERROR;
        }
        current_page = other_page;
        current_write_address = 0;
    }
    
    // Créer l'enregistrement
    eeprom_record_t record;
    record.address = address;
    record.data = data;
    record.crc = EEPROM_CalculateCRC((uint8_t*)&record, sizeof(record) - sizeof(record.crc));
    
    LOG_DEBUG("Record: a=0x%04X, d=0x%04X, c=0x%08lX", 
             record.address, record.data, record.crc);
    
    // Écrire l'enregistrement
    HAL_StatusTypeDef status = EEPROM_WriteRecord(
        EEPROM_PAGE_0_ADDR + (current_page * EEPROM_PAGE_SIZE),
        current_write_address,
        &record
    );
    
    if (status == HAL_OK) {
        current_write_address++;
        page_headers[current_page].write_counter++;
        
        LOG_DEBUG("Write OK: w=%lu, wc=%lu", 
                 current_write_address, page_headers[current_page].write_counter);
        
        // Mettre à jour l'en-tête de page
        EEPROM_WritePageHeader(
            EEPROM_PAGE_0_ADDR + (current_page * EEPROM_PAGE_SIZE),
            &page_headers[current_page]
        );
    } else {
        LOG_ERROR("Write error: %d", status);
    }
    
    return status;
}*/

/**
 * @brief Lit une donnée 32 bits (optimisée)
 * @param address Adresse de la donnée
 * @param data Pointeur vers la donnée lue
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_Read32(uint16_t address, uint32_t *data)
{
    if (data == NULL || address >= EEPROM_DATA_SIZE) {
        return HAL_ERROR;
    }
    //LOG_INFO("Page A %i %i", current_page, address);

    // Lire depuis la page la plus récente vers la plus ancienne
    for (int i = 0; i < EEPROM_PAGE_COUNT ; i++)
    {
        // Chercher d'abord dans la page courante (plus probable)
        int page = (current_page - i + EEPROM_PAGE_COUNT) % EEPROM_PAGE_COUNT;

        if (search_in_page(page, address, data) == HAL_OK) {
        	return HAL_OK;
        }
    }

    // Si pas trouvé, chercher dans les autres pages valides
    /*for (int page = 0; page < EEPROM_PAGE_COUNT; page++) {
        LOG_INFO("Page B %i %i", page, address);
        if (page != current_page && page_headers[page].page_status == EEPROM_PAGE_VALID_PAGE) {
            if (search_in_page(page, address, data) == HAL_OK) {
                return HAL_OK;
            }
        }
    }*/
    
    // Donnée non trouvée
    *data = 0;
    return HAL_ERROR;
}

HAL_StatusTypeDef search_in_page(uint8_t page, uint16_t address, uint32_t *data)
{
    uint32_t page_addr = EEPROM_PAGE_0_ADDR + (page * EEPROM_PAGE_SIZE);
    
    for (uint16_t i = 0; i < EEPROM_MAX_RECORDS; i++) {
    	uint16_t j = EEPROM_MAX_RECORDS - i - 1;
        eeprom_record_32_t record;
        if (EEPROM_ReadRecord32(page_addr, j, &record) == HAL_OK) {
            if (record.address == address && record.crc != 0xFFFF) {
                if (EEPROM_VerifyRecord32(&record)) {
                    *data = record.data;
                    //LOG_DEBUG("Found in page %d, index %d", page, j);
                    return HAL_OK;
                }
            }
        }
    }
    
    return HAL_ERROR;
}

/**
 * @brief Écrit une donnée 32 bits (optimisée)
 * @param address Adresse de la donnée
 * @param data Donnée à écrire
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_Write32(uint16_t address, uint32_t data)
{
    if (address >= EEPROM_DATA_SIZE) {
        return HAL_ERROR;
    }
    
    // Vérifier si la page actuelle est pleine
    if (current_write_address >= EEPROM_MAX_RECORDS_32) {
        // Transférer vers la page suivante
        if (move_to_next_page() != HAL_OK) {
            return HAL_ERROR;
        }
    }
    
    LOG_DEBUG("Write32: a=0x%04X, d=0x%08X, p=%lu, w=%lu",
                  address, data, current_page, current_write_address);

    // Créer l'enregistrement
    eeprom_record_32_t record;
    record.address = address;
    record.data = data;
    record.crc = EEPROM_CalculateCRC((uint8_t*)&record, sizeof(record) - sizeof(record.crc));
    
    // Écrire l'enregistrement
    HAL_StatusTypeDef status = EEPROM_WriteRecord(
        EEPROM_PAGE_0_ADDR + (current_page * EEPROM_PAGE_SIZE),
        current_write_address, &record );
    
    if (status == HAL_OK) {
        current_write_address++;
        page_headers[current_page].write_counter++;
        
        // Mettre à jour l'en-tête de page
        /*EEPROM_WritePageHeader(
            EEPROM_PAGE_0_ADDR + (current_page * EEPROM_PAGE_SIZE),
            &page_headers[current_page]
        );*/
    }
    
    return status;
}

/**
 * @brief Formate l'EEPROM
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_Format(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    // Effacer toutes les pages
    for (uint8_t i = 0; i < EEPROM_PAGE_COUNT; i++) {
    	LOG_INFO("eff page:%i", i);
    	osDelay(300);
        status = EEPROM_ErasePage(EEPROM_PAGE_0_ADDR + (i * EEPROM_PAGE_SIZE));
        if (status != HAL_OK) {
            return status;
        }
    }
    
    osDelay(1000);

    // Initialiser la page 0
    current_page = 0;
    current_write_address = 0;
    
    page_headers[0].page_status = EEPROM_PAGE_VALID_PAGE;
    page_headers[0].page_number = 0;
    page_headers[0].write_counter = 0;
    page_headers[0].reserved = 0;
    
    status = EEPROM_WritePageHeader(EEPROM_PAGE_0_ADDR, &page_headers[0]);
    
    return status;
}

/**
 * @brief Obtient les statistiques de l'EEPROM
 * @param write_count Compteur d'écritures
 * @param free_space Espace libre
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EEPROM_GetStats(uint32_t *write_count, uint32_t *free_space)
{
    if (write_count == NULL || free_space == NULL) {
        return HAL_ERROR;
    }
    
    *write_count = page_headers[current_page].write_counter;
    *free_space = EEPROM_MAX_RECORDS - current_write_address;
    
    LOG_INFO("eeprom:total:%i page:%i index:%i free sur page:%i", *write_count, current_page, current_write_address, *free_space);

    return HAL_OK;
}

// === FONCTIONS INTERNES ===

/**
 * @brief Lit l'en-tête d'une page
 */
static HAL_StatusTypeDef EEPROM_ReadPageHeader(uint32_t page_addr, eeprom_page_header_t *header)
{
   // LOG_DEBUG("Lecture en-tete page à 0x%08lX", page_addr);
    
    if (header == NULL) {
        return HAL_ERROR;
    }
    
    // Vérifier que l'adresse est alignée sur 32 bits
    if (page_addr % 4 != 0) {
        LOG_ERROR("Adresse non alignee: 0x%08lX", page_addr);
        return HAL_ERROR;
    }
    
    // Lire l'en-tête
    uint32_t* header_ptr = (uint32_t*)header;
    for (int i = 0; i < sizeof(eeprom_page_header_t) / 4; i++) {
        header_ptr[i] = *((uint32_t*)(page_addr + i * 4));
        //LOG_DEBUG("Header[%d] = 0x%08lX", i, header_ptr[i]);
    }
    
    return HAL_OK;
}

/**
 * @brief Écrit l'en-tête d'une page
 */
static HAL_StatusTypeDef EEPROM_WritePageHeader(uint32_t page_addr, eeprom_page_header_t *header)
{
    if (header == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    LOG_DEBUG("Header size: %lu bytes", sizeof(eeprom_page_header_t));

    // Désactiver les interruptions
    __disable_irq();
    
    // Déverrouiller la flash
    HAL_FLASH_Unlock();
    
    // Effacer la page
    /*FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = (page_addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase_init.NbPages = 1;
    
    uint32_t page_error;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);*/
    

    //if (status == HAL_OK) {
        // Écrire l'en-tête
        uint32_t *header_ptr = (uint32_t*)header;
        //for (int i = 0; i < sizeof(eeprom_page_header_t) / 4; i++) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                                     page_addr,
                                     *(uint64_t*)(header_ptr));
            //if (status != HAL_OK) break;
        //}
    //}
    
    // Verrouiller la flash
    HAL_FLASH_Lock();
    
    // Réactiver les interruptions
    __enable_irq();
    
    return status;
}

/**
 * @brief Trouve la page valide
 */
static HAL_StatusTypeDef EEPROM_FindValidPage(void)
{
    //LOG_INFO("Recherche de la page valide...");
    
    int valid_page = -1;
    
    for (int i = 0; i < EEPROM_PAGE_COUNT; i++) {
        //LOG_INFO("Pag %d - Status: 0x%04X, Write count: %lu",
          //       i, page_headers[i].page_status, page_headers[i].write_counter);
        
        if (page_headers[i].page_status == EEPROM_PAGE_VALID_PAGE)
        {
        	//LOG_INFO("page val:%i write:%i", i, page_headers[i].write_counter);
            //if (page_headers[i].write_counter >= max_write_count) {
             //   max_write_count = page_headers[i].write_counter;
                valid_page = i;
                //LOG_INFO("Nouvelle page valide: %d (write count: %lu)", i, max_write_count);
                break;
            //}
        }
    }
    
    if (valid_page == -1) {
        LOG_WARNING("Aucune page valide trouvee");
        return HAL_ERROR;
    }
    
    current_page = valid_page;
    current_write_address = 0;
    
    // Compter les enregistrements valides
    uint32_t page_addr = EEPROM_PAGE_0_ADDR + (valid_page * EEPROM_PAGE_SIZE);
    for (uint16_t j = 0; j < EEPROM_MAX_RECORDS; j++) {
        eeprom_record_32_t record;
        if (EEPROM_ReadRecord32(page_addr, j, &record) == HAL_OK) {
            if (record.crc == 0xFFFF) {
                current_write_address = j;
                page_headers[current_page].write_counter = current_write_address;
                break;
            }
        }
    }
    
    LOG_INFO("Page valide: %d index:%i", current_page, current_write_address);
    return HAL_OK;
}

/**
 * @brief Lit un enregistrement
 */
/*static HAL_StatusTypeDef EEPROM_ReadRecord(uint32_t page_addr, uint16_t record_index, eeprom_record_t *record)
{
    if (record == NULL || record_index >= EEPROM_MAX_RECORDS) {
        return HAL_ERROR;
    }
    
    uint32_t record_addr = page_addr + EEPROM_HEADER_SIZE + (record_index * EEPROM_RECORD_SIZE);
    memcpy(record, (void*)record_addr, sizeof(eeprom_record_t));
    
    return HAL_OK;
}*/

/**
 * @brief Écrit un enregistrement 32 bits
 */


static HAL_StatusTypeDef EEPROM_WriteRecord(uint32_t page_addr, uint16_t record_index, eeprom_record_32_t *record)
{
    if (record == NULL || record_index >= EEPROM_MAX_RECORDS_32) {
        return HAL_ERROR;
    }
    
    uint32_t record_addr = page_addr + EEPROM_HEADER_SIZE + (record_index * EEPROM_RECORD_SIZE);
    
    LOG_DEBUG("WriteRecord: p=0x%08lX, i=%d", page_addr, record_index);
    
    // Vérifier si l'adresse est alignée sur 8 octets (requis pour DOUBLEWORD)
    if (record_addr % 8 != 0) {
        LOG_ERROR("Addr not aligned: 0x%08lX", record_addr);
        return HAL_ERROR;
    }
    
    // Vérifier l'état de la flash avant écriture
    uint64_t *flash_data = (uint64_t*)record_addr;
    LOG_DEBUG("Flash before: 0x%016llX", *flash_data);
    
    // Si la flash n'est pas effacée (contient des 0), on ne peut pas écrire
    if (*flash_data != 0xFFFFFFFFFFFFFFFFULL) {
        LOG_ERROR("Flash not erased: 0x%016llX", *flash_data);
        return HAL_ERROR;
    }
    
    // Désactiver les interruptions
    __disable_irq();
    
    // Déverrouiller la flash
    HAL_FLASH_Unlock();
    
    // Écrire l'enregistrement par mots de 64 bits
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                                               record_addr, 
                                               *(uint64_t*)record);
    
    if (status != HAL_OK) {
        LOG_ERROR("Flash prog error: %d", status);
    } else {
        LOG_DEBUG("Flash prog OK");
        // Vérifier l'état après écriture
        LOG_DEBUG("Flash after: 0x%016llX", *flash_data);
    }
    
    // Verrouiller la flash
    HAL_FLASH_Lock();
    
    // Réactiver les interruptions
    __enable_irq();
    
    return status;
}

/**
 * @brief Transfère une page vers une autre
 */
/*static HAL_StatusTypeDef EEPROM_TransferPage(uint32_t from_page, uint32_t to_page)
{
    // Effacer la page de destination
    if (EEPROM_ErasePage(EEPROM_PAGE_0_ADDR + (to_page * EEPROM_PAGE_SIZE)) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Initialiser l'en-tête de la page de destination
    page_headers[to_page].page_status = EEPROM_PAGE_VALID_PAGE;
    page_headers[to_page].page_number = to_page;
    page_headers[to_page].write_counter = page_headers[from_page].write_counter;
    page_headers[to_page].reserved = 0;
    
    // Écrire l'en-tête
    if (EEPROM_WritePageHeader(EEPROM_PAGE_0_ADDR + (to_page * EEPROM_PAGE_SIZE), &page_headers[to_page]) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Transférer les enregistrements valides
    uint32_t from_addr = EEPROM_PAGE_0_ADDR + (from_page * EEPROM_PAGE_SIZE);
    uint32_t to_addr = EEPROM_PAGE_0_ADDR + (to_page * EEPROM_PAGE_SIZE);
    
    uint16_t to_record_index = 0;
    
    for (uint16_t i = 0; i < EEPROM_MAX_RECORDS; i++) {
        eeprom_record_t record;
        if (EEPROM_ReadRecord(from_addr, i, &record) == HAL_OK) {
            if (record.crc != 0xFFFFFFFF && EEPROM_VerifyRecord(&record)) {
                // Transférer l'enregistrement
                if (EEPROM_WriteRecord(to_addr, to_record_index, &record) == HAL_OK) {
                    to_record_index++;
                }
            }
        }
    }
    
    // Marquer l'ancienne page comme invalide
    page_headers[from_page].page_status = EEPROM_PAGE_INVALID;
    EEPROM_WritePageHeader(from_addr, &page_headers[from_page]);
    
    return HAL_OK;
}*/

/**
 * @brief Efface une page
 */
static HAL_StatusTypeDef EEPROM_ErasePage(uint32_t page_addr)
{
    LOG_DEBUG("Erase page: 0x%08lX", page_addr);
    osDelay(200);
    
    // Calculer le numéro de page
    uint32_t page_number = (page_addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    LOG_DEBUG("Page calc: %lu", page_number);
    LOG_INFO("Page: %lu", page_number);
    
    // Vérifier que la page est dans la plage valide
    if (page_number >= 128) {  // 256 KB / 2 KB = 128 pages
        LOG_ERROR("Invalid page: %lu (max: 127)", page_number);
        return HAL_ERROR;
    }
    
    // Vérifier l'état de la flash avant effacement
    uint32_t *flash_ptr = (uint32_t*)page_addr;
    LOG_INFO("Flash before: 0x%08lX 0x%08lX", flash_ptr[0], flash_ptr[1]);
    
    // Désactiver les interruptions
    __disable_irq();
    
    // Déverrouiller la flash
    HAL_StatusTypeDef unlock_status = HAL_FLASH_Unlock();
    if (unlock_status != HAL_OK) {
        LOG_ERROR("Flash unlock error: %d", unlock_status);
        __enable_irq();
        return unlock_status;
    }
    
    // Effacer la page
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = page_number;
    erase_init.NbPages = 1;
    
    LOG_DEBUG("Erase page: %lu", erase_init.Page);
    
    uint32_t page_error;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    
    if (status != HAL_OK) {
        LOG_ERROR("Erase error p%lu: %d, err: 0x%08lX", 
                 erase_init.Page, status, page_error);
    } else {
        LOG_DEBUG("Erase OK");
        // Vérifier l'état après effacement
        LOG_INFO("Flash after: 0x%08lX 0x%08lX", flash_ptr[0], flash_ptr[1]);
    }
    
    // Verrouiller la flash
    HAL_FLASH_Lock();
    
    // Réactiver les interruptions
    __enable_irq();
    
    return status;
}

/**
 * @brief Calcule le CRC
 */
static uint32_t EEPROM_CalculateCRC(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return ~crc;
}

/**
 * @brief Vérifie un enregistrement
 */
/*static HAL_StatusTypeDef EEPROM_VerifyRecord(const eeprom_record_t *record)
{
    if (record == NULL) {
        return HAL_ERROR;
    }
    
    uint32_t calculated_crc = EEPROM_CalculateCRC((uint8_t*)record, sizeof(eeprom_record_t) - sizeof(record->crc));
    return (calculated_crc == record->crc) ? HAL_OK : HAL_ERROR;
}*/

/**
 * @brief Lit un enregistrement 32 bits
 */
static HAL_StatusTypeDef EEPROM_ReadRecord32(uint32_t page_addr, uint16_t record_index, eeprom_record_32_t *record)
{
    if (record == NULL || record_index >= EEPROM_MAX_RECORDS_32) {
        return HAL_ERROR;
    }
    
    uint32_t record_addr = page_addr + EEPROM_HEADER_SIZE + (record_index * EEPROM_RECORD_SIZE_32);
    memcpy(record, (void*)record_addr, sizeof(eeprom_record_32_t));
    
    return HAL_OK;
}



/**
 * @brief Transfère une page vers une autre (pour 32 bits)
 */
/*static HAL_StatusTypeDef EEPROM_TransferPage32(uint32_t from_page, uint32_t to_page)
{
    // Effacer la page de destination
    if (EEPROM_ErasePage(EEPROM_PAGE_0_ADDR + (to_page * EEPROM_PAGE_SIZE)) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Initialiser l'en-tête de la page de destination
    page_headers[to_page].page_status = EEPROM_PAGE_VALID_PAGE;
    page_headers[to_page].page_number = to_page;
    page_headers[to_page].write_counter = page_headers[from_page].write_counter;
    page_headers[to_page].reserved = 0;
    
    // Écrire l'en-tête
    if (EEPROM_WritePageHeader(EEPROM_PAGE_0_ADDR + (to_page * EEPROM_PAGE_SIZE), &page_headers[to_page]) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Transférer les enregistrements valides
    uint32_t from_addr = EEPROM_PAGE_0_ADDR + (from_page * EEPROM_PAGE_SIZE);
    uint32_t to_addr = EEPROM_PAGE_0_ADDR + (to_page * EEPROM_PAGE_SIZE);
    
    uint16_t to_record_index = 0;
    
    for (uint16_t i = 0; i < EEPROM_MAX_RECORDS_32; i++) {
        eeprom_record_32_t record;
        if (EEPROM_ReadRecord32(from_addr, i, &record) == HAL_OK) {
            if (record.crc != 0xFFFFFFFF && EEPROM_VerifyRecord32(&record)) {
                // Transférer l'enregistrement
                if (EEPROM_WriteRecord(to_addr, to_record_index, &record) == HAL_OK) {
                    to_record_index++;
                }
            }
        }
    }
    
    // Marquer l'ancienne page comme invalide
    page_headers[from_page].page_status = EEPROM_PAGE_INVALID;
    EEPROM_WritePageHeader(from_addr, &page_headers[from_page]);
    
    return HAL_OK;
}*/

/**
 * @brief Vérifie un enregistrement 32 bits
 */
static HAL_StatusTypeDef EEPROM_VerifyRecord32(const eeprom_record_32_t *record)
{
    if (record == NULL) {
        return HAL_ERROR;
    }
    
    uint32_t calculated_crc = EEPROM_CalculateCRC((uint8_t*)record, sizeof(eeprom_record_32_t) - sizeof(record->crc));
    return (calculated_crc == record->crc) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief Test simple de l'EEPROM
 */
void test_eeprom_simple(void)
{
    LOG_INFO("=== TEST EEPROM SIMPLE ===");
    
    // Forcer le formatage pour utiliser les nouvelles adresses
    LOG_INFO("Formatage EEPROM avec nouvelles adresses...");
    HAL_StatusTypeDef format_status = EEPROM_Format();
    if (format_status != HAL_OK) {
        LOG_ERROR("Erreur formatage: %d", format_status);
        return;
    }
    LOG_INFO("Formatage OK");
    
    // Test d'écriture
    uint16_t test_data = 0x1234;
    HAL_StatusTypeDef status = EEPROM_Write16(0, test_data);
    if (status == HAL_OK) {
        LOG_INFO("Ecriture EEPROM: OK");
    } else {
        LOG_ERROR("Ecriture EEPROM: ERREUR %d", status);
        return;
    }
    
    // Test de lecture
    uint16_t read_data = 0;
    status = EEPROM_Read16(0, &read_data);
    if (status == HAL_OK) {
        LOG_INFO("Lecture EEPROM: 0x%04X", read_data);
        if (read_data == test_data) {
            LOG_INFO("Test EEPROM: SUCCES");
        } else {
            LOG_ERROR("Test EEPROM: ECHEC (attendu: 0x%04X, lu: 0x%04X)", test_data, read_data);
        }
    } else {
        LOG_ERROR("Lecture EEPROM: ERREUR %d", status);
    }
}

/*uint16_t calculate_write_counter(uint32_t page_addr)
{
    uint16_t count = 0;

    // Compter les records valides dans la page
    for (uint16_t i = 0; i < EEPROM_MAX_RECORDS; i++) {
        eeprom_record_32_t record;
        if (EEPROM_ReadRecord32(page_addr, i, &record) == HAL_OK) {
            if (record.crc != 0xFFFFFFFF && EEPROM_VerifyRecord(&record)) {
                count++;
            }
        }
    }

    return count;
}*/



// Fonction pour passer à la page suivante
HAL_StatusTypeDef move_to_next_page(void)
{
    // Marquer la page actuelle comme pleine
    page_headers[current_page].page_status = EEPROM_PAGE_RECEIVE_DATA;
    uint8_t old_page = current_page;


    // Passer à la page suivante
    current_page = (current_page + 1) % EEPROM_PAGE_COUNT;
    current_write_address = 0;

    // Vérifier si la nouvelle page est effacée
    if (page_headers[current_page].page_status != EEPROM_PAGE_ERASED) {
        // Effacer la page si nécessaire
        if (EEPROM_ErasePage(EEPROM_PAGE_0_ADDR + (current_page * EEPROM_PAGE_SIZE)) != HAL_OK) {
            LOG_ERROR("Failed to erase page %d", current_page);
            return HAL_ERROR;
        }
    }

    // Initialiser le header de la nouvelle page
    page_headers[current_page].page_status = EEPROM_PAGE_VALID_PAGE;
    page_headers[current_page].page_number = current_page;
    page_headers[current_page].write_counter = 0;
    page_headers[current_page].reserved = 0;

    // Écrire le new header en flash
    if (EEPROM_WritePageHeader(EEPROM_PAGE_0_ADDR + (current_page * EEPROM_PAGE_SIZE), &page_headers[current_page]) != HAL_OK) {
        LOG_ERROR("Failed to write new page header %d", current_page);
        return HAL_ERROR;
    }

    // Écrire le header en flash de la old page
    if (EEPROM_WritePageHeader(EEPROM_PAGE_0_ADDR + (old_page * EEPROM_PAGE_SIZE), &page_headers[old_page]) != HAL_OK) {
        LOG_ERROR("Failed to write old page header %d", old_page);
        return HAL_ERROR;
    }

    LOG_INFO("Moved to page %d", current_page);
    return HAL_OK;
}


uint16_t find_next_write_index(uint8_t page)
{
    uint32_t page_addr = EEPROM_PAGE_0_ADDR + (page * EEPROM_PAGE_SIZE);
    uint16_t index = 0;

    // Chercher le premier emplacement libre
    for (uint16_t i = 0; i < EEPROM_MAX_RECORDS; i++) {
        eeprom_record_32_t record;
        if (EEPROM_ReadRecord32(page_addr, i, &record) == HAL_OK) {
            if (record.crc == 0xFFFF) {
                // Emplacement libre trouvé
                index = i;
                break;
            }
        }
    }

    return index;
}
