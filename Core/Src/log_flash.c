#include "log_flash.h"
#include "stm32wlxx_hal_flash.h"
#include <string.h>
#include <stdbool.h>
#include <communication.h>

// Variables globales
static uint32_t current_log_page = 0;
static uint32_t current_write_index = 0;
static log_page_header_t log_page_headers[LOG_PAGE_COUNT];
static const uint32_t LOG_MAGIC_NUMBER = 0x12345678;

// Fonctions internes
static HAL_StatusTypeDef log_read_page_header(uint8_t i);
static HAL_StatusTypeDef log_write_page_header(uint8_t page_num);
static HAL_StatusTypeDef log_find_active_page(void);
static HAL_StatusTypeDef log_read_entry(uint32_t page_addr, uint16_t entry_index, LogEntry *entry);
static HAL_StatusTypeDef log_write_entry_to_page(uint32_t page_addr, uint16_t entry_index, const LogEntry *entry);
static HAL_StatusTypeDef log_erase_page(uint32_t page_addr);
static HAL_StatusTypeDef log_switch_to_next_page(void);
static uint16_t find_next_write_index(uint8_t page_num);
static uint8_t log_write_final(const LogEntry* entry);

//static uint32_t log_calculate_crc(const uint8_t *data, uint32_t length);

static log_header_flash_t header_f;

/**
 * @brief Initialise le système de logs flash
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef log_init(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    

    // Lire les en-têtes des pages de logs
    for (int i = 0; i < LOG_PAGE_COUNT; i++) {
        status = log_read_page_header(i);
        if (status != HAL_OK) {
        	//LOG_INFO("err:%i", i);
            return HAL_ERROR;
        }
    }
    

    // Trouver la page active
    if (status == HAL_OK)
    {
    	status = log_find_active_page();
    }

    if (status != HAL_OK) {
        // Aucune page active, initialiser la première page
        //LOG_VERBOSE("No active log page found, initializing...");
        status = LOG_Format();
    }
    
    return status;
}

/**
 * @brief Écrit un log avec paramètres individuels
 * @param code Code du log
 * @param c1 Paramètre 1
 * @param c2 Paramètre 2
 * @param c3 Paramètre 3
 * @param message Message du log
 * @retval 0 si succès, 1 si erreur
 */
uint8_t log_write(uint8_t code, uint8_t c1, uint8_t c2, uint8_t c3, const char* message)
{
    LogEntry entry;
    
    // Remplir la structure
    entry.timestamp = HAL_GetTick()/1000;  // en secondes
    entry.code = code;
    entry.c1 = c1;
    entry.c2 = c2;
    entry.c3 = c3;
    
    // Copier le message avec gestion intelligente de la terminaison
    uint8_t message_size = LOG_ENTRY_SIZE - 8;  // 8 octets pour le message
    uint8_t message_len = strlen(message);
    
    if (message_len >= message_size) {
        // Message de 8 caractères ou plus : copier les 8 premiers sans terminaison
        memcpy(entry.message, message, message_size);
    } else {
        // Message de moins de 8 caractères : copier avec terminaison null
        strcpy(entry.message, message);
    }
    
    return log_write_final(&entry);
}

/**
 * @brief Écrit une entrée de log complète
 * @param entry Pointeur vers l'entrée de log
 * @retval 0 si succès, 1 si erreur
 */
uint8_t log_write_entry ( LogEntry* entry)
{

    entry->timestamp = HAL_GetTick()/1000;  // en secondes

    return log_write_final(entry);
}

uint8_t log_write_final(const LogEntry* entry)
{
	if (entry == NULL) {
        return 1;
    }
    
    // Vérifier si la page actuelle est pleine
    if (current_write_index >= LOG_MAX_ENTRIES_PER_PAGE) {
        // Basculer vers la page suivante
        if (log_switch_to_next_page() != HAL_OK) {
            return 1;
        }
    }
    
    // Écrire l'entrée dans la page actuelle
    uint32_t page_addr = LOG_PAGE_ADDR(current_log_page);
    if (log_write_entry_to_page(page_addr, current_write_index, entry) != HAL_OK) {
        return 1;
    }
    
    // Mettre à jour les compteurs
    current_write_index++;
    log_page_headers[current_log_page].write_index = current_write_index;
    log_page_headers[current_log_page].entry_count++;
    
    
    return 0;
}

/**
 * @brief Lit les logs depuis la flash et les envoie
 * @param debut_entry Premier log à lire (depuis le plus récent)
 * @param max_entries Nombre maximum d'entrées à lire
 * @param dest Destinataire pour l'envoi des logs
 * @retval Nombre d'entrées lues
 */
uint16_t log_read(uint16_t debut_entry, uint16_t max_entries, uint8_t dest)
{
    if (max_entries == 0) {
        return 0;
    }
    
    uint16_t entries_read = 0;
    uint16_t current_entry = 0;
    
    uint16_t total_entries = 0;
    // Compter le nombre total d'entrées dans toutes les pages
    for (int i = 0; i < LOG_PAGE_COUNT; i++) {
        if (log_page_headers[i].page_status == LOG_PAGE_ACTIVE || 
            log_page_headers[i].page_status == LOG_PAGE_FULL) {
            total_entries += log_page_headers[i].entry_count;
        }
    }
    
    // Si pas d'entrées ou debut_entry trop grand, retourner 0
    if (total_entries == 0 || debut_entry >= total_entries) {
        return 0;
    }
    
    // Lire depuis la page la plus récente vers la plus ancienne
    for (int i = 0; i < LOG_PAGE_COUNT && entries_read < max_entries; i++)
    {
        int page = (current_log_page - i + LOG_PAGE_COUNT) % LOG_PAGE_COUNT;

        if (log_page_headers[page].page_status == LOG_PAGE_ACTIVE || 
            log_page_headers[page].page_status == LOG_PAGE_FULL)
        {
            
            uint32_t page_addr = LOG_PAGE_ADDR(page);
            uint32_t entries_in_page = log_page_headers[page].entry_count;
            
            // Lire les entrées de cette page (du plus récent vers le plus ancien)
            for (int i = entries_in_page; i >= 1 && entries_read < max_entries; i--) {
                // Vérifier si on doit lire cette entrée
                if (current_entry >= debut_entry)
                {
                    LogEntry entry;
                    if (log_read_entry(page_addr, i, &entry) == HAL_OK) {
                        // Formater le message pour l'envoi
                        char formatted_message[LOG_ENTRY_SIZE-7];  // 8 caractères + null terminator
                        uint8_t message_size = LOG_ENTRY_SIZE - 8;  // 8 octets
                        
                        // Vérifier si le message a une terminaison null
                        bool has_null_term = false;
                        for (int j = 0; j < message_size; j++) {
                            if (entry.message[j] == '\0') {
                                has_null_term = true;
                                break;
                            }
                        }
                        
                        if (has_null_term) {
                            // Message avec terminaison null : copier tel quel
                            strcpy(formatted_message, entry.message);
                        } else {
                            // Message de 8 caractères sans terminaison : ajouter null
                            memcpy(formatted_message, entry.message, message_size);
                            formatted_message[message_size] = '\0';
                        }
                        
                        // Envoyer le log via envoie_mess_ASC
                        envoie_mess_ASC("%cLO%i%c%c%c%s", dest, entry.code, entry.c1, entry.c2, entry.c3, formatted_message);
                        entries_read++;
                    }
                }
                current_entry++;
            }
        }
        else
        	break;
    }
    
    return entries_read;
}

/**
 * @brief Efface tous les logs
 * @retval HAL_StatusTypeDef
 */
/*HAL_StatusTypeDef log_clear(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    // Effacer toutes les pages de logs
    for (int i = 0; i < LOG_PAGE_COUNT; i++) {
        status = log_erase_page(LOG_PAGE_ADDR(i));
        if (status != HAL_OK) {
            return status;
        }
    }
    
    // Réinitialiser les variables
    current_log_page = 0;
    current_write_index = 0;
    
    // Initialiser la page 0
    log_page_headers[0].page_status = LOG_PAGE_ACTIVE;
    log_page_headers[0].entry_count = 0;
    log_page_headers[0].write_index = 0;
    log_page_headers[0].magic_number = LOG_MAGIC_NUMBER;
    
    status = log_write_page_header(LOG_PAGE_ADDR(0), &log_page_headers[0]);
    
    return status;
}*/

/**
 * @brief Obtient les statistiques des logs
 * @param total_entries Nombre total d'entrées
 * @param free_space Espace libre
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef log_get_stats(uint32_t* total_entries, uint32_t* free_space)
{
    if (total_entries == NULL || free_space == NULL) {
        return HAL_ERROR;
    }
    
    *total_entries = 0;
    *free_space = 0;
    
    // Compter les entrées dans toutes les pages
    for (int i = 0; i < LOG_PAGE_COUNT; i++) {
        if (log_page_headers[i].page_status == LOG_PAGE_ACTIVE || 
            log_page_headers[i].page_status == LOG_PAGE_FULL) {
            *total_entries += log_page_headers[i].entry_count;
        }
    }
    
    // Calculer l'espace libre
    if (log_page_headers[current_log_page].page_status == LOG_PAGE_ACTIVE) {
        *free_space = LOG_MAX_ENTRIES_PER_PAGE - current_write_index;
    }
    
    LOG_INFO("log:total:%i page:%i index:%i free sur page:%i", *total_entries, current_log_page, current_write_index, *free_space);

    return HAL_OK;
}

// === FONCTIONS INTERNES ===

/**
 * @brief Lit l'en-tête d'une page de logs
 */
static HAL_StatusTypeDef log_read_page_header(uint8_t i)
{
    //LOG_WARNING("LOG4");
    //osDelay(200);

	uint32_t page_addr = LOG_PAGE_ADDR(i);

    /*if (header_f == NULL) {
        return HAL_ERROR;
    }*/
    
    // Vérifier que l'adresse est alignée sur 4 octets
    if (page_addr % 4 != 0) {
        LOG_ERROR("Adresse non alignee: 0x%08lX", page_addr);
        return HAL_ERROR;
    }
    
    // Vérifier que l'adresse est dans la plage flash valide
    if (page_addr < FLASH_BASE || page_addr >= (FLASH_BASE + FLASH_SIZE)) {
        LOG_ERROR("Adresse flash invalide: 0x%08lX", page_addr);
        return HAL_ERROR;
    }

    
    // Désactiver les interruptions pendant la lecture
    __disable_irq();
    
    // Lire l'en-tête depuis la flash par mots de 32 bits
    uint32_t* header_ptr = (uint32_t*)&header_f;
    uint32_t* flash_ptr = (uint32_t*)page_addr;

    header_ptr[0] = flash_ptr[0];
    header_ptr[1] = flash_ptr[1];  //magic_number

    // Réactiver les interruptions
    __enable_irq();


    log_page_headers[i].page_status = header_ptr[0] & 0xFF;
    log_page_headers[i].entry_count=0;
    if (log_page_headers[i].page_status == LOG_PAGE_FULL)
    		log_page_headers[i].entry_count = LOG_MAX_ENTRIES_PER_PAGE -1;
    log_page_headers[i].write_index=0;

    if  ((header_ptr[1] != LOG_MAGIC_NUMBER) || ((header_ptr[0] & 0xFF00) != (i<<8)))
    {
    	//LOG_INFO( "header:%08X %08X ",header_ptr[0], header_ptr[1]);
        log_page_headers[i].page_status = LOG_PAGE_INVALID;
        //LOG_WARNING("inv:%i", i);
    }


    return HAL_OK;
}

/**
 * @brief Écrit l'en-tête d'une page de logs
 */
static HAL_StatusTypeDef log_write_page_header(uint8_t page_num)
{
    /*if (header_f == NULL) {
        LOG_DEBUG("err head");
        return HAL_ERROR;
    }*/

    uint32_t page_addr = LOG_PAGE_ADDR(page_num);
    
    header_f.page_status = log_page_headers[page_num].page_status;
    header_f.page_number = page_num;
    header_f.reserved = 0xFFFF;
    header_f.magic_number = LOG_MAGIC_NUMBER;

    
    // Écrire l'en-tête
    uint64_t *header_ptr = (uint64_t*)&header_f;  // Cast direct vers uint64_t*
    HAL_StatusTypeDef status = HAL_OK;

    //LOG_DEBUG("Page addr: 0x%08lX, align: %lu", page_addr, page_addr % 8);
    if (page_addr % 8 != 0) {
        LOG_ERROR("Page addr not aligned on 8 bytes");
        return HAL_ERROR;
    }

    // Log 2: Vérifier la taille de la structure
    //LOG_DEBUG("Header size: %lu bytes", sizeof(log_header_flash_t));

    // Log 3: Vérifier le contenu avant écriture
    //LOG_DEBUG("Header data: 0x%016llX", (unsigned long long)*header_ptr);

    // Log 4: Vérifier l'état de la flash avant écriture
    //uint64_t *flash_ptr = (uint64_t*)page_addr;
    //LOG_DEBUG("Flash before: 0x%016llX", (unsigned long long)*flash_ptr);

    // Désactiver les interruptions
    __disable_irq();

    // Déverrouiller la flash
    HAL_FLASH_Unlock();

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, page_addr, *header_ptr);


    // Verrouiller la flash
    HAL_FLASH_Lock();
    
    // Réactiver les interruptions
    __enable_irq();

    //LOG_DEBUG("Flash after: 0x%016llX", (unsigned long long)*flash_ptr);

    return status;
}

/**
 * @brief Trouve la page active
 */
static HAL_StatusTypeDef log_find_active_page(void)
{
    for (int i = 0; i < LOG_PAGE_COUNT; i++)
    {
        if (log_page_headers[i].page_status == LOG_PAGE_ACTIVE)
        {
            current_log_page = i;
            uint16_t ind = find_next_write_index(i);
            if (!ind)  // index non trouvé : page pleine
            {
            	LOG_ERROR("Page act pleine :%i", i);
                return HAL_ERROR;
            }
            current_write_index = ind;
            log_page_headers[i].write_index = current_write_index;
            log_page_headers[i].entry_count = current_write_index-1;
            LOG_VERBOSE("activ:page:%i, index:%i", current_log_page, current_write_index);
            return HAL_OK;
        }
    }
    
    return HAL_ERROR;
}

/**
 * @brief Lit une entrée de log - rajouter diable_irq
 */
static HAL_StatusTypeDef log_read_entry(uint32_t page_addr, uint16_t entry_index, LogEntry *entry)
{
    if (entry == NULL || entry_index >= LOG_MAX_ENTRIES_PER_PAGE || (!entry_index)) {
        return HAL_ERROR;
    }
    
    uint32_t entry_addr = page_addr +  (entry_index * LOG_ENTRY_SIZE);
    // Désactiver les interruptions pendant la lecture
    __disable_irq();
    memcpy(entry, (void*)entry_addr, sizeof(LogEntry));
    __enable_irq();
    return HAL_OK;
}

/**
 * @brief Écrit une entrée de log dans une page
 */
static HAL_StatusTypeDef log_write_entry_to_page(uint32_t page_addr, uint16_t entry_index, const LogEntry *entry)
{
    if ((entry == NULL) || (entry_index >= LOG_MAX_ENTRIES_PER_PAGE) || (!entry_index)) {
        return HAL_ERROR;
    }
    
    // Désactiver les interruptions
    __disable_irq();
    
    // Déverrouiller la flash
    HAL_FLASH_Unlock();
    
    uint32_t entry_addr = page_addr  + (entry_index * LOG_ENTRY_SIZE);
    
    // Écrire l'entrée
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                                               entry_addr, 
                                               *(uint64_t*)entry);
    
    // Verrouiller la flash
    HAL_FLASH_Lock();
    
    // Réactiver les interruptions
    __enable_irq();
    
    return status;
}

/**
 * @brief Efface une page de logs
 */
static HAL_StatusTypeDef log_erase_page(uint32_t page_addr)
{
    // Désactiver les interruptions
    __disable_irq();
    
    // Déverrouiller la flash
    HAL_FLASH_Unlock();
    
    // Effacer la page
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = (page_addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase_init.NbPages = 1;
    
    uint32_t page_error;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    
    // Verrouiller la flash
    HAL_FLASH_Lock();
    
    // Réactiver les interruptions
    __enable_irq();
    
    return status;
}

/**
 * @brief Bascule vers la page suivante
 */
static HAL_StatusTypeDef log_switch_to_next_page(void)
{
    // Marquer la page actuelle comme pleine
    log_page_headers[current_log_page].page_status = LOG_PAGE_FULL;
    log_write_page_header(current_log_page);
    
    // Passer à la page suivante
    current_log_page = (current_log_page + 1) % LOG_PAGE_COUNT;
    current_write_index = 1;
    
    // Effacer la nouvelle page
    uint32_t new_page_addr = LOG_PAGE_ADDR(current_log_page);
    if (log_erase_page(new_page_addr) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Initialiser l'en-tête de la nouvelle page
    log_page_headers[current_log_page].page_status = LOG_PAGE_ACTIVE;
    log_page_headers[current_log_page].entry_count = 0;
    log_page_headers[current_log_page].write_index = 1;
    
    return log_write_page_header(current_log_page);
}

/**
 * @brief Calcule le CRC (optionnel pour validation)
 */
/*static uint32_t log_calculate_crc(const uint8_t *data, uint32_t length)
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
}*/


HAL_StatusTypeDef LOG_Format(void)
{
    LOG_DEBUG("Force erase log pages...");

    HAL_StatusTypeDef status = HAL_OK;

	// Effacer toutes les pages
    for (int i = 0; i < LOG_PAGE_COUNT; i++)
    {
        uint32_t page_addr = LOG_PAGE_ADDR(i);
        //LOG_INFO("Er pg %d at 0x%08lX", i, page_addr);
        HAL_StatusTypeDef status = log_erase_page(page_addr);
        if (status == HAL_OK) {
            //LOG_INFO("Page %d erased successfully", i);
        } else {
            LOG_ERROR("Failed to erase page %d", i);
        }
    }

    current_log_page = 0;
    current_write_index = 1;

    // Initialiser l'en-tête de la page 0
    log_page_headers[0].page_status = LOG_PAGE_ACTIVE;
    log_page_headers[0].entry_count = 0;
    log_page_headers[0].write_index = 1;

    osDelay(1000);
    // Ecrire l'en-tête
    status = log_write_page_header(0);

    return status;
}


static uint16_t find_next_write_index(uint8_t page_num)
{
    uint16_t index = 0;

    uint32_t page_addr = LOG_PAGE_ADDR(page_num);

    // Parcourir toutes les entrées pour trouver la première vide
    for (uint16_t i = 1; i < LOG_MAX_ENTRIES_PER_PAGE; i++) {
        LogEntry entry;
        if (log_read_entry(page_addr, i, &entry) == HAL_OK) {
            // Vérifier si l'entrée est vide (timestamp = 0xFFFFFFFF)
        	//LOG_INFO("ind:%i, time:04X", i, entry.timestamp);
            if (entry.timestamp == 0xFFFFFFFF) {
                index = i;
                break;
            }
        }
    }

    return index;
}
