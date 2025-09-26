/*
 * functions.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Tocqueville
 */

#include "functions.h"
#include <string.h>

// Variable globale pour le niveau de verbosité
static uint8_t current_log_level = CURRENT_LOG_LEVEL;

// Buffer pour le formatage
static char log_buffer[256];

extern UART_HandleTypeDef hlpuart1;

/**
 * @brief Fonction principale de logging avec niveau de verbosité
 * @param level: Niveau de verbosité (1-10)
 * @param format: Format string (comme printf)
 * @param ...: Arguments variables
 */
void print_log(uint8_t level, const char* format, ...)
{
    // Vérifier si le niveau est suffisant pour afficher
    if (level > current_log_level) {
        return;
    }

    // Préfixe selon le niveau
    const char* prefix;
    switch (level) {
        case LOG_LEVEL_ERROR:
            prefix = "[ERROR] ";
            break;
        case LOG_LEVEL_WARNING:
            prefix = "[WARN]  ";
            break;
        case LOG_LEVEL_INFO:
            prefix = "[INFO]  ";
            break;
        case LOG_LEVEL_DEBUG:
            prefix = "[DEBUG] ";
            break;
        case LOG_LEVEL_VERBOSE:
            prefix = "[VERB]  ";
            break;
        default:
            prefix = "[LOG]   ";
            break;
    }

    // Ajouter le préfixe
    strcpy(log_buffer, prefix);

    // Formatage des arguments
    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer + strlen(prefix), sizeof(log_buffer) - strlen(prefix) - 1, format, args);
    va_end(args);

    // Ajouter un retour à la ligne
    strcat(log_buffer, "\r\n");

    // Envoyer via UART
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)log_buffer, strlen(log_buffer), HAL_MAX_DELAY);
}

/**
 * @brief Changer le niveau de verbosité à l'exécution
 * @param level: Nouveau niveau (1-10)
 */
void set_log_level(uint8_t level)
{
    if (level >= 1 && level <= 10) {
        current_log_level = level;
        LOG_INFO("Log level changed to %d", level);
    }
}

/**
 * @brief Obtenir le niveau de verbosité actuel
 * @retval Niveau actuel (1-10)
 */
uint8_t get_log_level(void)
{
    return current_log_level;
}
