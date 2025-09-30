/*
 * functions.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Tocqueville
 */

#include "cmsis_os.h"
#include "timers.h"
#include "functions.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// Variable globale pour le niveau de verbosité
static uint8_t current_log_level = CURRENT_LOG_LEVEL;

// Buffer pour le formatage
static char log_buffer[256];

extern UART_HandleTypeDef huart2;
extern osMessageQueueId_t Event_QueueHandle;
extern osThreadId_t Appli_TaskHandle;
extern osThreadId_t LORA_TX_TaskHandle;
extern osThreadId_t LORA_RX_TaskHandle;
extern osThreadId_t Uart1_TaskHandle;

// Timers
TimerHandle_t HTimer_24h;
TimerHandle_t HTimer_20min;

/* Definitions for Uart_TX_Task */
osThreadId_t Uart_TX_TaskHandle;
const osThreadAttr_t Uart_TX_Task_attributes = {
  .name = "Uart_TXTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};


static uint8_t mess_buffer[MESS_BUFFER_SIZE];
static uint16_t head = 0;
static uint16_t tail = 0;

static osMutexId_t bufferMutex;

#define nb_erreurs_enregistrees  20
#define nb_erreurs_envoyees      30
#define nb_erreurs_unique        0x20
#define nb_erreurs_4_fois        0x60

uint8_t code_erreur, comptage_erreur;
uint8_t err_donnee1, err_donnee2;
uint8_t enr_erreur[nb_erreurs_enregistrees];          // Enr_erreur enregistre les 30 premiÃ¨res erreurs
uint8_t erreurs_unique[nb_erreurs_unique/8];    // 1:erreur dÃ©ja envoyÃ©e, plus d'envoi
uint8_t erreurs_4_fois[nb_erreurs_4_fois/4];       // Nb d'erreurs dÃ©ja envoyÃ©es, max 4

uint8_t nb_reset=0;

uint8_t mess_enqueue(const uint8_t *data, uint8_t len);
uint8_t mess_dequeue(uint8_t *data, uint8_t *len);

void Uart_TXTsk(void *argument);
static void Timer24hCallback(TimerHandle_t xTimer);
static void Timer20minCallback(TimerHandle_t xTimer);

void init_functions(void)
{
	// Initialisation du mutex
	bufferMutex = osMutexNew(NULL);
	if (bufferMutex == NULL) {
		LOG_ERROR("Failed to create bufferMutex");
		return;
	}
	LOG_INFO("bufferMutex created: %p", bufferMutex);

	/* creation of Uart_TX_Task */

	Uart_TX_TaskHandle = osThreadNew(Uart_TXTsk, NULL, &Uart_TX_Task_attributes);

	// creation timers
	 HTimer_24h = xTimerCreate(
	        "Timer24h",                          // Nom
	        pdMS_TO_TICKS(24*3600*1000),     // Période en ticks
	        pdTRUE,                             // Auto-reload
	        (void*)0,                           // ID optionnel
	        Timer24hCallback                     // Callback
	    );
	    if (HTimer_24h != NULL) xTimerStart(HTimer_24h, 0);

		 HTimer_20min = xTimerCreate(
		        "Timer20min",                          // Nom
		        pdMS_TO_TICKS(TIMER_PERIOD_MS),     // Période en ticks
		        pdTRUE,                             // Auto-reload
		        (void*)0,                           // ID optionnel
		        Timer20minCallback                     // Callback
		    );
		    if (HTimer_20min != NULL) xTimerStart(HTimer_20min, 0);
}




// PRS0 -> PURS0 (lg=5)
uint8_t envoie_mess_ASC(const char* format, ...)
{
	uint16_t len;
	uint8_t mess[MESS_LG_MAX]; // Variable locale pour travailler
	char formatted_buffer[MESS_LG_MAX]; // Buffer pour le formatage

	if (format == NULL) {
		return 1; // Erreur : buffer nul
	}

	// Formatage des arguments variables
	va_list args;
	va_start(args, format);
	len = vsnprintf(formatted_buffer, sizeof(formatted_buffer) - 1, format, args);
	va_end(args);

	// Vérifier si le formatage a réussi
	if (len < 0) {
		return 3; // Erreur : formatage échoué
	}


	// Vérifier si len<3 ou la longueur dépasse la taille maximale
	if ((len < 3) || ( (len+3) >= MESS_LG_MAX)) {
		return 2; // Erreur : dépassement de buffer
	}

	// Copier buf dans mess
	memcpy(mess+1, formatted_buffer, len + 1); // decalage & +1 pour inclure le '\0'
	len += 2;
	mess[0] = formatted_buffer[0];
	mess[1] = EMETTEUR;


	// Envoyer le message
	uint8_t res = mess_enqueue(mess, len);
	//osDelay(100);
	//LOG_INFO("enqueue:%i %s", len, mess);
    //osDelay(100);
	return res;
}

// P2RS => PU2RS (lg=5)
uint8_t envoie_mess_bin(const uint8_t *buf)
{
	uint8_t len;
	uint8_t mess[MESS_LG_MAX]; // Variable locale pour travailler

	if (buf == NULL) {
		return 1; // Erreur : buffer nul
	}
	len = buf[1]+2;

	// Vérifier si len<4 ou la longueur dépasse la taille maximale
	if ((len < 4) || ( (len+2) >= MESS_LG_MAX)) {
		return 2; // Erreur : dépassement de buffer
	}

	// Copier buf dans mess
	memcpy(mess+1, buf, len); // decalage de 2
	len++; // pour emetteur  : 5
	mess[0] = buf[0] & 0x80;
	mess[1] = EMETTEUR;

	// Envoyer le message
	uint8_t res = mess_enqueue(mess, len);
	return res;

}

// Ajout d’un message
uint8_t mess_enqueue(const uint8_t *data, uint8_t len)
{
	osStatus_t status = osMutexAcquire(bufferMutex, 10000);
	if (status != osOK) return 3;

    uint16_t free_space;
    uint16_t head_prov = head;

    if (head >= tail)
        free_space = MESS_BUFFER_SIZE - (head - tail) - 1;
    else
        free_space = (tail - head) - 1;

    if (free_space < (len + 1)) {
        osMutexRelease(bufferMutex);
        return 1; // pas assez de place
    }

    if ((len < 5) || (len > MESS_LG_MAX))
    {
    	return 2;
    }
    if (head >= MESS_BUFFER_SIZE) {
            head = 0; // Reset si corruption
            tail = 0;
    }

    mess_buffer[head_prov] = len;
    head_prov = (head_prov + 1) % MESS_BUFFER_SIZE;

    for (uint8_t i = 0; i < len; i++) {
        mess_buffer[head_prov] = data[i];
        head_prov = (head_prov + 1) % MESS_BUFFER_SIZE;
    }

    head = head_prov;

	//osDelay(100);
	//LOG_INFO("enqueue:head:%d tail:%d", head, tail);
    //osDelay(100);
    osMutexRelease(bufferMutex);

    return 0;
}

// Extraction d’un message
uint8_t mess_dequeue(uint8_t *data, uint8_t *len)
{
	osStatus_t status = osMutexAcquire(bufferMutex, 10000);
	if (status != osOK) return 4;

	uint16_t tail_prov=tail;

    if (head == tail) {
        osMutexRelease(bufferMutex);
        return 1; // FIFO vide
    }

    if (data == NULL || len == NULL)
    {
        osMutexRelease(bufferMutex);
    	return 4;
    }

    *len = mess_buffer[tail];
    tail_prov = (tail_prov + 1) % MESS_BUFFER_SIZE;

	//osDelay(300);
	//LOG_INFO("dequeue1:head:%d tail:%d", head, tail);
    //osDelay(300);

    // Vérif longueur valide
	if ((*len < 5) || (*len > MESS_LG_MAX) || tail_prov >= MESS_BUFFER_SIZE)
	{
		head=0;
		tail=0;
		osMutexRelease(bufferMutex);
		return 2; // corruption détectée
	}

	// Vérif que les données tiennent dans la FIFO actuelle
	uint16_t available = (head >= tail_prov) ?
						 (head - tail_prov) :
						 (MESS_BUFFER_SIZE - (tail_prov - head));

	if (available < *len) {
		head=0;
		tail=0;
		osMutexRelease(bufferMutex);
		return 3; // corruption : message incomplet
	}

    for (uint8_t i = 0; i < *len; i++) {
        data[i] = mess_buffer[tail_prov];
        tail_prov = (tail_prov + 1) % MESS_BUFFER_SIZE;
    }
    tail = tail_prov;
	//osDelay(300);
	//LOG_INFO("dequeue2:head:%d tail:%d lg:%d", head, tail, *len);
    //osDelay(300);

    osMutexRelease(bufferMutex);
    return 0;
}

void Uart_TXTsk(void *argument)
{
    uint8_t msg[MESS_LG_MAX];
    uint8_t len;

    //osDelay(3000);

    for (;;)
    {
        uint8_t stat;
        stat = mess_dequeue(msg, &len);
        if (stat == 0)
        {
        	//LOG_INFO("U");
            //osDelay(300);
            //LOG_INFO("UART TX message dequeue lg:%d st:%s", len, msg);
            //osDelay(300);
            // Envoi bloquant : la tâche attend
            HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, msg, len, 10000);
            if (status) { code_erreur = code_erreur_envoi;err_donnee1=status; err_donnee2=len;}
            //osDelay(300);
            //LOG_INFO("UART TX failed: %d", status);
            //osDelay(300);
        }
        else
        {
        	if (stat!=1)
        	{
        		code_erreur = code_erreur_dequeue;
        		err_donnee1 = stat;
        		err_donnee2 = len;
        	}
        }
        //osDelay(1000); // rien à envoyer → on laisse tourner le CPU
        //LOG_INFO(".");
        osDelay(30); // rien à envoyer → on laisse tourner le CPU
    }
}


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
    log_buffer[0] = dest_log;
    strcpy(log_buffer+1, prefix);

    // Formatage des arguments
    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer + strlen(prefix), sizeof(log_buffer) - strlen(prefix) - 1, format, args);
    va_end(args);

    // Ajouter un retour à la ligne
    strcat(log_buffer, "\r\n");

    // Envoyer via UART
    //HAL_UART_Transmit(&huart2, (uint8_t*)log_buffer, strlen(log_buffer), 3000);
    envoie_mess_ASC("%s", log_buffer);
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

static void Timer24hCallback(TimerHandle_t xTimer)
{
	event_t evt = { EVENT_TIMER_24h, 0, 0 };
	osMessageQueuePut(Event_QueueHandle, &evt, 0, 0);
}

static void Timer20minCallback(TimerHandle_t xTimer)
{
	event_t evt = { EVENT_TIMER_20min, 0, 0 };
	osMessageQueuePut(Event_QueueHandle, &evt, 0, 0);
}

/**
 * @brief Retourne le temps restant avant expiration du timer, en millisecondes.
 * @param t Handle du timer FreeRTOS
 * @return Temps restant en ms (0 si expiré ou inactif)
 */
uint32_t TimerGetRemainingMs(TimerHandle_t t)
{
    if (t == NULL || xTimerIsTimerActive(t) == pdFALSE)
    {
        return 0; // Timer non actif ou invalide
    }

    TickType_t expiry = xTimerGetExpiryTime(t);
    TickType_t now = xTaskGetTickCount();

    if (expiry <= now)
    {
        return 0; // Déjà expiré ou en cours de callback
    }

    TickType_t remainingTicks = expiry - now;
    return (uint32_t) ((remainingTicks * 1000) / configTICK_RATE_HZ);
}

void check_stack_usage(void)
{
    UBaseType_t stack_high_water_mark;

    // Vérifier chaque tâche
    LOG_INFO("=== STACK USAGE REPORT ===");

    // Appli_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Appli_TaskHandle);
    LOG_INFO("Appli_Task: %lu bytes free", stack_high_water_mark);

    // LORA_TX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(LORA_TX_TaskHandle);
    LOG_INFO("LORA_TX_Task: %lu bytes free", stack_high_water_mark);

    // LORA_RX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(LORA_RX_TaskHandle);
    LOG_INFO("LORA_RX_Task: %lu bytes free", stack_high_water_mark);

    // Uart_TX_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Uart_TX_TaskHandle);
    LOG_INFO("Uart_TX_Task: %lu bytes free", stack_high_water_mark);

    // Uart1_Task
    stack_high_water_mark = uxTaskGetStackHighWaterMark(Uart1_TaskHandle);
    LOG_INFO("Uart1_Task: %lu bytes free", stack_high_water_mark);

    LOG_INFO("=== END STACK REPORT ===");
}


void envoi_code_erreur (void)        // envoie l'erreur a dest_erreur_reset
{
    uint8_t i3;

    // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
    // Erreurs 20 a 7F : 3 fois        (Appli >0x70)  comm:2,3 util/periph:4  radio:5  tab:6 conc/appli:7
    // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

    if (comptage_erreur < nb_erreurs_envoyees)  // 30 envoyees, 20 enregistrees
    {
       i3 = 0;
       if (comptage_erreur < nb_erreurs_enregistrees)  enr_erreur[comptage_erreur]=code_erreur;
        comptage_erreur++;
        if (code_erreur < (nb_erreurs_4_fois+nb_erreurs_unique))
        {
            if (code_erreur < nb_erreurs_unique) // 0x20  envoi_une seule fois
            {
                if (code_erreur < nb_erreurs_unique)
                {
                    uint8_t index, bit;
                    index = code_erreur/8;  // si code=0xA =>index=1, bit=2;
                    bit = code_erreur % 8;
                    if (! (erreurs_unique[index] & (1<<bit)))
                    {
                        i3=1;
                        erreurs_unique[index] |= (1<<bit);
                    }
                }
            }
            else      // envoi 4 fois max
            {
                uint8_t index, bit, cpt;
                index = (code_erreur-nb_erreurs_unique)/4;  // si code=0xA =>index=3, bit=4;
                bit = (code_erreur % 4)*2;
                cpt = (erreurs_4_fois[index] >> bit) & 3 ;
                if  (cpt < 3)
                {
                    i3=1;
                    erreurs_4_fois[index] += (1 << bit);
                }
            }
        }
        else i3=1;  //  0x80 Ã  0xFF

        if (i3)
          {
          #ifdef dest_erreur
           uint8_t message[30];
           message[0] = dest_erreur;
           message[1] ='E';
           message[2] ='L';
           message[3] = nb_reset + 48;
           message[4] = comptage_erreur + 48;
           message[5] = ':';
           message[6] =deci(code_erreur >> 4 );
           message[7] =deci(code_erreur & 15);
           message[8] = '-';
           message[9] = err_donnee1+48;
           message[10] = err_donnee2+48;
           i3=9;
           if (err_donnee1) i3=10;
           if (err_donnee2) i3=11;
           message[i3] = 0;
           envoie_mess_ASC((const char*)message);
          #endif
          }
    }
    code_erreur=0;
    err_donnee1=0;
    err_donnee2=0;
}

uint8_t deci (uint8_t val) //transforme un char hexa en son charactere ASCII   0->48 9->57 A->65 F->70 G->71
{
  uint8_t resul;
  if (val > 36)
    resul = 91;  // caractere apres Z pour indiquer une erreur
  else if (val < 10)
    resul = val + 48;
  else
    resul = val + 55;
  return resul;
}

