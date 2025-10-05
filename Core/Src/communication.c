/*
 * functions.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Tocqueville
 */

#include <communication.h>
#include <fonctions.h>
#include <eeprom_emul.h>
#include <log_flash.h>
#include "cmsis_os.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>


#define nb_ligne_routage 2  //  0->Loop  1->Uart   Add+1->Uart
uint8_t table_routage[nb_ligne_routage][6] = {
    {'1', '1', 3, 0, 0, 0},
    {'2', 'z', 7, 'Q', 0, 0}
};

/* gestion des erreurs
Code_erreur => utile dans les ISR, peut masquer les premieres erreurs simultanées, gestion des répétitions
LOG_WARN(..) => envoie un message sur la sortie prédéfinie. message complet, limite 4/10min, pas ISR
LOG en Flash => pb répétitions. Limiter à 4 / 10 minutes*/


QueueHandle_t in_message_queue;  // queue pour les messages entrants

// Timers
TimerHandle_t timer_handles[2];

static uint8_t log_buffer[MESS_LG_MAX_LOG];
static uint16_t len_ASC;
static uint8_t mess_ASC[MESS_LG_MAX]; // Variable locale pour travailler
static char formatted_buffer[MESS_LG_MAX]; // Buffer pour le formatage
static SemaphoreHandle_t log_mutex = NULL;

SUBGHZ_HandleTypeDef hsubghz;

UART_HandleTypeDef huart2;

static uint8_t rx_char;
static uint8_t expected_length = 0;
static uint8_t message_type = 0;
static in_message_t mess_rx_uart;
static uint8_t car_valid;
static uint8_t buffer_index = 0;

UartStruct UartSt[NB_UART];

uint8_t message[MESS_LG_MAX];

extern UART_HandleTypeDef huart2;
extern QueueHandle_t Event_QueueHandle;


void Uart_RX_Tsk(void *argument);
void Uart_TX_Tsk(void *argument);
uint8_t Uart2_receive (uint8_t* data, uint8_t type);
void traitement_rx (uint8_t*, uint8_t lg); // var :longueur n'inclut pas le car_fin_trame (inclus emetteur) : 4 pour RLT1

/* Definitions for Uart2_RX_Task */
osThreadId_t Uart_RX_TaskHandle;
const osThreadAttr_t Uart_RX_Task_attributes = {
  .name = "Uart_RX_Task",
  .priority = (osPriority_t) osPriorityLow5,
  .stack_size = 256 * 4    // 190 utilisé
};

/* Definitions for Uart2_TX_Task */
osThreadId_t Uart_TX_TaskHandle;
const osThreadAttr_t Uart_TX_Task_attributes = {
  .name = "Uart_TXTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4    // 89 utilisé (+121 pour LOG)
};


// Variable globale pour le niveau de verbosité
static uint8_t current_log_level = CURRENT_LOG_LEVEL;


static uint8_t mess_buffer[MESS_BUFFER_SIZE];
static uint16_t head = 0;
static uint16_t tail = 0;

static osMutexId_t bufferMutex;


uint8_t mess_enqueue(const uint8_t *data, uint8_t len);
uint8_t mess_dequeue(uint8_t *data, uint8_t *len);


void TIMEOUT_RX_Callback(TimerHandle_t xTimer)
{
	uint8_t num_uart = (uint8_t)(uint32_t)pvTimerGetTimerID(xTimer);  // timer_id
	LOG_WARNING("RX Timeout for UART %lu", num_uart);
	    code_erreur=timeout_RX;   //timeout apres 1 car Recu
    err_donnee1= num_uart+'0';
    buffer_index = 0;
    raz_Uart(num_uart);
}


void init_communication(void)
{

    // Créer la queue de messages
 	in_message_queue = xQueueCreate(10, sizeof(in_message_t));
 	/*if (in_message_queue == NULL)
 	{
 	  LOG_ERROR("Failed to create in_message queue");
 	  return;
 	}*/

	// mutex pour envoyer les log
    log_mutex = xSemaphoreCreateMutex();

	// Initialisation du mutex pour mettre enqueue
	bufferMutex = osMutexNew(NULL);
	/*if (bufferMutex == NULL) {
		LOG_ERROR("Failed to create bufferMutex");
		return;
	}*/
	//LOG_INFO("bufferMutex created: %p", bufferMutex);

	/* creation of Uart_TX_Task */
	Uart_TX_TaskHandle = osThreadNew(Uart_TX_Tsk, NULL, &Uart_TX_Task_attributes);

   UartSt[0].h_timeout_RX = xTimerCreate("TimeoutRX2", pdMS_TO_TICKS(10000), pdFALSE, ( void * ) 0, TIMEOUT_RX_Callback);  // name,period-tick, autoreload,id, callback
   //UartSt[0].h_timeout_TX = xTimerCreate("TimeoutTX2", pdMS_TO_TICKS(10000), pdFALSE, ( void * ) 0, TIMEOUT_TX_Callback);  // name,period-tick, autoreload,id, callback

   /* creation of Uart_RX_Task */
   Uart_RX_TaskHandle = osThreadNew(Uart_RX_Tsk, NULL, &Uart_RX_Task_attributes);


   raz_Uart(0);

   //HAL_UART_Transmit(&huart2, (uint8_t*)"Init2", 5, 3000);
   //HAL_Delay(500);
}


/**
* @brief Function implementing the Uart1_Task thread.
* @param argument: Not used
ACSII : longueur n'inclut pas le car_fin_trame (inclus emetteur) : 5 pour RLSLO (en fait 6)
Binaire : RL2SLO => lg=2 (en fait 6)
* @retval None
*/
void Uart_RX_Tsk(void *argument)
{

    // Démarrer la surveillance watchdog
    watchdog_task_start(WATCHDOG_TASK_UART_RX);
    //HAL_UART_Transmit(&huart2, (uint8_t*)"Init4", 5, 3000);
    //HAL_Delay(500);

    //LOG_INFO("Uart_RX_Task started with watchdog protection");

    for(;;)
    {
        // Heartbeat watchdog
        watchdog_task_heartbeat(WATCHDOG_TASK_UART_RX);

        // Lire un caractère
		uint8_t status = HAL_UART_Receive(&huart2, &rx_char, 1, 100);
        if (status == HAL_OK)
        {
        	car_valid=1;
   		  //LOG_INFO("Received: 0x%02X ('%c')", rx_char, (rx_char >= 32 && rx_char <= 126) ? rx_char : '.');

            // Premier caractère : déterminer le type

            if (buffer_index == 0)
            {
                // Vérifier le bit de poids fort
                if (rx_char & 0x80)
                {
                	if ((rx_char==13) || (rx_char==10) || (!rx_char)) car_valid=0;  // pas CR ou LF en premier car
                    message_type = 1;  // Message binaire
                    LOG_DEBUG("Binary message detected");
                }
                else
                {
                    message_type = 0;  // Message ASCII
                    //LOG_DEBUG("ASCII message detected");
                }
            }

            // Ajouter au buffer
            if ((buffer_index < sizeof(mess_rx_uart.data) - 2) && (car_valid))
            {
   			    if ((buffer_index==0) && (rx_char=='1'))  rx_char=My_Address;  // remplacement de 1 par mon adresse

            	mess_rx_uart.data[buffer_index++] = rx_char;
   			    xTimerReset(UartSt[0].h_timeout_RX, 0); // timeout au bout de x secondes si on ne recoit pas la fin du message

				#ifdef UART_AJOUT_EMETTEUR
				  if (buffer_index==1)
				  {
	            	mess_rx_uart.data[buffer_index++] = '1';
				  }
				#endif
                // Traitement selon le type
                if (message_type == 0)
                {
	  	  	  	  	// Message ASCII : fin par '\0'
                    if ((rx_char == '\0') || (rx_char == 13))
                    {
                        // Message ASCII terminé
   					    xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout
   					    mess_rx_uart.length = buffer_index;
   					    mess_rx_uart.type = 0; // ASCII
   					    mess_rx_uart.source = 2; // UART2
                        //mess_rx_uart.timestamp = HAL_GetTick();

   					    if (rx_char==13)  // remplace par 0
   					    {
   					    	mess_rx_uart.data[buffer_index-1] = 0;

   					    }
                        //LOG_INFO("%s lg:%d",mess_rx_uart.data, mess_rx_uart.length-1 );
                        // Envoyer à la queue
                        if (buffer_index>4)  // minimum xxSL
                        {
                        	mess_rx_uart.length--;
							if (xQueueSend(in_message_queue, &mess_rx_uart, 0) != pdPASS)
							{
								code_erreur = erreur_RX_queue;
								LOG_ERROR("UART message queue full");
							}
							else
							{  // envoi de l'evenement a la tache appli
								event_t evt = {EVENT_UART_RX, SOURCE_UART, 0};
								if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
									code_erreur = erreur_queue_appli;
							}
                        }
                        // Réinitialiser
                        buffer_index = 0;
                        expected_length = 0;
                    }
                }
                else
                {
                    // Message binaire : longueur dans le 3ème octet
                    if (buffer_index == 3)
                    {
                        expected_length = mess_rx_uart.data[2]; // 3ème octet = longueur
                        LOG_DEBUG("Binary message length: %d", expected_length);
                        if ((expected_length <3) || (expected_length>(MESS_LG_MAX+3)))
                        {
                        	code_erreur=erreur_rx_uart_bin;
                        	err_donnee1 = 1;
                        	err_donnee2 = expected_length;
                            buffer_index = 0;
                        }
                    }

                    // Vérifier si on a reçu toute la longueur
                    if (buffer_index >= 3 && buffer_index >= expected_length + 3)
                    {
                        // Message binaire terminé
   					    xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout
   					    mess_rx_uart.length = buffer_index;
   					    mess_rx_uart.type = 1; // Binaire
   					    mess_rx_uart.source = 2; // UART2
                        //message.timestamp = HAL_GetTick();

                        // Envoyer à la queue
                        if (xQueueSend(in_message_queue, &mess_rx_uart, 0) != pdPASS)
                        {
                            LOG_ERROR("uartin_message queue full");
                        }
                        else
                        {  // envoi de l'evenement a la tache appli
			                event_t evt = {EVENT_UART_RX, SOURCE_UART, 0};
			                if (xQueueSend(Event_QueueHandle, &evt, 0) != pdPASS)
	                            code_erreur = erreur_queue_appli;
                        }

                        // Réinitialiser
                        buffer_index = 0;
                        expected_length = 0;
                    }
                }
            }
            else
            {
                // Buffer plein, réinitialiser
            	code_erreur = erreur_RX_full;
            	LOG_WARNING("UART buffer overflow, resetting");
                buffer_index = 0;
                expected_length = 0;
            }
		}
        else if (status == HAL_TIMEOUT) {
		    //LOG_WARNING("UART timeout - no data received");
		}
        else {
		    LOG_ERROR("UART receive error: %d", status);
		    osDelay(100);
		}

		osDelay(100);
		//LOG_INFO(".");
	}
  /* USER CODE END Uart1_Tsk */
}

// Tache appli : TODO attendre que message_in soit libre (CC:30us) ou mettre en queue
void reception_message_Uart2(in_message_t *msg)
{
	if (msg->type == 0)
	{
		// Message ASCII
		//LOG_INFO("Received ASCII message: %.*s", msg->length, msg->data);
		//LOG_INFO("Received ASCII message: %s lg:%d", msg->data, msg->length);
	}
	else
	{
		// Message binaire
		LOG_INFO("Received binary message, length: %d", msg->length);
		LOG_DEBUG("Binary data: ");
		for (int i = 0; i < msg->length; i++)
		{
			LOG_DEBUG("%02X ", msg->data[i]);
		}
	}

	// Appeler votre fonction de traitement existante
	traitement_rx(msg->data, msg->length);
}




void raz_Uart(uint8_t num_uart)  // raz car en reception (suite timeout)
{
   buffer_index = 0;
   xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout RX
}


// PRS0 -> PURS0 (lg=5)
uint8_t envoie_mess_ASC(const char* format, ...)
{

	if (format == NULL) {
		return 1; // Erreur : buffer nul
	}

	// Formatage des arguments variables
	va_list args;
	va_start(args, format);
	len_ASC = vsnprintf(formatted_buffer, sizeof(formatted_buffer) - 1, format, args);
	va_end(args);

	// Vérifier si le formatage a réussi
	if (len_ASC < 0) {
		return 3; // Erreur : formatage échoué
	}


	// Vérifier si len<3 ou la longueur dépasse la taille maximale
	if ((len_ASC < 3) || ( (len_ASC+3) >= MESS_LG_MAX)) {
		return 2; // Erreur : dépassement de buffer
	}

	// Copier buf dans mess
	memcpy(mess_ASC+1, formatted_buffer, len_ASC + 1); // decalage & +1 pour inclure le '\0'
	len_ASC += 2;
	mess_ASC[0] = formatted_buffer[0];
	mess_ASC[1] = My_Address;


	// Envoyer le message
	uint8_t res = envoie_routage(mess_ASC, len_ASC);
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
	mess[1] = My_Address;

	// Envoyer le message
	uint8_t res = envoie_routage(mess, len);
	return res;

}

// utilise la table de routage pour envoyer le message
uint8_t envoie_routage( uint8_t *mess, uint8_t len)  // envoi du message
{
	uint8_t destinataire, i, j, retc;
	retc=1;

	destinataire = mess[0] & 0x7F;
	//long_def = message_temp[0] & 0x80; // 0 si message texte, 80 si message hexa

	if (((destinataire == My_Address) && (My_Address!='1')) || (destinataire == '0'))   // envoi sur soi-meme -loop
	{
	    if (len < MESS_LG_MAX -1)
	    {
	        //message_in[0] = '0' + message[0] & 0x80;  // emetteur=loop
	        traitement_rx (mess, len);
	        retc=0;
	    }
	}
	else
	{
	  j = 0;
	  for (i = 0; i < nb_ligne_routage; i++) // recherche de la liaison a utiliser dans la table de routage
	  {
		  if ((destinataire >= table_routage[i][0])
			  && (destinataire <= table_routage[i][1]))
			{
			  j = table_routage[i][2]; // type liaison
			  break;
			}
	  }
	  if (j)
	  {
		   if (j==3)
			   retc = mess_enqueue(mess, len);
		   if (j==6)
			  retc = send_lora_message((const char*) mess, len, destinataire);
  		   if (j==7)
			  retc = send_lora_message((const char*)mess, len,  table_routage[i][3]);
 	  }
  }
  return retc;

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

void Uart_TX_Tsk(void *argument)
{
    uint8_t msg[MESS_LG_MAX];
    uint8_t len;

    // Démarrer la surveillance watchdog pour cette tâche
    watchdog_task_start(WATCHDOG_TASK_UART_TX);
    //LOG_INFO("Uart_TX_Task started with watchdog protection");

    for (;;)
    {
        // Enregistrer un heartbeat pour le watchdog
        watchdog_task_heartbeat(WATCHDOG_TASK_UART_TX);
        
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
 * @brief Envoyer un message LoRa
 * @param message: Message à envoyer (string)
 * @retval HAL_StatusTypeDef: Statut de l'opération
 */
HAL_StatusTypeDef send_lora_message(const char* message, uint8_t message_length, uint8_t dest)
{
    uint8_t radio_status;
    //uint16_t message_length = strlen(message);

    // Vérifier que le radio est libre
    if (HAL_SUBGHZ_ReadRegister(&hsubghz, 0x01, &radio_status) != HAL_OK) {
        LOG_ERROR("Failed to read radio status");
        return HAL_ERROR;
    }

    if (radio_status & 0x01) { // En transmission
        LOG_WARNING("LoRa radio busy");
        return HAL_BUSY;
    }

    // Écrire le message dans le buffer radio
    if (HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x00, (uint8_t*)message, message_length) != HAL_OK) {
        LOG_ERROR("Failed to write LoRa buffer");
        return HAL_ERROR;
    }

    // Démarrer la transmission
    uint8_t tx_cmd = 0x83; // Commande TX
    HAL_StatusTypeDef status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, tx_cmd, NULL, 0);

    if (status == HAL_OK) {
        LOG_INFO("LoRa transmission started: %s", message);
    } else {
        LOG_ERROR("Failed to start LoRa transmission: %d", status);
    }

    return status;
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

    if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return; // Échec d'acquisition du mutex
        }
    else
    {
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
		log_buffer[1] = My_Address;
		strcpy((char*)log_buffer+2, prefix);

		// Formatage des arguments
		va_list args;
		va_start(args, format);
		vsnprintf((char*)log_buffer + strlen(prefix)+2, sizeof(log_buffer) - strlen(prefix) - 4, format, args);
		va_end(args);

		// Ajouter un retour à la ligne
		strcat((char*)log_buffer, "\r\n");
		//len_ASC+=2;

		// Envoyer via UART
		//HAL_UART_Transmit(&huart2, (uint8_t*)log_buffer, strlen(log_buffer), 3000);
		//envoie_mess_ASC("%s", log_buffer);
		len_ASC = strlen((char*)log_buffer);
		envoie_routage(log_buffer, len_ASC);
	    xSemaphoreGive(log_mutex);
    }
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

void traitement_rx (uint8_t* message_in, uint8_t longueur_m) // var :longueur n'inclut pas le car_fin_trame (inclus emetteur) : 4 pour RLT1
{       // def :longueur n'inclut pas la longueur (inclus l'emetteur) : 4 pour RL1T1
  uint8_t a,  crc, long_def;//, emet_m;
  //uint8_t tempo1, tempo2, tempo3, tempo4;
  //unsigned char volatile   * pregistre_xdata;  // pointeur vers Memory (ROM, RAM, EEprom, registres)
  //unsigned int volatile    * pregistre_int_xdata;  // pointeur vers Memory (ROM, RAM, EEprom, registres)
  //uint32_t i32;

  //raz_timer_sleep ();
  message[0] = message_in[1]; // emetteur devient le destinataire du futur message
  //emet_m = message[0];

  if (((message_in[0] & 0x7F) != My_Address) && ((message_in[0] & 0x7F) != '0'))  // Transfert ailleurs
  {
      if (longueur_m  < MESS_LG_MAX-1)
      {
        #ifdef CC13xx
          // Message long
          if ((message_in[0]==('L'+0x80)) && (message_in[3]=='Y'))    // MESSAGE LONG : L-Em-Lg-Y-type-num*2
          {
              uint8_t code;
              send_long_buffer_tx=0;
              if (port_dest(message_in[1])==3)  // Node et STM32 : si reception de l'uart
              {
                  message_test[1] = 1;  // TODO
                  recep_mess_long_uart();
                  code = 1;
              }
              else
              {
                  code = recep_long( message_in);   // Concentrateur : reception de RF
                  recep_mess_long_rf(code);   // Concentrateur : reception de RF
              }

              if ((send_long_actif) && (code < 4))
              {
                  if (message_in[4] == 2)   // derniere trame
                  {
                      send_long_actif = 0;
                      CLOCK_STOP( xTimer_envoi);
                  }
                  memcpy (message, message_in, longueur_m + 1);
                  flag_comm_transfert=1;
                  if ((port_dest ('L') & 0b110) == 6)  // vers RF : 6 ou 7
                  {
                       uint16_t num_trame;
                       num_trame = ((uint16_t)message_in[5]<<8) + message_in[6];
                       Mess_statut = 0x10; // pas d'Ack  // bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)  bit3:diffï¿½rï¿½   bit4:pas d'ack  bit5:RX apres
                       if (!(num_trame%10)) Mess_statut = 0;  // Ack pour 1 message sur 10  TODO
                  }

                  transf_buff (longueur_m);  // transfert du message long
              }
          }
          else  // message normal a transferer
          #endif
          {
              memcpy (message, message_in, longueur_m + 1);
              envoie_routage (message, longueur_m);  // envoi du message
          }
      }
      else
      {
          code_erreur = erreur_mess;  // et '2'
          err_donnee1 = '2';
          err_donnee2 = message_in[0];
      }
  }
    #ifdef Uart2    // Transfert les messages vers l'uart
      else if ( (message_in[0] == My_Address) && (message_in[2] == '1'))  // transfert vers Uart : XL1HLH->1LHLH - message texte
      {
          if (longueur_m+1 < MESSAGE_SIZE)
          {
              memcpy (message, message_in+1, longueur_m );
              message[0] = '1';
              message[1] = message_in[1];
              flag_comm_transfert=1;
              transf_buff (longueur_m-1);
          }
      }
      else if ((message_in[0] == (My_Address|0x80)) && (message_in[3] == '1'))  // transfert vers Uart : XL31HLH->1L2HLH - message longueur definie
      {
          if (longueur_m+1 < MESSAGE_SIZE)
          {
              memcpy (message, message_in+1, longueur_m );
              message[0] = '1'+0x80;
              message[1] = message_in[1];
              message[2] = message_in[2]-1;
              flag_comm_transfert=1;
              transf_buff (longueur_m-1);
          }
      }
    #endif
  else
  {
      if (longueur_m >= MESS_LG_MAX)
      {
          longueur_m = MESS_LG_MAX;
          code_erreur =erreur_mess;
      }

      long_def = message_in[0] & 0x80;
      if (long_def)         // Si longueur definie dans le message
      {
          for (a = 2; a < longueur_m; a++) // 2 pour long=3, RL0T - suppression de la longueur
            message_in[a] = message_in[a + 1];
      }


      crc = 1;

      if (message_in[2] & 0x80)   // si CRC avec 1er car de la payload
      {
          crc = 0;
          for (a = 2; a < (longueur_m - 1); a++)   // long=5 si RLT1cf
            crc = crc + message_in[a];
          message_in[2] = message_in[2] & 0x7F;
          if (message_in[longueur_m - 1] != crc)
            crc = 0;
          else
            {
              longueur_m--;
            }
          message[1] = 'S';  // Accusï¿½ reception ok : S1
          message[2] = '1';
          message[3] = car_fin_trame;
          envoie_mess_ASC((const char*)message);
          if (!crc)
              message[2] = '0'; // si crc errone => envoi de S0
      }

      if (crc)
      {
          #ifdef CC13xx
          Mess_statut = 0;  // statut par defaut pour tout envoi de message
          if (rxrf_numero_device > lastAddedSensorNode)   // 0:pas de device  1:device 0 etc...
              rxrf_numero_device=0;
          #endif

          // ******************************** AAAAAAAAAAAAAAAAAAAAA  ********************

          if ((message_in[2] == 'A') && (message_in[3] == 'L')) // AL Lecture entree analogique
          {
              if ( (message_in[4] >='0') &&  (message_in[4] <= '9') && (longueur_m==5)) // Read ALx  : Lecture entree analog x
              {
                  uint8_t ret;
                  uint16_t ent_anal=13;
                  //ret = lecture_analog((message_in[4]-'0'), &ent_anal);  // 0:30(solaire)   1:27(batterie)
                  ret=0;
                  if (!ret)
                  {
                      envoie_mess_ASC("%cVAnal %i", message[0], ent_anal);
                  }
              }
          }
          if ((message_in[2] == 'S') && (message_in[3] == 'L'))
          {
              if ( (message_in[4] =='O')  && (longueur_m==5))  // 1SLO
              {
                  envoie_mess_ASC("1OKK");
              }
              if ( (message_in[4] =='T') && (message_in[5] =='a') && (longueur_m==6))  // SLTa  Stack des taches
  				 check_stack_usage();

              if ( (message_in[4] =='W') && (message_in[5] =='a') && (longueur_m==6))  // SLWa  Watchdog etat
            	  watchdog_print_status();

              if ( (message_in[4] =='R') && (message_in[5] =='e') && (longueur_m==6))  // SLWa  Reset cause et diagnostic
            	  display_reset_cause();
          }
          if ((message_in[2] == 'T') && (message_in[3] == 'E'))  // Test eeprom/log_flash
          {
			  if ( (message_in[4] =='E')  && (longueur_m==7))  // Ecriture eeprom  1TEE12
			  {
			     if (EEPROM_Write8(message_in[5]-'0', message_in[6]-'0') == HAL_OK) {
				    LOG_INFO("EEPROM écrit");
			     } else {
				    LOG_ERROR("Erreur écriture EEPROM ");
			     }
			  }
			  if ( (message_in[4] =='R')  && (longueur_m==6))  // Lecture eeprom  1TER1
			  {
				uint8_t value;
				if (EEPROM_Read8(message_in[5]-'0', &value) == HAL_OK) {
					LOG_INFO("EEPROM lu: adresse %d = 0x%02X", message_in[5]-'0', value);
				} else {
					LOG_ERROR("Erreur lecture EEPROM ");
				}
			  }
			  if ( (message_in[4] =='L')  && (longueur_m==7))  // Ecriture LOG  1TEL12
			  {
				char short_message[8];
				strncpy(short_message, (char*)message_in, 7);
				short_message[7] = '\0';
			    if (log_write(5, 0x01, 0x02, 0x03, short_message) == 0) {
				  LOG_INFO("LOG écrit");
			    }   else {
				  LOG_ERROR("Erreur écriture LOG ");
			    }
		      }
		      if ( (message_in[4] =='A')  && (longueur_m==7))  // LEcture Log 1TEA01
		      {
			     uint16_t logs_read = log_read(message_in[5]-'0', message_in[6]-'0', '1');
			     LOG_INFO("Logs lus: %i", logs_read);
		      }
          }
          if ((message_in[2] == 'L') && (message_in[3] == 'O'))  // log_flash
          {
		      if ( (message_in[4] =='R')  && (message_in[5] =='a') &&  (message_in[6] =='z')
		    		  && (longueur_m==7))  // Force effacement page Log 1LORaz
		      {
		    	  LOG_Format();
		      }
		      if ( (message_in[4] =='S')   && (longueur_m==5))  // Log:stats 1LOS
		      {
		    	  uint32_t total_entries, free_space;
		    	  log_get_stats(&total_entries, &free_space);
		      }
		      if ( (message_in[4] =='E')  && (message_in[5] =='E') &&  (message_in[6] =='Z')
		    		  && (longueur_m==7))  // Force effacement page eeprom 1LOEEZ
		      {
		    	  EEPROM_Format();
		      }
		      if ( (message_in[4] =='E') && (message_in[5] =='E') &&(longueur_m==6))  // EEprom:stats 1LOEE
		      {
		    	  uint32_t total_entries, free_space;
		    	  EEPROM_GetStats(&total_entries, &free_space);
		      }
         }
      }
  }
}

/**
 * @brief Vérifie la configuration flash
 */
void check_flash_config(void)
{
    LOG_INFO("=== CONFIGURATION FLASH ===");
    LOG_INFO("FLASH_BASE: 0x%08lX", FLASH_BASE);
    LOG_INFO("FLASH_SIZE: %d Ko", FLASH_SIZE);
    LOG_INFO("FLASH_PAGE_SIZE: %d octets", FLASH_PAGE_SIZE);
    
    // Vérifier que les adresses EEPROM sont dans la plage flash
    if (EEPROM_PAGE_0_ADDR >= FLASH_BASE && EEPROM_PAGE_0_ADDR < (FLASH_BASE + FLASH_SIZE * 1024)) {
        LOG_INFO("Adresses EEPROM dans la plage flash: OK");
    } else {
        LOG_ERROR("Adresses EEPROM hors plage flash!");
    }
    
    // Vérifier que les pages ne sont pas dans la zone du programme
    if (EEPROM_PAGE_0_ADDR < (FLASH_BASE + 50 * 1024)) {  // 50KB de programme
        LOG_ERROR("Pages EEPROM dans la zone du programme!");
    } else {
        LOG_INFO("Pages EEPROM hors zone programme: OK");
    }
    
    LOG_INFO("EEPROM_PAGE_0_ADDR: 0x%08lX", EEPROM_PAGE_0_ADDR);
}

/**
 * @brief Vérifie les permissions flash
 */
void check_flash_permissions(void)
{
    LOG_INFO("=== PERMISSIONS FLASH ===");
    
    // Vérifier les erreurs ECC
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCR_ERRORS)) {
        LOG_ERROR("Erreurs ECC detectees!");
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCR_ERRORS);
    }
    
    // Vérifier les erreurs d'opération
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR)) {
        LOG_ERROR("Erreur d'operation flash!");
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    }
    
    // Vérifier les erreurs de protection
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR)) {
        LOG_ERROR("Erreur de protection d'ecriture!");
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    }
    
    // Vérifier les erreurs d'alignement
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR)) {
        LOG_ERROR("Erreur d'alignement!");
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    }
    
    LOG_INFO("Flash accessible: OK");
}

