/*
 * functions.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Tocqueville
 */

#include <communication.h>
#include <fonctions.h>
#include "cmsis_os.h"
#include "timers.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>


#define nb_ligne_routage 2  //  0->Loop  1->Uart   Add+1->Uart
uint8_t table_routage[nb_ligne_routage][6] = {
    {'1', '1', 3, 0, 0, 0},
    {'2', 'z', 7, 'Q', 0, 0}
};

// Timers
TimerHandle_t timer_handles[2];

SUBGHZ_HandleTypeDef hsubghz;

UART_HandleTypeDef huart2;

uint8_t  flag_trame_rx_Uart2;
uint8_t  Uart2_rx_buff[MESS_LG_MAX+20];
uint8_t  Uart2_rx_head, Uart2_rx_tail;
uint8_t  Uart2_rx_type_trame;
uint8_t  Uart2_rx_nb_car_recu;
uint8_t  Uart2_rx_longu;
uint8_t  Uart2_rx_debut_trame, Uart2_rx_attente;
uint8_t mess_ok, flag_comm_transfert;

UartStruct UartSt[NB_UART];

uint8_t message_in[MESS_LG_MAX];
uint8_t longueur_message_in;
uint8_t message[MESS_LG_MAX];

extern UART_HandleTypeDef huart2;
extern osMessageQueueId_t Event_QueueHandle;

void Uart_RX_Tsk(void *argument);
void Uart_TX_Tsk(void *argument);
uint8_t Uart2_receive (uint8_t* data, uint8_t type);
void traitement_rx (char unsigned longueur_m); // var :longueur n'inclut pas le car_fin_trame (inclus emetteur) : 4 pour RLT1

/* Definitions for Uart2_RX_Task */
osThreadId_t Uart_RX_TaskHandle;
const osThreadAttr_t Uart_RX_Task_attributes = {
  .name = "Uart_RX_Task",
  .priority = (osPriority_t) osPriorityLow5,
  .stack_size = 256 * 4
};

/* Definitions for Uart2_TX_Task */
osThreadId_t Uart_TX_TaskHandle;
const osThreadAttr_t Uart_TX_Task_attributes = {
  .name = "Uart_TXTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};


// Variable globale pour le niveau de verbosité
static uint8_t current_log_level = CURRENT_LOG_LEVEL;

// Buffer pour le formatage
static char log_buffer[256];  // TODO : non protégé par un appel multiple

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
    raz_Uart(num_uart);
}


void init_communication(void)
{
   /* creation of Uart_RX_Task */
   Uart_RX_TaskHandle = osThreadNew(Uart_RX_Tsk, NULL, &Uart_RX_Task_attributes);

	// Initialisation du mutex
	bufferMutex = osMutexNew(NULL);
	if (bufferMutex == NULL) {
		LOG_ERROR("Failed to create bufferMutex");
		return;
	}
	//LOG_INFO("bufferMutex created: %p", bufferMutex);

	/* creation of Uart_TX_Task */
	Uart_TX_TaskHandle = osThreadNew(Uart_TX_Tsk, NULL, &Uart_TX_Task_attributes);

   UartSt[0].h_timeout_RX = xTimerCreate("TimeoutRX2", pdMS_TO_TICKS(10000), pdFALSE, ( void * ) 0, TIMEOUT_RX_Callback);  // name,period-tick, autoreload,id, callback
   //UartSt[0].h_timeout_TX = xTimerCreate("TimeoutTX2", pdMS_TO_TICKS(10000), pdFALSE, ( void * ) 0, TIMEOUT_TX_Callback);  // name,period-tick, autoreload,id, callback

   raz_Uart(0);

}


/**
* @brief Function implementing the Uart1_Task thread.
* @param argument: Not used
* @retval None
*/
void Uart_RX_Tsk(void *argument)
{
	uint8_t c, free;
    HAL_StatusTypeDef status;

    // Démarrer la surveillance watchdog pour cette tâche
    watchdog_task_start(WATCHDOG_TASK_UART1);
    LOG_INFO("Uart RX_Task started with watchdog protection");

	memset(Uart2_rx_buff, 0, sizeof(Uart2_rx_buff));

    osDelay(1000);

	for(;;)
	{
		// Enregistrer un heartbeat pour le watchdog
		watchdog_task_heartbeat(WATCHDOG_TASK_UART1);

		// Lire depuis UART1
		status = HAL_UART_Receive(&huart2, &c, 1, 1000);

		if (status == HAL_OK)
		{
		  LOG_INFO("Received: 0x%02X ('%c')", c, (c >= 32 && c <= 126) ? c : '.');

		  // Calcul du nb de places dispo dans le buffer (en laisser au moins 1)
		  if (Uart2_rx_head < Uart2_rx_tail)
			   free =  Uart2_rx_tail - Uart2_rx_head ;
		  else
			   free = (MESS_LG_MAX+20) - Uart2_rx_head + Uart2_rx_tail;

		  if (free < (2) )
		  {
			 code_erreur = erreur_RX_full;   // Rx_buffer full
			 //suppression message et attente jusqu'a la fin de ce message
			 Uart2_rx_head = Uart2_rx_debut_trame;
			 Uart2_rx_attente=1;
			 //raz_Uart(0);
		  }
		  if ((( (c&0x7F)>0x1F ) && ((c&0x7F) < 0x7B)) || (Uart2_rx_nb_car_recu))   // ne prend pas 0x0A ni 0xFF en premier caractere
		  {
			  uint8_t debu_rx = Uart2_rx_head;
			  if (!Uart2_rx_attente)
			  {
				  Uart2_rx_buff[Uart2_rx_head++] = c; /* copy over this byte of data dans le buffer */
				  if( Uart2_rx_head == (MESS_LG_MAX+20) ) /* if wrapping around */
					   Uart2_rx_head = 0; /* reset pointer */
			  }
			  Uart2_rx_nb_car_recu ++;
			  xTimerReset(UartSt[0].h_timeout_RX, 0); // timeout au bout de x secondes si on ne recoit pas la fin du message
			  if (Uart2_rx_nb_car_recu==1)
			  {
				  Uart2_rx_debut_trame = debu_rx;
				  #ifdef DEBUG_TIME
					  etat_cpu[index_time] = 0x31;  // phase reception courte  TODO
					  etat_detail[index_time] = 0;
					  time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
					  if (index_time >MAX_Index) index_time=0;
				  #endif
				  Uart2_rx_type_trame = c & 0x80;  // enregistre le type de trame (hexa ou texte)
				  Uart2_rx_longu = 10;
			  }
			 #ifdef UART_AJOUT_EMETTEUR
				  if (Uart2_rx_nb_car_recu==2)
					  Uart2_rx_longu = (c & 0x7F) - 1;   // enregistre la longueur de trame pour message hexa
			 #else
				  if (Uart2_rx_nb_car_recu==3)
					  Uart2_rx_longu = c & 0x7F;   // enregistre la longueur de trame pour message hexa
			  #endif
			  // Fin de trame
			  if (Uart2_rx_type_trame)
			  {
				  // Trame hexa
				 if (Uart2_rx_nb_car_recu >= (Uart2_rx_longu+4))
				 {
				  #ifdef DEBUG_TIME
					 etat_cpu[index_time] = 0x32;  // phase reception courte  TODO
					 etat_detail[index_time] = Uart2_rx_nb_car_recu;
					 time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
					 if (index_time >MAX_Index) index_time=0;
				  #endif
					 Uart2_rx_nb_car_recu = 0;   // A REVOIR
					 xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout
					 if (!Uart2_rx_attente)
					 {
						 flag_trame_rx_Uart2++;  // nb de car recu => flag pour fin de trame
			                LOG_INFO("Message complet: %s", Uart2_rx_buff);

			                // Envoyer événement
			                event_t evt = {EVENT_UART_RX, SOURCE_UART, 0};
			                osMessageQueuePut(Event_QueueHandle, &evt, 0, 0);
					 }
					 Uart2_rx_attente = 0;
				 }
			  }
			  else   // Trame texte
			  {
				 if ((c == car_fin_trame) || (c == 0xA))  // fin de trame => flag pour reception trame
				 {
				  #ifdef DEBUG_TIME
					 etat_cpu[index_time] = 0x32;  // phase reception courte  TODO
					 etat_detail[index_time] = Uart2_rx_nb_car_recu;
					 time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
					 if (index_time >MAX_Index) index_time=0;
				  #endif

					 Uart2_rx_nb_car_recu = 0;
					 xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout
					 if (!Uart2_rx_attente)
					 {
						flag_trame_rx_Uart2++;
		                LOG_INFO("Message complet: %s", Uart2_rx_buff);

		                // Envoyer événement
		                event_t evt = {EVENT_UART_RX, SOURCE_UART, 0};
		                osMessageQueuePut(Event_QueueHandle, &evt, 0, 0);
					 }
					 Uart2_rx_attente = 0;
				 }
			  }
		  }

		} else if (status == HAL_TIMEOUT) {
		    //LOG_WARNING("UART timeout - no data received");
		} else {
		    LOG_ERROR("UART receive error: %d", status);
		    osDelay(100);
		}

		osDelay(100);
		//LOG_INFO(".");
	}
  /* USER CODE END Uart1_Tsk */
}

// Tache appli : TODO attendre que message_in soit libre (CC:30us) ou mettre en queue
void reception_message_Uart2(void)
{
    uint8_t long_mess, retc;

    if (flag_trame_rx_Uart2)
    {            // Reception message Uart2
        do
        {
          #ifdef DEBUG_TIME
            etat_cpu[index_time] = 0x33;  // phase reception courte  TODO
            etat_detail[index_time] = 0;
            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
            if (index_time >MAX_Index) index_time=0;
          #endif
            //retc = SEMAPHORE_TAKE(MessageInSemHandle, 20); //   20 ms (correspondant a 1 message de 100 car 50kbps)
            retc=1;

          #ifdef DEBUG_TIME
            etat_cpu[index_time] = 0x34;  // phase reception courte  TODO
            etat_detail[index_time] = 0;
            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
            if (index_time >MAX_Index) index_time=0;
          #endif

            if (retc)  // 1:ok
            {
                long_mess = Uart2_receive (message_in, 1); // QLHLH -> 5  QL1HLH->5
                if (long_mess) {
                    #ifdef NODE
                        Mess_statut = 0x00;  // 22:Ack/RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:diffï¿½rï¿½   bit4:pas d'ack  bit5:RX apres
                    #endif
                    //ESP_LOGW(TAG, "\nmessage recu:%i %s\n", long_mess, message_in);
                    traitement_rx(long_mess);
                }
                #ifdef DEBUG_TIME
                etat_cpu[index_time] = 0x35;  // phase reception courte  TODO
                etat_detail[index_time] = 0;
                time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                if (index_time >MAX_Index) index_time=0;
                #endif

                //SEMAPHORE_GIVE( MessageInSemHandle);
            }
            else // suppression message
            {
                Uart2_receive (message_in, 0);
                code_erreur = erreur_messageIn;
                err_donnee1 = '1';
            }
        }
        while ((Uart2_rx_tail != Uart2_rx_head ) && (flag_trame_rx_Uart2));
   }
}

//type : 1:reel  0:factice
uint8_t Uart2_receive (uint8_t* data, uint8_t type)
// Transfert message de la pile Uart2_rx_buff vers message_in  PZZx  => 3
// retourne : longueur (sans car fin, avec emetteur)
{
  uint8_t c, count, type_trame, i1,longueur_message;
  #ifdef C10_CAMERA_PIR   // Uart reorientation messages
    uint8_t destin;
  #endif

  count = 0;
  /* if the buffer is empty  */
  if( Uart2_rx_tail == Uart2_rx_head ) code_erreur = erreur_RX_buff_vide;
  else
  {
      type_trame = Uart2_rx_buff[Uart2_rx_tail] & 0x80;
      i1=1;
      longueur_message=10;
      do /* retrieve the frame/data from the buffer */
      {
         c = Uart2_rx_buff[Uart2_rx_tail++]; /* copy data from buffer */
         if( Uart2_rx_tail == (MESS_LG_MAX+20) )
             Uart2_rx_tail = 0;   /* if wrapping around  reset pointer */
         if(count == 2)
             longueur_message = ( c & 0x7F) + 3;
         if (type_trame)
          {
            if (count >= longueur_message) i1=0;
          }
          else
          {
            if ((c==car_fin_trame) || (c==0xA))
            {
                i1=0;
                c = car_fin_trame;
            }
          }
          count++;
          if (count > MESS_LG_MAX )
          {
            code_erreur = erreur_RX_pas_fin;  // erreur : pas de fin, ca boucle
            i1=0;
            c = car_fin_trame;
          }
          if (count==1)  // Premier caractere : destinataire
          {
              if ((c & 0x7F)=='1')  // destinataire 1 => my_adress  1yyy -> Xyyy
                  c= My_Address + (c & 0x80);
          }
          #ifdef UART_AJOUT_EMETTEUR
            if (count==2)  // 2Â°car : ajout emetteur 1yyy->X1yyy   Xyyy->X1yyy   Qyyy->QXyyy
            {
               /* if ((UART_AJOUT_EMETTEUR=='1') && (((message_in[0] & 0x7F) != My_Address) && ((message_in[0] & 0x7F) !='1')))
                    *data++ = My_Address | 0x80; //
                else*/
                    *data++ = UART_AJOUT_EMETTEUR;
                count++;
            }
          #endif

            #ifdef C10_CAMERA_PIR   // Uart reorientation messages
            //  reception uart:reoriente vers L des messages de l'ESP32 qui commencent par 1B et 5B
            if (count == 1)
                  destin = c & 0x7F;
            if (count == 2)  //
            {
                if ( c != '1' )   // message warning ou erreur de l'ESP32
                {
                    if (type)
                    {
                        *data = My_Address;  // L My C1 C2
                        data--;
                        uint8_t temp = *data;
                        *data = 'L';
                        data = data + 2;
                        *data++ = temp;
                        count = count+2;
                    }
                }
                else  // message normal : Remplace emetteur par My_address+0x80 quand ca vient du port serie
                {
                    if (destin == 'L')   // Permet d'avoir un message lisible pour L
                        c = My_Address | 0x80;
                    else if (destin != My_Address)
                        c = My_Address | 0x80;
                }
            }
          #endif
          if (type)
              *data++ = c;
        } while ( i1 );
  }


/*  if (message_test[1])  // TODO
  {
      message_test[2]++;
      if ((message_in[0]==0xCC) && (message_in[1]==0xE1)  && (message_in[3]=='Y'))
      {
          if ( (message_in[4]==1))
          {
              if ((message_in[2]!=0x67) || (count !=107))
                  test_err(2);
          }
          if ( (message_in[4]==2))
              message_test[1]=0;
      }
      else
          test_err(3);
  }*/


   if (flag_trame_rx_Uart2) flag_trame_rx_Uart2-- ;  // BSP_CRITICAL_STATEMENT
  if (count) count--;
  return count; /* indicate the number of bytes retrieved from the buffer, sans car fin trame */
}


void raz_Uart(uint8_t num_uart)  // raz car en reception (suite timeout)
{
   Uart2_rx_attente=0;
   Uart2_rx_head=0;
   Uart2_rx_tail=0;
   Uart2_rx_nb_car_recu = 0;
   Uart2_rx_longu = 0;
   xTimerStop(UartSt[0].h_timeout_RX, 0);  // raz timeout RX
   flag_trame_rx_Uart2 = 0;
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
	mess[1] = My_Address;


	// Envoyer le message
	uint8_t res = envoie_routage(mess, len);
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
uint8_t envoie_routage(const uint8_t *mess, uint8_t len)  // envoi du message
{
	uint8_t destinataire, i, j, retc;
	retc=1;

	destinataire = mess[0] & 0x7F;
	//long_def = message_temp[0] & 0x80; // 0 si message texte, 80 si message hexa

	if (((destinataire == My_Address) && (My_Address!='1')) || (destinataire == '0'))   // envoi sur soi-meme -loop
	{
	    if (len < MESS_LG_MAX -1)
	    {
	        memcpy (message_in, message, len + 1);
	        //message_in[0] = '0' + message[0] & 0x80;  // emetteur=loop
	        traitement_rx (len);
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
    LOG_INFO("Uart_TX_Task started with watchdog protection");

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

void traitement_rx (char unsigned longueur_m) // var :longueur n'inclut pas le car_fin_trame (inclus emetteur) : 4 pour RLT1
{       // def :longueur n'inclut pas la longueur (inclus l'emetteur) : 4 pour RL1T1
  uint8_t a,  crc, long_def;//, emet_m;
  //uint8_t tempo1, tempo2, tempo3, tempo4;
  //unsigned char volatile   * pregistre_xdata;  // pointeur vers Memory (ROM, RAM, EEprom, registres)
  //unsigned int volatile    * pregistre_int_xdata;  // pointeur vers Memory (ROM, RAM, EEprom, registres)
  //uint32_t i32;
  mess_ok=0;

  //char unsigned cpuSR;  // uint8_t

  flag_comm_transfert=0;
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
              flag_comm_transfert=1;
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
              if ( (message_in[4] =='O')  && (longueur_m==5))  // SLO
              {
                  envoie_mess_ASC("1OKK");
              }
              if ( (message_in[4] =='T') && (message_in[5] =='a') && (longueur_m==6))  // SLTa
              {
  				 check_stack_usage();
                 envoie_mess_ASC("1OKK");
              }
          }
      }
  }
}

