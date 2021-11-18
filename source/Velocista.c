
/**
 * @file    Velocista.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "Util.h"
#include "string.h"

/***********************************************************************************************************************
 * PROTOTIPOS
 **********************************************************************************************************************/
void DecodificarHeaderESP();
void CksVerif();
void CMD();

void ESP_SendATCommand(void);    //, uint16_t msTimeout
void ESP_chargebuf(const char *str,uint8_t size);
void InitAT(void);
void HeaderToTX();
void Send_Sensores();
void Sensar(uint8_t Nsen);

/***********************************************************************************************************************
 * DEFINICIONES Y CONSTANTES
 **********************************************************************************************************************/
volatile _sFlag flag1;
volatile _sFlag flag1, flag2;
_sWork PWM1, PWM2,TimerMS;
_sWork Sensor1,Sensor2,Sensor3,Sensor4,Sensor5,Sensor6,Sensor7,Sensor8;
_unionNd Nd,proxBytes;


#define START flag1.bit.b0
#define STOP flag1.bit.b1
#define PWM flag1.bit.b3
#define SENSAR_OK flag1.bit.b4
#define Send_sensors flag1.bit.b5

const char ATmux[]={"AT+CIPMUX=0\r\n"};
const char ATmode[]={"AT+CWMODE=3\r\n"};
const char ATReset[]={"AT+RST\r\n"};

const char wifimicros[]= {'A','T','+','C','W','J','A','P','=','"','M','I','C','R','O','S','"',',','"','m','i','c','r','o','s','1','2','3','4','5','6','7','"','\r','\n'};
const char wificasa[]= {'A','T','+','C','W','J','A','P','=','"','2','0','8','e','8','8','"',',','"','2','4','6','7','4','9','3','6','1','"','\r','\n'};

const char ATudpcasa[]={'A','T','+','C','I','P','S','T','A','R','T','=','"','U','D','P','"',',','"','1','9','2','.','1','6','8','.','0','.','1','1','"',',','3','0','0','1','4',',','3','0','1','4','\r','\n'};
const char ATudpmicros[]={'A','T','+','C','I','P','S','T','A','R','T','=','"','U','D','P','"',',','"','1','9','2','.','1','6','8','.','1','.','8','"',',','3','0','0','1','4',',','3','0','1','4','\r','\n'};  //192.168.1.5

const char ATsendData[]={"AT+CIPSEND=9\r\n"}; // Envia ESP CONECTADO

//const char ATclose[]={"AT+CIPCLOSE"};
//const char ATCifsr[]={"AT+CIFSR\r\n"};

// AT Respuesta Comandos

const char ResATmux[]={"AT+CIPMUX=0\r\n\r\nOK\r\n"};
const char ResATmode[]={"AT+CWMODE=3\r\n\r\nOK\r\n"};

const char Reswificasa2[]="AT+CWJAP=\"208e88\",\"246749361\"\r\nWIFI DISCONNECT\r\nWIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";
const char ResATudpcasa[]="AT+CIPSTART=\"UDP\",\"192.168.0.11\",30014,3014\r\nCONNECT\r\n\r\nOK\r\n";
const char Reswificasa[]="AT+CWJAP=\"208e88\",\"246749361\"\r\nWIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";


const char Reswifimicros[]="AT+CWJAP=\"MICROS\",\"micros1234567\"\r\nWIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";
const char ResATudpmicros[]="AT+CIPSTART=\"UDP\",\"192.168.1.8\",30014,3014\r\nCONNECT\r\n\r\nOK\r\n";

const char ResATsendData[]="AT+CIPSEND=9\r\n\r\nOK\r\n> ";
const char ResATsendData2[]="\r\nRecv 9 bytes\r\n\r\nSEND OK\r\n";

const char ResATsendData10[]="AT+CIPSEND=10\r\n\r\nOK\r\n> ";
const char ResATsendData101[]="\r\nRecv 10 bytes\r\n\r\nSEND OK\r\n";

/***********************************************************************************************************************
 * VARIABLES
 **********************************************************************************************************************/
uint8_t StatusProgram;                                                                         // Variable para controlar estado del programa
volatile uint32_t timerCounter,timerConvercion,timerReset,TimerPWM;



               /******** VARIABLES PARA ESP******************/

volatile _rx ESP_Rx;
volatile _tx ESP_Tx;
enum state {ST_ATmux, ST_ATmode, ST_ATwifi, ST_udp,ST_ATsend,ST_ATdata,ST_Conect,ST_Error};    // Estados de Comandos AT
enum state current_state;
uint8_t charge=0,n=0;
uint8_t RxBufEsp[256],TxBufEsp[256];
uint8_t indexHeaderESP;                                                                        // Para lectura del Heder
uint8_t cks,cmdPos_inBuff,Command,cksSend;                                                     // cheksun y guardo posicion del comando


              /******** VARIABLES PARA USB******************/
uint8_t RxBufUsb[256],TxBufUsb[256];                                                           // Variables para USB

volatile uint8_t Sensores,posSen;
int32_t Cyn70[8];

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitLEDsPins();
    BOARD_InitButtonsPins();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif


    /* Force the counter to be placed into memory. */

    ESP_Tx.buf=TxBufEsp;                          // Buffer de transmisión
    ESP_Tx.iW=0;
    ESP_Tx.iR=0;

    ESP_Rx.buf=RxBufEsp;                          // Buffer de recepción
    ESP_Rx.iW=0;
    ESP_Rx.iR=0;

    current_state=ST_Error;
    indexHeaderESP=0;
    StatusProgram=0;
    timerReset=200;
    n=0;
    charge=0;

    Sensores=0;
    posSen=0;

    PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_1);              // Inicio el PIT canal 2
    /* Enter an infinite loop, just incrementing a counter. */



    while(1) {

    	switch(StatusProgram){
    	      case 0:
    	             InitAT();

    	               if((current_state==ST_Conect) && (ESP_Rx.iW==ESP_Rx.iR) ){

    	            	   	GPIO_PortToggle(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
    	            	   	StatusProgram=1;

    	               }
    	      break;

    	      case 1:

    	    	  DecodificarHeaderESP();

    	      break;

    	      case 2:

    	    	  CksVerif();

    	      break;

    	      case 3:
    	    	  ESP_Rx.iR++;
    	    	  CMD();

    	      break;

    	      case 4:

    		           switch(Command){

							   case 0xF0:
									  if(ESP_Rx.iW != ESP_Rx.iR){

										 if(ESP_Rx.buf[ESP_Rx.iR]!='>'){
										   ESP_Rx.iR++;
										  }
										 else{
											   Nd.value=0x03;
											   HeaderToTX();
											   ESP_Tx.buf[ESP_Tx.iW++]=0xF0;
											   cksSend^=0xF0;
											   ESP_Tx.buf[ESP_Tx.iW++]=0xD0;
											   cksSend^=0xF0;
											   ESP_Tx.buf[ESP_Tx.iW++]=cksSend;
											   StatusProgram=1;
											   charge=0;

													   }
										}

							   break;

							   case 0xD0:                                       // Tiempo Iniciado
									  if(ESP_Rx.iW != ESP_Rx.iR){

										 if(ESP_Rx.buf[ESP_Rx.iR]!='>'){
										   ESP_Rx.iR++;
										  }
										 else{
											   Nd.value=0x03;
											   HeaderToTX();
											   ESP_Tx.buf[ESP_Tx.iW++]=0xD0;
											   cksSend^=0xF0;
											   ESP_Tx.buf[ESP_Tx.iW++]=0x0D;
											   cksSend^=0xF0;
											   ESP_Tx.buf[ESP_Tx.iW++]=cksSend;
											   StatusProgram=1;
											   charge=0;

													   }
										}

							    break;

    		           }



    	      break;
    	    }


      if(Send_sensors==1 && timerConvercion==0){


    	if(charge==0){

        ESP_chargebuf("AT+CIPSEND=41\r\n",sizeof("AT+CIPSEND=41\r\n"));
        charge=1;

    	}


    	if(ESP_Rx.iW != ESP_Rx.iR){

		       if(ESP_Rx.buf[ESP_Rx.iR]!='>'){
				  ESP_Rx.iR++;
				}

				    Nd.value=0x22;
					HeaderToTX();
					Send_Sensores();
					timerConvercion=10;
					charge=0;
    	          }


       }


       ESP_SendATCommand();


        __asm volatile ("nop");
    }
    return 0 ;
}


/***********************************************************************************************************************
 * FUNCIONES PARA UAR3 (Llegan datos del ESP)
 **********************************************************************************************************************/


void UART3_SERIAL_RX_TX_IRQHANDLER(void) {
  uint32_t intStatus;
  /* Reading all interrupt flags of status registers */
  intStatus = UART_GetStatusFlags(UART3_PERIPHERAL);

  /* Flags can be cleared by reading the status register and reading/writing data registers.
    See the reference manual for details of each flag.
    The UART_ClearStatusFlags() function can be also used for clearing of flags in case the content of data regsiter is not used.
    For example:
        status_t status;
        intStatus &= ~(kUART_RxOverrunFlag | kUART_NoiseErrorFlag | kUART_FramingErrorFlag | kUART_ParityErrorFlag);
        status = UART_ClearStatusFlags(UART3_PERIPHERAL, intStatus);
  */

  /* Place your code here */
  if(intStatus & UART_S1_RDRF_MASK){

	 uint8_t data;

	  if( timerReset==0){
	  data = UART_ReadByte(UART3);
	  ESP_Rx.buf[ESP_Rx.iW]= data;  // Guardo los datos en el buffer recibidos en el UART y incremento el indice de escritura
	  ESP_Rx.iW++;
	  }else{  data = UART_ReadByte(UART3);}


  }

  intStatus &= ~(kUART_RxOverrunFlag | kUART_NoiseErrorFlag | kUART_FramingErrorFlag | kUART_ParityErrorFlag);
  UART_ClearStatusFlags(UART3_PERIPHERAL, intStatus);




  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}



/***********************************************************************************************************************
 * FUNCIONES PARA ESP
 **********************************************************************************************************************/



/*Funcion para enviar datos por UDP */

void ESP_SendATCommand(){



  if(kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(UART3)){
	if(ESP_Tx.iR != ESP_Tx.iW){

 		UART_WriteByte(UART3,ESP_Tx.buf[ESP_Tx.iR++]);

    }
  }


}


/*Funcion para cargar datos en buffer  */

void ESP_chargebuf(const char *str,uint8_t size){
	uint8_t i ;

	for(i=0; i<size;i++){

		ESP_Tx.buf[ESP_Tx.iW]=*(str+i);
		ESP_Tx.iW++;
	}

}

/*Funcion de inicializacion ESP */

void InitAT(void){

static uint8_t n=0,charge=0;


if((!timerCounter) & (GPIO_PinRead(BOARD_SW2_GPIO,BOARD_SW2_PIN))){              // Parpadeo de LED
        GPIO_PortToggle(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
        timerCounter=25;
        }


if (current_state!=ST_Conect){                  // si no esta conectado el ESP realizo rutina de conexion

	  switch (current_state){

	  case ST_ATmux:

	    if(timerReset==0){

	    	 if(charge==0 ){                     // carga comando AT mux cuando el tiempo del reset es 0 y chargue es 0

	    		ESP_chargebuf(ATmux,sizeof(ATmux));
	    		charge=1;
	    	 }

	    if(ESP_Rx.iW != ESP_Rx.iR){              // si los indices son distintos llegaron datos (respuestas del ESP)

	          if(ESP_Rx.buf[ESP_Rx.iR]==ResATmux[n]){     // comparo los datos con los cargados en la flash
	    		  ESP_Rx.iR++;
	    		  n++;

	    		   if(ResATmux[n]=='\0'){                 // Si termine de comparar  reseteo las variables y cambio de estado
	    		       n=0;
	    		       charge=0;
	    		       current_state=ST_ATmode;
	    		     }

	    		}else{

	    			current_state=ST_Error;           // Si algun dato llego erroneo paso al estado error sonde reseto el ESP
  	        	    timerReset=200;

	  	    		  }
       }

       }
	   break;

	   case ST_ATmode:

	      if(charge==0){
	         ESP_chargebuf(ATmode,sizeof(ATmode));
	         charge=1;
	            }

	      if(ESP_Rx.iW != ESP_Rx.iR){

	    	  if(ESP_Rx.buf[ESP_Rx.iR]==ResATmode[n]){
    		      ESP_Rx.iR++;
    		      n++;

    		      if(ResATmode[n]=='\0'){
    		          n=0;
    		          charge=0;
    		          current_state=ST_ATwifi;

    		       }
             }else{
                      current_state=ST_Error;
                      timerReset=200;
    	           }

	       }

         break;

	     case ST_ATwifi:

	        if(charge==0){
	          ESP_chargebuf(wificasa,sizeof(wificasa));
	          charge=1;
	        }

	        if(ESP_Rx.iW != ESP_Rx.iR){

	        	if(ESP_Rx.buf[ESP_Rx.iR]==Reswificasa[n]){

	        		ESP_Rx.iR++;
    		         n++;

    		       if(Reswificasa[n]=='\0'){
    		        	n=0;
    		        	charge=0;
    		        	current_state=ST_udp;

    		       }

    		 }else{
                      current_state=ST_Error;
                      timerReset=200;
    		       }
             }

          break;

	      case ST_udp:

	         if(charge==0){
	             ESP_chargebuf(ATudpcasa,sizeof(ATudpcasa));
	        	 charge=1;
	            }

	    	 if(ESP_Rx.iW != ESP_Rx.iR){

	    		if(ESP_Rx.buf[ESP_Rx.iR]==ResATudpcasa[n]){
    		        ESP_Rx.iR++;
    		        n++;

    		        if(ResATudpcasa[n]=='\0'){
    		        	n=0;
    		        	charge=0;
    		        	current_state=ST_ATsend;
                    }

    		   }else{

  		        	  current_state=ST_Error;
  		        	  timerReset=200;
  		            }
	    	}

          break;

          case ST_ATsend:

	        if(charge==0){
	           ESP_chargebuf(ATsendData,sizeof(ATsendData));
	            charge=1;
	        }

	        if(ESP_Rx.iW != ESP_Rx.iR){

	            if(ESP_Rx.buf[ESP_Rx.iR]==ResATsendData[n]){
	        	   ESP_Rx.iR++;
	        	    n++;

	        	    if(ResATsendData[n]=='\0'){
	          		 n=0;
	        	  	 charge=0;
	           		 current_state=ST_ATdata;
                    }

	            }else{


	        	   	  current_state=ST_Error;
	        	   	  timerReset=200;
	                  }
	          }
            break;


	        case ST_ATdata:
	        	 if(charge==0){
	             Nd.value=0x02;
	             HeaderToTX();
	             ESP_Tx.buf[ESP_Tx.iW++]=0x0A;
	        	 cksSend^=0x0A;
		         ESP_Tx.buf[ESP_Tx.iW++]=cksSend;
		         charge=1;


	        	 }


	        	 if(ESP_Rx.iW != ESP_Rx.iR){

	        	 if(ESP_Rx.buf[ESP_Rx.iR]==ResATsendData2[n]){
	        	 	ESP_Rx.iR++;
	        	 	 n++;

	        	      if(ResATsendData2[n]=='\0'){
	        	 	    n=0;
	        	 	    charge=0;
	        	 	    current_state=ST_Conect;
	        	                            }

	        	}
	        	 	          }
	      	    break;


	        case ST_Error:

	        	 GPIO_PinWrite(BOARD_PTA1_GPIO, BOARD_PTA1_PIN, 0U);
	        	 if(timerReset<=100){
	        	 GPIO_PinWrite(BOARD_PTA1_GPIO, BOARD_PTA1_PIN, 1U);
	        	 current_state=ST_ATmux;
	        	 }

	        	 n=0;
	        	 charge=0;

	        break;

	    }
	}


}


void HeaderToTX(){           //Ncoman numero de comando a enviar

	ESP_Tx.buf[ESP_Tx.iW++]='U';
	cksSend='U';
    ESP_Tx.buf[ESP_Tx.iW++]='N';
    cksSend^='N';
    ESP_Tx.buf[ESP_Tx.iW++]='E';
    cksSend^='E';
    ESP_Tx.buf[ESP_Tx.iW++]='R';
    cksSend^='R';
    ESP_Tx.buf[ESP_Tx.iW++]=Nd.v[0];
    cksSend^=Nd.v[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Nd.v[1];
    cksSend^=Nd.v[1];
    ESP_Tx.buf[ESP_Tx.iW++]=':';
    cksSend^=':';

}



/***********************************************************************************************************************
 * FUNCIONES PARA LEER LOS DATOS
 **********************************************************************************************************************/



/* DECODIFICACION Y LECTURA DE COMANDOS */

void DecodificarHeaderESP(){
	uint8_t aux;
	aux=ESP_Rx.iW;


	if((ESP_Rx.iR!=aux)&&(StatusProgram==1)) {

	    	switch(indexHeaderESP) {

	    	case 0:
	    		if((ESP_Rx.buf[ESP_Rx.iR])==':'){
	    			ESP_Rx.iR++;
	    			indexHeaderESP++;

	    		}else{

	    			ESP_Rx.iR++;
	    		}

	    		break;

             case 1://U
	                if(ESP_Rx.buf[ESP_Rx.iR]=='U') {
	                    cks='U';
	                    ESP_Rx.iR++;
	                    indexHeaderESP++;
	                }
	                else {
	                	ESP_Rx.iR++;
	                    indexHeaderESP=-1;
	                }
	                break;
	            case 2://UN
	                if(ESP_Rx.buf[ESP_Rx.iR]=='N') {
	                    cks^='N';
	                    ESP_Rx.iR++;
	                    indexHeaderESP++;
	                }
	                else {
	                	ESP_Rx.iR--;
	                    indexHeaderESP=-1;
	                }
	                break;
	            case 3://UNE
	                if(ESP_Rx.buf[ESP_Rx.iR]=='E') {
	                    cks^='E';
	                    ESP_Rx.iR++;
	                    indexHeaderESP++;
	                }
	                else {
	                	ESP_Rx.iR--;
	                    indexHeaderESP=-1;
	                }
	                break;
	            case 4://UNER
	                if(ESP_Rx.buf[ESP_Rx.iR]=='R') {
	                    cks^='R';
	                    ESP_Rx.iR++;
	                    indexHeaderESP++;
	                }
	                else {
	                	ESP_Rx.iR--;
	                    indexHeaderESP=-1;
	                }
	                break;

	            case 5: //byte menos significativo
	                if(ESP_Rx.buf[ESP_Rx.iR]>0) {
	                    cks^=(ESP_Rx.buf[ESP_Rx.iR]);
	                    proxBytes.v[0]=ESP_Rx.buf[ESP_Rx.iR];
	                    ESP_Rx.iR++;
	                    indexHeaderESP++;
	                }
	                else {
	                	ESP_Rx.iR--;
	                    indexHeaderESP=-1;
	                }
	                break;

	            case 6://byte mas significativo
	                if(ESP_Rx.buf[ESP_Rx.iR]==0) {
	                    cks^=ESP_Rx.buf[ESP_Rx.iR];
	                    proxBytes.v[1]=ESP_Rx.buf[ESP_Rx.iR];
	                    ESP_Rx.iR++;
	                    indexHeaderESP++;
	                }
	                else {
	                	ESP_Rx.iR--;
	                    indexHeaderESP=-1;
	                }
	                break;

	            case 7: // ':'
	                if(ESP_Rx.buf[ESP_Rx.iR]==':') {
	                    cks^=ESP_Rx.buf[ESP_Rx.iR];
	                    ESP_Rx.iR++;
	                    indexHeaderESP++;
	                    StatusProgram=2;//llego toda la cabecera
	                }
	                else {
	                	ESP_Rx.iR--;
	                    indexHeaderESP=-1;
	                }
	                break;
	        }
	    }
	}

void CksVerif(){
    static uint8_t p=1;

    while((ESP_Rx.iR!=ESP_Rx.iW)&&(p<proxBytes.value)){         // suo el cks hasta que p sea igual al numero de byte enviados

    	if(p==1){
    		cmdPos_inBuff=ESP_Rx.iR;
    	}

        cks^=ESP_Rx.buf[ESP_Rx.iR];
        ESP_Rx.iR++;
        p++;
    }

    if( (p==proxBytes.value)&&(ESP_Rx.iR!=ESP_Rx.iW) ){
        p=1;
        if(cks==ESP_Rx.buf[ESP_Rx.iR]){
        	StatusProgram=3;
        }
        else{
        	StatusProgram=1;
            ESP_Rx.iR=ESP_Rx.iW;
        }
        indexHeaderESP=0;
    }
}


/* LEER COMANDOS */

void CMD(){

 switch(ESP_Rx.buf[cmdPos_inBuff]){

	case 0xF0:

		ESP_chargebuf("AT+CIPSEND=10\r\n",sizeof("AT+CIPSEND=10\r\n"));

	    StatusProgram=4;
        Command=0xF0;

	    break;
	case 0xD0:             // Cargar PWM a motores
		ESP_chargebuf("AT+CIPSEND=10\r\n",sizeof("AT+CIPSEND=10\r\n"));

		cmdPos_inBuff++;
        PWM1.u8[0]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        PWM1.u8[1]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        PWM1.u8[2]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        PWM1.u8[3]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        PWM2.u8[0]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        PWM2.u8[1]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        PWM2.u8[2]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        PWM2.u8[3]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        TimerMS.u8[0]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        TimerMS.u8[1]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        TimerMS.u8[2]=ESP_Rx.buf[cmdPos_inBuff];
        cmdPos_inBuff++;
        TimerMS.u8[3]=ESP_Rx.buf[cmdPos_inBuff];

        TimerPWM=(TimerMS.u32/10);

        FTM_StopTimer(FTM0_PERIPHERAL);
        FTM0_PERIPHERAL->CONTROLS[1].CnV=PWM1.u16[0];
        FTM0_PERIPHERAL->CONTROLS[2].CnV=PWM2.u16[0];
         //FTM0_PERIPHERAL->CONTROLS[1].CnV=PWM11;
         //	FTM0_PERIPHERAL->CONTROLS[2].CnV=PWM21;
        FTM_StartTimer(FTM0_PERIPHERAL, kFTM_SystemClock);
        PWM=1;
		GPIO_PortToggle(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		StatusProgram=4;
		Command=0xD0;

	break;

	case 0XA0:

		StatusProgram=1;
		START=1;

	break;

	case 0X0A:

		StatusProgram=1;
		STOP=1;

	break;

	case 0X0E:                                // Pido valores de sensores

		Send_sensors=1;
		StatusProgram=1;
		timerConvercion=10;

	break;
	    }
	}

/***********************************************************************************************************************
 * TEMPORIZADORES
 **********************************************************************************************************************/

/* PIT0_IRQn interrupt handler */
void PIT_CHANNEL_0_IRQHANDLER(void) {
  uint32_t intStatus;
  /* Reading all interrupt flags of status register */
  intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
  PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);

  /* Place your code here */

  if(timerCounter != 0U){
    	  timerCounter--;

  }

    if(timerReset != 0U){
  	  timerReset--;
        }

    if(timerConvercion != 0U){
  	  timerConvercion--;
         }

    if(TimerPWM != 0U){
    	TimerPWM--;

        }


  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


/* PIT1_IRQn interrupt handler */
void PIT_CHANNEL_1_IRQHANDLER(void) {
  uint32_t intStatus;
  /* Reading all interrupt flags of status register */
  intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_1);
  PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_1, intStatus);

//  /* Place your code here */


  Sensar(Sensores);
  SENSAR_OK=0;

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/***********************************************************************************************************************
 * ANALOGICOS
 **********************************************************************************************************************/

/* ADC1_IRQn interrupt handler */
void ADC1_IRQHANDLER(void) {


  /* Place your code here */
	Cyn70[Sensores++]=ADC16_GetChannelConversionValue(ADC1_PERIPHERAL,0);


  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


/* ADC0_IRQn interrupt handler */
void ADC0_IRQHANDLER(void) {


  /* Place your code here */

	Cyn70[Sensores++]=ADC16_GetChannelConversionValue(ADC0_PERIPHERAL,0);

    if(Sensores>=8){
    	Sensores=0;

    }


  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


void Sensar(uint8_t Nsen){

	switch(Nsen){

   case 0:
	   ADC16_SetChannelConfig(ADC1_PERIPHERAL,0,&ADC1_channelsConfig[0]);

   break;
   case 1:
	   ADC16_SetChannelConfig(ADC1_PERIPHERAL,0,&ADC1_channelsConfig[1]);

   break;

   case 2:
	   ADC16_SetChannelConfig(ADC1_PERIPHERAL,0,&ADC1_channelsConfig[2]);

   break;

   case 3:
	   ADC16_SetChannelConfig(ADC1_PERIPHERAL,0,&ADC1_channelsConfig[3]);

   break;

   case 4:
	   ADC16_SetChannelConfig(ADC0_PERIPHERAL,0,&ADC0_channelsConfig[0]);

   break;

   case 5:
	   ADC16_SetChannelConfig(ADC0_PERIPHERAL,0,&ADC0_channelsConfig[1]);

   break;

   case 6:
	   ADC16_SetChannelConfig(ADC0_PERIPHERAL,0,&ADC0_channelsConfig[2]);

   break;

   case 7:
   ADC16_SetChannelConfig(ADC0_PERIPHERAL,0,&ADC0_channelsConfig[3]);

   break;

   }
}

void Send_Sensores(){

	Sensor1.u32=Cyn70[0];
	Sensor2.u32=Cyn70[1];
	Sensor3.u32=Cyn70[2];
	Sensor4.u32=Cyn70[3];
	Sensor5.u32=Cyn70[4];
	Sensor6.u32=Cyn70[5];
	Sensor7.u32=Cyn70[6];
	Sensor8.u32=Cyn70[7];

	ESP_Tx.buf[ESP_Tx.iW++]=0x0E;
	cksSend^=0x0E;
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor1.u8[0];
    cksSend^=Sensor1.u8[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor1.u8[1];
    cksSend^=Sensor1.u8[1];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor1.u8[2];
    cksSend^=Sensor1.u8[2];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor1.u8[3];
    cksSend^=Sensor1.u8[3];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor2.u8[0];
    cksSend^=Sensor2.u8[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor2.u8[1];
    cksSend^=Sensor2.u8[1];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor2.u8[2];
    cksSend^=Sensor2.u8[2];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor2.u8[3];
    cksSend^=Sensor2.u8[3];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor3.u8[0];
    cksSend^=Sensor3.u8[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor3.u8[1];
    cksSend^=Sensor3.u8[1];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor3.u8[2];
    cksSend^=Sensor3.u8[2];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor3.u8[3];
    cksSend^=Sensor3.u8[3];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor4.u8[0];
    cksSend^=Sensor4.u8[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor4.u8[1];
    cksSend^=Sensor4.u8[1];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor4.u8[2];
    cksSend^=Sensor4.u8[2];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor4.u8[3];
    cksSend^=Sensor4.u8[3];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor5.u8[0];
    cksSend^=Sensor5.u8[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor5.u8[1];
    cksSend^=Sensor5.u8[1];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor5.u8[2];
    cksSend^=Sensor5.u8[2];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor5.u8[3];
    cksSend^=Sensor5.u8[3];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor6.u8[0];
    cksSend^=Sensor6.u8[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor6.u8[1];
    cksSend^=Sensor6.u8[1];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor6.u8[2];
    cksSend^=Sensor6.u8[2];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor6.u8[3];
    cksSend^=Sensor6.u8[3];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor7.u8[0];
    cksSend^=Sensor7.u8[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor7.u8[1];
    cksSend^=Sensor7.u8[1];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor7.u8[2];
    cksSend^=Sensor7.u8[2];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor7.u8[3];
    cksSend^=Sensor7.u8[3];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor8.u8[0];
    cksSend^=Sensor8.u8[0];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor8.u8[1];
    cksSend^=Sensor8.u8[1];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor8.u8[2];
    cksSend^=Sensor8.u8[2];
    ESP_Tx.buf[ESP_Tx.iW++]=Sensor8.u8[3];
    cksSend^=Sensor8.u8[3];
    ESP_Tx.buf[ESP_Tx.iW++]=cksSend;

}
