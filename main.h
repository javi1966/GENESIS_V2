/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>



typedef unsigned char byte;

#define LED LATCbits.LATC1

#define CE11 LATCbits.LATC7
#define CE12 LATCbits.LATC6
#define A0   LATAbits.LATA5
#define A1   LATAbits.LATA1
#define A2   LATAbits.LATA2
#define WR   LATAbits.LATA3

#define datoDisplay LATB

#define DQ18B20       PORTCbits.RC0
#define TRIS_DQ18B20  TRISCbits.TRISC0

#define BTN_UP        PORTAbits.RA0
#define BTN_MD        PORTAbits.RA4
#define BTN_DW        PORTAbits.RA7 


//comandos diplay PD3535
#define CLEARDISPLAY  0x80
#define LAMPTEST      0x40
#define BLINK         0x20
#define ATRIBUTOS     0x10
#define CURSORON      0x00
#define BLINKCARACTER 0x01
#define CURSORPARPA   0x02
#define CARACURSOR    0x03
#define BLANCO        0x00
#define BRI25         0x01
#define BRI50         0x02
#define BRI100        0x03

#define POS4 A2=1;A1=0;A0=0;
#define POS5 A2=1;A1=0;A0=1;
#define POS6 A2=1;A1=1;A0=0;
#define POS7 A2=1;A1=1;A0=1;


#define SEGUNDOS 0
#define MINUTOS  1
#define HORAS    2
#define SEMDIA   3
#define DIAS     4
#define MESES    5
#define ANNOS    6

#define MAKE16(x,y) (((((unsigned int)x)<<8)) | ((unsigned int)y))


//*********************************************************

byte bcd2bin(byte BCD);
byte bin2bcd(byte bin);
void init_I2C();
void start_I2C();
void restart_I2C();
void stop_I2C();
void ack_I2C();
void nack_I2C();
byte read_I2C();
byte write_I2C(byte dato);
byte ackStatus_I2C();
void idle_I2C();
void initDS1307();
void setDatoDS1307(byte dato,byte addr);
byte getDatoDS1307(byte addr);
void getHoraDS1307();
void getDiaDS1307();
char * getDiaSemana(byte num);
void initPD3535(byte modo);
void disChPD3535(char ch,int,int);
void printStrPD3535(char *str);
short int initErrDS18B20();
void initDS18B20();
void enviaDS18B20(int dato);
byte leeDS18B20();
byte leeRAMDS18B20(int);
int  ponresDS18B20(byte res);
float leeTempDS18B20(byte Resol);



#endif	/* XC_HEADER_TEMPLATE_H */

