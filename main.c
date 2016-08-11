/*
 * File:   main.c
 * Author: t133643
 *
 * Created on 23 de julio de 2016, 8:57
 */


#include <xc.h>
#include <stdbool.h>
#include <stdint.h>

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits->Oscillator not enabled
#pragma config RSTOSC = HFINT32    // Power-up default value for COSC bits->HFINTOSC with 2x PLL (32MHz)
#pragma config CLKOUTEN = OFF    // Clock Out Enable bit->CLKOUT function is disabled; I/O or oscillator function on OSC2
#pragma config CSWEN = ON    // Clock Switch Enable bit->Writing to NOSC and NDIV is allowed
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config MCLRE = ON    // Master Clear Enable bit->MCLR/VPP pin function is MCLR; Weak pull-up enabled
#pragma config PWRTE = OFF    // Power-up Timer Enable bit->PWRT disabled
#pragma config WDTE = OFF    // Watchdog Timer Enable bits->WDT disabled; SWDTEN is ignored
#pragma config LPBOREN = OFF    // Low-power BOR enable bit->ULPBOR disabled
#pragma config BOREN = ON    // Brown-out Reset Enable bits->Brown-out Reset enabled, SBOREN bit ignored
#pragma config BORV = LO    // Brown-out Reset Voltage selection bit->Brown-out voltage (Vbor) set to 2.45V
#pragma config ZCD  = ON
#pragma config PPS1WAY = ON    // PPSLOCK bit One-Way Set Enable bit->The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable bit->Stack Overflow or Underflow will cause a Reset
//#pragma config DEBUG = ON    // Debugger enable bit->Background debugger disabled

// CONFIG3
#pragma config WRT = OFF    // User NVM self-write protection bits->Write protection off
#pragma config LVP = OFF    // Low Voltage Programming Enable bit->Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.

// CONFIG4
#pragma config CP = OFF    // User NVM Program Memory Code Protection bit->User NVM code protection disabled
#pragma config CPD = OFF    // Data NVM Memory Code Protection bit->Data NVM code protection disabled


#define _XTAL_FREQ 32000000   // 0,125 us

typedef unsigned char byte;

#define LED LATCbits.LATC1

#define CE11 LATCbits.LATC7
#define CE11 LATCbits.LATC6
#define A0 LATAbits.LATA5
#define A1 LATAbits.LATA1
#define A2 LATAbits.LATA2
#define WR LATAbits.LATA3

#define datoDisplay LATB
#define DQ18B20    LATCbits.LATC0



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


bool bLec5seg;
bool bVis;

void main(void) {

    int dato;
    float fTemperatura;
    byte szGrado = 0x1B;
    int iTarea = 0;
    char *s;

    byte hora, minu, seg;
    byte dia, mes, anno, day;




    // NOSC HFINTOSC with 2x PLL; NDIV 1; 
    OSCCON1 = 0x00;
    // CSWHOLD may proceed; SOSCPWR Low power; SOSCBE crystal oscillator; 
    OSCCON3 = 0x00;
    // LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCEN = 0x00;
    // HFFRQ0 16_MHz;
    OSCFRQ = 0x06;
    // HFTUN 0; 
    OSCTUNE = 0x00;

    LATA = 0B00000000;
    LATB = 0B00000000;
    LATC = 0B00000000;
    WPUA = 0B00000000;
    WPUB = 0B00000000;
    WPUC = 0B00000000;
    ANSELA = 0B11111111;
    ANSELB = 0B00000000;
    ANSELC = 0B00000000;
    TRISA = 0B00000000;
    TRISB = 0B00000000;
    TRISC = 0B00100001;

    LED = 1;
  
    // Configura timer0 8192 us
    T0CON0 = 0x00;
    T0CON0bits.T016BIT=1;
    T0CON1bits.T0CKPS=0b1000; //256
    T0CON1bits.T0CS=0b010;    //FOSC/4
    
    T0CON0bits.T0EN = 1;
    TMR0L=0;
    TMR0H=0;

    TMR0IF = 0;
    TMR0IE = 1;

  //********************************


    PEIE = 1;
    GIE = 1;


    while (1) {


    }//while

    return;
}

//****************************************************

void interrupt INTERRUPT_InterruptManager(void) {
    
    static byte lTemp=0;
    static byte lLec=0;
    
    if (TMR0IE && TMR0IF) {
        
        if(++lTemp > 3){
            bVis=true;
            lTemp=0;
        }
        
        if(++lLec > 125)
        {
            LED ^= 1;
            
            bLec5seg=true;
            lLec=0;
        }    
      TMR0IF=1;
    } 
    
}  
