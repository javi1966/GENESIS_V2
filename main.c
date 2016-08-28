/*
 * File:   main.c
 * Author: t133643
 *
 * Created on 23 de julio de 2016, 8:57
 */


#include "main.h"

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
#pragma config LVP = ON    // Low Voltage Programming Enable bit->Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.

// CONFIG4
#pragma config CP = OFF    // User NVM Program Memory Code Protection bit->User NVM code protection disabled
#pragma config CPD = OFF    // Data NVM Memory Code Protection bit->Data NVM code protection disabled


byte hora, minu, seg;
byte dia, mes, anno, day;
bool bLec5seg;
bool bVis;


//*********************** PD3535 ************************************
void initPD3535(byte modo){ 
    
    CE11=1;CE12=0;WR=0;
    A2=0;
    datoDisplay=modo;
    WR=1;CE11=0;
    
    CE11=0;CE12=1;WR=0;
    A2=0;
    datoDisplay=modo;
    WR=1;CE12=0;
}
//*****************************************************************
void disChPD3535(char ch){
    
    static volatile byte pos=7;
    static volatile byte iCh=0;
    
    switch(pos)
    {
        case 7:POS7;
               break;
        case 6:POS6;
               break;
        case 5:POS5;
               break;
        case 4:POS4;
               break;
        
        default:break;
    }  
    
    if(--pos < 4)
        pos=7;
    
      if(iCh < 4)
      {
        CE11=1;CE12=0;WR=0;
        datoDisplay=ch;
        WR=1;CE11=0;
      } 
    else
        {
        CE11=0;CE12=1;WR=0;
        datoDisplay=ch;
        WR=1;CE12=0;
        }
    
    if(++iCh > 7)
        iCh=0;
}

//*******************************************************************************
void printStrPD3535(char *str){
    
    int i;
    
    i=0;
    while (*str){
        
        disChPD3535(*str++);
        i++;
    }
}


//********************** DS1307 ****************************************************


 byte bcd2bin(byte BCD) {
     
     byte number;
     
     number=(BCD >> 4)*10;
     number+=(BCD & 0x0F);
     
     return number;
 }
 
 byte bin2bcd(byte bin){
     
     byte bcd;
     
     bcd=(bin/10) << 4;
     bcd += (bin%10);
     
     return bcd;
 }
 
 
 void init_I2C()
 {
     
     //remapeo RC3->CLK ,RC4->data
     PPSLOCK=0x55;
     PPSLOCK=0xAA;
     PPSLOCKbits.PPSLOCKED=0x00;
     
     SSP1CLKPPS =0x13; //RC3->SCL
     RC3PPS=0x14;
     RC4PPS=0x15;   //RC4-> SDA
     SSP1DATPPS=0x14;
     
     PPSLOCK=0x55;
     PPSLOCK=0xAA;
     PPSLOCKbits.PPSLOCKED=0x01;
     
     SSP1STAT=0x80;
     SSP1CON1=0x28;
     SSP1CON3=0x00;
     SSP1ADD=0x03;
     SSP1BUF=0x00;
     
 }
 
 void start_I2C(){
     SSP1CON2bits.SEN =1;
     while(SSP1CON2bits.SEN);
     
 }
 
 void restart_I2C(){
     
     SSP1CON2bits.RSEN =1;
     while(SSP1CON2bits.RSEN);
 }
 
 void stop_I2C(){
     
     SSP1CON2bits.PEN =1;
     while(SSP1CON2bits.PEN);
     
 }
 
 
 
 void ack_I2C(){
     SSP1CON2bits.ACKDT=0;
     SSP1CON2bits.ACKEN=1;
     while(SSP1CON2bits.ACKEN);
 }
 byte ackStatus_I2C(){
     
    
 }
 
 void nack_I2C(){
     
     SSP1CON2bits.ACKDT=1;
     SSP1CON2bits.ACKEN=1;
     while(SSP1CON2bits.ACKEN);
     SSP1CON2bits.ACKDT=0;
 }
 
 void idle_I2C(){
     
     while(SSP1STATbits.R_nW | SSP1CON2bits.SEN | SSP1CON2bits.RSEN 
             |SSP1CON2bits.PEN | SSP1CON2bits.RCEN | SSP1CON2bits.ACKEN )
     {};
 }
 
 byte write_I2C(byte dato){
     
     SSP1BUF=dato;
     idle_I2C();
     return (byte)(!SSP1CON2bits.ACKSTAT);
     
     
 }
 
 byte read_I2C(){
     
     SSP1CON2bits.RCEN=1;
     while( SSP1CON2bits.RCEN);
     return SSP1BUF;
 } 
 
 void initDS1307(){
     byte sec;
     
         init_I2C();
         sec=getDatoDS1307(0) & 0x7F;
         setDatoDS1307(sec & 0x7F,0);
         write_I2C(0xD0);
         write_I2C(0x07);
         write_I2C(0x80);
         stop_I2C(); 
 }
 
 
 void setDatoDS1307(byte dato,byte addr){
     
     start_I2C();
     write_I2C(0xD0);
     write_I2C(addr);
     write_I2C(bin2bcd(dato));
     stop_I2C(); 
 }
 
 byte getDatoDS1307(byte addr){
     
     byte data;
       
          start_I2C();
          write_I2C(0xD0);
          write_I2C(addr);
          start_I2C();
          write_I2C(0xD1);
          data=read_I2C();
          stop_I2C();
          
          return data;
 }
 
 
 void getHoraDS1307()
 {
     
          start_I2C();
          write_I2C(0xD0);
          write_I2C(0x00);
          start_I2C();
          write_I2C(0xD1);
          seg=bcd2bin(read_I2C() & 0x7F);
          minu=bcd2bin(read_I2C() & 0x7F);
          hora=bcd2bin(read_I2C() & 0x3F);
          stop_I2C();
 }
 
 void getDiaDS1307(){
     
          start_I2C();
          write_I2C(0xD0);
          write_I2C(0x03);
          start_I2C();
          write_I2C(0xD1);
          day=bcd2bin(read_I2C() & 0x07);
          dia=bcd2bin(read_I2C() & 0x3F);
          mes=bcd2bin(read_I2C() & 0x1F);
          anno=bcd2bin(read_I2C() & 0x0F);
          stop_I2C();
     
 }
 
 
 
 //******************************************************************************************************
void main(void) {

    int dato;
    float fTemperatura;
    byte szGrado = 0x1B;
    int iTarea = 0;
    char *s;

   



  
    // NOSC HFINTOSC with 2x PLL; NDIV 1; 
    OSCCON1 = 0x60;
    // CSWHOLD may proceed; SOSCPWR Low power; SOSCBE crystal oscillator; 
    OSCCON3 = 0x00;
    // LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCEN = 0x00;
    // HFFRQ0 32_MHz;
    OSCFRQ = 0x06;
    //MFOR
    OSCSTAT=0x00;
    // HFTUN 0; 
    OSCTUNE = 0x00;
   

    LATA = 0B00000000;
    LATB = 0B00000000;
    LATC = 0B00000000;
    WPUA = 0B00000000;
    WPUB = 0B00000000;
    WPUC = 0B00000000;
    ANSELA = 0B11111111;
    ANSELB = 0B11111111;
    ANSELC = 0B11100001;
    TRISA = 0B00000000;
    TRISB = 0B00000000;
    TRISC = 0B00100001;

    LED = 1;
  
    // Configura timer0 8192 us
    T0CON0 = 0x00;
    
    T0CON1bits.T0CKPS=0b1001; //512
    T0CON1bits.T0CS=0b010;    //FOSC/4
   // T0CON1=0x49;
    //0,125 * 512=64us ,10000us/64 156->0x9B
    TMR0H=0x9B;   //registro de periodo 10ms
    TMR0L=0;
    PIR0bits.TMR0IF = 0;
    PIE0bits.TMR0IE = 1;
    T0CON0bits.T0EN = 1;
    

  //******************************************************************


    PEIE = 1;
    GIE = 1;
    
    
    initDS1307();
    
    hora=0;
    setDatoDS1307(hora & 0x3F,HORAS);
    __delay_ms(100);
    getHoraDS1307();
    dato=hora;
    initPD3535(CLEARDISPLAY);
    __delay_ms(100);
    initPD3535(BRI50);
    printStrPD3535("@JEG2016");
    
    __delay_ms(1000);
   


    while (1) {
        
       /* 
        LATCbits.LATC2=0;
        __delay_ms(500);
        LATCbits.LATC2=1;
        __delay_ms(500);*/


    }//while

    return;
}

//****************************************************

void interrupt INTERRUPT_InterruptManager(void) {
    
    static volatile byte lTemp=0;
    static volatile uint16_t lLec=0;
    
    if (PIR0bits.TMR0IF && PIE0bits.TMR0IE) { //cada 10ms
        
        if(++lTemp > 30){
            bVis=true;
            lTemp=0;
        }
        
        if(++lLec >= 500)  // 10ms 500 5 seg
        {
            LED ^= 1;
            
            bLec5seg=true;
            lLec=0;
        }    
      PIR0bits.TMR0IF=0;
    } 
    
}  
