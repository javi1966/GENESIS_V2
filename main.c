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

#define _XTAL_FREQ 32000000   // 0,125 us


int hora , minu;
byte seg;
byte dia, mes,day;
byte anno ;
bool bLec5seg;
bool bVis;

char buff[8];


//*********************** PD3535 ************************************

void initPD3535(byte modo) {

    CE11 = 1;
    CE12 = 0;
    WR = 0;
    A2 = 0;
    datoDisplay = modo;
    WR = 1;
    CE11 = 0;

    CE11 = 0;
    CE12 = 1;
    WR = 0;
    A2 = 0;
    datoDisplay = modo;
    WR = 1;
    CE12 = 0;
}
//*****************************************************************

void disChPD3535(char ch, int pos, int iCh) {


    switch (pos) {
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



    if (iCh < 4) {
        CE11 = 1;
        CE12 = 0;
        WR = 0;
        datoDisplay = ch;
        WR = 1;
        CE11 = 0;
    }
    else {
        CE11 = 0;
        CE12 = 1;
        WR = 0;
        datoDisplay = ch;
        WR = 1;
        CE12 = 0;
    }


}

//*******************************************************************************

void printStrPD3535(char *str) {

    int i = 7, j = 0; //i digito MSB,j display MSB


    while (*str) {

        disChPD3535(*str++, i, j);
        if (--i < 4) //digito 7..4
            i = 7;
        if (++j > 7) //cifra entre displays
            j = 0;
    }
}


//********************** DS1307 ****************************************************

byte bcd2bin(byte BCD) {

    byte number;

    number = (BCD >> 4)*10;
    number += (BCD & 0x0F);

    return number;
}

byte bin2bcd(byte bin) {

    byte bcd;

    bcd = (bin / 10) << 4;
    bcd += (bin % 10);

    return bcd;
}

void init_I2C() {
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    //remapeo RC3->CLK ,RC4->data
    bool state = GIE;
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00;

    SSP1CLKPPS = 0x13; //RC3->SCL
    RC3PPS = 0x14;
    RC4PPS = 0x15; //RC4-> SDA
    SSP1DATPPS = 0x14;

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01;

    GIE = state;

    SSP1STAT = 0x80;
    SSP1CON1 = 0x28;
    SSP1CON3 = 0x00;
    SSP1ADD = 0x4F;
    SSP1BUF = 0x00;

}

void start_I2C() {
    SSP1CON2bits.SEN = 1;
    while (SSP1CON2bits.SEN);

}

void restart_I2C() {

    SSP1CON2bits.RSEN = 1;
    while (SSP1CON2bits.RSEN);
}

void stop_I2C() {

    SSP1CON2bits.PEN = 1;
    while (SSP1CON2bits.PEN);

}

void ack_I2C() {
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
    while (SSP1CON2bits.ACKEN);
}

byte ackStatus_I2C() {

    return 0;
}

void nack_I2C() {

    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
    while (SSP1CON2bits.ACKEN);
    SSP1CON2bits.ACKDT = 0;
}

void idle_I2C() {

    while (SSP1STATbits.R_nW | SSP1CON2bits.SEN | SSP1CON2bits.RSEN
            | SSP1CON2bits.PEN | SSP1CON2bits.RCEN | SSP1CON2bits.ACKEN) {
    };
}

byte write_I2C(byte dato) {

    SSP1BUF = dato;
    idle_I2C();
    return (byte) (!SSP1CON2bits.ACKSTAT);


}

byte read_I2C() {

    SSP1CON2bits.RCEN = 1;
    while (SSP1CON2bits.RCEN);
    return SSP1BUF;
}

void initDS1307() {

    byte sec = 0;

    init_I2C();
    
    start_I2C();
    write_I2C(0xD0);      // WR to RTC
    write_I2C(0x00);      // REG 0
    start_I2C();
    write_I2C(0xD1);      // RD from RTC
    sec = bcd2bin(read_I2C(0)); // Read current "seconds" in DS1307
    stop_I2C();
    sec &= 0x7F;

    __delay_us(3);

   start_I2C();
   write_I2C(0xD0);      // WR to RTC
   write_I2C(0x00);      // REG 0
   write_I2C(bin2bcd(sec));     // Start oscillator with current "seconds value
   start_I2C();
   write_I2C(0xD0);      // WR to RTC
   write_I2C(0x07);      // Control Register
   write_I2C(0x80);     // Disable squarewave output pin
   stop_I2C(); 
   /* sec = getDatoDS1307(0) & 0x7F;
    __delay_us(3); 
    setDatoDS1307(sec & 0x7F, 0);
    write_I2C(0xD0);
    write_I2C(0x07);
    write_I2C(0x80);
    stop_I2C();*/
}

void setDatoDS1307(byte dato, byte addr) {

    start_I2C();
    write_I2C(0xD0);
    write_I2C(addr);
    write_I2C(bin2bcd(dato));
    stop_I2C();
}

byte getDatoDS1307(byte addr) {

    byte data;

    start_I2C();
    write_I2C(0xD0);
    write_I2C(addr);
    start_I2C();
    write_I2C(0xD1);
    data = read_I2C();
    stop_I2C();

    return data;
}

void getHoraDS1307() {

    start_I2C();
    write_I2C(0xD0);
    write_I2C(0x00);
    start_I2C();
    write_I2C(0xD1);
    seg = bcd2bin(read_I2C() & 0x7F);
    ack_I2C();
    minu = bcd2bin(read_I2C() & 0x7F);
    ack_I2C();
    hora = bcd2bin(read_I2C() & 0x3F);

    nack_I2C();
    stop_I2C();
}

void getDiaDS1307() {

    start_I2C();
    write_I2C(0xD0);
    write_I2C(0x03);
    start_I2C();
    write_I2C(0xD1);
    day = bcd2bin(read_I2C() & 0x07);
    ack_I2C();
    dia = bcd2bin(read_I2C() & 0x3F);
    ack_I2C();
    mes = bcd2bin(read_I2C() & 0x1F);
    ack_I2C();
    anno = bcd2bin(read_I2C() & 0x1F);
    nack_I2C();
    stop_I2C();

}

char * getDiaSemana(byte num) {

    char str[2];

    switch (num) {

        case 1:strcpy(str, "Lu");
            break;
        case 2:strcpy(str, "Ma");
            break;
        case 3:strcpy(str, "Mi");
            break;
        case 4:strcpy(str, "Ju");
            break;
        case 5:strcpy(str, "Vi");
            break;
        case 6:strcpy(str, "Sa");
            break;
        case 7:strcpy(str, "Do");
            break;

        default:
            break;

    }
    return (str);
}

void ponDay(int day) {
    setDatoDS1307(day, SEMDIA);
    sprintf(buff, "DIA: %02u", day);
    printStrPD3535(buff);

}

void ponDia(int dia) {


    setDatoDS1307(dia, DIAS);
    sprintf(buff, "DIA: %02u", dia);
    printStrPD3535(buff);

}

void ponMes(int mes) {


    setDatoDS1307(mes, MESES);
    sprintf(buff, "MES:  %02u", mes);
    printStrPD3535(buff);

}

void ponAnno(int anno) {


    setDatoDS1307(anno, ANNOS);
    sprintf(buff, "ANNO: %02u", anno);
    printStrPD3535(buff);

}

void ponHora(int hora) {

    setDatoDS1307(hora & 0x3F, HORAS);
    sprintf(buff, "HORA: %02u", hora);
    printStrPD3535(buff);

}

void ponMinuto(int minu) {

    setDatoDS1307(minu & 0x7F, MINUTOS);
    sprintf(buff, "MINU: %02u", minu);
    printStrPD3535(buff);

}
//**************** DS18B20 ***********************************

short int initErrDS18B20() {


    TRIS_DQ18B20 = 0;
    DQ18B20 = 0;
    __delay_us(500);

    TRIS_DQ18B20 = 1;
    __delay_us(5);

    if (DQ18B20 == 0)
        return 0;

    __delay_us(80);

    if (DQ18B20 == 1)
        return 0;



    __delay_us(420);

    TRIS_DQ18B20 = 1;
    return 1;



}

void initDS18B20() {

    TRIS_DQ18B20 = 0;
    DQ18B20 = 0;
    __delay_us(500);
    TRIS_DQ18B20 = 1;
    __delay_us(80);
    __delay_us(420);
    TRIS_DQ18B20 = 1;
}

void enviaDS18B20(int dato) {
    byte cnt, d;

    for (cnt = 0; cnt < 8; ++cnt) {

        d = dato & 0x01;

        TRIS_DQ18B20 = 0;
        DQ18B20 = 0;
        __delay_us(2);
        DQ18B20 = d;
        __delay_us(60);
        TRIS_DQ18B20 = 1;
        __delay_us(2);
        dato = dato >> 1;
    }

}

byte leeDS18B20() {

    byte cnt, dato = 0;
    byte d;

    for (cnt = 0; cnt < 8; ++cnt) {



        TRIS_DQ18B20 = 0;
        DQ18B20 = 0;
        __delay_us(2);
        TRIS_DQ18B20 = 1;
        __delay_us(8);
        d = DQ18B20;
        dato >>= 1;
        if (d)
            dato |= 0x80;


        __delay_us(120);

    }

    return dato;
}

byte leeRAMDS18B20(int pos) {

    byte dato[9];

    if (pos > 8)
        return 0;

    initDS18B20();
    enviaDS18B20(0xCC);
    enviaDS18B20(0xBE);

    dato[0] = leeDS18B20(); //Temp. LSB
    dato[1] = leeDS18B20(); //Temp. MSB 
    dato[2] = leeDS18B20(); //TH limit
    dato[3] = leeDS18B20(); //TL limit
    dato[4] = leeDS18B20(); //CONFIG
    dato[5] = leeDS18B20(); //RES 0
    dato[6] = leeDS18B20(); //RES 1
    dato[7] = leeDS18B20(); //RES 2
    dato[8] = leeDS18B20(); //CRC


    return dato[pos];

}

int ponresDS18B20(byte res) {

    byte cfg;

    if ((res < 9) || (res > 12))
        res = 9;

    cfg = ((res - 9) << 5);

    if (!initErrDS18B20())
        return 99;

    enviaDS18B20(0xCC);
    enviaDS18B20(0x4E);
    enviaDS18B20(0b01111101);
    enviaDS18B20(0b11001001);
    enviaDS18B20(cfg);

    initDS18B20();
    enviaDS18B20(0xCC);
    enviaDS18B20(0x48);
    __delay_ms(15);

    return ((leeRAMDS18B20(4) & 0b01100000) >> 5);

}

float leeTempDS18B20(byte Resol) {

    byte iTempLSB, iTempMSB, iConfig, iDly;

    long lTemperatura;
    float fTemperatura;

    iConfig = ponresDS18B20(Resol);
    //iConfig = ((leeRAMDS18B20(4) & 0b01100000) >> 5);

    if (!initErrDS18B20())
        return 99;

    enviaDS18B20(0xCC);
    enviaDS18B20(0x44);

    iDly = 1 << iConfig; //retardo medida segun resolucion

    while (iDly--)
        __delay_ms(100);



    iTempLSB = leeRAMDS18B20(0);
    iTempMSB = leeRAMDS18B20(1);

    lTemperatura = MAKE16(iTempMSB, iTempLSB);

    fTemperatura = 0.0;
    fTemperatura = (lTemperatura >> 4) & 0x00FF;

    if (lTemperatura & 0x0001) fTemperatura += 0.06250;
    if (lTemperatura & 0x0002) fTemperatura += 0.12500;
    if (lTemperatura & 0x0004) fTemperatura += 0.25000;
    if (lTemperatura & 0x0008) fTemperatura += 0.50000;

    return fTemperatura;
}



//******************************************************************************************************

void main(void) {

   
    byte modo = 0;
    bool bProg=false;
    
    float fTemperatura;
    byte szGrado = 0x1B;
    int iTarea = 0;
    char *s;
    char buffer[8];


    // NOSC HFINTOSC with 2x PLL; NDIV 1; 
    OSCCON1 = 0x60;
    // CSWHOLD may proceed; SOSCPWR Low power; SOSCBE crystal oscillator; 
    OSCCON3 = 0x00;
    // LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCEN = 0x00;
    // HFFRQ0 32_MHz;
    OSCFRQ = 0x06;
    //MFOR
    OSCSTAT = 0x00;
    // HFTUN 0; 
    OSCTUNE = 0x00;


    LATA = 0B00000000;
    LATB = 0B00000000;
    LATC = 0B00000000;
    WPUA = 0B00000000;
    WPUB = 0B00000000;
    WPUC = 0B00000000;
    ANSELA = 0B00101110;
    ANSELB = 0B11111111;
    ANSELC = 0B00100000;
    TRISA = 0B10010001;
    TRISB = 0B00000000;
    TRISC = 0B00101101;

    LED = 1;

    // Configura timer0 8192 us
    T0CON0 = 0x00;

    T0CON1bits.T0CKPS = 0b1001; //512
    T0CON1bits.T0CS = 0b010; //FOSC/4
    // T0CON1=0x49;
    //0,125 * 512=64us ,10000us/64 156->0x9B
    TMR0H = 0x9B; //registro de periodo 10ms
    TMR0L = 0;
    PIR0bits.TMR0IF = 0;
    PIE0bits.TMR0IE = 1;
    T0CON0bits.T0EN = 1;


    //******************************************************************


    PEIE = 1;
    GIE = 1;


    initDS1307();
    initPD3535(CLEARDISPLAY);
    __delay_ms(100);
    initPD3535(BRI50);
    printStrPD3535("@JEG2016");
    fTemperatura = leeTempDS18B20(10);

    __delay_ms(2000);

    while (1) {

        //boton modo
        if (!BTN_MD) {

            __delay_ms(70);
            if (!BTN_MD) {
              
                if (++modo > 7) 
                    modo = 0;
                    
                
                PIE0bits.TMR0IE = 0; 
                bProg=true;  
             
            }
            
          while(!BTN_MD){};  
        }

        //Boton +
        if (!BTN_UP) {
            __delay_ms(70);
            if (!BTN_UP) {

                switch (modo) {
                    case 0:
                         break;
                    case 1: if (++hora >= 24)
                                hora = 0;
                            setDatoDS1307(hora & 0x3F, HORAS);
                            break;
                    case 2: if (++minu >= 59)
                                 minu = 0;
                            setDatoDS1307(minu & 0x7F, MINUTOS);
                            break;
                    case 3:if (++dia > 31)
                            dia = 1;
                           setDatoDS1307(dia, DIAS);
                           break;
                    case 4: if (++mes > 12)
                               mes = 1;
                            setDatoDS1307(mes, MESES);
                            break;
                    case 5: anno++;
                            setDatoDS1307(anno, ANNOS);
                            break;
                    case 6: if (++day > 7)
                               day = 1;
                            setDatoDS1307(day, SEMDIA);
                            break;
                    default:
                        break;
                }
            }
          while(!BTN_UP){};  
        }

        //Boton -
        if (!BTN_DW) {
            __delay_ms(70);
            if (!BTN_DW) {
                switch (modo) {
                    case 0: break;
                    case 1: if (--hora == 0xFFFF)
                                 hora = 24;
                            setDatoDS1307(hora & 0x3F, HORAS);
                            break;
                    case 2: if (--minu == 0xFFFF)
                                minu = 59;
                            setDatoDS1307(minu & 0x7F, MINUTOS);
                            break;
                    case 3:if (--dia == 0xFF)
                               dia = 31;
                           setDatoDS1307(dia, DIAS);
                           break;
                    case 4: if (--mes == 0xFF)
                               mes = 12;
                            setDatoDS1307(mes, MESES);
                            break;
                    case 5: anno--;
                            setDatoDS1307(anno, ANNOS);
                            break;
                    case 6: if (--day == 0xFF)
                               day = 7;
                            setDatoDS1307(day, SEMDIA);
                            break;
  
                    default:
                        break;
                }

            }
            
             
        }
        
        
        if(bProg){
                switch (modo) {
                    
                    case 0:printStrPD3535("<<PROG>>");
                           break;
                    case 1:sprintf(buff, "HORA: %02u", hora);
                           break;
                    case 2:sprintf(buff, "MINU: %02u", minu);
                           break;
                    case 3:sprintf(buff, "DIA:  %02u", dia);
                           break;
                    case 4:sprintf(buff, "MES:  %02u", mes);
                           break;
                    case 5:sprintf(buff, "ANNO: %02u", anno);
                           break;
                    case 6: s = getDiaSemana(day);
                           sprintf(buff, "DIAS: %c%c",s[0],s[1]);
                           break; 
                           
                    case 7:
                           printStrPD3535("<<PROG>>");
                            __delay_ms(1500);
                           modo=0; 
                           PIE0bits.TMR0IE = 1;
                           bProg=false;
                          
                           break;      
                           
                    default:
                        break;
                }
                
          printStrPD3535(buff);
          
          __delay_ms(100);
        }
          

        if (bVis) {

            switch (iTarea) {

                case 0: getDiaDS1307();
                    s = getDiaSemana(day);
                    sprintf(buffer, "%c%c %02u/%02u", s[0], s[1], dia, mes);
                    printStrPD3535(buffer);
                    break;
                case 1: getHoraDS1307();
                    sprintf(buffer, "%02u:%02u:%02u", hora, minu, seg);
                    printStrPD3535(buffer);
                    break;
                case 2: sprintf(buffer, " %2.2f%c ", fTemperatura, szGrado);
                    printStrPD3535(buffer);
                    break;

                default:break;
            }

            bVis = false;

        }


        if (bLec5seg) {


            fTemperatura = leeTempDS18B20(10);
            if (++iTarea > 2)
                iTarea = 0;
            bLec5seg = false;
        }



    }//while

    return;
}

//****************************************************

void interrupt INTERRUPT_InterruptManager(void) {

    static volatile byte lTemp = 0;
    static volatile uint16_t lLec = 0;

    if (PIR0bits.TMR0IF && PIE0bits.TMR0IE) { //cada 10ms

        if (++lTemp > 30) {
            bVis = true;
            lTemp = 0;
        }

        if (++lLec >= 500) // 10ms 500 5 seg
        {
            LED ^= 1;

            bLec5seg = true;
            lLec = 0;
        }
        PIR0bits.TMR0IF = 0;
    }

}
