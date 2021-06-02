/*
 * File:   machorrobot.c
 * Author: angel
 *
 * Created on 19 de mayo de 2021, 04:49 PM
 */
#include <stdint.h>
#include <stdlib.h>
#include <xc.h>
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


#define _XTAL_FREQ 4000000

#define SERVO1 PORTCbits.RC0
#define SERVO2 PORTCbits.RC1//RC1 //CCP2
#define SERVO3 PORTCbits.RC2 //CCP1
#define SERVO4 PORTCbits.RC3
#define SERVO5 PORTDbits.RD0
#define SERVO6 PORTDbits.RD1
//----------------------
#define DELAY_CAMINAR 100
volatile char angulos[] = {64,64,64,64};
volatile uint8_t contador = 0;
volatile char lecturaUart[50];
volatile char cont = 0;

volatile char adc1, adc2; // variables para el adc

volatile int tiempo; // variable para medir el tiempo para calcular la distancia
volatile char leerSensor = 1;

volatile char comando = 0; 
volatile char numero[4];
volatile char contadorDatos = 0;
volatile char hayComando=0; 
volatile char ejecutaComando = 0;
//------- antirrebote de botonoes ----------------------
char portAactual = 0;
char portAanterior = 0;

char debug[6];
// funciones -------
char cicloDeTrabajo(char valor);
void moveCCPservo(char angle, char servo);
void aletear();
void moonwalk();
void caminar();
void SendChar(const char caracter);
void initUart();
void SendString(const char* string, const uint8_t largo);
void ejecutarComando();
void readPosition();
void savePosition();
// para generar el pwm por tiempo se uso el tmr0 con su interrupcion
/* de forma que ocn un contador se establece cual es el canal de pwm es en el
 que se debe de colocar en 1 y el anterior en 0, al terminar los 8 pwm se deja
 en 0 los 4 canales durante aproximadamente  4 ms para poder lograr un periodo 
 de entre 6ms y 14ms para la señal del servo este periodo depende de los 4
 anchos de pulso */
/*para establecer el valor del tmr0 se le resta el valor requerido a 0 para 
 que el valor resultante equivalga a 256-valor debido que que son numeros sin 
 signo de 8 bits */
void __interrupt() ISR(){
    
    if(INTCONbits.T0IF){
        
        
        switch(contador){
            case 0 :
                SERVO1 = 1;
                TMR0 = 0-cicloDeTrabajo(angulos[contador]);
                break;
            case 1:
                SERVO1 = 0;
                SERVO4 = 1;
                TMR0 = 0-cicloDeTrabajo(angulos[contador]);
                break;
            case 2:
                SERVO4 = 0;
                SERVO5 = 1;
                TMR0 = 0-cicloDeTrabajo(angulos[contador]);
                break;
            case 3:
                SERVO5 = 0;
                SERVO6 = 1;
                TMR0 = 0-cicloDeTrabajo(angulos[contador]);
                break;
            case 4:
                SERVO6 = 0;
                TMR0 = 0; // se estan en 0 todo el resto del periodo
                contador = 255; // para que cuando se sume 1 sea 0
                break;
            
        }
        contador++;
        T0IF = 0;
    }
    
    if (PIR1bits.RCIF){
        char aux = RCREG;
        if(!ejecutaComando){
           switch(aux){
            
               case 'A':
               case 'B':
               case 'C':
               case 'D':
               case 'E':
               case 'F':
                   comando = aux;
                   hayComando = 1;
                   break;
               default:
                   if(hayComando){
                       if( aux != ';'  && contadorDatos<3){
                           numero[contadorDatos] = aux;
                           contadorDatos++;
                           
                       }else{
                           numero[contadorDatos] = '\0';
                           
                           ejecutaComando = 1;
                           contadorDatos = 0;
                       }
                              
                   }
                   break;
                
            
            }
        }

        
        
        
    }
    if (PIR1bits.ADIF){
        
        switch(ADCON0bits.CHS){
            case 0:
                adc1 = ADRESH;
                ADCON0bits.CHS = 1;
                ADCON0bits.GO = 1;
                break;
            case 1:
                adc2 = ADRESH;
                ADCON0bits.CHS = 0;
                ADCON0bits.GO =1;
                break;
        }
        
        PIR1bits.ADIF = 0;
        
    }
    
    if(INTCONbits.INTF){
        tiempo = TMR1;
        TMR1 = 0;
        INTCONbits.INTF = 0;
        leerSensor = 1;
        
    }
    
}

void main(void) {
    ANSEL = 0;\
    ANSELH = 0;
    TRISA = 255;
    TRISAbits.TRISA3 = 0; // salida
    TRISC = 0;
    TRISD = 0;
    TRISB = 255; 
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 0;
    
    PORTBbits.RB6 = 1;
    PORTBbits.RB7 = 1;
    T2CONbits.TOUTPS = 0; // prescaler de 1
    T2CONbits.T2CKPS = 2; // post scaler de 16
    //CONFIGURACION DEL CCP
    CCP2CONbits.CCP2M = 0b1100; //pwm
    CCP2CONbits.DC2B0 =  0;
    CCP2CONbits.DC2B1 =  0;
    CCP1CONbits.CCP1M = 0b1100; 
    CCP1CONbits.P1M = 0;
    CCP1CONbits.DC1B = 0;
    
    // timer 2
    T2CONbits.T2CKPS = 0b11; //prescaler de 16
    T2CONbits.TMR2ON = 1;
    PR2 = 249; // PERIODO DE PWM DE 4.096 mS
    // timer 0
    OPTION_REGbits.T0CS = 0; // fuente de reloj el reloj interno
    OPTION_REGbits.PSA = 0; // PRESCALER ASIGANDO AL TMR0 
    OPTION_REGbits.PS = 0b011; // prescaler de 1:16 (T max 4mS)
    
    // interrupciones
    INTCONbits.GIE = 1; // SE ACTIVAN LAS INTERRUPCIONES
    INTCONbits.PEIE = 1;
    INTCONbits.T0IE = 1; // interrupcion del timer 0 activada
    TMR0 = 0;
    
    initUart();
    
    
   // memoria eprom
    
    EECON1bits.EEPGD = 0; // memoria de datos
    
    
   // adc
    
    ANSEL = 3; // ADC 0 Y 1
    
    ADCON0bits.ADCS = 0b01; // fosc/8
    ADCON0bits.CHS = 0; // canal 0
    ADCON1 = 0; // justificado a la izquyuerda
    ADCON0bits.ADON = 1; // se enciende
    ADCON0bits.GO = 1; // inicia la transmision
    
    //PIE1bits.ADIE = 1; // interrupcion de adc
    
    //---- configuracion del timer 1
    
    T1CONbits.T1GINV = 1; // va a contar cuando gate esta en 1
    T1CONbits.TMR1GE = 1; // se habilita el gate
    T1CONbits.T1CKPS = 0; // prescaler 1:1
    T1CONbits.TMR1CS = 0; // fuente de reloj interna
    T1CONbits.TMR1ON = 1; // lo enciendo
    
    TMR1 = 0;
    // configuracion de interrupcion int
    OPTION_REGbits.INTEDG = 0; // interrupcion en flanco de bajada
    INTCONbits.INTE = 1; // interrupcion de hardware activda
    
    numero[0] = ' ';
    numero[1] = ' ';
    numero[2] = ' ';
    numero[3] = '\0';
    
    PORTA  = 0;
    
    while(1){
        portAanterior = portAactual;
        portAactual = PORTA;
        
        if((portAanterior&0b100)==4 && (portAactual&0b100)==0){
            savePosition();
        }
        if((portAanterior&0b10000)==16 && (portAactual&0b10000)==0){
            readPosition();
        }
        //moveCCPservo(64,1);
        //moveCCPservo(64,2);
        //for (int i = 0; i<4; i++) tiempos[i] =cicloDeTrabajo(angulos[i]);
        //aletear();
        //__delay_ms(500); 
        // si el valor del sensor de distancia es mayor que el el pot aletea
        if (adc1>=adc2 )aletear();
        //moonwalk();
        
        if(leerSensor){
            PORTAbits.RA3 = 1;
            __delay_us(10);
            PORTAbits.RA3 = 0;
            leerSensor =0;
            
        }
        // que la mano este a 4cm aprox 
        
        if(tiempo <= 250) moonwalk();
        if(ejecutaComando){
            ejecutaComando=0;
            hayComando = 0;
            ejecutarComando();
            
        }
        //itoa(debug,tiempo,10);
        //SendString(debug, 10);
        //SendChar('\n');
        //cont = 0;
        
        
    }
    
    return;
}

//----- FUNCIONES PARA DETERMINAR VALROES DE REGISTROS PARA MOVER SERVOS
//convierte un angulo requerido al valor necesitado
// el  valor de char es entre 0-127 que es directamente proporcionar a 0-180
// se pede calcular usando lo siguiente valor = (127/180) *angulo 
char cicloDeTrabajo(char valor){
    return valor+31;
}

// FUNCIONES PARA EL MOVIMIENTO DE LOS SERVOS
void moveCCPservo(char angle, char servo){
    unsigned char valor = cicloDeTrabajo(angle);
    
    switch(servo){
        case 1:
            CCPR1L = valor;
            break;
        case 2:
            CCPR2L = valor;
            break;
    }
    
    
    
}

void aletear(){
    PORTBbits.RB6 = 0;
    
    for (char i = 0; i < 6 ; i++) angulos[i] = 64;
    for (char i = 10; i <70 ; i++ ){
        moveCCPservo(i,1);
        moveCCPservo(i,2);
        __delay_ms(20);
    }
    for (char i = 70; i >10 ; i-- ){
        moveCCPservo(i,1);
        moveCCPservo(i,2);
        __delay_ms(20);
    }
    
}

void moonwalk(){
    PORTBbits.RB6 = 1;
    
    moveCCPservo(70,1);
    moveCCPservo(70,2);
    
    angulos[3]= 64;
    angulos[2]= 64;
    
    for (char i = 64; i > 31; i--){
        angulos[0] = i;
        __delay_ms(10);
    }
    for (char i = 64; i<95; i++){
       angulos[1] = i;
        __delay_ms(10);
    }
    for (char i = 31; i <64; i++){
        angulos[0] = i;
        __delay_ms(10);
    }
    for (char i = 95; i >64; i--){
        angulos[1] = i;
        __delay_ms(10);
    }
    
}
void caminar(){
    // piernas y pie derecho 1 
    for (char i = 64; i > 42; i--){
        angulos[2] = i;
        angulos[0] = i;
        //angulos[1] = 127-i;
        __delay_ms(DELAY_CAMINAR);
    }
//    for (char i = 42; i < 64; i++){
//        angulos[2] = i;
//        angulos[0] = i;
//        angulos[3] = 106-i;
//        angulos[1] = 22+i;
//        //angulos[1] = 127-i;
//        __delay_ms(DELAY_CAMINAR);
//    }
//    
//    for (char i = 64; i > 42; i--){
//        angulos[3] = 106-i;
//        angulos[1] = 22+i;
//        //angulos[1] = 127-i;
//        __delay_ms(DELAY_CAMINAR);
//    }
//    
//    
//    for (char i = 64; i < 86; i++){
//        angulos[2] = i;
//        angulos[0] =127 - i;
//        //angulos[1] = 127-i;
//        __delay_ms(DELAY_CAMINAR);
//    }
//    
//     for (char i = 86; i > 64; i--){
//        angulos[2] = i;
//        angulos[0] = 128-i;
//        angulos[1] = 150-i;
//        angulos[3] = 150-i;
//        __delay_ms(DELAY_CAMINAR);
//    }
//    
//    for (char i = 64; i < 86; i++){
//        angulos[3] = 150-i;
//        angulos[1] = 150-i;
//        __delay_ms(DELAY_CAMINAR);
//    }
// 
   
}

//COMUNICACIÓN SERIAL 

void initUart(){
    
    TXSTAbits.TX9 = 0; // 8 bits
    TXSTAbits.TXEN = 1; // trrnasmicion habilitada
    TXSTAbits.SYNC = 0; //modod asincrono 
    
    TXSTAbits.BRGH = 1; // alta velocidad 
    
    RCSTAbits.SPEN = 1; // habiolita pines seriales 
    RCSTAbits.RX9 = 0 ; // recepcion de 8 bits
    RCSTAbits.CREN = 1; // recepcion habilitada
    
    
    //baud rate de 115200
    
    SPBRG = 12;  // realmente es de 125000
    
    PIE1bits.RCIE = 1; // se activa la interupcion
    //
    INTCONbits.PEIE = 1;
    
}


void SendChar(const char caracter) {
    while (TXSTAbits.TRMT == 0);    // Wait for buffer to be empty
    TXREG = caracter ;
}


void SendString(const char* string, const uint8_t largo) {
    int i = 0;
    for (i=0 ; i<largo && string[i]!='\0' ; i++) {
        SendChar(string[i]);
    }
}

void ejecutarComando(){
    
    int numeros = atoi(numero);
    
    
    PORTBbits.RB7 = ~PORTBbits.RB7;
    SendString(numero,4);
    SendChar('\n');
    numero[0] = ' ';
    numero[1] = ' ';
    numero[2] = ' ';
    numero[3] = '\0';
    switch(comando){
        case 'A':
            angulos[0] = numeros;
            break;
        case 'B':
            angulos[1] = numeros;
            break;
        case 'C':
            moveCCPservo(numeros,1);
            break;
        case 'D':
            moveCCPservo(numeros,2);
            break;
        case 'E':
            angulos[2] = numeros;
            break;
        case 'F':
            angulos[3] = numeros;
            break;
    }
    
}


// ------- USO DE LA EEPROM --------------
void writeEEPROM(char data, char direccion){
   
    EEADR = direccion;
    EEADRH = 0;
    
    EEDATA = data;
    
    EECON1bits.EEPGD = 0; 
    EECON1bits.WREN = 1; 
    
    /// se deshabilitan las interrupciones
    INTCONbits.GIE = 0;
    
    while(INTCONbits.GIE); // mientras no se desabiliten
    
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; 
    
    INTCONbits.GIE = 1; // habilito interrupciones
    EECON1bits.WREN = 0; // deshabilito la escritura
    
}

char readEEPROM(char direccion){
    
    EEADR = direccion;
    EECON1bits.EEPGD = 0; // lectura de eeprom
    EECON1bits.RD = 1;// lectura 
    
    return EEDATA;
}

void savePosition(){
    // se guardan los valores en la eeeprom
    SendString("Guardando Posiciones:\n",75);
    for (char i = 0; i<4; i++){
        writeEEPROM(angulos[i],i);
        
        itoa(debug,i+1,10);
        SendString(debug,6);
        SendString(") ",5);
        itoa(debug,angulos[i],10);
        SendString(debug,6);
        SendChar('\n');
    }    
    writeEEPROM(CCPR1L,4);
    SendString("5) ",4);
    itoa(debug,CCPR1L,10);
    SendString(debug,6);
    SendChar('\n');
    writeEEPROM(CCPR2L,5);
    SendString("6) ",4);
    itoa(debug,CCPR2L,10);
    SendString(debug,6);
    SendChar('\n');
    
}

void readPosition(){
    SendString("Colocando Posiciones:\n",75);
    for (char i = 0; i<4; i++){
        
        angulos[i] = readEEPROM(i);
        itoa(debug,i+1,10);
        SendString(debug,6);
        SendString(") ",5);
        itoa(debug,angulos[i],10);
        SendString(debug,6);
        SendChar('\n');
    }
    CCPR1L = readEEPROM(4);
    SendString("5) ",4);
    itoa(debug,CCPR1L,10);
    SendString(debug,6);
    SendChar('\n');
    CCPR2L = readEEPROM(5);
    SendString("6) ",4);
    itoa(debug,CCPR2L,10);
    SendString(debug,6);
    SendChar('\n');
}


