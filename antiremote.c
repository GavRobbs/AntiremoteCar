#define F_CPU 1000000
#define BUTTON_PIN PIND
#define SPEED_JUMP 20
#define CONSTANT_SPEED 80
#define MAX_PWM_VALUE 1024UL

#define STOPPED 0
#define FORWARDS 1
#define BACKWARDS 2

#define STEADY 0
#define ACCELERATING 1
#define DECELERATING 2

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<stdlib.h>

uint8_t MOVEMENT_STATE = 0;
uint8_t MOVEMENT_PHASE = 0;
uint8_t TARGET_STATE = STOPPED;

int pwmSpeedTarget;
int pwmSpeedCurrent;

int lastTime;
int delta;

void stopVehicle(void);
void startVehicle(void);
void reverseVehicle(void);
void continueCurrently(void);

ISR(PCINT2_vect){
    if(bit_is_clear(PIND, PD0)){
        cli();
        stopVehicle();
    } else if(bit_is_clear(PIND, PD1)){
        cli();
        startVehicle();
    } else if(bit_is_clear(PIND, PD2)){
        cli();
        reverseVehicle(); 
    } else{

    }
    sei();
}

void initInterrupts(){
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PD0) | (1 << PD1) | (1 << PD2);
    sei();
}

void initADC(){
    ADMUX |= (1 << REFS0) | (1 << MUX2) | (1 << MUX0);
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADEN);
}

void initPWMAndTimers(){
    TCCR1B = (1 << CS10) |(1 << WGM13) | (1 << WGM11);
    TCCR1A = (1 << COM1A1);
    ICR1 = 150;
    OCR1A = 0;

    TCCR0B = (1 << CS02) | (1 << CS00); //1 tick is 1 ms
}

void initInputs(){
	DDRD = 0b00000000;
	PORTD = 0b00000111;

	DDRB = 0b00100110;
}

void setPWMSpeed(){
    uint16_t ocrval = (((uint16_t)pwmSpeedCurrent) * 150) / MAX_PWM_VALUE;
    OCR1A = ocrval;
}

uint16_t readADC(){
    ADCSRA |= (1 << ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    return ADC;
}

void stopVehicle(){

    TARGET_STATE = STOPPED;
    pwmSpeedTarget = 0;

    if(pwmSpeedCurrent == 0){
        if(MOVEMENT_STATE == STOPPED){
            MOVEMENT_PHASE = STEADY;
        } else{
            MOVEMENT_STATE = STOPPED;
            MOVEMENT_PHASE = STEADY;            
        }
	DDRB &= ~(1 << PB1);
        PORTB &= ~((1 << PB5) | (1 << PB2));
    } else{
        MOVEMENT_PHASE = DECELERATING;
    }
}

void startVehicle(){
    //Read the value of PWM speed target, don't forget to put it here
    pwmSpeedTarget = readADC();

    /*When this button is pressed, checks if the vehicle is already moving forwards. If it isn't, it slows the vehicle down to a complete halt*/
    if(MOVEMENT_STATE != FORWARDS || MOVEMENT_STATE != STOPPED){
        stopVehicle();
        while(pwmSpeedCurrent != 0){
            continueCurrently();
        }
    }

    TARGET_STATE = FORWARDS;
    DDRB &= (1 << PB1);
    PORTB &= ~((1 << PB5) | (1 << PB2));
    PORTB |= (1 << PB2);

    /*If the vehicle is already going forwards, all we have to do is change the current speed. The new speed can be higher and lower than the old, so it can be accelerating or decelerating*/
    if(MOVEMENT_STATE == FORWARDS){
        if(abs(pwmSpeedCurrent - pwmSpeedTarget) <= SPEED_JUMP){
            pwmSpeedCurrent = pwmSpeedTarget;
            MOVEMENT_PHASE = STEADY;
        } else if(pwmSpeedTarget > pwmSpeedCurrent){
            MOVEMENT_PHASE = ACCELERATING;
        } else{
            MOVEMENT_PHASE = DECELERATING;
        }
        return;
    } else{
        MOVEMENT_STATE = FORWARDS;
        MOVEMENT_PHASE = ACCELERATING;
    }
}

void reverseVehicle(){
    //Read the value of PWM speed target, don't forget to put it here
    pwmSpeedTarget = readADC();

    /*When this button is pressed, checks if the vehicle is already moving forwards. If it isn't, it slows the vehicle down to a complete halt*/
    if(MOVEMENT_STATE != BACKWARDS || MOVEMENT_STATE != STOPPED){
        stopVehicle();
        while(pwmSpeedCurrent != 0){
            continueCurrently();
        }
    }

    TARGET_STATE = BACKWARDS;
    DDRB &= (1 << PB1);
    PORTB = ~((1 << PB5) | (1 << PB2));
    PORTB |= (1 << PB5);

    if(MOVEMENT_STATE == BACKWARDS){
        if(abs(pwmSpeedCurrent - pwmSpeedTarget) <= SPEED_JUMP){
            pwmSpeedCurrent = pwmSpeedTarget;
            MOVEMENT_PHASE = STEADY;
        } else if(pwmSpeedTarget > pwmSpeedCurrent){
            MOVEMENT_PHASE = ACCELERATING;
        } else{
            MOVEMENT_PHASE = DECELERATING;
        }
        return;
    } else{
        MOVEMENT_STATE = BACKWARDS;
        MOVEMENT_PHASE = ACCELERATING;
    }
}

void continueCurrently(){

    int ct = TCNT0;
    delta = ct - lastTime;
    lastTime = ct;

    if(MOVEMENT_PHASE == STEADY){
        return;
    }

    if(abs(pwmSpeedCurrent - pwmSpeedTarget) <= SPEED_JUMP){
        pwmSpeedCurrent = pwmSpeedTarget;
        MOVEMENT_PHASE = STEADY;
    }

    if(MOVEMENT_PHASE == ACCELERATING){
        pwmSpeedCurrent += ((uint8_t)CONSTANT_SPEED * (uint8_t)delta)/255;
        if(pwmSpeedCurrent >= pwmSpeedTarget){
            pwmSpeedCurrent = pwmSpeedTarget;
            MOVEMENT_PHASE = STEADY;
        }
    } else{
        pwmSpeedCurrent -= ((uint8_t)CONSTANT_SPEED * (uint8_t)delta)/255;
        if(pwmSpeedCurrent <= pwmSpeedTarget){
            pwmSpeedCurrent = pwmSpeedTarget;
            MOVEMENT_PHASE = STEADY;
        }
    }

    _delay_ms(1);
    setPWMSpeed();
}

int main(void){
    pwmSpeedTarget = 0;
    pwmSpeedCurrent = 0;
    MOVEMENT_STATE = STOPPED;
    TARGET_STATE = STOPPED;
    MOVEMENT_PHASE = STEADY;
    TCNT0 = 0;

    initInputs();
    initADC();
    initPWMAndTimers();

    lastTime = TCNT0;

    initInterrupts();
    
    while(1){
        continueCurrently();
    }

    return 0;
}
