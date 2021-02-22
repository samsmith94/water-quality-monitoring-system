
//nem kell irányt állítani setup-ban
#define TEMPERATURE_PIN         PB8

//szintén nem kell az i2c lábakhoz sem irányt állítani

// OUTPUT
#define USER_LED_PIN            PC13

//EZ LENNE A NYÁK SZERINT:
//#define MIXER_MOTOR_PIN         PA1     //nem működik
//#define PUMP_PWM_PIN            PA2     //működik, de az irányváltás miatt ez legyen a mixer inkább
//#define FAN_PWM_PIN             PA3     //EZ LESZ A MIXER (A FAN MEHETNE A H-HÍD MÁSIK KIMENETÉRE...)

//DE ÁT KELL DEFÍNIÁLNI, MERT AZ ELSŐ NEM MŰKÖDIK SAJNOS

#define MIXER_MOTOR_PIN         PA2
#define PUMP_PWM_PIN            PB0

// H-híd irányállító lábak
#define IN1 PB10
#define IN2 PB1

#define PELTIER_PIN             PA8
#define VALVE_1_PIN             PB12
#define VALVE_2_PIN             PB13
#define VALVE_3_PIN             PB14
#define VALVE_4_PIN             PB15

// INPUT
#define WATER_LOW_LEVEL_PIN     PA12
#define WATER_HIGH_LEVEL_PIN    PA11
