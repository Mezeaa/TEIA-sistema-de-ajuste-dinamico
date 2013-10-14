/* 
 * File:   main.c
 * Author: Miguel Zea
 *
 * Created on 19 de junio de 2013, 11:10 AM
 */
#define COLLAPSE_INCLUDES
#define COLLAPSE_CONFIGURATION_BITS
#define COLLAPSE_ISRS

#ifdef COLLAPSE_INCLUDES
#include <xc.h>
#include "hardware_profile.h"
#include <libpic30.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <libq.h>
#include <dsp.h>
#include <timer.h>
#include "uart1_simple.h"
#include "i2c1_master_simple.h"
#include "lsm303dlhc.h"
#include "l3gd20.h"
#include "ds65hb_oc.h"
#include "freq_measure_ic.h"
#endif

#ifdef COLLAPSE_CONFIGURATION_BITS
// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)
#endif

#define ACC_X_MIN   (-1117)
#define ACC_X_MAX   (1017)
#define ACC_Y_MIN   (-998)
#define ACC_Y_MAX   (1021)
#define ACC_Z_MIN   (-1027)
#define ACC_Z_MAX   (1052)

#define MAG_X_MIN   (-511)
#define MAG_X_MAX   (261)
#define MAG_Y_MIN   (-511)
#define MAG_Y_MAX   (511)
#define MAG_Z_MIN   (-512)
#define MAG_Z_MAX   (223)

#define GYRO_X_ZOFF (558)
#define GYRO_Y_ZOFF (-306)
#define GYRO_Z_ZOFF (-288)

#define ACC_SENSITIVITY         (1000)
#define MAG_SENSITIVITY_XY      (1100)
#define MAG_SENSITIVITY_Z       (980)
#define GYRO_SENSITIVITY_BY4    (35)
#define GYRO_DPS_FACTOR         (4000)

#define GYRO_PITCH_BIAS (775)
#define GYRO_ROLL_BIAS  (395)

#define Q16_FRACTIONAL_RESOLUTION (65536)

#define Q16_DEGREES(x)  ((x) * 65536)

//#define CONTROL_LOOP_SAMPLE_FREQ (100U)
#define CONTROL_LOOP_SAMPLE_FREQ (100)

#define PITCH_PID_KP (0.2f)
#define PITCH_PID_KI (0.0f)
#define PITCH_PID_KD (0.1f)//(0.05f)

#define PITCH_SETPOINT_DEGREES (0)
#define PITCH_NORM_FACTOR (180)

#define YAW_PID_KP (0.2f)
#define YAW_PID_KI (0.0f)
#define YAW_PID_KD (0.05f)

#define YAW_SETPOINT_DEGREES (90)
#define YAW_NORM_FACTOR (360)

//#define TEST_SERVOS
#define PITCH_CONTROL
#define YAW_CONTROL
//#define SEND_DATA_TO_PC
//define PITCH_STEP_RESPONSE

void
device_configuration(void);

_Q16
_Q16atan2ByPI(_Q16 y, _Q16 x);

volatile int16_t sensor_data[9];
volatile bool samples_ready = false;

#ifdef TEST_SERVOS
_Q16 servo_pw = 0U;
uint16_t servo_pw_aux = 0U;
uint16_t freq_aux = 0U;
#endif

#ifdef PITCH_CONTROL
static tPID pitch_pid_controller;
__xdata static fractional pitch_control_gains[3];
__far __ydata static fractional pitch_control_buf[3];
#endif

#ifdef YAW_CONTROL
static tPID yaw_pid_controller;
__xdata static fractional yaw_control_gains[3];
__far __ydata static fractional yaw_control_buf[3];
#endif

#ifdef SEND_DATA_TO_PC
volatile bool sync_received = false;
volatile bool send_data_to_pc = false;
volatile char host_command;
#endif

#ifdef PITCH_STEP_RESPONSE
volatile bool sr_start = true;
volatile bool sr_stablish_new_setpoint = false;
volatile bool sr_restore_setpoint = false;
#endif

int
main(void)
{
    device_configuration();

#undef LSM303_CALIBRATE
#if ((defined LSM303_CALIBRATE) && (defined L3GD20_CALIBRATE))
    printf("Press \'k\' to start the calibration\n");
    while(uart1_read() != 'k');
    printf("Starting calibration...\n");
    printf("Calibrating accelerometer, slowly turn the device around the 3\n");
    printf("axes, then press \'a\' to generate the parameters.\n");
    lsm303_acc_calibrate();
    printf("Calibrating gyroscope, place the device on a motionless flat\n");
    printf("surface, then press \'g\' to generate the parameters.\n");
    l3gd20_calibrate();
    printf("Calibrating magnetometer, slowly turn the device around the 3\n");
    printf("axes, then press \'m\' to generate the parameters.\n");
    lsm303_mag_calibrate();
#endif

    // Normalized Q15.16 sensor readings
    _Q16 q_acc_x = 0;
    _Q16 q_acc_y = 0;
    _Q16 q_acc_z = 0;

    _Q16 q_gyro_x = 0;
    _Q16 q_gyro_y = 0;
    _Q16 q_gyro_z = 0;

    _Q16 q_mag_x = 0;
    _Q16 q_mag_y = 0;
    _Q16 q_mag_z = 0;

    // Gyro auxiliary registers for angle measurement
    _Q16 pq_gyro_y = 0;
    _Q16 pq_gyro_x = 0;

    // Mag auxiliary registers for angle measurement
    _Q16 q_mag_cx = 0;
    _Q16 q_mag_cy = 0;
    _Q16 q_mag_cz = 0;

    // Registers for angle measurement
    _Q16 q_acc_pitch = 0;
    _Q16 q_acc_roll = 0;
    _Q16 q_gyro_pitch = 0;
    _Q16 q_gyro_roll = 0;
    _Q16 pq_gyro_pitch = 0;
    _Q16 pq_gyro_roll = 0;
    _Q16 q_fusion_pitch = 0;
    _Q16 q_fusion_roll = 0;
    _Q16 q_mag_yaw = 0;
    _Q16 pq_mag_yaw = 0;

#ifdef PITCH_CONTROL
    fractional pitch_pid_gains[3];
    fractional pitch_control_setpoint;
    fractional pitch_control_measure;
    fractional pitch_control_output;
    int16_t pitch_control_outnorm;
    uint16_t pitch_control_servo;
#endif

#ifdef YAW_CONTROL
    fractional yaw_pid_gains[3];
    fractional yaw_control_setpoint;
    fractional yaw_control_measure;
    fractional yaw_control_output;
    int16_t yaw_control_outnorm;
    uint16_t yaw_control_servo;
    //bool yaw_first_measure = true;
#endif

#ifdef PITCH_CONTROL
    pitch_pid_gains[0] = Float2Fract(PITCH_PID_KP);
    pitch_pid_gains[1] = Float2Fract(PITCH_PID_KI);
    pitch_pid_gains[2] = Float2Fract(PITCH_PID_KD);

    pitch_control_setpoint =
            Float2Fract(((float)PITCH_SETPOINT_DEGREES) / PITCH_NORM_FACTOR);
    
    pitch_pid_controller.abcCoefficients = pitch_control_gains;
    pitch_pid_controller.controlHistory = pitch_control_buf;
    pitch_pid_controller.controlReference = pitch_control_setpoint;

    PIDInit(&pitch_pid_controller);
    PIDCoeffCalc(pitch_pid_gains, &pitch_pid_controller);
#endif

#ifdef YAW_CONTROL
    yaw_pid_gains[0] = Float2Fract(YAW_PID_KP);
    yaw_pid_gains[1] = Float2Fract(YAW_PID_KI);
    yaw_pid_gains[2] = Float2Fract(YAW_PID_KD);

    //yaw_control_setpoint = Float2Fract(-90.0f / YAW_NORM_FACTOR);
    yaw_control_setpoint =
            - Float2Fract(((float)YAW_SETPOINT_DEGREES) / YAW_NORM_FACTOR);

    yaw_pid_controller.abcCoefficients = yaw_control_gains;
    yaw_pid_controller.controlHistory = yaw_control_buf;
    yaw_pid_controller.controlReference = yaw_control_setpoint;

    PIDInit(&yaw_pid_controller);
    PIDCoeffCalc(yaw_pid_gains, &yaw_pid_controller);
#endif

#ifdef PITCH_STEP_RESPONSE
    fractional sr_pitch_setpoint = Float2Fract(30.0f / PITCH_NORM_FACTOR);
    fractional sr_prev_setpoint = pitch_control_setpoint;
#endif

    while(1)
    {
        if(samples_ready)
        {
            q_acc_x = (Q16_FRACTIONAL_RESOLUTION * (2 * (_Q16)sensor_data[0]
                    - (ACC_X_MAX + ACC_X_MIN))) / (ACC_X_MAX - ACC_X_MIN);
            q_acc_y = (Q16_FRACTIONAL_RESOLUTION * (2 * (_Q16)sensor_data[1]
                    - (ACC_Y_MAX + ACC_Y_MIN))) / (ACC_Y_MAX - ACC_Y_MIN);
            q_acc_z = (Q16_FRACTIONAL_RESOLUTION * (2 * (_Q16)sensor_data[2]
                    - (ACC_Z_MAX + ACC_Z_MIN))) / (ACC_Z_MAX - ACC_Z_MIN);

            q_gyro_x = (((Q16_FRACTIONAL_RESOLUTION / 16) *
                    ((_Q16)sensor_data[6] - GYRO_X_ZOFF)) /
                    (GYRO_DPS_FACTOR / 16)) * GYRO_SENSITIVITY_BY4;
            q_gyro_y = (((Q16_FRACTIONAL_RESOLUTION / 16) *
                    ((_Q16)sensor_data[7] - GYRO_Y_ZOFF)) /
                    (GYRO_DPS_FACTOR / 16)) * GYRO_SENSITIVITY_BY4;
            q_gyro_z = (((Q16_FRACTIONAL_RESOLUTION / 16) *
                    ((_Q16)sensor_data[8] - GYRO_Z_ZOFF)) /
                    (GYRO_DPS_FACTOR / 16)) * GYRO_SENSITIVITY_BY4;

            q_mag_x = (Q16_FRACTIONAL_RESOLUTION * (2 * (_Q16)sensor_data[3] -
                    (MAG_X_MAX + MAG_X_MIN))) / (2 * MAG_SENSITIVITY_XY);
            q_mag_y = (Q16_FRACTIONAL_RESOLUTION * (2 * (_Q16)sensor_data[4] -
                    (MAG_Y_MAX + MAG_Y_MIN))) / (2 * MAG_SENSITIVITY_XY);
            q_mag_z = (Q16_FRACTIONAL_RESOLUTION * (2 * (_Q16)sensor_data[5] -
                    (MAG_Z_MAX + MAG_Z_MIN))) / (2 * MAG_SENSITIVITY_Z);

            q_acc_pitch = -_Q16atan2ByPI(q_acc_x, q_acc_z) * 180;
            q_acc_roll = _Q16atan2ByPI(q_acc_y, q_acc_z) * 180;

            q_gyro_pitch = q_gyro_pitch + ((q_gyro_y + pq_gyro_y)
                    / (2 * CONTROL_LOOP_SAMPLE_FREQ));// + GYRO_PITCH_BIAS;
            q_gyro_roll = q_gyro_roll + ((q_gyro_x + pq_gyro_x)
                    / (2 * CONTROL_LOOP_SAMPLE_FREQ));// + GYRO_ROLL_BIAS;
            pq_gyro_y = q_gyro_y;
            pq_gyro_x = q_gyro_x;

            q_fusion_pitch = ((98 * (q_fusion_pitch + q_gyro_pitch
                    - pq_gyro_pitch)) / 100) + ((2 * q_acc_pitch) / 100);
            q_fusion_roll = ((98 * (q_fusion_roll + q_gyro_roll
                    - pq_gyro_roll)) / 100) + ((2 * q_acc_roll) / 100);
            pq_gyro_pitch = q_gyro_pitch;
            pq_gyro_roll = q_gyro_roll;

            q_mag_cx = q_mag_x * _Q16cosPI(q_fusion_pitch / 180)
                    + q_mag_z * _Q16sinPI(q_fusion_pitch / 180);
            q_mag_cy = q_mag_y * _Q16cosPI(q_fusion_roll / 180)
                    - q_mag_z * _Q16sinPI(q_fusion_roll / 180);
            q_mag_cz = q_mag_z * _Q16cosPI(q_fusion_pitch / 180) 
                    - q_mag_x * _Q16sinPI(q_fusion_pitch / 180);

            // North = 0.0
            // East  = 90.0
            // South = 180.0
            // West  = 270.0
            q_mag_yaw = (_Q16atan2ByPI(- q_mag_cy, q_mag_cx) * 180)
                    + (180 * Q16_FRACTIONAL_RESOLUTION);

            // Low Pass Filtered Yaw
            //pq_mag_yaw = (_Q16atan2ByPI(- q_mag_cy, q_mag_cx) * 180)
            //        + (180 * Q16_FRACTIONAL_RESOLUTION);
            //q_mag_yaw = ((99 * q_mag_yaw) / 100) + ((1 * pq_mag_yaw) / 100);

            //printf("Fusion q15 \tRoll: %.2f Pitch: %.2f\n",
            //        (double)_itofQ16(q_fusion_roll), (double)_itofQ16(q_fusion_pitch));
            //printf("Gyro Pitch \tDouble: %f Q16: %f\n", gyro_pitch,
            //        (double)_itofQ16(q_gyro_y));
            //printf("Mag \tYaw: %.2f\n", (double)_itofQ16(q_mag_yaw));
            //printf("Mag \tYaw: %d\n", (int16_t)(q_mag_yaw / Q16_FRACTIONAL_RESOLUTION));

#ifdef PITCH_CONTROL
            // Pitch Control
            pitch_control_measure = -(fractional)(q_fusion_pitch
                    / Q16_FRACTIONAL_RESOLUTION);

            pitch_pid_controller.measuredOutput = 
                Float2Fract(((float)pitch_control_measure) / PITCH_NORM_FACTOR);

            PID(&pitch_pid_controller);
            pitch_control_output = pitch_pid_controller.controlOutput;

            pitch_control_outnorm =
                    (int16_t)ds65hb_oc_get_pulse_width(DS65HB_OC1);
            pitch_control_outnorm += (int16_t)(Fract2Float(pitch_control_output)
                    * (PITCH_NORM_FACTOR) * DS65HB_DEADBAND);

            pitch_control_servo = (uint16_t)pitch_control_outnorm;
            
            if(pitch_control_servo > DS65HB_MAX_PULSE_WIDTH)
            {
                pitch_control_servo = DS65HB_MAX_PULSE_WIDTH;
            }
            else if(pitch_control_servo < DS65HB_MIN_PULSE_WIDTH)
            {
                pitch_control_servo = DS65HB_MIN_PULSE_WIDTH;
            }

            ds65hb_oc_set_pulse_width(DS65HB_OC1, pitch_control_servo);
#endif

#ifdef YAW_CONTROL
            // Yaw Control
            /*if(yaw_first_measure)
            {
                yaw_first_measure = false;
                yaw_control_setpoint = (fractional)(q_mag_yaw
                        / Q16_FRACTIONAL_RESOLUTION);
                yaw_pid_controller.controlReference =
                   Float2Fract(((float)yaw_control_setpoint) / YAW_NORM_FACTOR);
            }*/

            yaw_control_measure = -(fractional)(q_mag_yaw
                    / Q16_FRACTIONAL_RESOLUTION);

            yaw_pid_controller.measuredOutput =
                Float2Fract(((float)yaw_control_measure) / YAW_NORM_FACTOR);

            PID(&yaw_pid_controller);
            yaw_control_output = yaw_pid_controller.controlOutput;

            yaw_control_outnorm =
                    (int16_t)ds65hb_oc_get_pulse_width(DS65HB_OC2);
            yaw_control_outnorm += (int16_t)(Fract2Float(yaw_control_output)
                    * (YAW_NORM_FACTOR) * DS65HB_DEADBAND);

            yaw_control_servo = (uint16_t)yaw_control_outnorm;

            if(yaw_control_servo > DS65HB_MAX_PULSE_WIDTH)
            {
                yaw_control_servo = DS65HB_MAX_PULSE_WIDTH;
            }
            else if(yaw_control_servo < DS65HB_MIN_PULSE_WIDTH)
            {
                yaw_control_servo = DS65HB_MIN_PULSE_WIDTH;
            }

            ds65hb_oc_set_pulse_width(DS65HB_OC2, yaw_control_servo);
#endif

#ifdef SEND_DATA_TO_PC
            if(send_data_to_pc)
            {
                //printf("%ld %ld %ld %ld %ld %ld\n",
                //        q_acc_pitch, q_gyro_pitch, q_fusion_pitch,
                //        q_acc_roll, q_gyro_roll, q_fusion_roll);
                //printf("%ld %ld\n", pq_mag_yaw, q_mag_yaw);
                printf("%ld %ld %ld\n",
                        (_Q16)pitch_pid_controller.controlReference,
                        (_Q16)pitch_pid_controller.measuredOutput,
                        (_Q16)pitch_pid_controller.controlOutput);
                send_data_to_pc = false;
            }
#endif

#ifdef PITCH_STEP_RESPONSE
            if(sr_stablish_new_setpoint)
            {
                pitch_pid_controller.controlReference = sr_pitch_setpoint;
                sr_stablish_new_setpoint = false;
            }
            if(sr_restore_setpoint)
            {
                pitch_pid_controller.controlReference = sr_prev_setpoint;
                sr_restore_setpoint = false;
            }
#endif

            samples_ready = false;
        }

#ifdef TEST_SERVOS
        if(uart1_data_available())
        {
            if('u' == uart1_read())
            {
                if(servo_pw_aux < DS65HB_MAX_PULSE_WIDTH)
                {
                    servo_pw_aux += DS65HB_DEADBAND;
                    ds65hb_oc_set_pulse_width(DS65HB_OC1, servo_pw_aux);
                }
                printf("Servo DC: %d\n", servo_pw_aux);
                printf("Pitch: %.0f\n", (double)_itofQ16(q_fusion_pitch));
            }
            else if('d' == uart1_read())
            {
                if(servo_pw_aux > DS65HB_MIN_PULSE_WIDTH)
                {
                    servo_pw_aux -= DS65HB_DEADBAND;
                    ds65hb_oc_set_pulse_width(DS65HB_OC1, servo_pw_aux);
                }
                printf("Servo DC: %d\n", servo_pw_aux);
                printf("Pitch: %.0f\n", (double)_itofQ16(q_fusion_pitch));
            }
            else if('f' == uart1_read())
            {
                freq_aux = freq_measure_ic_get_freq(CHANNEL_IC1);
                printf("Freq measurement: %d\n", freq_aux);
            }
        }
#endif

    }

    return 0;
}

void
device_configuration(void)
{
    uint16_t match_value;

    // PLL settings
    CLKDIVbits.PLLPRE = 0U;     // PLLPRE<4:0> = 0  ->  N1 = 2
                                // 7.37MHz / 2 = 3.685MHz
    PLLFBD = 41U;               // PLLDIV<8:0> = 41  ->  M = 43
                                // 3.685MHz * 43 = 158.455MHz
    CLKDIVbits.PLLPOST = 0U;    // PLLPOST<1:0> = 0  ->  N2 = 2
                                // 158.455MHz / 2 = 79.2275MHz
    while(!OSCCONbits.LOCK)     // Wait for PLL to lock
    {

    }

    // Port settings
    AD1PCFGL = 0xFFFF;          // Set All pins as digital
    TRISA = 0U;                 // Initialize PORTA as output
    TRISB = 0x0028;             // Initialize RB3 (RP3) and RB5 (RP5)as inputs,
                                // remaining pins as outputs
    LATA = 0U;                  // Set PORTA to zero
    LATB = 0U;                  // Set PORTB to zero

    // UART1 settings
    RPINR18bits.U1RXR = 0x3;    // UART1 RX assigned to RP3 (RB3)
    RPOR1bits.RP2R = 0x3;       // UART1 TX assigned to RP2 (RB2)

#ifdef SEND_DATA_TO_PC
    uart1_open(460800UL, HIGH_SPEED_MODE);
    uart1_enable_rx_interrupt();
#else
#ifdef UART1_FAST_MODE
    uart1_open(115200UL, LOW_SPEED_MODE);
#else
    uart1_open(9600UL, LOW_SPEED_MODE);
#endif
#endif
    __delay_ms(100);

    // I2C1 settings
#ifdef I2C1_FAST_MODE
    i2c1_open(400000UL);
#else
    i2c1_open(100000UL);
#endif
    __delay_ms(100);

    // Inertial measurement unit settings
    lsm303_acc_init_default();
    lsm303_mag_init_default();
    l3gd20_init_default();

    // Control loop sample clock timer settings
    ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_ON);
    WriteTimer1(0);
    match_value = (FCY / 8) / CONTROL_LOOP_SAMPLE_FREQ;
    OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP &
               T1_PS_1_8 & T1_SYNC_EXT_OFF &
               T1_SOURCE_INT, match_value);

    // Servos settings
    RPOR3bits.RP6R = 0x12;      // OC1 assigned to RP6 (RB6)
    RPOR3bits.RP7R = 0x13;      // OC2 assigned to RP7 (RB7)
    ds65hb_oc_init(DS65HB_OC1 & DS65HB_OC2);

#ifdef TEST_SERVOS
    servo_pw_aux = ds65hb_oc_get_pulse_width(DS65HB_OC1);
#endif

    // Freq measure settings
    RPINR7bits.IC1R = 0x05;     // IC1 assigned to RP5 (RB5)
    freq_measure_ic_init(CHANNEL_IC1, RANGE_MIDDLE_FREQ);

}


#ifdef COLLAPSE_ISRS
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0U;
    LATBbits.LATB15 = ~LATBbits.LATB15;
    sensor_data[0] = lsm303_acc_get_sample(X_AXIS);
    sensor_data[1] = lsm303_acc_get_sample(Y_AXIS);
    sensor_data[2] = lsm303_acc_get_sample(Z_AXIS);
    sensor_data[3] = lsm303_mag_get_sample(X_AXIS);
    sensor_data[4] = lsm303_mag_get_sample(Y_AXIS);
    sensor_data[5] = lsm303_mag_get_sample(Z_AXIS);
    sensor_data[6] = l3gd20_get_sample(X_AXIS);
    sensor_data[7] = l3gd20_get_sample(Y_AXIS);
    sensor_data[8] = l3gd20_get_sample(Z_AXIS);
    samples_ready = true;
}

#ifdef SEND_DATA_TO_PC
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0U;
    host_command = uart1_read();

    switch(host_command)
    {
        case 'k':
            sync_received = true;
            break;
        case 'c':
            if(sync_received)
            {
                send_data_to_pc = true;

#ifdef PITCH_STEP_RESPONSE
                if(sr_start)
                {
                    sr_stablish_new_setpoint = true;
                    sr_start = false;
                }
#endif

            }
            break;
        case 's':
            if(sync_received)
            {
                sync_received = false;
                send_data_to_pc = false;

#ifdef PITCH_STEP_RESPONSE
                sr_start = true;
                sr_restore_setpoint = true;
#endif
            }
            break;
        default:
            break;
    }
}
#endif
#endif

_Q16
_Q16atan2ByPI(_Q16 y, _Q16 x)
{
    _Q16 result;

    if(x > 0)
    {
        result = _Q16atanYByXByPI(x, y);
    }
    else if((y >= 0) && (x < 0))
    {
        result = _Q16atanYByXByPI(x, y) + 65536;
    }
    else if((y < 0) && (x < 0))
    {
        result = _Q16atanYByXByPI(x, y) - 65536;
    }
    else if((y > 0) && (0 == x))
    {
        result = 32768;
    }
    else if((y < 0) && (0 == x))
    {
        result = -32768;
    }
    else
    {
        result = 0xFFFFFFFF;
    }

    return result;
}