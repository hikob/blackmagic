/*
 * jaguar.c
 *
 *  Created on: Mar 10, 2014
 *      Author: burindes
 */

#include <stdint.h>
#include <math.h>

#include "em_device.h"
#include "em_adc.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_usart.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"

#include "jaguar.h"

#undef DEBUG
#define DEBUG(...) uart_printf(__VA_ARGS__)

static volatile I2C_TransferReturn_TypeDef I2C_Status;

/*
 * AD5142A
 */
enum
{
  AD5142A_CMD_NOP                    = 0x00,
  AD5142A_CMD_WRITE_SERIAL_TO_RDAC   = 0x10,
  AD5142A_CMD_WRITE_SERIAL_TO_INPUT  = 0x20,
  AD5142A_CMD_READ_BACK              = 0x30,
  AD5142A_CMD_LINEAR_RDAC_INC_DEC    = 0x40,
  AD5142A_CMD_6DB_RDAC_INC_DEC	     = 0x50,
  AD5142A_CMD_COPY_INPUT_TO_RDAC     = 0x60,
  AD5142A_CMD_XCHG_RDAC_AND_EEPROM   = 0x70,
  AD5142A_CMD_WRITE_SERIAL_TO_EEPROM = 0x80,
  AD5142A_CMD_SCALE                  = 0x90,
  AD5142A_CMD_SOFT_RESET             = 0xB0,
  AD5142A_CMD_SOFT_SHUTDOWN          = 0xC0,
  AD5142A_CMD_COPY_SERAIL_TO_CTRL    = 0xD0,
};

enum
{
  AD5142A_ADDR_ALL   = 0x08,
  AD5142A_ADDR_RDAC1 = 0x00,
  AD5142A_ADDR_RDAC2 = 0x01,
};

enum
{
  AD5142A_READ_INPUT  = 0x00,
  AD5142A_READ_EEPROM = 0x01,
  AD5142A_READ_CTRL   = 0x02,
  AD5142A_READ_RDAC   = 0x03,
};

enum
{
  AD5142A_RDAC1= 0,
  AD5142A_RDAC2= 1,
};    

#define AD5142A_I2C_ADDR 0x5E

/*
 * RDAC1 => VDD_TARGET
 * RDAC2 => VTARGET
 */

static void ad5142a_write_RDAC(uint8_t rdac, uint8_t val)
{

  I2C_TransferSeq_TypeDef seq;
  uint8_t data[2];

  // Start Reading Current
  seq.addr = AD5142A_I2C_ADDR;
  seq.flags = I2C_FLAG_WRITE;
  
  /* Issue Command */
  seq.buf[0].data = data;
  switch(rdac){
  case 0:
    data[0] = AD5142A_CMD_WRITE_SERIAL_TO_RDAC | AD5142A_ADDR_RDAC1;  
    data[1] = val;
    break;
  case 1:
    data[0] = AD5142A_CMD_WRITE_SERIAL_TO_RDAC | AD5142A_ADDR_RDAC2;  
    data[1] = val;
    break;
  }
  seq.buf[0].len = 2;

  I2C_Status = I2C_TransferInit(I2C0, &seq);
  while (I2C_Status == i2cTransferInProgress)
    {
      asm volatile ("nop");
    }
  
}

static uint8_t ad5142a_read_RDAC(uint8_t rdac)
{

  uint8_t val = 0;
  I2C_TransferSeq_TypeDef seq;
  uint8_t data[2];

  // Start Reading Current
  seq.addr = AD5142A_I2C_ADDR;
  seq.flags = I2C_FLAG_WRITE_READ;
  
  /* Issue Command */
  seq.buf[0].data = data;
  switch(rdac){
  case 0:
    data[0] = AD5142A_CMD_READ_BACK | AD5142A_ADDR_RDAC1;  
    data[1] = AD5142A_READ_RDAC;
    break;
  case 1:
    data[0] = AD5142A_CMD_READ_BACK | AD5142A_ADDR_RDAC2;  
    data[1] = AD5142A_READ_RDAC;
    break;
  }
  seq.buf[0].len = 2;

  seq.buf[1].data = &val;
  seq.buf[1].len = 1;

  I2C_Status = I2C_TransferInit(I2C0, &seq);
  while (I2C_Status == i2cTransferInProgress)
    {
      asm volatile ("nop");
    }

  DEBUG("AD5142A RDAC%d reg: 0x%02x\n", rdac, val); 
  
  return val;
}

/*
 * INA226
 */

enum
{
    INA226_REG_CONFIGURATION = 0x00,
    INA226_REG_SHUNT_VOLTAGE = 0x01,
    INA226_REG_BUS_VOLTAGE = 0x02,
    INA226_REG_POWER = 0x03,
    INA226_REG_CURRENT = 0x04,
    INA226_REG_CALIBRATION = 0x05,
    INA226_REG_MASK_ENABLE = 0x06,
    INA226_REG_ALERT_LIMIT = 0x07,
    INA226_REG_DIE_ID = 0xFF,
};

enum
{
    INA226_CONFIGURATION__RST = 0x8000,

    INA226_CONFIGURATION__AVG_MASK = 0x0E00,
    INA226_CONFIGURATION__AVG_SHIFT = 9,

    INA226_CONFIGURATION__BUS_CONV_TIME_MASK = 0x01C0,
    INA226_CONFIGURATION__BUS_CONV_TIME_SHIFT = 6,

    INA226_CONFIGURATION__SHUNT_CONV_TIME_MASK = 0x0038,
    INA226_CONFIGURATION__SHUNT_CONV_TIME_SHIFT = 3,

    INA226_CONFIGURATION__MODE_MASK = 0x0007,
    INA226_CONFIGURATION__MODE_SHIFT = 0,
};

static void ina226_reg_write(uint8_t reg, uint16_t value);
static void state_machine(int event);

enum
{
    EVENT_ALERT = 1, EVENT_I2C_DONE = 2,
};

enum
{
    STATE_IDLE = 0,
    STATE_READING_CURRENT = 1,
    STATE_READING_SHUNT = 2,
    STATE_READING_VOLTAGE = 3,
    STATE_CLEARING_MASK = 4,
};

static struct
{
    jaguar_power_handler_t handler;
    float current_lsb;

    int state;

    /** Data for I2C reads */
    I2C_TransferSeq_TypeDef seq;
    uint8_t regid[1];
    uint8_t data[2];

    float current, voltage, shunt;
    uint32_t timestamp;

    const char *target_voltage, *target_vdd_voltage, *target_5v_voltage;
} power;

void jaguar_init()
{
    DEBUG("Jaguar Init\n");

    // Enable Target Power
    GPIO_PinModeSet(TARGET_EN_PORT, TARGET_EN_PIN, gpioModePushPull, 1);
    GPIO_PinModeSet(TARGET_5V_PORT, TARGET_5V_PIN, gpioModePushPull, 1);

    /* ALERT: Pull-up Input */ 
    GPIO_PinModeSet(POWER_ALERT_PORT, POWER_ALERT_PIN, gpioModeInputPull, 1);

    // Enable I2C for Power Consumption measure
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
    CMU_ClockEnable(cmuClock_I2C0, true);

    /* Use location 3: SDA - PA0, SCL - PA1 */
    GPIO_PinModeSet(POWER_I2C_SDA_PORT, POWER_I2C_SDA_PIN, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(POWER_I2C_SCL_PORT, POWER_I2C_SCL_PIN, gpioModeWiredAnd, 1);

    /* Enable pins at location 0 */
    I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN
            | (0 << _I2C_ROUTE_LOCATION_SHIFT);

    I2C_Init(I2C0, &i2cInit);

    /* Clear and enable interrupt from I2C module */
    NVIC_ClearPendingIRQ(I2C0_IRQn);
    NVIC_EnableIRQ(I2C0_IRQn);

    power.handler = NULL;

    // Configure and calibrate
    uint16_t config_reg = /* BusConv Time */(0 << 6)
            | /* ShuntConv Time */(0 << 3) | /* Average */(4 << 9) | /* Mode*/7;
    ina226_reg_write(INA226_REG_CONFIGURATION, config_reg);

    // Compute the current LSB as max_expected_current/2**15
    power.current_lsb = 0.16 / (1 << 15);
    
    float r_shunt = 2;
    
    // Compute calibration register as 0.00512 / (current_lsb * r_shunt)
    float calib = 0.00512 / (power.current_lsb * r_shunt);
    
    // Register is a 16bit unsigned integer, thus convert and round above
    uint16_t calib_reg = (uint16_t) floorf(calib);
    
    // Re-compute and store real current LSB
    power.current_lsb = 0.00512 / (r_shunt * calib_reg);
    
    // Write calibration
    ina226_reg_write(INA226_REG_CALIBRATION, calib_reg);
    
    // Enable ALERT pin
    ina226_reg_write(INA226_REG_MASK_ENABLE, 1 << 10);
    
    // Reset State
    power.state = 0;

    /* Configure PB9 interrupt on falling edge */
    GPIO_IntConfig(POWER_ALERT_PORT, POWER_ALERT_PIN, false, true, true);

    /* Configure the ADC VRESET input */
    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
    ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

    //CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_ADC0, true);
    
    /* Initialize timebases */
    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(400000,0);
    ADC_Init(ADC0, &init);

    /* Set input. Reference must be 1.25V */
    /* 
     * Default is:
     * Ref: 1V25
     * Res: 12bits
     */
    sInit.input = adcSingleInpCh5;
    ADC_InitSingle(ADC0, &sInit);
    
    /* Setup interrupt generation on completed conversion. */
    ADC_IntEnable(ADC0, ADC_IF_SINGLE);
    NVIC_EnableIRQ(ADC0_IRQn);
   
}

void jaguar_target_measure_VRESET(void){
    
    //jaguar_target_select_voltage(JAGUAR_VOLTAGE_OFF);
    /* Start one ADC sample */
    ADC_IntClear(ADC0, ADC_IF_SINGLE);
    ADC_Start(ADC0, adcStartSingle);

    /* Wait in EM1 for ADC to complete */
    while (!(ADC_IntGet(ADC0) & ADC_IF_SINGLE));
    //EMU_EnterEM1();

    /* Read sensor value */
    uint32_t toto = ADC_DataSingleGet(ADC0);

    DEBUG("ADC read: 0x%08x\n", toto);
}

void jaguar_target_select_voltage(enum JaguarVoltage voltage)
{
  uint8_t val = 0;

  // Disable regulator
    GPIO_PinOutClear(TARGET_EN_PORT, TARGET_EN_PIN);

    if (voltage == JAGUAR_VOLTAGE_OFF)
    {
        power.target_voltage = "0";
        return;
    }
    
    switch (voltage)
    {
    case JAGUAR_VOLTAGE_2p0:
      val = 245;
      power.target_voltage = "2";
      break;
    case JAGUAR_VOLTAGE_2p5:
      val = 159;
      power.target_voltage = "2.5";
      break;
    case JAGUAR_VOLTAGE_3p3:
      val = 62;
      power.target_voltage = "3.3";
      break;
    case JAGUAR_VOLTAGE_3p6:
      val = 39;
      power.target_voltage = "3.6";
      break;
    case JAGUAR_VOLTAGE_4p1:
      val = 10;
      power.target_voltage = "4.1";
      break;
    case JAGUAR_VOLTAGE_4p2:
      val = 5;
      power.target_voltage = "4.2";
      break;
    case JAGUAR_VOLTAGE_OFF:
      break;
    }

    ad5142a_write_RDAC(AD5142A_RDAC2, val);
    
    // Enable regulator
    GPIO_PinOutSet(TARGET_EN_PORT, TARGET_EN_PIN);
}
void jaguar_target_select_vdd_voltage(enum JaguarVddVoltage voltage)
{

  uint8_t val = 0;

  power.target_vdd_voltage = "0";

  switch (voltage)
  {
  case JAGUAR_VDD_VOLTAGE_2p0:
    val = 178;
    power.target_vdd_voltage = "2";
    break;
  case JAGUAR_VDD_VOLTAGE_2p5:
    val = 92;
    power.target_vdd_voltage = "2.5";
    break;
  case JAGUAR_VDD_VOLTAGE_2p8:
    val = 47;
    power.target_vdd_voltage = "2.8";
    break;
  case JAGUAR_VDD_VOLTAGE_3p3:
    val= 0;
    power.target_vdd_voltage = "3.3";
    break;
  }

  ad5142a_write_RDAC(AD5142A_RDAC1, val);
}

void jaguar_target_5V(int enable)
{
    if (enable)
    {
        DEBUG("Enabling 5V\n");
        GPIO_PinOutSet(TARGET_5V_PORT, TARGET_5V_PIN);
        power.target_5v_voltage = "5";
    }
    else
    {
        DEBUG("Disabling 5V\n");
        GPIO_PinOutClear(TARGET_5V_PORT, TARGET_5V_PIN);
        power.target_5v_voltage = "0";
    }
}

const char* jaguar_target_voltage()
{
    return power.target_voltage;
}
const char* jaguar_target_vdd_voltage()
{
    return power.target_vdd_voltage;
}
const char* jaguar_target_5V_voltage()
{
    return power.target_5v_voltage;
}

void jaguar_power_sensing(jaguar_power_handler_t power_handler)
{
    DEBUG("Starting Power Sensing %p\n", power_handler);
    power.handler = power_handler;
}
void jaguar_power_alert(uint32_t rtc_timestamp)
{

  power.timestamp = rtc_timestamp;

    // Process Alert
    state_machine(EVENT_ALERT);
}

void I2C0_IRQHandler(void)
{
    /* Just run the I2C_Transfer function that checks interrupts flags and returns */
    /* the appropriate status */
    I2C_Status = I2C_Transfer(I2C0);

    if (I2C_Status == 0)
    {
        state_machine(EVENT_I2C_DONE);
    }
}

#define INA226_I2C_ADDR 0x80

static void ina226_reg_write(uint8_t reg, uint16_t value)
{
    I2C_TransferSeq_TypeDef seq;
    uint8_t data[3];

    seq.addr = INA226_I2C_ADDR;
    seq.flags = I2C_FLAG_WRITE;
    /* Select register to be written */
    data[0] = reg;
    seq.buf[0].data = data;
    data[1] = (uint8_t) (value >> 8);
    data[2] = (uint8_t) value;
    seq.buf[0].len = 3;

    /* Do a polled transfer */
    I2C_Status = I2C_TransferInit(I2C0, &seq);
    while (I2C_Status == i2cTransferInProgress)
    {
        asm volatile ("nop");
    }
}

/** READ STATE MACHINE **/
static void state_machine(int event)
{

  //  DEBUG("STATE: %d, EVENT: %d\n", power.state, event); 
  switch (power.state)
    {
    case STATE_IDLE:
        switch (event)
        {
        case EVENT_ALERT:
            // Read Current
        {
            // Start Reading Current
            power.seq.addr = INA226_I2C_ADDR;
            power.seq.flags = I2C_FLAG_WRITE_READ;

            /* Select register to be read */
            power.regid[0] = INA226_REG_CURRENT;
            power.seq.buf[0].data = power.regid;
            power.seq.buf[0].len = 1;

            power.seq.buf[1].data = power.data;
            power.seq.buf[1].len = 2;

            I2C_TransferInit(I2C0, &power.seq);
            power.state = STATE_READING_CURRENT;
        }

            break;
        case EVENT_I2C_DONE:
            break;
        }
        break;
    case STATE_READING_CURRENT:
        switch (event)
        {
        case EVENT_ALERT:
            break;
        case EVENT_I2C_DONE:
            // Store Current converted to Amperes
        {
            int16_t current_reg = (((uint16_t) (power.data[0])) << 8)
                    | power.data[1];
            power.current = (float) current_reg * power.current_lsb;
        }
            // Start Reading Shunt
            {
                power.seq.addr = INA226_I2C_ADDR;
                power.seq.flags = I2C_FLAG_WRITE_READ;

                /* Select register to be read */
                power.regid[0] = INA226_REG_SHUNT_VOLTAGE;
                power.seq.buf[0].data = power.regid;
                power.seq.buf[0].len = 1;

                power.seq.buf[1].data = power.data;
                power.seq.buf[1].len = 2;

                I2C_TransferInit(I2C0, &power.seq);
                power.state = STATE_READING_SHUNT;
            }
            break;
        }
        break;
    case STATE_READING_SHUNT:
        switch (event)
        {
        case EVENT_ALERT:
            break;
        case EVENT_I2C_DONE:
            // Store Shunt Voltage
        {
            int16_t shunt_reg = (((uint16_t) (power.data[0])) << 8)
                    | power.data[1];
            power.shunt = (float) shunt_reg * 2.5e-6;
        }
            // Start Reading Voltave
            {
                power.seq.addr = INA226_I2C_ADDR;
                power.seq.flags = I2C_FLAG_WRITE_READ;

                /* Select register to be read */
                power.regid[0] = INA226_REG_BUS_VOLTAGE;
                power.seq.buf[0].data = power.regid;
                power.seq.buf[0].len = 1;

                power.seq.buf[1].data = power.data;
                power.seq.buf[1].len = 2;

                I2C_TransferInit(I2C0, &power.seq);
                power.state = STATE_READING_VOLTAGE;
            }
            break;
        }
        break;
    case STATE_READING_VOLTAGE:
        switch (event)
        {
        case EVENT_ALERT:
            break;
        case EVENT_I2C_DONE:
            // Store Voltage
        {
            uint16_t voltage_reg = (((uint16_t) (power.data[0])) << 8)
                    | power.data[1];
            power.voltage = (float) voltage_reg * 1.25e-3;
        }
            // Clear Mask
            {
                power.seq.addr = INA226_I2C_ADDR;
                power.seq.flags = I2C_FLAG_WRITE_READ;

                /* Select register to be read */
                power.regid[0] = INA226_REG_MASK_ENABLE;
                power.seq.buf[0].data = power.regid;
                power.seq.buf[0].len = 1;

                power.seq.buf[1].data = power.data;
                power.seq.buf[1].len = 2;

                I2C_TransferInit(I2C0, &power.seq);
                power.state = STATE_CLEARING_MASK;
            }
            break;
        }
        break;
    case STATE_CLEARING_MASK:
        switch (event)
        {
        case EVENT_ALERT:
            break;
        case EVENT_I2C_DONE:

            power.state = STATE_IDLE;

            if (power.handler)
            {
                power.handler(power.timestamp, power.voltage, power.shunt,
                        power.current);
            }
            break;
        }
        break;
    }
}

