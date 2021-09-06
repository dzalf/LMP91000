/*
 FILENAME:	LMP91000.h
 AUTHOR:	Orlando S. Hoilett
 EMAIL:     ohoilett@purdue.edu
 
 
 Please see .cpp file for extended descriptions, instructions, and version updates
 
 
 Linnes Lab code, firmware, and software is released under the MIT License
 (http://opensource.org/licenses/MIT).
 
 The MIT License (MIT)
 
 Copyright (c) 2016 Linnes Lab, Purdue University, West Lafayette, IN, USA
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do
 so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 
 */

#ifndef LMP91000_H
#define LMP91000_H

#include "Arduino.h"
#include "Wire.h"

#warning "This is a work in progress. Custom clone of the LMP91000 library..." // dzalf 05/09/2021

#define DEBUG false

const float TEMP_INTERCEPT = 1562.2f;
const float TEMPSLOPE = -8.16f;
const float TEMP_FACTOR = 1E3;
const uint8_t LMP91000_I2C_ADDRESS = 0x48;

// Registers group
const uint8_t LMP91000_STATUS_REG = 0x00; /* Read only status register */
const uint8_t LMP91000_LOCK_REG = 0x01;   /* Protection Register */
const uint8_t LMP91000_TIACN_REG = 0x10;  /* TIA Control Register */
const uint8_t LMP91000_REFCN_REG = 0x11;  /* Reference Control Register*/
const uint8_t LMP91000_MODECN_REG = 0x12; /* Mode Control Register */

/** AFE's bitmask group
 */

// System status labels
const uint8_t LMP91000_READY = 0x01;
const uint8_t LMP91000_NOT_READY = 0x00;

// TIA gain values
const uint8_t LMP91000_TIA_GAIN_EXT = 0x00;  //default
const uint8_t LMP91000_TIA_GAIN_2P75K = 0x04;
const uint8_t LMP91000_TIA_GAIN_3P5K = 0x08;
const uint8_t LMP91000_TIA_GAIN_7K = 0x0C;
const uint8_t LMP91000_TIA_GAIN_14K = 0x10;
const uint8_t LMP91000_TIA_GAIN_35K = 0x14;
const uint8_t LMP91000_TIA_GAIN_120K = 0x18;
const uint8_t LMP91000_TIA_GAIN_350K = 0x1C;

const uint8_t LMP91000_RLOAD_10OHM = 0X00;
const uint8_t LMP91000_RLOAD_33OHM = 0X01;
const uint8_t LMP91000_RLOAD_50OHM = 0X02;
const uint8_t LMP91000_RLOAD_100OHM = 0X03;  //default

const uint8_t LMP91000_REF_SOURCE_INT = 0x00;  //default
const uint8_t LMP91000_REF_SOURCE_EXT = 0x80;

const uint8_t LMP91000_INT_Z_20PCT = 0x00;
const uint8_t LMP91000_INT_Z_50PCT = 0x20;  //default
const uint8_t LMP91000_INT_Z_67PCT = 0x40;
const uint8_t LMP91000_INT_Z_BYPASS = 0x60;

const uint8_t LMP91000_BIAS_SIGN_NEG = 0x00;  //default
const uint8_t LMP91000_BIAS_SIGN_POS = 0x10;

const uint8_t LMP91000_BIAS_0PCT = 0x00;  //default
const uint8_t LMP91000_BIAS_1PCT = 0x01;
const uint8_t LMP91000_BIAS_2PCT = 0x02;
const uint8_t LMP91000_BIAS_4PCT = 0x03;
const uint8_t LMP91000_BIAS_6PCT = 0x04;
const uint8_t LMP91000_BIAS_8PCT = 0x05;
const uint8_t LMP91000_BIAS_10PCT = 0x06;
const uint8_t LMP91000_BIAS_12PCT = 0x07;
const uint8_t LMP91000_BIAS_14PCT = 0x08;
const uint8_t LMP91000_BIAS_16PCT = 0x09;
const uint8_t LMP91000_BIAS_18PCT = 0x0A;
const uint8_t LMP91000_BIAS_20PCT = 0x0B;
const uint8_t LMP91000_BIAS_22PCT = 0x0C;
const uint8_t LMP91000_BIAS_24PCT = 0x0D;

const uint8_t LMP91000_FET_SHORT_DISABLED = 0x00;  //default
const uint8_t LMP91000_FET_SHORT_ENABLED = 0x80;
const uint8_t LMP91000_OP_MODE_DEEP_SLEEP = 0x00;  //default
const uint8_t LMP91000_OP_MODE_GALVANIC = 0x01;
const uint8_t LMP91000_OP_MODE_STANDBY = 0x02;
const uint8_t LMP91000_OP_MODE_AMPEROMETRIC = 0x03;
const uint8_t LMP91000_OP_MODE_TIA_OFF = 0x06;
const uint8_t LMP91000_OP_MODE_TIA_ON = 0x07;

const uint8_t LMP91000_WRITE_LOCK = 0x01;  //default
const uint8_t LMP91000_WRITE_UNLOCK = 0x00;

const uint8_t LMP91000_NOT_PRESENT = 0xA8;  // arbitrary library status code

const double TIA_GAIN[] = {2750, 3500, 7000, 14000, 35000, 120000, 350000};
const double TIA_BIAS[] = {0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14,
                           0.16, 0.18, 0.2, 0.22, 0.24};
const uint8_t NUM_TIA_BIAS = 14;
const double TIA_ZERO[] = {0.2, 0.5, 0.67};

class LMP91000 {
   public:
    typedef enum {
        DEEP_SLEEP = LMP91000_OP_MODE_DEEP_SLEEP,
        TWO_LEAD_GALVANIC = LMP91000_OP_MODE_GALVANIC,
        STANDBY = LMP91000_OP_MODE_STANDBY,
        THREE_LEAD_AMPEROMETRIC = LMP91000_OP_MODE_AMPEROMETRIC,
        TEMP_MEASURE_TIA_OFF = LMP91000_OP_MODE_TIA_OFF,
        TEMP_MEASURE_TIA_ON = LMP91000_OP_MODE_TIA_ON  // Sends the temperature to Vout while the Pstat voltage is at C2

    } OP_MODES;

    //CONSTRUCTORS
    LMP91000();  //tested

    LMP91000(double adc_vref, uint8_t adc_bits);

    

    // set ADC reference voltage
    void setADCReference(float volts);

    // set number of bits fro our MCU's ADC
    void setADCBits(uint8_t adc_bits);

    //sets and gets MENB pin for enabling and disabling I2C commands
    void setMENB(uint8_t pin);
    uint8_t getMENB();

    //sets and gets pin for reading output of temperature sensor
    void setTempSensor(uint8_t pin);
    uint8_t getTempSensor();

    //enables and disables LMP91000 for I2C commands
    //default state is not ready
    void enable();
    void disable();
    bool isReady();

    //locks and unlocks the transimpedance amplifier
    //and reference control registers for editing
    //default state is locked (read-only)
    void lock();
    void unlock();
    bool isLocked();

    //sets the gain of the transimpedance amplifier
    void setGain(uint8_t gain);
    double getGain();

    //sets the load for compensating voltage differences
    //between working and reference electrodes
    void setRLoad(uint8_t load);

    //sets the source for the bias voltage for the
    //electrochemical cell
    void setRefSource(uint8_t source);
    void setIntRefSource();
    void setExtRefSource();

    //sets reference voltage for transimpedance amplifier
    void setIntZ(uint8_t intZ);
    double getIntZ();

    //sets bias voltage for electrochemical cell
    void setBiasSign(uint8_t sign);
    void setNegBias();
    void setPosBias();
    void setBias(uint8_t bias);
    void setBias(uint8_t bias, signed char sign);

    //enable and disable FET for deep sleep mode
    void setFET(uint8_t selection);
    void disableFET();
    void enableFET();

    //set operating modes for the LMP91000
    void setMode(uint8_t mode);
    void sleep();
    void setTwoLead();
    void standby();
    void setThreeLead();
    void measureCell();
    void getTemp();
    void setTempReadMode(uint8_t);
    double getTemp(uint16_t raw_sensor_value);

    //reading the output of the LMP91000
    uint16_t getOutput(uint8_t raw_sensor_value);
    double getVoltage(uint16_t adcVal);
    double getCurrent(uint16_t adcVal);
    double getCurrent(uint16_t adcVal, double extGain);

    /*
    double getVoltage(uint8_t sensor, double adc_ref, uint8_t adc_bits) const;
    double getCurrent(uint8_t sensor, double adc_ref, uint8_t adc_bits) const;
    double getCurrent(uint8_t sensor, double adc_ref, uint8_t adc_bits,
                      double extGain) const;
    */

   private:

    uint8_t _MENB_Pin;  //IO pin for enabling and disabling I2C commands
    uint8_t _gain_index;
    uint8_t _zero_index;

    float _ADC_Vref;
    uint8_t _ADC_Bits;

    //reads and writes to LMP91000 via I2C
    void writeAFE(uint8_t reg, uint8_t data);
    uint8_t readAFE(uint8_t reg);
};

#endif
