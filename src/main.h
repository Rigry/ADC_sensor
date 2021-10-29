#pragma once

#include "pin.h"
#include "pwm_.h"
#include "adc.h"
#include "flash.h"
#include "timers.h"
#include "encoder.h"
#include "modbus_slave.h"

// #if defined(USE_MOCK_PWM)
// using PWM_t = mock::PWM;
// #else
// using PWM_t = ::PWM;
// #endif

constexpr auto _3V3 {3.3};
constexpr auto conversion_on_channel {16};

struct ADC_{
   ADC_average& control = ADC_average::make<mcu::Periph::ADC1>(conversion_on_channel);
   ADC_channel& voltage = control.add_channel<mcu::PA4>();
};


struct State {
   bool enable_PWM_auto   :1; // enable auto PWM 
   bool enable_PWM_manual :1; // enable manual PWM
   bool enable_encoder    :1; // enable encoder
   uint16_t res           :13;
}__attribute__((packed));

struct In_regs {
   
   UART::Settings uart_set;         // 0
   uint16_t modbus_address;         // 1
   uint16_t password;               // 2
   uint16_t factory_number;         // 3
   uint16_t voltage_pwm;            // 4
   State state;                     // 5

}__attribute__((packed));

struct Out_regs {

   uint16_t device_code;            // 0
   uint16_t factory_number;         // 1
   UART::Settings uart_set;         // 2
   uint16_t modbus_address;         // 3
   uint16_t voltage_pwm;            // 4
   uint16_t voltage;                // 5
   int16_t encoder;                 // 6

}__attribute__((packed));

struct Flash_data {
   uint16_t factory_number = 0;
   UART::Settings uart_set = {
      .parity_enable  = false,
      .parity         = USART::Parity::even,
      .data_bits      = USART::DataBits::_8,
      .stop_bits      = USART::StopBits::_1,
      .baudrate       = USART::Baudrate::BR9600,
      .res            = 0
   };
   uint8_t  modbus_address = 7;
   uint16_t model_number   = 0;
};

#define ADR(reg) GET_ADR(In_regs, reg)

template<class Flash, class Modbus>
class Task
{
   ADC_& adc;
   PWM& pwm;
   Flash& flash;
   Modbus& modbus;
   Encoder& encoder;
   Timer timer{1000};
   int16_t step_pwm {100};

public:
   Task(ADC_& adc, PWM& pwm, Flash& flash, Modbus& modbus, Encoder& encoder) 
      : adc {adc}
      , pwm {pwm}
      , flash {flash}
      , modbus {modbus}
      , encoder {encoder}
   {
      adc.control.start();
   }

   void operator()() {

      modbus.outRegs.voltage_pwm = pwm.duty_cycle * _3V3 / 1000;

      modbus.outRegs.voltage = adc.voltage;

      modbus.outRegs.encoder = encoder;

      modbus([&](uint16_t registrAddress) {
            static bool unblock = false;
         switch (registrAddress) {
            case ADR(uart_set):
               flash.uart_set
                  = modbus.outRegs.uart_set
                  = modbus.inRegs.uart_set;
            break;
            case ADR(modbus_address):
               flash.modbus_address 
                  = modbus.outRegs.modbus_address
                  = modbus.inRegs.modbus_address;
            break;
            case ADR(password):
               unblock = modbus.inRegs.password == 208;
            break;
            case ADR(factory_number):
               if (unblock) {
                  unblock = false;
                  flash.factory_number 
                     = modbus.outRegs.factory_number
                     = modbus.inRegs.factory_number;
               }
               unblock = true;
            break;
            case ADR (state):
               if (modbus.inRegs.state.enable_PWM_auto 
               or  modbus.inRegs.state.enable_PWM_manual){
                  pwm.out_enable(); 
               } else {
                  pwm.out_disable();
                  pwm.duty_cycle = 0;
               }
            break;
         } // switch
      });

      if (modbus.inRegs.state.enable_PWM_auto) {
         pwm.duty_cycle += timer.event() ? step_pwm : 0;
         step_pwm = (pwm.duty_cycle >= 900 or pwm.duty_cycle <= 100) ? -step_pwm : step_pwm;
      }

      if (modbus.inRegs.state.enable_PWM_manual) {
         pwm.duty_cycle = 1000 * modbus.inRegs.voltage_pwm / _3V3;
      }

   }//operator
};