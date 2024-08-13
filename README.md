EPICS support for Measurement Computing Corporation daq hats for Raspberry Pi
=============================================================================

vendor of hats:
  * Measurement Computing Corporation
    <https://www.mccdaq.com/>
  * now Digilent
    <https://digilent.com/>
  * now Emerson

possible types:
  * MCC118:
    8-ch 12-bit 100kS/s analog input, single ended -10V…+10V
    common clock input/output, common trigger input
  * MCC128:
    8-ch 16-bit 100kS/s analog input, single ended or
    4-ch 16-bit 100kS/s analog input, differential
    -10V…+10V, -5V…+5V, -2V…+2V, -1V…+1V
    common clock input/output, common trigger input
  * MCC134:
    4-ch 24-bit thermocouple input
  * MCC152:
    2-ch 12-bit analog output 0V…5V
    8-ch bit-configureable digital I/O
  * MCC172:
    2-ch 24-bit 51.2kS/s analog input, differential -5V…+5V
    common trigger input

some types might blink the status LED (118,128,152,172)

required Github library <https://github.com/mccdaq/daqhats>

required EPICS <https://epics-controls.org/>
  * base <https://github.com/epics-base/epics-base>
  * asyn <https://github.com/epics-modules/asyn>
