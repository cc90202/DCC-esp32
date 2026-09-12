# DCC and RailCom Reference Sources

Last checked: 2026-04-24

These are the external references used for DCC electrical/protocol and RailCom hardware review work in this repository.

## NMRA DCC Standards

- NMRA Standards and Recommended Practices index, section S-9 Electrical / DCC:
  <https://www.nmra.org/index-nmra-standards-and-recommended-practices>
- NMRA S-9.1, Electrical Standards for Digital Command Control:
  <https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.1_electrical_standards_for_digital_command_control_2021.pdf>
- NMRA S-9.3.2, Communications Standard for Digital Command Control Basic Decoder Transmission:
  <https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/S-9.3.2_2012_12_10.pdf>

## RailCommunity RailCom Standards

- RailCommunity current standards index:
  <https://www.railcommunity.org/index.php?Itemid=61&id=49&option=com_content&view=article>
- RailCommunity RCN-217, RailCom DCC feedback protocol, current PDF found during lookup:
  <https://normen.railcommunity.de/RCN-217.pdf>

## Notes

- As of 2026-04-24, the RailCommunity RCN-217 PDF found online is issue 24.11.2025.
- For RailCom-specific electrical review, prefer RCN-217 for current RailCom cutout, transmitter, detector, and timing details.
- For baseline DCC packet/electrical compatibility, use NMRA S-9.1 and the NMRA S-9 standards index as the primary references.

## Parser conformance review (2026-09-07)

RCN-217 issue 24.11.2025, sections 2.5 and 3, is the reference for
strict 4-of-8 decoding, prohibition of ACK/NACK in CH1, and the prohibition
of datagrams after an initial CH2 ACK/NACK. Further control symbols and ACK
padding after data remain supported. A one-byte `0xF8` response is invalid;
the former breadboard workaround is no longer accepted as an ACK.

For POM writes, an ACK followed by NACK indicates an unsupported CV and must
not be reported as success. Host regressions cover this sequence from wire
bytes through POM response matching, invalid symbols, and independent CH1/CH2
processing. These checks do not establish full RailCom/POM conformance.

Implementation comparisons (hardware-specific behavior must not be copied
without ESP32-C6 validation):

- [OpenMRN STM32 capture](https://github.com/bakerstu/openmrn/blob/9f1199c9c60bba6bcc608d270275b53f9dc776d1/src/freertos_drivers/st/Stm32Railcom.hxx):
  channel-specific framing-error handling and receiver recovery between channels.
- [ZIMO dissector](https://github.com/ZIMO-Elektronik/DCC/blob/7ae8711767caa1616687ee002bd9fa1f4d0358bf/include/dcc/bidi/dissector.hpp):
  separate channel validation.

UART registers, FIFO reset, and cutout timing were not changed by this review.
Before changing capture behavior, record synchronized DCC/GPIO4/UART traces
with CH1 collisions and valid CH2 responses, earliest/latest channel timings,
both ACK codes, ACK/NACK, and POM data with ACK padding under WiFi load.
Verify that CH1 recovery finishes before CH2 starts and that UART error flags
are attributed to the correct channel. Hardware validation remains outstanding.
