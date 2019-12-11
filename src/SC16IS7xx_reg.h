#ifndef __SC16IS7XX_REG_H
#define __SC16IS7XX_REG_H

// General Registers
#define SC16IS7XX_REG_RHR        (0x00)
#define SC16IS7XX_REG_THR        (0x00)
#define SC16IS7XX_REG_IER        (0x01)
#define SC16IS7XX_REG_FCR        (0x02)
#define SC16IS7XX_REG_IIR        (0x02)
#define SC16IS7XX_REG_LCR        (0x03)
#define SC16IS7XX_REG_MCR        (0x04)
#define SC16IS7XX_REG_LSR        (0x05)
#define SC16IS7XX_REG_MSR        (0x06)
#define SC16IS7XX_REG_SPR        (0x07)
#define SC16IS7XX_REG_TCR        (0X06)
#define SC16IS7XX_REG_TLR        (0X07)
#define SC16IS7XX_REG_TXLVL      (0X08)
#define SC16IS7XX_REG_RXLVL      (0X09)
#define SC16IS7XX_REG_IODIR      (0X0A)
#define SC16IS7XX_REG_IOSTATE    (0X0B)
#define SC16IS7XX_REG_IOINTENA   (0X0C)
#define SC16IS7XX_REG_IOCONTROL  (0X0E)
#define SC16IS7XX_REG_EFCR       (0X0F)

// Special Registers
#define SC16IS7XX_REG_DLL        (0x00)
#define SC16IS7XX_REG_DLH        (0X01)

//Enhanced Registers
#define SC16IS7XX_REG_EFR        (0X02)
#define SC16IS7XX_REG_XON1       (0X04)
#define SC16IS7XX_REG_XON2       (0X05)
#define SC16IS7XX_REG_XOFF1      (0X06)
#define SC16IS7XX_REG_XOFF2      (0X07)

//
#define SC16IS7XX_INT_CTS        (0X80)
#define SC16IS7XX_INT_RTS        (0X40)
#define SC16IS7XX_INT_XOFF       (0X20)
#define SC16IS7XX_INT_SLEEP      (0X10)
#define SC16IS7XX_INT_MODEM      (0X08)
#define SC16IS7XX_INT_LINE       (0X04)
#define SC16IS7XX_INT_THR        (0X02)
#define SC16IS7XX_INT_RHR        (0X01)

#endif // __SC16IS7XX_REG_H
