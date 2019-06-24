/* 
 * Application-specific configuration values for nRF24L01+
 *
 * Requires definitions for:
 * NRF24_CSN - CSN (active low) pin on nRF24L01+
 * NRF24_CE - Chip Enable pin on nRF24L01+
 * NRF24_IRQ - Interrupt pin (active low) on nRF24L01+
 * NRF24_XFER_SPI(x) - Transfer one byte to/from SPI bus without changing CSN
 * 
 * Modify includes as necessary for processor architecture
 * 
 */

#ifndef NRF24L01P_CFG_H
#define	NRF24L01P_CFG_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define NRF24_CSN   LATCbits.LATC3
#define NRF24_CE    LATCbits.LATC4
#define NRF24_IRQ   PORTCbits.RC5

#define NRF24_XFER_SPI(x)   transfer_spi(x)
    
/* Function prototype for definition of NRF24_XFER_SPI(x) must be speficied here */
uint8_t transfer_spi(uint8_t data);
    
#ifdef	__cplusplus
}
#endif

#endif	/* NRF24L01P_CFG_H */

