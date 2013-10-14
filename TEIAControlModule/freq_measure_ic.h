/* 
 * File:   freq_measure_ic.h
 * Author: Miguel Zea
 *
 * Created on 20 de agosto de 2013, 07:34 AM
 */

#ifndef FREQ_MEASURE_IC_H
#define	FREQ_MEASURE_IC_H

#ifdef	__cplusplus
extern "C" {
#endif

#define CHANNEL_IC1 (~0x0001)
#define CHANNEL_IC2 (~0x0002)
#define CHANNEL_IC7 (~0x0004)
#define CHANNEL_IC8 (~0x0008)
#define RANGE_HIGHER_FREQ   (1U)    // From ~500Hz to ~39MHz
#define RANGE_HIGH_FREQ     (2U)    // From ~70Hz to ~4.5MHz
#define RANGE_MIDDLE_FREQ   (3U)    // From ~9Hz to ~600kHz
#define RANGE_LOW_FREQ      (4U)    // From ~2Hz to ~150kHz

void
freq_measure_ic_init(unsigned int channels_to_enable, unsigned int freq_range);

uint16_t
freq_measure_ic_get_freq(unsigned int channel);

#ifdef	__cplusplus
}
#endif

#endif	/* FREQ_MEASURE_IC_H */

