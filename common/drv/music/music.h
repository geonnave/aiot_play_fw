#ifndef __MUSIC_H
#define __MUSIC_H

#include "stdint.h"
#include "notes.h"

//=========================== define ==========================================

//=== speed

// RTC1 runs at 32 kHz
// we set the speed of the song by tuning the prescaler of RTC1
// the number is in 1/8th notes
// speed (in bpm) = (32000*60)/(8*(value+1))
// value          = ((32000*60)/(8*speed)-1)

//=== songs

typedef struct {
    uint32_t val;      // PWM period
    uint16_t duration; // in 1/8th
} note_t;

typedef enum {
    SONGTITLE_STAR_WARS,
    SONGTITLE_HARRY_POTTER,
    SONGTITLE_SYNCTEST
} songtitle_t;

static const uint16_t SONGSPEED[] = {
    150, // SONGTITLE_STAR_WARS
    220, // SONGTITLE_HARRY_POTTER
    220, // SONGTITLE_SYNCTEST
};

#include "song_synctest.h"
#include "Star_Wars_Medley.h"
#include "harry_potter.h"

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void music_init(void);
void music_play(songtitle_t songtitle, uint8_t track);

#endif
