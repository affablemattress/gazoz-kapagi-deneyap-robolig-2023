
#pragma once
#include "musicPlayer.h"

static const uint16_t noodleScale[7] = { 164, 174, 185, 196, 207, 220, 233 };
static const uint8_t noodleNotes[10] = { 0, 1, 2, 3, 2, 5, 4, 6, 1, 0 };
static const TickType_t noodleLengths[10] =   { 800, 800, 400, 400, 800, 800, 1600, 400, 400, 1600 };
const SongData noodle( 0, 10, noodleScale, noodleNotes, noodleLengths);

static const uint16_t E3[13] = { 164, 174, 185, 196, 207, 220, 233, 466, 493, 523, 554, 587, 622 };
static const uint8_t masterNotes[27] = { 0, 0, 12, 0, 0, 11, 0, 0, 10, 9, 8, 0, 0, 7, 0, 0, 6, 0, 0, 5, 0, 4, 0, 3, 0, 2, 1 };
static const TickType_t masterLengths[27] =   { 142, 142, 142, 150, 142, 142, 142, 150, 284, 284, 570, 142, 142, 142, 150, 142, 142, 142, 
                                                150, 142, 142, 142, 150, 142, 142, 142, 150 };
const SongData master(1, 27, E3, masterNotes, masterLengths);