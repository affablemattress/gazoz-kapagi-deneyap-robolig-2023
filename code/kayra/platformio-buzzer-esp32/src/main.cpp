#include <Arduino.h>

#define LEDC_RES 10

#define MELODY_CHANNEL 7
#define MELODY_PIN GPIO_NUM_14

#define CHORD_CHANNEL 6
#define CHORD_PIN GPIO_NUM_12

void playerTaskMelody(void*);
TaskHandle_t playerTaskMelodyHandle = NULL;
void playerTaskChord(void*);
TaskHandle_t playerTaskChordHandle = NULL;


struct SongData {
  const char* songName;
  uint8_t songID;
  uint16_t noteCount;
  const uint16_t* scaleArray;
  const uint16_t* melodyNotesArray;
  const int8_t* chordNotesArray;
  const TickType_t* noteLengthsArray;

  SongData() = delete;
  SongData(SongData &&) = delete;
  SongData(const char songName[], 
           uint8_t songID, 
           uint8_t noteCount, 
           const uint16_t scaleArray[], 
           const uint16_t melodyNotesArray[], 
           const int8_t chordNotesArray[], 
           const TickType_t noteLengthsArray[])
    : songName(songName), songID(songID), noteCount(noteCount), scaleArray(scaleArray), melodyNotesArray(melodyNotesArray), 
      chordNotesArray(chordNotesArray), noteLengthsArray(noteLengthsArray) { }
};

static const uint16_t phrygianDominant[7] =   { 262, 277, 330, 349, 392, 415, 466};
static const uint16_t noodleMelodyNotes[10] = { 0, 1, 2, 3, 2, 5, 4, 6, 1, 0 };
static const int8_t noodleChordNotes[10] =    { -1, -1, -1, 1, 1, 1, 1, 0, 0, -1 };
static const TickType_t noodleLengths[10] =   { 800, 800, 400, 400, 800, 800, 1600, 400, 400, 1600 };
const SongData noodle("Noodle", 0, 10, phrygianDominant, noodleMelodyNotes, noodleChordNotes,  noodleLengths);

static const uint16_t E3[13] = { 164, 174, 185, 196, 207, 220, 233, 466, 493, 523, 554, 587, 622 };
static const uint16_t masterMelodyNotes[27] = {   0,   0,  12,   0,   0,  11,   0,   0,
                                                 10,   9,   8,
                                                  0,   0,   7,   0,   0,   6,   0,   0,
                                                  5,   0,   4,   0,   3,   0,   2,   1 };
static const int8_t masterChordNotes[27] =    {  -1,  -1,   0,  -1,  -1,   0,  -1,  -1,
                                                  1,   1,   1,
                                                 -1,  -1,   0,  -1,   0,   1,   0,   0,
                                                  0,   0,   1,   0,   1,   1,   1,   1 };
static const TickType_t masterLengths[27] =   { 142, 142, 142, 150, 142, 142, 142, 150,
                                                284, 284, 570,
                                                142, 142, 142, 150, 142, 142, 142, 150,
                                                142, 142, 142, 150, 142, 142, 142, 150 };
const SongData master("MoP", 1, 27, E3, masterMelodyNotes, masterChordNotes, masterLengths);

void setup() {
  ledcSetup(MELODY_CHANNEL, 50, LEDC_RES);
  ledcAttachPin(MELODY_PIN, MELODY_CHANNEL);
  ledcSetup(CHORD_CHANNEL, 50, LEDC_RES);
  ledcAttachPin(CHORD_PIN, CHORD_CHANNEL);

  xTaskCreatePinnedToCore((TaskFunction_t)playerTaskChord, "Player Chord Task", 1200, (void*)&master, 2, &playerTaskChordHandle, APP_CPU_NUM);
  xTaskCreatePinnedToCore((TaskFunction_t)playerTaskMelody, "Player Melody Task", 1200, (void*)&master, 2, &playerTaskMelodyHandle, APP_CPU_NUM);
}

void loop() {
  vTaskDelete(NULL);
}

void playerTaskMelody(void* songData) {
  static const TickType_t strumBreakLength = 35;
  SongData* song = (SongData*) songData;

  while(1) {
    for(size_t i = 0; i < song->noteCount; i++) {
      ledcWriteTone(MELODY_CHANNEL, song->scaleArray[song->melodyNotesArray[i]]);
      vTaskDelay(song->noteLengthsArray[i] - strumBreakLength);
      ledcWriteTone(MELODY_CHANNEL, 1);
      vTaskDelay(strumBreakLength);
    }

  }
}

void playerTaskChord(void* songData) {
  static const TickType_t strumBreakLength = 20;
  SongData* song = (SongData*) songData;

  while(1) {
    for(size_t i = 0; i < song->noteCount; i++) {
      switch (song->chordNotesArray[i])
      {
        case -1:
          ledcWriteTone(CHORD_CHANNEL, song->scaleArray[song->melodyNotesArray[0]] / 4);
          break;
        case 0:
          ledcWriteTone(CHORD_CHANNEL, song->scaleArray[song->melodyNotesArray[0]]);
          break;
        case 1:
          ledcWriteTone(CHORD_CHANNEL, song->scaleArray[song->melodyNotesArray[i]] * 3 / 2);
          break;
      }
      vTaskDelay(song->noteLengthsArray[i] - strumBreakLength);
      ledcWriteTone(CHORD_CHANNEL, 1);
      vTaskDelay(strumBreakLength);
    }
  }
}