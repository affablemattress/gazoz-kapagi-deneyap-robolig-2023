#include "vars.h"
#include <Arduino.h>

struct SongData {
  uint8_t songID;
  uint16_t noteCount;
  const uint16_t* scaleArray;
  const uint8_t* notesArray;
  const TickType_t* noteLengthsArray;

  SongData(uint8_t songID, 
           uint8_t noteCount, 
           const uint16_t scaleArray[], 
           const uint8_t notesArray[], 
           const TickType_t noteLengthsArray[])
    : songID(songID), noteCount(noteCount), scaleArray(scaleArray), notesArray(notesArray), noteLengthsArray(noteLengthsArray) { }

};

// <--------------------------------------------------------------------------------------------------------------------->
// <------------------------------------------------------- TASKS ------------------------------------------------------->
// <--------------------------------------------------------------------------------------------------------------------->
void playerTask(void*);
TaskHandle_t playerTaskHandle = NULL;
void playerTask(void* songData) {
  static const TickType_t strumBreakLength = 35;
  SongData* song = (SongData*) songData;

  ledcSetup(SONG_CHANNEL, 50, 13);
  ledcAttachPin(BUZZ, SONG_CHANNEL);

  while(1) {
    for(size_t i = 0; i < song->noteCount; i++) {
      ledcWriteTone(SONG_CHANNEL, song->scaleArray[song->notesArray[i]]);
      vTaskDelay((song->noteLengthsArray[i] - strumBreakLength) / portTICK_PERIOD_MS);
      ledcWriteTone(SONG_CHANNEL, 1);
      vTaskDelay(strumBreakLength / portTICK_PERIOD_MS);
    }
  }
}
