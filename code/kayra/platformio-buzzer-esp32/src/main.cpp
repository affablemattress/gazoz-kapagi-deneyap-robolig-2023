#include <Arduino.h>

#define LEDC_RES 10

#define PLAYER_CHANNEL 6
#define PLAYER_PIN GPIO_NUM_12

void playerTask(void*);
TaskHandle_t playerTaskHandle = NULL;

struct SongData {
  const char* songName;
  uint8_t songID;
  uint16_t noteCount;
  const uint16_t* scaleArray;
  const uint16_t* notesArray;
  const uint16_t* noteLengthsArray;

  SongData() = delete;
  SongData(SongData &&) = delete;
  SongData(const char songName[], uint8_t songID, 
           uint8_t noteCount, const uint16_t scaleArray[], const uint16_t noteArray[], const uint16_t noteLengthArray[])
    : songName(songName), songID(songID), 
      noteCount(noteCount), scaleArray(scaleArray), notesArray(noteArray), noteLengthsArray(noteLengthArray) {}
};

const uint16_t phrygianDominant[7] = {262, 277, 330, 349, 392, 415, 466};
const uint16_t noodleNotes[10] =   { 0,   1,   2,   3,   2,   5,   4,   6,   1,   0 };
const uint16_t noodleLengths[10] = { 4 * 200, 4 * 200, 4 * 100, 4 * 100, 4 * 200, 4 * 400, 4 * 400, 4 * 100, 4 * 100, 4 * 400 };
const SongData noodle("Noodle", 0, 10, phrygianDominant, noodleNotes, noodleLengths);

void setup() {
  ledcSetup(PLAYER_CHANNEL, 50, LEDC_RES);
  ledcAttachPin(PLAYER_PIN, PLAYER_CHANNEL);

  xTaskCreatePinnedToCore((TaskFunction_t)playerTask, "Player Task", 1024, (void*)&noodle, 2, &playerTaskHandle, APP_CPU_NUM);
}

void loop() {
  vTaskDelete(NULL);
}

void playerTask(void* songData) {
  SongData* song = (SongData*) songData;

  while(1) {
    for(size_t i = 0; i < song->noteCount; i++) {
      ledcWriteTone(PLAYER_CHANNEL, song->scaleArray[song->notesArray[i]]);
      vTaskDelay(song->noteLengthsArray[i]);
    }

  }
}