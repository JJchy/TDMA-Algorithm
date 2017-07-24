#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#define SUBFRAME 25
#define MINISLOT 220
#define SUBFRAME_SIZE 40000 // us
#define RATIO 3 // 1:3
#define PILOT(x) (ceil((((double)(x))/RATIO)+(x)+4)) // Allocation
// data -> allocation area
#define PILOT_CHECK(x) (floor(((double)RATIO/(RATIO+1))*((x)-4))) // Check
// remain area -> maximum data

#define TC_SLOT PILOT(2)       // 4.265Mbps, 102Bytes
#define WITHOUT_TC (MINISLOT)-(TC_SLOT)
#define SLOT_LOCATION(x) (WITHOUT_TC-(x))
#define TM_SIZE 1024      // 128Bytes
#define VIDEO_SIZE 512000 // 64KB
#define AUDIO_SIZE 29600  // 3.7KB

// Input structure
typedef struct
{
  int drone;
  double rate[15]; //Mbps
  int video[15];
  int audio[15];
} Graph;

// Output structure
typedef struct
{
  int data[SUBFRAME][MINISLOT];
} Schedule;

// For sorting
typedef struct list Link;
typedef struct list
{
  int drone_num;
  Link *next;
} Link;

Link *Priority_list;

Schedule *round_robin (Graph *);
Schedule *evenly_distribute (Graph *);
Schedule *modified_RR (Graph *);
