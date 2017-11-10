#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#define PACKET_DATA 1024   // It should be multiple of 8
                           // 128Bytes
#define HEADER_SIZE 224   // 28Bytes
#define PACKET_SIZE (PACKET_DATA+HEADER_SIZE)
#define PACKET_NUM(x) (ceil(((double)(x))/PACKET_DATA))
#define TC_SLOT_PACKET (PILOT(ceil((((PACKET_NUM(816))*PACKET_SIZE)/4.265)/\
                                   (((double)SUBFRAME_SIZE)/MINISLOT))))
#define WITHOUT_TC_PACKET (MINISLOT-TC_SLOT_PACKET)
#define SLOT_LOCATION_PACKET(x) (WITHOUT_TC_PACKET-(x))


/*
#define PILOT_PACKET(x) (PILOT((x)*PACKET_SIZE))
#define PILOT_CHECK_PACKET(x) (floor(PILOT_CHECK(x)/PACKET_SIZE))
#define TC_SLOT_PACKET (ceil((PILOT_PACKET(PACKET_NUM(816)))/4.265)
#define WITHOUT_TC_PACKET (MINISLOT-TC_SLOT_PACKET)
*/

#define SUBFRAME 24 
#define MINISLOT 220
#define SUBFRAME_SIZE 40000 // us
#define RATIO 3 // 1:3
#define PILOT(x) (ceil((((double)(x))/RATIO)+(x)+4)) // Allocation
// data slot -> allocation area slot
#define PILOT_CHECK(x) (floor(((double)RATIO/(RATIO+1))*((x)-4))) // Check
// remain area slot -> maximum data slot

#define TC_SLOT PILOT(2)       // 4.265Mbps, 102Bytes
#define WITHOUT_TC (MINISLOT-TC_SLOT)
#define SLOT_LOCATION(x) (WITHOUT_TC-(x))
#define TM_SIZE 1024      // 128Bytes
#define VIDEO_SIZE 512000 // 64KB
#define AUDIO_SIZE 29600  // 3.7KB

#define MIN(x,y) ((x) < (y) ? (x) : (y))

// Input structure
typedef struct
{
  int drone;
  double rate[15];  //Mbps
  double buffering; //

  bool video[15];
  bool video_preexist[15];
  bool audio[15];
  bool audio_preexist[15];

  bool video_first;
  bool preexist_first;
} Graph;

// For sorting
typedef struct list Link;
typedef struct list
{
  int drone_num;
  Link *next;
} Link;

Link *Priority_list;

int **round_robin (Graph *);
int **modified_RR (Graph *);
