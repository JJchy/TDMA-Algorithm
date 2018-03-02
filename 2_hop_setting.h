#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <assert.h>

#define PACKET_DATA 1024   // It should be multiple of 8
                           // 128Bytes
#define HEADER_SIZE 224    // 28Bytes
#define PACKET_SIZE (PACKET_DATA+HEADER_SIZE)
#define PACKET_NUM(x) (ceil(((double)(x))/PACKET_DATA))
#define TC_SLOT_PACKET_GCS (PILOT(ceil((((PACKET_NUM(816))*PACKET_SIZE)/4.265)/\
                                       (((double)SUBFRAME_SIZE)/MINISLOT))))
#define TC_SLOT_PACKET(x) (PILOT(ceil((((PACKET_NUM(816))*PACKET_SIZE)/(x))/\
                                      (((double)SUBFRAME_SIZE)/MINISLOT))))
#define WITHOUT_TC_PACKET(x) (MINISLOT-TC_SLOT_PACKET_GCS-TC_SLOT_PACKET(x))

#define SUBFRAME 24 
#define MINISLOT 220
#define SUBFRAME_SIZE 40000 // us
#define TM_SIZE 1024      // 128Bytes
#define VIDEO_SIZE 512000 // 64KB
#define AUDIO_SIZE 29600  // 3.7KB
#define RATIO 3 // 1:9
//#define IDEAL_PILOT
// data slot -> allocation area slot
// remain area slot -> maximum data slot
#ifdef IDEAL_PILOT
  #define PILOT(x) (ceil((((double)(x))/RATIO)+(x)+4)) // Allocation -ideal
  #define PILOT_CHECK(x) (floor(((double)RATIO/(RATIO+1))*((x)-4))) // Check -ideal
#else
  #define PILOT(x) ((ceil(((double)(x))/RATIO)*(RATIO+1))+4) // -real
  #define PILOT_CHECK(x) ((floor(((double)((x)-4))/(RATIO+1)))*RATIO)
#endif

#define SET_X(x) (1<<((x)-1))

#define MIN(x,y) ((x) < (y) ? (x) : (y))

//#define SIMUL

// Input structure
typedef struct
{
  int drone;
  int parent_num;
  int parent[15];
  int children[15][15];

  double rate[15];  //Mbps
  double buffering; //

  bool video[15];
  bool video_preexist[15];
  bool audio[15];
  bool audio_preexist[15];

  bool video_first;
  bool preexist_first;
} Graph;

typedef struct
{
  bool video[15];
  bool audio[15];
} Media_Result;

// For sorting
typedef struct list Link;
typedef struct list
{
  int drone_num;
  Link *next;
} Link;

typedef struct using_list ULink;
typedef struct using_list
{
  int usage;
  int start;
  int end;
  ULink *next;
} ULink;

Link *Priority_list;

int **MRR (Graph *, Media_Result *);
