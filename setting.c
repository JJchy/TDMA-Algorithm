#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define SUBFRAME 25
#define MINISLOT 220
#define SUBFRAME_SIZE 40000 // us

#define TC_SLOT 30       // 4.265Mbps, 2500Bytes
#define WITHOUT_TC 190
#define SLOT_LOCATION(x) (190-(x))
#define TM_SIZE 8000     // 1000Bytes
#define VIDEO_SIZE 20480 // 2560Bytes
#define AUDIO_SIZE 1184  // 148Bytes

typedef struct
{
  int drone;
  double rate[15]; //Mbps
  int video[15];
  int audio[15];
} Graph;

typedef struct
{
  int data[SUBFRAME][MINISLOT];
} Schedule;

Schedule *round_robin (Graph *setting)
{
  Schedule *result = (Schedule *) calloc (1, sizeof (Schedule));

  for (int i = 0; i < SUBFRAME / 4; i++)
    for (int j = 1; j <= TC_SLOT; j++)
      result->data[i][MINISLOT - j] = 16; // 16 : GCS -> Drone (Broadcast)

  double tm_time;
  int tm_slot, tm_location = 0, tm_subframe = 0;
  int temp;
  for (int i = 1; i <= setting->drone; i++)
  {
    tm_time = TM_SIZE / setting->rate[i-1];
    tm_slot = (tm_time / ((double) SUBFRAME_SIZE / MINISLOT)) + 1; // ceiling
    
    if (tm_subframe >= 4)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n",  i);
      printf ("NO MORE SLOT\n");
      for (int j = 1; j < SUBFRAME / 4; j++)
        memcpy (result, ((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                (sizeof (int) * MINISLOT * 4));
      return result;
    }

    if (WITHOUT_TC - tm_location >= tm_slot + 4) // preamble
    {
      tm_slot += 4;
      for (int j = 0; j < tm_slot; j++)
        result->data[tm_subframe][tm_location + j] = i;

      tm_location += tm_slot;
        
      if (WITHOUT_TC - tm_location <= 4)
      {
        tm_subframe++;
        tm_location = 0;
      }
    }

    else
    {
      temp = ((4 - tm_subframe) * (WITHOUT_TC - 4)) - tm_location;
      if (temp < tm_slot)
      {
        printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", i);
        printf ("%d(th/st/nd/rd) TM DATA is TOO big\n", i);  
        for (int j = 1; j < SUBFRAME / 4; j++)
          memcpy (result, ((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                  (sizeof (int) * MINISLOT * 4));
        return result;
      }

      while (true)
      {
        tm_slot += 4;
        if (WITHOUT_TC - tm_location >= tm_slot)
        {
          for (int j = 0; j < tm_slot; j++)
            result->data[tm_subframe][tm_location + j] = i;

          tm_location += tm_slot;

          if (WITHOUT_TC - tm_location <= 4)
          {
            tm_subframe++;
            tm_location = 0;
          }

          break;
        }

        else
        {
          for (int j = tm_location; j < WITHOUT_TC; j++)
            result->data[tm_subframe][j] = i;
          
          tm_slot -= WITHOUT_TC - tm_location;
          tm_subframe++;
          tm_location = 0;
        }
      }
    }

    for (int j = 0; j < SUBFRAME / 4; j++)
      memcpy (result, ((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
              (sizeof (int) * MINISLOT * 4));

  }

  int remain_data[SUBFRAME - 1];
  for (int i = 0; i < 4; i++)
  {
    if (i < tm_subframe)       remain_data[i] = 0;
    else if (i == tm_subframe) remain_data[i] = WITHOUT_TC - tm_location;
    else                       remain_data[i] = WITHOUT_TC;
    
    if (remain_data[i] < 4) remain_data[i] = 0;
  }  
  
  for (int i = 0; i < SUBFRAME / 4; i++)
    memcpy (remain_data, ((void *) result) + (sizeof (int) * 4 * i),\
            (sizeof (int) * 4));

  //

  int subframe_slot = 0;
  int temp_location;
  double media_time;
  int media_num, media_size, media_slot;
  bool is_video = true;

  while (true)
  {
    for (int i = 1; i <= setting->drone; i++)
    {
      if (is_video == true) 
      {
        media_num = setting->video[i-1];
        media_size = VIDEO_SIZE;
      }
      else  
      {
        media_num = setting->audio[i-1];
        media_size = AUDIO_SIZE;
      }

      while (media_num != 0)
      {
        media_num--;
        media_time = media_size / setting->rate[i-1];
        media_slot = (media_time / ((double) SUBFRAME_SIZE / MINISLOT)) + 1;
        if (subframe_slot >= SUBFRAME - 1)
        {
          if (is_video == true)
            printf ("DROP : %d(th/st/nd/rd)'s VIDEO %d is drop\n",\
                    i, setting->video[i-1] - media_num);
          else
            printf ("DROP : %d(th/st/nd/rd)'s AUDIO %d is drop\n",\
                    i, setting->audio[i-1] - media_num);
          printf ("NO MORE SLOT\n");
          return result;
        }

        if (remain_data[subframe_slot] >= media_slot + 4)
        {
          media_slot += 4;
          temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
          for (int j = 0; j < media_slot; j++)
            result->data[subframe_slot][temp_location + j] = i;

          remain_data[subframe_slot] -= media_slot;
        
          if (remain_data[subframe_slot] <= 4)
            subframe_slot++;

        }
      
        else
        {
          temp = 0;
          for (int j = 0; j < SUBFRAME - 1; j++)
            temp += remain_data[subframe_slot] - 4;

          if (temp < media_slot)
          {
            if (is_video == true)
              printf ("DROP : %d(th/st/nd/rd)'s VIDEO %d is drop\n",\
                      i, setting->video[i-1] - media_num);
            else
              printf ("DROP : %d(th/st/nd/rd)'s AUDIO %d is drop\n",\
                      i, setting->audio[i-1] - media_num);
            printf ("The DATA is TOO big\n");  
            return result;
          }

          while (true)
          {
            media_slot += 4;
            if (media_slot <= remain_data[subframe_slot])
            {
              temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
              for (int k = 0; k < media_slot; k++)
                result->data[subframe_slot][temp_location + k] = i;

              remain_data[subframe_slot] -= media_slot;

              if (remain_data[subframe_slot] <= 4)
                subframe_slot++;

              break;
            }

            else
            {
              temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
              for (int k = 0; k < remain_data[subframe_slot]; k++)
                result->data[subframe_slot][temp_location + k] = i;
          
              media_slot -= remain_data[subframe_slot];
              remain_data[subframe_slot] = 0;
              subframe_slot++;

            }
          }
        }
      }
    }

    is_video = false;
  }

  return result;
}

Schedule *evenly_distribute (Graph *setting)
{
  Schedule *result = (Schedule *) calloc (1, sizeof (Schedule));

  for (int i = 0; i < SUBFRAME / 4; i++)
    for (int j = 1; j <= TC_SLOT; j++)
      result->data[i][MINISLOT - j] = 16; // 16 : GCS -> Drone (Broadcast)

  double tm_time;
  int tm_slot, tm_location = 0, tm_subframe = 0;
  int temp;
  for (int i = 1; i <= setting->drone; i++)
  {
    tm_time = TM_SIZE / setting->rate[i-1];
    tm_slot = (tm_time / ((double) SUBFRAME_SIZE / MINISLOT)) + 1; // ceiling
    
    if (tm_subframe >= 4)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n",  i);
      printf ("NO MORE SLOT\n");
      for (int j = 1; j < SUBFRAME / 4; j++)
        memcpy (result, ((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                (sizeof (int) * MINISLOT * 4));
      return result;
    }

    if (WITHOUT_TC - tm_location >= tm_slot + 4) // preamble
    {
      tm_slot += 4;
      for (int j = 0; j < tm_slot; j++)
        result->data[tm_subframe][tm_location + j] = i;

      tm_location += tm_slot;
        
      if (WITHOUT_TC - tm_location <= 4)
      {
        tm_subframe++;
        tm_location = 0;
      }
    }

    else
    {
      temp = ((4 - tm_subframe) * (WITHOUT_TC - 4)) - tm_location;
      if (temp < tm_slot)
      {
        printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", i);
        printf ("%d(th/st/nd/rd) TM DATA is TOO big\n", i);  
        for (int j = 1; j < SUBFRAME / 4; j++)
          memcpy (result, ((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                  (sizeof (int) * MINISLOT * 4));
        return result;
      }

      while (true)
      {
        tm_slot += 4;
        if (WITHOUT_TC - tm_location >= tm_slot)
        {
          for (int j = 0; j < tm_slot; j++)
            result->data[tm_subframe][tm_location + j] = i;

          tm_location += tm_slot;

          if (WITHOUT_TC - tm_location <= 4)
          {
            tm_subframe++;
            tm_location = 0;
          }

          break;
        }

        else
        {
          for (int j = tm_location; j < WITHOUT_TC; j++)
            result->data[tm_subframe][j] = i;
          
          tm_slot -= WITHOUT_TC - tm_location;
          tm_subframe++;
          tm_location = 0;
        }
      }
    }
  }

  int remain_data[4];
  for (int i = 0; i < 4; i++)
  {
    if (i < tm_subframe)       remain_data[i] = 0;
    else if (i == tm_subframe) remain_data[i] = WITHOUT_TC - tm_location;
    else                       remain_data[i] = WITHOUT_TC;
    
    if (remain_data[i] < 4) remain_data[i] = 0;
  }  

  //

  int subframe_slot = 0;
  int temp_location;
  double media_time;
  int media_num, media_size, media_slot;
  bool media_list[setting->drone];
  bool is_video = true;
  bool is_finish = false;

  while (is_finish != true)
  {
    if (is_video == true) 
    {
      for (int i = 1; i <= setting->drone; i++)
      {
        if (setting->video[i-1] != 0)
        {
          media_list[i-1] = true;
          setting->video[i-1]--;
        }
        else
          media_list[i-1] = false;
      }
      media_size = VIDEO_SIZE;
    }

    else  
    {
      for (int i = 1; i <= setting->drone; i++)
      {
        if (setting->audio[i-1] != 0)
        {
          media_list[i-1] = true;
          setting->audio[i-1]--;
        }
        else
          media_list[i-1] = false;
      }
      media_size = AUDIO_SIZE;
    }

    for (int i = 1; i <= setting->drone; i++)
    {
      if (media_list[i-1] == false) continue;
      
      media_time = media_size / (setting->rate[i-1] * 6);
      media_slot = (media_time / ((double) SUBFRAME_SIZE / MINISLOT)) + 1;
      
      if (subframe_slot >= 4)
      {
        if (is_video == true)
          printf ("DROP : %d(th/st/nd/rd)'s VIDEO %d is drop\n",\
                  i, setting->video[i-1] - media_num);
        else
          printf ("DROP : %d(th/st/nd/rd)'s AUDIO %d is drop\n",\
                    i, setting->audio[i-1] - media_num);
        printf ("NO MORE SLOT\n");
          
        for (int j = 0; j < SUBFRAME / 4; j++)
          memcpy (result, ((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                  (sizeof (int) * MINISLOT * 4));

        return result;
      }

      if (remain_data[subframe_slot] >= media_slot + 4)
      {
        media_slot += 4;
        temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
        for (int j = 0; j < media_slot; j++)
          result->data[subframe_slot][temp_location + j] = i;

        remain_data[subframe_slot] -= media_slot;
        
        if (remain_data[subframe_slot] <= 4)
          subframe_slot++;
      }
      
      else
      {
        temp = 0;
        for (int j = 0; j < SUBFRAME / 6; j++)
          temp += remain_data[subframe_slot] - 4;

        if (temp < media_slot)
        {
          if (is_video == true)
            printf ("DROP : %d(th/st/nd/rd)'s VIDEO %d is drop\n",\
                    i, setting->video[i-1] - media_num);
          else
            printf ("DROP : %d(th/st/nd/rd)'s AUDIO %d is drop\n",\
                    i, setting->audio[i-1] - media_num);
          printf ("The DATA is TOO big\n");  
          
          for (int j = 0; j < SUBFRAME / 4; j++)
            memcpy (result, ((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                    (sizeof (int) * MINISLOT * 4));
          
          return result;
        }

        while (true)
        {
          media_slot += 4;
          if (media_slot <= remain_data[subframe_slot])
          {
            temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
            for (int k = 0; k < media_slot; k++)
              result->data[subframe_slot][temp_location + k] = i;

            remain_data[subframe_slot] -= media_slot;

            if (remain_data[subframe_slot] <= 4)
              subframe_slot++;

            break;
          }

          else
          {
            temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
            for (int k = 0; k < remain_data[subframe_slot]; k++)
            result->data[subframe_slot][temp_location + k] = i;
          
            media_slot -= remain_data[subframe_slot];
            remain_data[subframe_slot] = 0;
            subframe_slot++;
          }
        }
      }
    }
    
    if (is_video == true)
    {
      is_video = false;
      for (int i = 0; i < setting->drone; i++)
      {
        if (setting->video[i] != 0) 
        {
          is_video = true;
          break;
        }
      }
    }

    if (is_video == false)
    {
      is_finish = true;
      for (int i = 0; i < setting->drone; i++)
      {
        if (setting->audio[i] != 0) 
        {
          is_finish = false;
          break;
        }
      }
    }
  }

  for (int j = 0; j < SUBFRAME / 4; j++)
    memcpy (result, ((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
            (sizeof (int) * MINISLOT * 4));

  return result;
}

   
