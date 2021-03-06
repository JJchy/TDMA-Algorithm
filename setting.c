/* TDMA Scheduling function 
 * by CHY                    */

#include "setting.h"

Schedule *round_robin (Graph *setting)
{
  Schedule *result = (Schedule *) calloc (1, sizeof (Schedule));

  //TC
  for (int i = 0; i < 4; i++)
    for (int j = 1; j <= TC_SLOT; j++)
      result->data[i][MINISLOT - j] = 16; // 16 : GCS -> Drone (Broadcast)

  //TM
  double tm_time;
  int tm_slot, tm_location = 0, tm_subframe = 0;
  int temp;
  for (int i = 1; i <= setting->drone; i++)
  {
    tm_time = TM_SIZE / setting->rate[i-1];
    tm_slot = ceil(tm_time / ((double) SUBFRAME_SIZE / MINISLOT));

    if (tm_subframe >= 4)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", i);
      printf ("NO MORE SLOT\n");
      for (int j = 1; j < SUBFRAME / 4; j++)
        memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                result, (sizeof (int) * MINISLOT * 4));
      return result;
    }

    if (PILOT_CHECK(WITHOUT_TC - tm_location) >= tm_slot)
    {
      for (int j = 0; j < PILOT(tm_slot); j++)
        result->data[tm_subframe][tm_location + j] = i;

      tm_location += PILOT(tm_slot);
      assert (tm_location <= WITHOUT_TC);
        
      if (WITHOUT_TC - tm_location <= 5)
      {
        tm_subframe++;
        tm_location = 0;
      }
    }

    else
    {
      temp = ((3 - tm_subframe) * PILOT_CHECK(WITHOUT_TC)) +\
             PILOT_CHECK(WITHOUT_TC - tm_location);
      if (temp < tm_slot)
      {
        printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", i);
        printf ("%d(th/st/nd/rd) TM DATA is TOO big\n", i); 
        for (int j = 1; j < SUBFRAME / 4; j++)
          memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                  result, (sizeof (int) * MINISLOT * 4));
        return result;
      }

      while (true)
      {
        if (PILOT_CHECK(WITHOUT_TC - tm_location) >= tm_slot)
        {
          for (int j = 0; j < PILOT(tm_slot); j++)
            result->data[tm_subframe][tm_location + j] = i;

          tm_location += PILOT(tm_slot);
          assert (tm_location <= WITHOUT_TC);

          if (WITHOUT_TC - tm_location <= 5)
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
          
          tm_slot -= PILOT_CHECK(WITHOUT_TC - tm_location);
          tm_subframe++;
          tm_location = 0;
        }
      }
    }
  }

  for (int i = 0; i < SUBFRAME / 4; i++)
    memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * i),\
            result, (sizeof (int) * MINISLOT * 4));

  // Ready to put media
  int remain_data[SUBFRAME - 1];
  for (int i = 0; i < 4; i++)
  {
    if (i < tm_subframe)       remain_data[i] = 0;
    else if (i == tm_subframe) remain_data[i] = WITHOUT_TC - tm_location;
    else                       remain_data[i] = WITHOUT_TC;
    
    if (remain_data[i] <= 5) remain_data[i] = 0;
  }  
  
  for (int i = 0; i < SUBFRAME / 4; i++)
    memcpy (((void *) remain_data) + (sizeof (int) * 4 * i),\
            remain_data, (sizeof (int) * 4));

  //

  // Media
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
        media_slot = ceil(media_time / ((double) SUBFRAME_SIZE / MINISLOT));
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

        if (PILOT_CHECK(remain_data[subframe_slot]) >= media_slot)
        {
          temp_location = SLOT_LOCATION(remain_data[subframe_slot]);
          for (int j = 0; j < PILOT(media_slot); j++)
            result->data[subframe_slot][temp_location + j] = i;

          remain_data[subframe_slot] -= PILOT(media_slot);
          assert (remain_data[subframe_slot] >= 0);
        
          if (remain_data[subframe_slot] <= 5)
            subframe_slot++;
        }
      
        else
        {
          temp = 0;
          for (int j = 0; j < SUBFRAME - 1; j++)
            if (remain_data[j] > 5)  
              temp += PILOT_CHECK(remain_data[j]);

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
            if (PILOT_CHECK(remain_data[subframe_slot]) >= media_slot)
            {
              temp_location = SLOT_LOCATION(remain_data[subframe_slot]);
              for (int k = 0; k < PILOT(media_slot); k++)
                result->data[subframe_slot][temp_location + k] = i;

              remain_data[subframe_slot] -= PILOT(media_slot);
              assert (remain_data[subframe_slot] >= 0);

              if (remain_data[subframe_slot] <= 5)
                subframe_slot++;

              break;
            }

            else
            {
              temp_location = SLOT_LOCATION(remain_data[subframe_slot]);
              for (int k = 0; k < remain_data[subframe_slot]; k++)
                result->data[subframe_slot][temp_location + k] = i;
          
              media_slot -= PILOT_CHECK(remain_data[subframe_slot]);
              remain_data[subframe_slot] = 0;
              subframe_slot++;
            }
          }
        }
      }
    }

    if (is_video == false) break;
    is_video = false;
  }
  return result;
}


Schedule *evenly_distribute (Graph *setting)
{
  Schedule *result = (Schedule *) calloc (1, sizeof (Schedule));

  //TC
  for (int i = 0; i < 4; i++)
    for (int j = 1; j <= TC_SLOT; j++)
      result->data[i][MINISLOT - j] = 16; // 16 : GCS -> Drone (Broadcast)

  //TM
  double tm_time;
  int tm_slot, tm_location = 0, tm_subframe = 0;
  int temp;
  for (int i = 1; i <= setting->drone; i++)
  {
    tm_time = TM_SIZE / setting->rate[i-1];
    tm_slot = ceil(tm_time / ((double) SUBFRAME_SIZE / MINISLOT));
    
    if (tm_subframe >= 4)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n",  i);
      printf ("NO MORE SLOT\n");
      for (int j = 1; j < SUBFRAME / 4; j++)
        memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j), result,\
                (sizeof (int) * MINISLOT * 4));
      return result;
    }

    if (PILOT_CHECK(WITHOUT_TC - tm_location) >= tm_slot)
    {
      for (int j = 0; j < PILOT(tm_slot); j++)
        result->data[tm_subframe][tm_location + j] = i;

      tm_location += PILOT(tm_slot);
      assert (tm_location <= WITHOUT_TC);
        
      if (WITHOUT_TC - tm_location <= 5)
      {
        tm_subframe++;
        tm_location = 0;
      }
    }

    else
    {
      temp = ((3 - tm_subframe) * PILOT_CHECK(WITHOUT_TC)) +\
             PILOT_CHECK(WITHOUT_TC - tm_location);
      if (temp < tm_slot)
      {
        printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", i);
        printf ("%d(th/st/nd/rd) TM DATA is TOO big\n", i); 
        for (int j = 1; j < SUBFRAME / 4; j++)
          memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                  result, (sizeof (int) * MINISLOT * 4));
        return result;
      }

      while (true)
      {
        if (PILOT_CHECK(WITHOUT_TC - tm_location) >= tm_slot)
        {
          for (int j = 0; j < PILOT(tm_slot); j++)
            result->data[tm_subframe][tm_location + j] = i;

          tm_location += PILOT(tm_slot);
          assert (tm_location <= WITHOUT_TC);

          if (WITHOUT_TC - tm_location <= 5)
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
          
          tm_slot -= PILOT_CHECK(WITHOUT_TC - tm_location);
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

  //Media
  int subframe_slot = 0;
  int temp_location;
  int media_num;
  double media_time;
  int media_size, media_slot;
  bool media_list[setting->drone];
  bool is_video = true;
  bool is_finish = false;

  while (is_finish != true)
  {
    media_num = 1;
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
      media_slot = ceil(media_time / ((double) SUBFRAME_SIZE / MINISLOT));
      
      if (subframe_slot >= 4)
      {
        if (is_video == true)
          printf ("DROP : %d(th/st/nd/rd)'s VIDEO %d is drop\n", i, media_num);
        else
          printf ("DROP : %d(th/st/nd/rd)'s AUDIO %d is drop\n", i, media_num);
        printf ("NO MORE SLOT\n");
          
        for (int j = 0; j < SUBFRAME / 4; j++)
          memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                  result, (sizeof (int) * MINISLOT * 4));

        return result;
      }

      if (PILOT_CHECK(remain_data[subframe_slot]) >= media_slot)
      {
        temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
        for (int j = 0; j < PILOT(media_slot); j++)
          result->data[subframe_slot][temp_location + j] = i;

        remain_data[subframe_slot] -= PILOT(media_slot);
        assert (remain_data[subframe_slot] >= 0);
        
        if (remain_data[subframe_slot] <= 5)
          subframe_slot++;
      }
      
      else
      {
        temp = 0;
        for (int j = 0; j < SUBFRAME / 6; j++)
          if (remain_data[j] > 5)
            temp += PILOT_CHECK(remain_data[j]);

        if (temp < media_slot)
        {
          if (is_video == true)
            printf ("DROP : %d(th/st/nd/rd)'s VIDEO %d is drop\n", i,\
                    media_num);
          else
            printf ("DROP : %d(th/st/nd/rd)'s AUDIO %d is drop\n", i,\
                    media_num);
          printf ("The DATA is TOO big\n");  
          
          for (int j = 0; j < SUBFRAME / 4; j++)
            memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                    result, (sizeof (int) * MINISLOT * 4));
          
          return result;
        }

        while (true)
        {
          if (PILOT_CHECK(remain_data[subframe_slot]) >= media_slot)
          {
            temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
            for (int k = 0; k < PILOT(media_slot); k++)
              result->data[subframe_slot][temp_location + k] = i;

            remain_data[subframe_slot] -= PILOT(media_slot);
            assert (remain_data[subframe_slot] >= 0);

            if (remain_data[subframe_slot] <= 5)
              subframe_slot++;

            break;
          }

          else
          {
            temp_location = SLOT_LOCATION (remain_data[subframe_slot]);
            for (int k = 0; k < remain_data[subframe_slot]; k++)
              result->data[subframe_slot][temp_location + k] = i;
          
            media_slot -= PILOT_CHECK(remain_data[subframe_slot]);
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
          media_num++;
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
          media_num++;
          break;
        }
      }
    }
  }

  for (int j = 0; j < SUBFRAME / 4; j++)
    memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
            result, (sizeof (int) * MINISLOT * 4));

  return result;
}

Schedule *modified_RR (Graph *setting)
{
  Schedule *result = (Schedule *) calloc (1, sizeof (Schedule));

  //TC
  for (int i = 0; i < 4; i++)
    for (int j = 1; j <= TC_SLOT; j++)
      result->data[i][MINISLOT - j] = 16; // 16 : GCS -> Drone (Broadcast)

  //Link
  Link *temp_list, *before_list = NULL;
  bool is_last = false;
  for (int i = 0; i < setting->drone; i++)
  {
    Link *temp_list = (Link *) calloc (1, sizeof (Link));
    temp_list->drone_num = i+1;

    if (i == 0)
    {
      Priority_list = temp_list;
      continue;
    }

    before_list = NULL;
    is_last = true;
    for (Link *j = Priority_list; j != NULL; j = j->next)
    {
      if (setting->rate[j->drone_num - 1] < setting->rate[i])
      {
        if (before_list == NULL)
        {
          temp_list->next = Priority_list;
          Priority_list = temp_list;
        }

        else
        {
          before_list->next = temp_list;
          temp_list->next = j;
        }

        is_last = false;
        break;
      }

      before_list = j;
    }

    if (is_last == true)
      before_list->next = temp_list;
  }

  //TM
  int priority_num;
  double tm_time;
  int tm_slot, tm_location = 0, tm_subframe = 0;
  int temp;
  
  temp_list = Priority_list;
  for (int i = 1; i <= setting->drone; i++)
  {
    priority_num = temp_list->drone_num;  

    tm_time = TM_SIZE / setting->rate[priority_num-1];
    tm_slot = ceil(tm_time / ((double) SUBFRAME_SIZE / MINISLOT));
    
    if (tm_subframe >= 4)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", priority_num);
      printf ("(Success : %d)\n", i - 1);
      printf ("NO MORE SLOT\n");
      for (int j = 1; j < SUBFRAME / 4; j++)
        memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                result, (sizeof (int) * MINISLOT * 4));
      return result;
    }

    if (PILOT_CHECK(WITHOUT_TC - tm_location) >= tm_slot)
    {
      for (int j = 0; j < PILOT(tm_slot); j++)
        result->data[tm_subframe][tm_location + j] = priority_num;

      tm_location += PILOT(tm_slot);
      assert (tm_location <= WITHOUT_TC); 

      if (WITHOUT_TC - tm_location <= 5)
      {
        tm_subframe++;
        tm_location = 0;
      }
    }

    else
    {
      temp = ((3 - tm_subframe) * PILOT_CHECK(WITHOUT_TC)) +\
             PILOT_CHECK(WITHOUT_TC - tm_location);
      if (temp < tm_slot)
      {
        printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", priority_num);
        printf ("%d(th/st/nd/rd) TM DATA is TOO big\n", priority_num);
        printf ("(Success : %d)\n", i - 1);
        for (int j = 1; j < SUBFRAME / 4; j++)
          memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * j),\
                  result, (sizeof (int) * MINISLOT * 4));
        return result;
      }

      while (true)
      {
        if (PILOT_CHECK(WITHOUT_TC - tm_location) >= tm_slot)
        {
          for (int j = 0; j < PILOT(tm_slot); j++)
            result->data[tm_subframe][tm_location + j] = priority_num;

          tm_location += PILOT(tm_slot);
          assert (tm_location <= WITHOUT_TC);

          if (WITHOUT_TC - tm_location <= 5)
          {
            tm_subframe++;
            tm_location = 0;
          }

          break;
        }

        else
        {
          for (int j = tm_location; j < WITHOUT_TC; j++)
            result->data[tm_subframe][j] = priority_num;
          
          tm_slot -= PILOT_CHECK(WITHOUT_TC - tm_location);
          tm_subframe++;
          tm_location = 0;
        }
      }
    }
    
    temp_list = temp_list->next;
  }

  for (int i = 0; i < SUBFRAME / 4; i++)
    memcpy (((void *) result) + (sizeof (int) * MINISLOT * 4 * i),\
            result, (sizeof (int) * MINISLOT * 4));

  // Ready to put media
  int remain_data[SUBFRAME - 1];
  for (int i = 0; i < 4; i++)
  {
    if (i < tm_subframe)       remain_data[i] = 0;
    else if (i == tm_subframe) remain_data[i] = WITHOUT_TC - tm_location;
    else                       remain_data[i] = WITHOUT_TC;
    
    if (remain_data[i] <= 5) remain_data[i] = 0;
  }  
  
  for (int i = 0; i < SUBFRAME / 4; i++)
    memcpy (((void *) remain_data) + (sizeof (int) * 4 * i),\
            remain_data, (sizeof (int) * 4));

  //

  // Media
  int subframe_slot = 0;
  int temp_location;
  double media_time;
  int media_num, media_size, media_slot;
  bool is_video = true;

  while (true)
  {
    temp_list = Priority_list;
    for (int i = 1; i <= setting->drone; i++)
    {
      priority_num = temp_list->drone_num;

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
        media_time = media_size / setting->rate[priority_num-1];
        media_slot = ceil(media_time / ((double) SUBFRAME_SIZE / MINISLOT));
        if (subframe_slot >= SUBFRAME - 1)
        {
          if (is_video == true)
            printf ("DROP : %d(th/st/nd/rd)'s VIDEO %d is drop\n",\
                    priority_num,\
                    setting->video[priority_num-1] - media_num);
          else
            printf ("DROP : %d(th/st/nd/rd)'s AUDIO %d is drop\n",\
                    priority_num,\
                    setting->audio[priority_num-1] - media_num);
          printf ("(Success : %d)\n", i-1);
          printf ("NO MORE SLOT\n");
          return result;
        }

        if (PILOT_CHECK(remain_data[subframe_slot]) >= media_slot)
        {
          temp_location = SLOT_LOCATION(remain_data[subframe_slot]);
          for (int j = 0; j < PILOT(media_slot); j++)
            result->data[subframe_slot][temp_location + j] = priority_num;
          
          remain_data[subframe_slot] -= PILOT(media_slot);
          assert (remain_data[subframe_slot] >= 0);
        
          if (remain_data[subframe_slot] <= 5)
            subframe_slot++;
        }
      
        else
        {
          temp = 0;
          for (int j = 0; j < SUBFRAME - 1; j++)
            if (remain_data[j] > 5)
              temp += PILOT_CHECK(remain_data[j]);

          if (temp < media_slot)
          {
            if (is_video == true)
              printf ("DROP : %d(th/st/nd/rd)'s VIDEO %d is drop\n",\
                      priority_num,\
                      setting->video[priority_num-1] - media_num);
            else
              printf ("DROP : %d(th/st/nd/rd)'s AUDIO %d is drop\n",\
                      priority_num,\
                      setting->audio[priority_num-1] - media_num);
            printf ("(Success : %d)\n", i-1);
            printf ("The DATA is TOO big\n");  
            return result;
          }

          while (true)
          {
            if (PILOT_CHECK(remain_data[subframe_slot]) >= media_slot)
            {
              temp_location = SLOT_LOCATION(remain_data[subframe_slot]);
              for (int k = 0; k < PILOT(media_slot); k++)
                result->data[subframe_slot][temp_location + k] = priority_num;

              remain_data[subframe_slot] -= PILOT(media_slot);
              assert (remain_data[subframe_slot] >= 0);

              if (remain_data[subframe_slot] <= 5)
                subframe_slot++;

              break;
            }

            else
            {
              temp_location = SLOT_LOCATION(remain_data[subframe_slot]);
              for (int k = 0; k < remain_data[subframe_slot]; k++)
                result->data[subframe_slot][temp_location + k] = priority_num;
          
              media_slot -= PILOT_CHECK(remain_data[subframe_slot]);
              remain_data[subframe_slot] = 0;
              subframe_slot++;
            }
          }
        }
      }
      
      temp_list = temp_list->next;
    }

    if (is_video == false) break;
    is_video = false;
  }

  return result;
}

/*
int main ()
{
  Graph *setting = (Graph *) calloc (1, sizeof (Graph));
  Graph *setting_1 = (Graph *) calloc (1, sizeof (Graph));
  Graph *setting_2 = (Graph *) calloc (1, sizeof (Graph));
  Graph *setting_3 = (Graph *) calloc (1, sizeof (Graph));
  
  Schedule *result, *result_1, *result_2;
  int cut = 55;

  setting->drone = 15;
  setting->rate[0] = 1.06;
  setting->rate[1] = 0.26;
  setting->rate[2] = 1.06;
  setting->rate[3] = 2.13;
  setting->rate[4] = 0.525;
  setting->rate[5] = 3.2;
  setting->rate[6] = 2.84;
  setting->rate[7] = 0.26;
  setting->rate[8] = 0.975;
  setting->rate[9] = 0.525;
  setting->rate[10] = 1.06;
  setting->rate[11] = 0.395;
  setting->rate[12] = 2.13;
  setting->rate[13] = 0.71;
  setting->rate[14] = 1.06;

  for (int i = 0; i < setting->drone; i++)
  {
    setting->video[i] = 1;
    setting->audio[i] = 1;
  }

  //memcpy (setting_1, setting, sizeof (Graph));
  //memcpy (setting_2, setting, sizeof (Graph));
  
  for (int i = 1; i <= 15; i++)
  {
    memcpy (setting_1, setting, sizeof (Graph));
    memcpy (setting_2, setting, sizeof (Graph));
    memcpy (setting_3, setting, sizeof (Graph));
    setting_1->drone = i;
    setting_2->drone = i;
    setting_3->drone = i;
    printf ("drone num : %d\n", i);
    result = round_robin (setting_1);
    result_1 = evenly_distribute (setting_2);
    result_2 = modified_RR (setting_3);
    free (result);
    free (result_1);
    free (result_2);
  }

  printf ("\n\nRR Algorithm\n");
  Schedule *result = round_robin (setting);
  for (int i = 0; i < SUBFRAME; i++)
  {
    printf ("subframe %d :\n", i+1);
    for (int j = 0; j < 4; j++)
    {
      for (int k = 0; k < cut; k++)
        printf ("%2d", result->data[i][(cut * j) + k]);
      printf ("\n");
    }
  }
  free (setting);
  free (result);

  printf ("\n\nED Algorithm\n");
  Schedule *result_1 = evenly_distribute (setting_1);
  for (int i = 0; i < SUBFRAME; i++)
  {
    printf ("subframe %d :\n", i+1);
    for (int j = 0; j < 4; j++)
    {
      for (int k = 0; k < cut; k++)
        printf ("%2d", result_1->data[i][(cut * j) + k]);
      printf ("\n");
    }
  }
  free (setting_1);
  free (result_1);
  
  printf ("\n\nMRR Algorithm\n");
  Schedule *result_2 = modified_RR (setting_2);
  for (int i = 0; i < SUBFRAME; i++)
  {
    printf ("subframe %d :\n", i+1);
    for (int j = 0; j < 4; j++)
    {
      for (int k = 0; k < cut; k++)
        printf ("%2d", result_2->data[i][(cut * j) + k]);
      printf ("\n");
    }
  }
  free (setting_2);
  free (result_2);
}
  */




