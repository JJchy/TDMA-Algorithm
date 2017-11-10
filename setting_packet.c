/* TDMA Scheduling function
 * by CHY                    */

#include "setting.h"

int find_packet_area (double packet_time, int left_slot)
{
  if (left_slot <= 5) return 0;
  double left_time = (PILOT_CHECK(left_slot)) * (((double)SUBFRAME_SIZE)/MINISLOT);
  return floor (left_time / packet_time);
}

int **round_robin (Graph *setting)
{
  int subframe = ceil (SUBFRAME * setting->buffering);

  int **result = (int **) calloc (subframe, sizeof (int *));
  for (int i = 0; i < subframe; i++)
    result[i] = (int *) calloc (MINISLOT, sizeof (int));

  //TC
  for (int i = 0; i < 4; i++)
    for (int j = 1; j <= TC_SLOT_PACKET; j++)
      result[i][MINISLOT - j] = 16;

  //TM
  double packet_time;
  int tm_packet_num, tm_packet_slot, able_packet_num;
  int tm_location = 0, tm_subframe = 0;
  int temp;
  for (int i = 1; i <= setting->drone; i++)
  {
    tm_packet_num = PACKET_NUM(TM_SIZE);
    packet_time = PACKET_SIZE / setting->rate[i-1];
    
    if (tm_subframe >= 4)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", i);
      printf ("NO MORE SLOT\n");
      for (int j = 1; j < subframe / 4; j++)
        for (int k = 0; k < 4; k++)
          memcpy (result[4 * j + k], result[k], (sizeof (int) * MINISLOT));
      return result;
    }

    temp = ((3 - tm_subframe) *\
            find_packet_area (packet_time, WITHOUT_TC_PACKET)) +\
           find_packet_area (packet_time, WITHOUT_TC_PACKET - tm_location);
    if (temp < tm_packet_num)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", i);
      printf ("%d(th/st/nd/rd) TM DATA is TOO big\n", i); 
      for (int j = 1; j < subframe / 4; j++)
        for (int k = 0; k < 4; k++)
          memcpy (result[4 * j + k], result[k], (sizeof (int) * MINISLOT));
      return result;
    }

    while (tm_packet_num != 0)
    {
      assert (tm_subframe < 4);
      able_packet_num = find_packet_area (packet_time,\
                                          WITHOUT_TC_PACKET - tm_location);
      if (able_packet_num == 0) 
      {
        tm_subframe++;
        tm_location = 0;
        continue;
      }

      able_packet_num = MIN(able_packet_num, tm_packet_num);
      tm_packet_slot = PILOT(ceil((able_packet_num * packet_time) /\
                                  ((double) SUBFRAME_SIZE / MINISLOT)));
      for (int j = 0; j < tm_packet_slot; j++)
        result[tm_subframe][tm_location + j] = i;
      
      tm_location += tm_packet_slot;
      tm_packet_num -= able_packet_num;
      assert (tm_location <= WITHOUT_TC_PACKET);
    }
  }

  for (int j = 1; j < subframe / 4; j++)
    for (int k = 0; k < 4; k++)
      memcpy (result[4 * j + k], result[k], (sizeof (int) * MINISLOT));

  // Ready to put media
  int remain_data[subframe];
  for (int i = 0; i < 4; i++)
  {
    if (i < tm_subframe)       remain_data[i] = 0;
    else if (i == tm_subframe) remain_data[i] = WITHOUT_TC_PACKET - tm_location;
    else                       remain_data[i] = WITHOUT_TC_PACKET;
    
    if (remain_data[i] <= 5) remain_data[i] = 0;
  }  
  
  for (int i = 0; i < subframe / 4; i++)
    memcpy (((void *) remain_data) + (sizeof (int) * 4 * i),\
            remain_data, (sizeof (int) * 4));

  // Media
  int subframe_slot = 0;
  int temp_location;
  double media_time;
  int media_packet_num, media_packet_slot;
  int media_size, media_slot;
  bool current_video, current_preexist, media_preexist;

  if (setting->video_first == true) current_video = true;
  else current_video = false;
 
  current_preexist = true;

  while (true)
  {
    for (int i = 1; i <= setting->drone; i++)
    {
      if (current_video == true) 
      {
        if (!setting->video[i-1])
          continue;

        if (current_preexist)
          if (!setting->video_preexist[i-1])
            continue;

        setting->video[i-1] = false;
        media_preexist = setting->video_preexist[i-1];
        media_size = VIDEO_SIZE * setting->buffering;
      }
      else  
      {
        if (!setting->audio[i-1])
          continue;
        
        if (current_preexist)
          if (!setting->audio_preexist[i-1])
            continue;
        
        setting->audio[i-1] = false;
        media_preexist = setting->audio_preexist[i-1];
        media_size = AUDIO_SIZE * setting->buffering;
      }

      media_packet_num = PACKET_NUM (media_size);
      packet_time = PACKET_SIZE / setting->rate[i-1];

      if (subframe_slot >= subframe)
      {
        if (current_video == true)
          printf ("DROP : %d(th/st/nd/rd)'s VIDEO is drop\n", i);
        else
          printf ("DROP : %d(th/st/nd/rd)'s AUDIO is drop\n", i);
        printf ("NO MORE SLOT\n");
        return result;
      }

      temp = 0;
      for (int j = subframe_slot; j < subframe; j++)
        temp += find_packet_area (packet_time, remain_data[j]);

      if (temp < media_packet_num)
      {
        if (current_video == true)
          printf ("DROP : %d(th/st/nd/rd)'s VIDEO is drop\n", i);
        else
          printf ("DROP : %d(th/st/nd/rd)'s AUDIO is drop\n", i);
        printf ("The DATA is TOO big\n");  
        return result;
      }

      while (media_packet_num != 0)
      {
        assert (subframe_slot < subframe);
        able_packet_num = find_packet_area (packet_time,\
                                            remain_data[subframe_slot]);
        if (able_packet_num == 0)
        {
          subframe_slot++;
          continue;
        }

        able_packet_num = MIN(able_packet_num, media_packet_num);
        media_packet_slot = PILOT(ceil((able_packet_num * packet_time) /\
                                       ((double) SUBFRAME_SIZE / MINISLOT)));
        temp_location = SLOT_LOCATION_PACKET(remain_data[subframe_slot]);
        for (int j = 0; j < media_packet_slot; j++)
          result[subframe_slot][temp_location + j] = i;

        remain_data[subframe_slot] -= media_packet_slot;
        media_packet_num -= able_packet_num;
        assert (remain_data[subframe_slot] >= 0);
      }
    }
    
    if ((!setting->preexist_first) && current_preexist)
      current_preexist = false;

    else if ((setting->preexist_first) && (current_video && setting->video_first))
      current_video = (!current_video);
    
    else
    {
      current_video = (!current_video);
      current_preexist = (!current_preexist);
    }

    if ((current_video == setting->video_first) &&\
        (current_preexist == true))
      break;
    
  }
  return result;
}

int **modified_RR (Graph *setting)
{
  int subframe = ceil (SUBFRAME * setting->buffering);

  int **result = (int **) calloc (subframe, sizeof (int *));
  for (int i = 0; i < subframe; i++)
    result[i] = (int *) calloc (MINISLOT, sizeof (int));
 
  //TC
  for (int i = 0; i < 4; i++)
    for (int j = 1; j <= TC_SLOT_PACKET; j++)
      result[i][MINISLOT - j] = 16; // 16 : GCS -> Drone (Broadcast)

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
  double packet_time;
  int tm_packet_num, tm_packet_slot, able_packet_num;
  int tm_location = 0, tm_subframe = 0;
  int temp;
  
  temp_list = Priority_list;
  for (int i = 1; i <= setting->drone; i++)
  {
    priority_num = temp_list->drone_num;  

    tm_packet_num = PACKET_NUM (TM_SIZE);
    packet_time = PACKET_SIZE / setting->rate[priority_num-1];

    if (tm_subframe >= 4)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", priority_num);
      printf ("(Success : %d)\n", i - 1);
      printf ("NO MORE SLOT\n");
      for (int j = 1; j < subframe / 4; j++)
        for (int k = 0; k < 4; k++)
          memcpy (result[4 * j + k], result[k], (sizeof (int) * MINISLOT));
      return result;
    }

    temp = ((3 - tm_subframe) *\
            find_packet_area (packet_time, WITHOUT_TC_PACKET)) +\
           find_packet_area (packet_time, WITHOUT_TC_PACKET - tm_location);
    if (temp < tm_packet_num)
    {
      printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", priority_num);
      printf ("%d(th/st/nd/rd) TM DATA is TOO big\n", priority_num);
      printf ("(Success : %d)\n", i - 1);
      for (int j = 1; j < subframe / 4; j++)
        for (int k = 0; k < 4; k++)
          memcpy (result[4 * j + k], result[k], (sizeof (int) * MINISLOT));
      return result;
    }

    while (tm_packet_num != 0)
    {
      assert (tm_subframe < 4);
      able_packet_num = find_packet_area (packet_time,\
                                          WITHOUT_TC_PACKET - tm_location);
      if (able_packet_num == 0) 
      {
        tm_subframe++;
        tm_location = 0;
        continue;
      }

      able_packet_num = MIN(able_packet_num, tm_packet_num);
      tm_packet_slot = PILOT(ceil((able_packet_num * packet_time) /\
                                  ((double) SUBFRAME_SIZE / MINISLOT)));
      for (int j = 0; j < tm_packet_slot; j++)
        result[tm_subframe][tm_location + j] = priority_num;
      
      tm_location += tm_packet_slot;
      tm_packet_num -= able_packet_num;
      assert (tm_location <= WITHOUT_TC_PACKET);
    }

    temp_list = temp_list->next;
  }

  for (int j = 1; j < subframe / 4; j++)
    for (int k = 0; k < 4; k++)
      memcpy (result[4 * j + k], result[k], (sizeof (int) * MINISLOT));

  // Ready to put media
  int remain_data[subframe];
  for (int i = 0; i < 4; i++)
  {
    if (i < tm_subframe)       remain_data[i] = 0;
    else if (i == tm_subframe) remain_data[i] = WITHOUT_TC_PACKET - tm_location;
    else                       remain_data[i] = WITHOUT_TC_PACKET;
    
    if (remain_data[i] <= 5) remain_data[i] = 0;
  }  
  
  for (int i = 0; i < subframe / 4; i++)
    memcpy (((void *) remain_data) + (sizeof (int) * 4 * i),\
            remain_data, (sizeof (int) * 4));

  //

  // Media
  int subframe_slot = 0;
  int temp_location;
  double media_time;
  int media_packet_num, media_packet_slot;
  int media_size, media_slot;
  bool is_video = true;
  bool current_video, current_preexist, media_preexist;

  if (setting->video_first == true) current_video = true;
  else current_video = false;
  
  current_preexist = true;
  
  while (true)
  {
    temp_list = Priority_list;
    for (int i = 1; i <= setting->drone; i++)
    {
      priority_num = temp_list->drone_num;

      if (current_video == true) 
      {
        if (!setting->video[priority_num - 1])
          continue;
        
        if (current_preexist)
          if (!setting->video_preexist[i-1])
            continue;
        
        setting->video[priority_num - 1] = false;
        media_preexist = setting->video_preexist[priority_num - 1];
        media_size = VIDEO_SIZE * setting->buffering;
      }
      else  
      {
        if (!setting->audio[priority_num - 1])
          continue;
        
        if (current_preexist)
          if (!setting->audio_preexist[i-1])
            continue;
        
        setting->audio[priority_num - 1] = false;
        media_preexist = setting->audio_preexist[priority_num - 1];
        media_size = AUDIO_SIZE * setting->buffering;
      }

      media_packet_num = PACKET_NUM (media_size);
      packet_time = PACKET_SIZE / setting->rate[priority_num - 1];

      if (subframe_slot >= subframe)
      {
        if (current_video == true)
          printf ("DROP : %d(th/st/nd/rd)'s VIDEO is drop\n",\
                    priority_num);
        else
          printf ("DROP : %d(th/st/nd/rd)'s AUDIO is drop\n",\
                  priority_num);
        printf ("(Success : %d)\n", i-1);
        printf ("NO MORE SLOT\n");
        return result;
      }

      temp = 0;
      for (int j = subframe_slot; j < subframe; j++)
        temp += find_packet_area (packet_time, remain_data[j]);

      if (temp < media_packet_num)
      {
        if (current_video == true)
          printf ("DROP : %d(th/st/nd/rd)'s VIDEO is drop\n",\
                  priority_num);
        else
          printf ("DROP : %d(th/st/nd/rd)'s AUDIO is drop\n",\
                    priority_num);
        printf ("(Success : %d)\n", i-1);
        printf ("The DATA is TOO big\n");  
        return result;
      }

      while (media_packet_num != 0)
      {
        assert (subframe_slot < subframe);
        able_packet_num = find_packet_area (packet_time,\
                                             remain_data[subframe_slot]);
        if (able_packet_num == 0)
        {
          subframe_slot++;
          continue;
        }

        able_packet_num = MIN(able_packet_num, media_packet_num);
        media_packet_slot = PILOT(ceil((able_packet_num * packet_time) /\
                                       ((double) SUBFRAME_SIZE / MINISLOT)));
        temp_location = SLOT_LOCATION_PACKET(remain_data[subframe_slot]);
        for (int j = 0; j < media_packet_slot; j++)
          result[subframe_slot][temp_location + j] = priority_num;

        remain_data[subframe_slot] -= media_packet_slot;
        media_packet_num -= able_packet_num;
        assert (remain_data[subframe_slot] >= 0);
      }
      
      temp_list = temp_list->next;
    }

    if ((!setting->preexist_first) && current_preexist)
      current_preexist = false;

    else if ((setting->preexist_first) && (current_video && setting->video_first))
      current_video = (!current_video);
    
    else
    {
      current_video = (!current_video);
      current_preexist = (!current_preexist);
    }
    
    if ((current_video == setting->video_first) &&\
        (current_preexist == true))
      break;
  }

  return result;
}

int main (int argc, char **argv)
{
  Graph *setting = (Graph *) calloc (1, sizeof (Graph));
 
  double data_rate[13] = {4.265, 3.2, 2.84, 2.13, 1.595, 1.42, 1.06, 0.975, 0.71, 0.525, 0.395, 0.35, 0.26};
  
  setting->drone = 5;
  setting->rate[0] = data_rate [2];
  setting->rate[1] = data_rate [2];
  setting->rate[2] = data_rate [2];
  setting->rate[3] = data_rate [2];
  setting->rate[4] = data_rate [2];

  setting->buffering = 1;

  for (int i = 0; i < setting->drone; i++)
  {
    setting->video[i] = true;
    setting->audio[i] = true;
    setting->audio_preexist [i] = true;
  }

  setting->video_first = true;
  setting->preexist_first = false;
  
  if (argc != 1)
  {
    setting->drone = atoi (argv[1]);
    setting->video[0] = (argv[4][0] == 't') ? true : false;
    setting->audio[0] = (argv[4][1] == 't') ? true : false;
    setting->buffering = atof (argv[3]);
    setting->video_first = (argv[5][0] == 't') ? true : false;
    
    for (int i = 0; i < setting->drone; i++)
    {
      setting->rate[i] = data_rate[atoi (argv[2]) - 1];
      if (i == 0) continue;
      setting->video[i] = setting->video[i - 1];
      setting->audio[i] = setting->audio[i - 1];
    }
  }

  printf ("MRR Algorithm\n");
  //int **result = modified_RR (setting);
  int **result = round_robin (setting);
  int index = 0;
  for (int i = 0; i < ceil (SUBFRAME * setting->buffering); i++)
  {
    if (i % 24 == 23) index++;
    
    printf ("subframe %d :\n", index + 1);
    for (int j = 0; j < 4; j++)
    {
      for (int k = 0; k < 55; k++)
        printf ("%2d", result[i][(55 * j) + k]);
      printf ("\n");
    }
    index++;
  }
  free (setting);
  free (result);
}




