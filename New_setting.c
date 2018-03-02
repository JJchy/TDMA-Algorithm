/* TDMA Scheduling function
 * by CHY                    */

#include "New_setting.h"

int find_packet_area (double packet_time, int left_slot)
{
  if (left_slot <= 5) return 0;
  double left_time = (PILOT_CHECK(left_slot)) * (((double)SUBFRAME_SIZE)/MINISLOT);
  return floor (left_time / packet_time);
}

int **MRR (Graph *setting, Media_Result *mresult)
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

  int subframe_slot = 0;
  int temp_location;
  double media_time, reply_packet_time;
  int media_packet_num, media_packet_slot, reply_packet_num = 0;
  int media_size, media_slot;
  bool current_video, current_preexist, media_preexist;
  int temp_data;

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
        {
          temp_list = temp_list->next;
          continue;
        }
        
        if (current_preexist)
          if (!setting->video_preexist[priority_num-1])
          {
            temp_list = temp_list->next;
            continue;
          }
        
        setting->video[priority_num - 1] = false;
        media_preexist = setting->video_preexist[priority_num - 1];
        if (setting->buffering == 0.16) 
          media_size = ceil (VIDEO_SIZE / 6);
        else
          media_size = ceil (VIDEO_SIZE * setting->buffering);
      }
      else  
      {
        if (!setting->audio[priority_num - 1])
        {
          temp_list = temp_list->next;
          continue;
        }
        
        if (current_preexist)
          if (!setting->audio_preexist[priority_num-1])
          {
            temp_list = temp_list->next;
            continue;
          }
        
        setting->audio[priority_num - 1] = false;
        media_preexist = setting->audio_preexist[priority_num - 1];
        if (setting->buffering == 0.16) 
          media_size = ceil (AUDIO_SIZE / 6);
        else
          media_size = ceil (AUDIO_SIZE * setting->buffering);
      }

      media_packet_num = PACKET_NUM (media_size);
      packet_time = PACKET_SIZE / setting->rate[priority_num - 1];

      if (!current_video)
      {
        reply_packet_num = PACKET_NUM (media_size);
        reply_packet_time = PACKET_SIZE / 4.265;
      }
        
      if (subframe_slot >= subframe)
      {
        if (current_video == true)
          printf ("DROP : %d(th/st/nd/rd)'s VIDEO is drop\n",\
                    priority_num);
        else
          printf ("DROP : %d(th/st/nd/rd)'s AUDIO is drop\n",\
                  priority_num);
        printf ("NO MORE SLOT\n");
        temp_list = temp_list->next;
        continue;
      }

      int remain = media_packet_num;
      int reply_remain = reply_packet_num;
      for (int j = subframe_slot; j < subframe; j++)
      {
        if (remain > 0)
        {
          temp = find_packet_area (packet_time, remain_data[j]);
          remain -= temp;
          if ((!current_video) && (remain <= 0))
          {
            temp_data = remain_data[j] - PILOT(ceil(((temp + remain) * packet_time) /\
                                                   ((double) SUBFRAME_SIZE / MINISLOT)));
            temp = find_packet_area (reply_packet_time, temp_data);
            reply_remain -= temp;
            continue;
          }
        }
        if ((!current_video) && (remain <= 0))
        {
          temp = find_packet_area (reply_packet_time, remain_data[j]);
          reply_remain -= temp;
        }
      }

      if ((remain > 0) || (reply_remain > 0))
      {
        if (current_video == true)
          printf ("DROP : %d(th/st/nd/rd)'s VIDEO is drop\n",\
                  priority_num);
        else
          printf ("DROP : %d(th/st/nd/rd)'s AUDIO is drop\n",\
                    priority_num);
        printf ("The DATA is TOO big\n");  
        temp_list = temp_list->next;
        continue;
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

      if (!current_video)
      {
        while (reply_packet_num != 0)
        {
          assert (subframe_slot < subframe);
          able_packet_num = find_packet_area (reply_packet_time,\
                                              remain_data[subframe_slot]);
          if (able_packet_num == 0)
          {
            subframe_slot++;
            continue;
          }

          able_packet_num = MIN(able_packet_num, reply_packet_num);
          media_packet_slot = PILOT(ceil((able_packet_num * reply_packet_time) /\
                                         ((double) SUBFRAME_SIZE / MINISLOT)));
          temp_location = SLOT_LOCATION_PACKET(remain_data[subframe_slot]);
          for (int j = 0; j < media_packet_slot; j++)
            result[subframe_slot][temp_location + j] = 16;

          remain_data[subframe_slot] -= media_packet_slot;
          reply_packet_num -= able_packet_num;
          assert (remain_data[subframe_slot] >= 0);
        }
      }
      
      if (current_video)
        mresult->video[priority_num - 1] = true;
      else
        mresult->audio[priority_num - 1] = true;

      temp_list = temp_list->next;
    }

    if ((!setting->preexist_first) && current_preexist)
      current_preexist = false;

    else if ((setting->preexist_first) && (current_video == setting->video_first))
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

#ifdef SIMUL
int main ()
{
  Graph *setting = (Graph *) calloc (1, sizeof (Graph));
  Media_Result *mresult = (Media_Result *) calloc (1, sizeof (Media_Result));
  double data_rate[13] = {4.265, 3.2, 2.84, 2.13, 1.595, 1.42, 1.06, 0.975, 0.71, 0.525, 0.395, 0.35, 0.26};
  
  int rate = 9;
  setting->drone = 7;
  setting->buffering = 1;
  
  for (int i = 0; i < setting->drone; i++)
  {
    setting->rate[i] = data_rate [rate];
  }

  /*
  setting->rate[0] = data_rate [6];
  setting->rate[1] = data_rate [3];
  setting->rate[2] = data_rate [5];
  setting->rate[3] = data_rate [12];
  setting->rate[4] = data_rate [2];
  setting->rate[5] = data_rate [9];
  setting->rate[6] = data_rate [7];
  */

  setting->video_first = false;
  setting->preexist_first = true;

  bool isvideo[setting->drone];
  bool isaudio[setting->drone];
  int con_time_video[setting->drone];
  int con_time_audio[setting->drone];
  int num_video[setting->drone];
  int num_audio[setting->drone];
  int preexist_video[setting->drone];
  int preexist_audio[setting->drone];
  int fall_video[setting->drone];
  int fall_audio[setting->drone];
  int preexist_fall_video[setting->drone];
  int preexist_fall_audio[setting->drone];

  for (int i = 0; i < setting->drone; i++)
  {
    isvideo[i] = false;
    isaudio[i] = false;
    con_time_video[i] = 0;
    con_time_audio[i] = 0;
    num_video[i] = 0;
    num_audio[i] = 0;
    preexist_video[i] = 0;
    preexist_audio[i] = 0;
    fall_video[i] = 0;
    fall_audio[i] = 0;
    preexist_fall_video[i] = 0;
    preexist_fall_audio[i] = 0;
  }

  srand (time (NULL));
  for (int i = 0; i < 100000; i++)
  {
    for (int j = 0; j < setting->drone; j++)
    {
      if (isvideo[j])
      {
        if (((float) rand() / RAND_MAX) < 1 - pow((1 - 0.5), ((con_time_video[j]++) + 1)))
        {
          isvideo[j] = false;
          con_time_video[j] = 0;
          setting->video[j] = false;
        }

        else
        {
          setting->video[j] = true;
          setting->video_preexist[j] = true;
          num_video[j]++;
          preexist_video[j]++;
        }
      }
      else
      {
        if (((float) rand() / RAND_MAX) < 1 - pow((1 - 0.01), ((con_time_video[j]++) + 1)))
        {
          isvideo[j] = true;
          con_time_video[j] = 0;
          setting->video[j] = true;
          setting->video_preexist[j] = false;
          num_video[j]++;
        }

        else
          setting->video[j] = false;
      }

      if (isaudio[j])
      {
        if (((float) rand() / RAND_MAX) < 1 - pow((1 - 0.5), ((con_time_audio[j]++) + 1)))
        {
          isaudio[j] = false;
          con_time_audio[j] = 0;
          setting->audio[j] = false;
        }

        else
        {
          setting->audio[j] = true;
          setting->audio_preexist[j] = true;
          num_audio[j]++;
          preexist_audio[j]++;
        }
      }
      else
      {
        if (((float) rand() / RAND_MAX) < 1 - pow((1 - 0.5), ((con_time_audio[j]++) + 1)))
        {
          isaudio[j] = true;
          con_time_audio[j] = 0;
          setting->audio[j] = true;
          setting->audio_preexist[j] = false;
          num_audio[j]++;
        }

        else
          setting->audio[j] = false;
      }
    }

    MRR (setting, mresult);
      
    for (int j = 0; j < setting->drone; j++)
    {
      if ((isvideo[j]) && (!mresult->video[j]))
      {
        if (con_time_video[j] > 0)
        {
          preexist_fall_video[j]++;
          isvideo[j] = false;
          con_time_video[j] = 0;
        }
        fall_video[j]++;
      }


      if ((isaudio[j]) && (!mresult->audio[j]))
      {
        if (con_time_audio[j] > 0)
        {
          preexist_fall_audio[j]++;
          isaudio[j] = false;
          con_time_audio[j] = 0;
        }
        fall_audio[j]++;
      }
    }

    memset (mresult, 0, sizeof (Media_Result));
  }

  int total_video = 0;
  int total_audio = 0;
  int total_preexist_video = 0;
  int total_preexist_audio = 0;
  int total_fall_video = 0;
  int total_fall_audio = 0;
  int total_preexist_fall_video = 0;
  int total_preexist_fall_audio = 0;
  
  for (int i = 0; i < setting->drone; i++)
  {
    total_video += num_video[i];
    total_audio += num_audio[i];
    total_preexist_video += preexist_video[i];
    total_preexist_audio += preexist_audio[i];
    total_fall_video += fall_video[i];
    total_fall_audio += fall_audio[i];
    total_preexist_fall_video += preexist_fall_video[i];
    total_preexist_fall_audio += preexist_fall_audio[i];
  }

  printf ("Video - Success Preexist: %f, New: %f\n", \
      1 - ((float)total_preexist_fall_video) / total_preexist_video, \
      1 - ((float)(total_fall_video - total_preexist_fall_video)) / (total_video - total_preexist_video)); 
  printf ("Audio - Success Preexist: %f, New: %f\n", \
      1 - ((float)total_preexist_fall_audio) / total_preexist_audio, \
      1 - ((float)(total_fall_audio - total_preexist_fall_audio)) / (total_audio - total_preexist_audio)); 

}







#else
int main (int argc, char **argv)
{
  Graph *setting = (Graph *) calloc (1, sizeof (Graph));
  Media_Result *mresult = (Media_Result *) calloc (1, sizeof (Media_Result));
 
  double data_rate[13] = {4.265, 3.2, 2.84, 2.13, 1.595, 1.42, 1.06, 0.975, 0.71, 0.525, 0.395, 0.35, 0.26};
 
  setting->drone = 7;
  int rate = 3;
  setting->buffering = 0.16;

  setting->video [0] = true;
  setting->video [1] = true;
  setting->audio [2] = true;
  setting->audio [3] = true;
  setting->audio [4] = true;
  setting->audio [5] = true;

  /*
  for (int i = 0; i < setting->drone; i++)
  {
    setting->rate[i] = data_rate [rate];
  } 
  */
  
  setting->rate[0] = data_rate [6];
  setting->rate[1] = data_rate [3];
  setting->rate[2] = data_rate [5];
  setting->rate[3] = data_rate [12];
  setting->rate[4] = data_rate [2];
  setting->rate[5] = data_rate [9];
  setting->rate[6] = data_rate [7];

  setting->video_first = false;
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
  //int **result = round_robin (setting);
  int **result = MRR (setting, mresult);

  /*
  int index = 0;
  for (int i = 0; i < ceil (SUBFRAME * setting->buffering); i++)
  {    
    if (i % 24 == 0) 
      if (i != 0)
        index++;
    
    printf ("subframe %d :\n", index + 1);
    for (int j = 0; j < 4; j++)
    {
      for (int k = 0; k < 55; k++)
        printf ("%2d", result[i][(55 * j) + k]);
      printf ("\n");
    }
    index++;
  }
  */
  
  free (setting);
  free (result);
  free (mresult);
}
#endif


