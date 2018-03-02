/* TDMA Scheduling function
 * by CHY                    */

#include "2_hop_setting.h"

int find_packet_area (double packet_time, int left_slot)
{
  if (left_slot <= 5) return 0;
  double left_time = (PILOT_CHECK(left_slot)) * (((double)SUBFRAME_SIZE)/MINISLOT);
  return floor (left_time / packet_time);
}

int whoisparent (Graph *setting, int node)
{
  int location;
  for (int i = 0; i < setting->parent_num; i++)
  {
    location = 0;
    while (setting->children[i][location] != 0)
      if (node == setting->children[i][location++])
        return setting->parent[i];
  }
  return 16;
}

int *wherecaniuse (ULink *list, double packet_time, int parent)
{
  int *result = (int *) calloc (1, sizeof (int));

  int size = 0, temp = 0, start = 0, previous_end = 0, filter;
  while (list != NULL)
  {
    filter = (parent == 16) ? 0xFFFF : SET_X (parent);
    if ((list->usage & filter) != 0)
    {
      temp = find_packet_area (packet_time, size);
      if (temp != 0)
      {
        result = (int *) realloc ((void *) result, sizeof (int) * (*result + 2));
        result[*result * 2 + 1] = temp;
        result[*result * 2 + 2] = start;
        (*result)++;
      }
      size = 0;

    }
    else
    {
      if (size == 0)
        start = list->start;

      previous_end = list->end + 1;
      size += list->end + 1 - list->start;
    }
    list = list->next;
  }

  if (size != 0)
  {
    temp = find_packet_area (packet_time, size);
    if (temp != 0)
    {
      result = (int *) realloc ((void *) result, sizeof (int) * (*result + 2));
      result[*result * 2 + 1] = temp;
      result[*result * 2 + 2] = start;
      (*result)++;
    }
  }

  return result;
}

void change_usage_status (ULink *list, int start, int size, int parent)
{
  int temp;
  ULink *new_list;
  while (list != NULL)
  {
    if (list->start == start)
      break;
    list = list->next;
  }
  if (list == NULL) printf ("IMPOSSIBLE\n");

  while ((list != NULL) && (list->start < start + size))
  {
    assert ((list->usage & SET_X(parent)) == 0);
    if (list->end > start + size - 1)
    {
      temp = list->end;
      list->end = start + size - 1;
      new_list = (ULink *) calloc (1, sizeof (ULink));
      new_list->usage = list->usage;
      new_list->start = start + size;
      new_list->end = temp;
      new_list->next = list->next;
      list->next = new_list;
    }

    list->usage |= (parent == 16) ? 0xffff : SET_X(parent);
    list = list->next;
  }
}

void copy_usage_status (ULink *source, ULink *target)
{
  ULink *temp, *previous;
  memcpy (target, source, sizeof (ULink));

  source = source->next;
  previous = target;
  while (source != NULL)
  { 
    temp = (ULink *) calloc (1, sizeof (ULink));
    memcpy (temp, source, sizeof (ULink));
    previous->next = temp;

    source = source->next;
    previous = temp;
  }
}

int **MRR (Graph *setting, Media_Result *mresult)
{
  int subframe = ceil (SUBFRAME * setting->buffering);
    
  int **result = (int **) calloc (subframe, sizeof (int *));
  ULink *usage_list[subframe];
  for (int i = 0; i < subframe; i++)
  {
    result[i] = (int *) calloc (MINISLOT, sizeof (int));
    usage_list[i] = (ULink *) calloc (1, sizeof (ULink));
  }
 
  //TC
  double min_rate = 4.265;
  for (int i = 0; i < setting->parent_num; i++)
    if (min_rate > setting->rate[setting->parent[i]-1])
      min_rate = setting->rate[setting->parent[i]-1];

  int TC_SLOT_parent = TC_SLOT_PACKET(min_rate);
  
  for (int i = 0; i < 4; i++)
    for (int j = 1; j <= TC_SLOT_PACKET_GCS; j++)
      result[i][MINISLOT - j - TC_SLOT_parent] = SET_X(16); // 16 : GCS -> Drone (Broadcast)

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < setting->parent_num; j++)
      for (int k = 0; k < TC_SLOT_PACKET(setting->rate[setting->parent[j]-1]); k++)
        result[i][MINISLOT - TC_SLOT_parent + k] |= SET_X(setting->parent[j]);

  for (int i = 0; i < 4; i++)
    usage_list[i]->end = WITHOUT_TC_PACKET(min_rate) - 1;

  //Link
  Link *temp_list, *before_list = NULL;
  int temp_parent;
  double temp_rate, temp_list_rate;
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
      temp_parent = whoisparent (setting, i+1);
      if (temp_parent == 16)
        temp_rate = setting->rate[i];
      else
        temp_rate = (setting->rate[i] * setting->rate[temp_parent - 1]) /\
                    (setting->rate[i] + setting->rate[temp_parent - 1]) / 2;
      
      temp_parent = whoisparent (setting, j->drone_num);
      if (temp_parent == 16)
        temp_list_rate = setting->rate[j->drone_num - 1];
      else
        temp_list_rate = (setting->rate[j->drone_num - 1] * setting->rate[temp_parent - 1]) /\
                         (setting->rate[j->drone_num - 1] + setting->rate[temp_parent - 1]) / 2;

      if (temp_list_rate < temp_rate)
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
  int tm_subframe = 0;
  int *able_packet_status;
  int temp;
  
  int success = 0;
  for (int l = 0; l < 2; l++)
  {
    temp_list = Priority_list;
    for (int i = 1; i <= setting->drone; i++)
    {
      priority_num = temp_list->drone_num;  

      if (l == 0) 
      {
        if (whoisparent (setting, priority_num) == 16)
        {
          temp_list = temp_list->next;
          continue;
        }
      }
      else
      {
        if (whoisparent (setting, priority_num) != 16)
        {
          temp_list = temp_list->next;
          continue;
        }
      }


      tm_packet_num = PACKET_NUM (TM_SIZE);
      packet_time = PACKET_SIZE / setting->rate[priority_num-1];

      temp = 0;
      for (int j = 0; j < 4; j++)
      {
        able_packet_status = wherecaniuse (usage_list[j], packet_time,\
                                           whoisparent (setting, priority_num));
        for (int k = 0; k < *able_packet_status; k++)
          temp += able_packet_status [k * 2 + 1];
        free (able_packet_status);
      }

      if (temp < tm_packet_num)
      {
        printf ("DROP : %d(th/st/nd/rd) TM DATA is drop\n", priority_num);
        printf ("%d(th/st/nd/rd) TM DATA is TOO big\n", priority_num);
        printf ("(Success : %d)\n", success);
        for (int j = 1; j < subframe / 4; j++)
          for (int k = 0; k < 4; k++)
            memcpy (result[4 * j + k], result[k], (sizeof (int) * MINISLOT));
        return result;
      }

      tm_subframe = 0;
      while (tm_packet_num != 0)
      {
        assert (tm_subframe < 4);
        able_packet_status = wherecaniuse (usage_list[tm_subframe], packet_time,\
                                           whoisparent (setting, priority_num));
        if (*able_packet_status == 0)
        {
          tm_subframe++;
          free (able_packet_status);
          continue;
        }

        for (int j = 0; j < *able_packet_status; j++)
        {
          able_packet_num = MIN (able_packet_status[j * 2 + 1], tm_packet_num);
          tm_packet_slot = PILOT(ceil((able_packet_num * packet_time) /\
                                      ((double) SUBFRAME_SIZE / MINISLOT)));
          for (int k = 0; k < tm_packet_slot; k++)
            result[tm_subframe][able_packet_status[j * 2 + 2] + k] |= SET_X(priority_num);
     
          change_usage_status (usage_list[tm_subframe], able_packet_status[j * 2 + 2],\
                               tm_packet_slot, whoisparent (setting, priority_num));
          tm_packet_num -= able_packet_num;
        }
        free (able_packet_status);
      }
      success++;

      temp_list = temp_list->next;
    }
  }
 
  for (int j = 1; j < subframe / 4; j++)
    for (int k = 0; k < 4; k++)
    {
      memcpy (result[4 * j + k], result[k], (sizeof (int) * MINISLOT));
      copy_usage_status (usage_list[k], usage_list[4 * j + k]);
    }

  // Ready to put media
  int subframe_slot = 0;
  int temp_location;
  double media_time, reply_packet_time, parent_packet_time;
  int media_packet_num, media_packet_slot, reply_packet_num;
  int parent_packet_num, parent_reply_packet_num;
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
      parent_packet_num = 0; 
      parent_reply_packet_num = 0;
      reply_packet_num = 0;
      int parent_remain = 0, parent_reply_remain = 0;
      if (whoisparent (setting, priority_num) != 16)
      {
        parent_packet_time = PACKET_SIZE /\
                             setting->rate[whoisparent (setting, priority_num) - 1];
        parent_remain = media_packet_num;
        parent_packet_num = media_packet_num;
      }

      if (!current_video)
      {
        reply_packet_num = media_packet_num;
        reply_packet_time = PACKET_SIZE / 4.265;
        if (parent_remain != 0)
        {
          parent_reply_remain = media_packet_num;
          parent_reply_packet_num = media_packet_num;
        }
      }
     
      printf ("Priority_num : %d\n", priority_num);
      int remain = media_packet_num;
      int reply_remain = reply_packet_num;
      ULink *temp_usage, *temp_remove;
      bool isfinish_original = false, isfinish_parent = false, isfinish_reply = false;
      for (int j = 0; j < subframe; j++)
      {
        temp_usage = (ULink *) calloc (1, sizeof (ULink));
        copy_usage_status (usage_list[j], temp_usage);

        if (remain > 0)
        {
          able_packet_status = wherecaniuse (temp_usage, packet_time,\
                                             whoisparent (setting, priority_num));
          for (int k = 0; k < *able_packet_status; k++)
          {
            remain -= able_packet_status [k * 2 + 1];
            if (remain <= 0)
            {
              change_usage_status (temp_usage, able_packet_status [k * 2 + 2],\
                                   PILOT(ceil(((able_packet_status [k * 2 + 1] + remain) * packet_time) /\
                                              ((double) SUBFRAME_SIZE / MINISLOT))),\
                                   whoisparent (setting, priority_num));
              break;
            }
            
            change_usage_status (temp_usage, able_packet_status [k * 2 + 2],\
                                 PILOT(ceil((able_packet_status [k * 2 + 1] * packet_time) /\
                                            ((double) SUBFRAME_SIZE / MINISLOT))),\
                                 whoisparent (setting, priority_num));
          }

          free (able_packet_status);
        }

        if (remain <= 0)
          isfinish_original = true;

        if ((isfinish_original) && (parent_remain > 0))
        {
          able_packet_status = wherecaniuse (temp_usage, parent_packet_time, 16);
          
          for (int k = 0; k < *able_packet_status; k++)
          {
            parent_remain -= able_packet_status [k * 2 + 1];
            if (parent_remain <= 0)
            {
              change_usage_status (temp_usage, able_packet_status [k * 2 + 2],\
                                   PILOT(ceil(((able_packet_status [k * 2 + 1] + parent_remain) *\
                                               parent_packet_time) /\
                                              ((double) SUBFRAME_SIZE / MINISLOT))), 16);
              break;
            }
            
            change_usage_status (temp_usage, able_packet_status [k * 2 + 2],\
                                 PILOT(ceil((able_packet_status [k * 2 + 1] * parent_packet_time) /\
                                            ((double) SUBFRAME_SIZE / MINISLOT))), 16);
          }

          free (able_packet_status);
        }

        if ((remain <= 0) && (parent_remain <= 0))
          isfinish_parent = true;

        if ((isfinish_parent) && (reply_remain > 0))
        {
          able_packet_status = wherecaniuse (temp_usage, reply_packet_time, 16);
          
          for (int k = 0; k < *able_packet_status; k++)
          {
            reply_remain -= able_packet_status [k * 2 + 1];
            if (reply_remain <= 0)
            {
              change_usage_status (temp_usage, able_packet_status [k * 2 + 2],\
                                   PILOT(ceil(((able_packet_status [k * 2 + 1] + reply_remain) *\
                                               reply_packet_time) /\
                                              ((double) SUBFRAME_SIZE / MINISLOT))), 16);
              break;
            }
            
            change_usage_status (temp_usage, able_packet_status [k * 2 + 2],\
                                 PILOT(ceil((able_packet_status [k * 2 + 1] * reply_packet_time) /\
                                            ((double) SUBFRAME_SIZE / MINISLOT))), 16);
          }

          free (able_packet_status);
        }

        if (reply_remain <= 0)
          isfinish_reply = true;

        if ((isfinish_reply) && (parent_reply_remain > 0))
        {
          able_packet_status = wherecaniuse (temp_usage, parent_packet_time, 16);
          
          for (int k = 0; k < *able_packet_status; k++)
          {
            parent_reply_remain -= able_packet_status [k * 2 + 1];
            if (parent_reply_remain <= 0)
            {
              change_usage_status (temp_usage, able_packet_status [k * 2 + 2],\
                                   PILOT(ceil(((able_packet_status [k * 2 + 1] + parent_reply_remain) *\
                                               parent_packet_time) /\
                                              ((double) SUBFRAME_SIZE / MINISLOT))), 16);
              break;
            }
            
            change_usage_status (temp_usage, able_packet_status [k * 2 + 2],\
                                 PILOT(ceil((able_packet_status [k * 2 + 1] * parent_packet_time) /\
                                            ((double) SUBFRAME_SIZE / MINISLOT))), 16);
          }

          free (able_packet_status);
        }

        temp_remove = temp_usage;
        while (temp_remove != NULL)
        {
          temp_usage = temp_remove->next;
          free (temp_remove);
          temp_remove = temp_usage;
        }
      }

      if ((remain > 0) || (parent_remain > 0) ||\
          (reply_remain > 0) || (parent_reply_remain > 0))
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
 
      subframe_slot = 0;
      while (media_packet_num != 0)
      {
        assert (subframe_slot < subframe);
        able_packet_status = wherecaniuse (usage_list[subframe_slot], packet_time,\
                                           whoisparent (setting, priority_num));
        if (*able_packet_status == 0)
        {
          subframe_slot++;
          free (able_packet_status);
          continue;
        }

        for (int j = 0; j < *able_packet_status; j++)
        {
          able_packet_num = MIN (able_packet_status[j * 2 + 1], media_packet_num);
          media_packet_slot = PILOT(ceil((able_packet_num * packet_time) /\
                                         ((double) SUBFRAME_SIZE / MINISLOT)));

          for (int k = 0; k < media_packet_slot; k++)
            result[subframe_slot][able_packet_status[j * 2 + 2] + k] |= SET_X (priority_num);

          change_usage_status (usage_list[subframe_slot], able_packet_status[j * 2 + 2],\
                               media_packet_slot, whoisparent (setting, priority_num));
          media_packet_num -= able_packet_num;
        }
        free (able_packet_status);
      }

      while (parent_packet_num != 0)
      {
        assert (subframe_slot < subframe);
        able_packet_status = wherecaniuse (usage_list[subframe_slot],\
                                           parent_packet_time, 16);
        if (*able_packet_status == 0)
        {
          subframe_slot++;
          free (able_packet_status);
          continue;
        }

        for (int j = 0; j < *able_packet_status; j++)
        {
          able_packet_num = MIN (able_packet_status[j * 2 + 1], parent_packet_num);
          media_packet_slot = PILOT(ceil((able_packet_num * parent_packet_time) /\
                                         ((double) SUBFRAME_SIZE / MINISLOT)));

          for (int k = 0; k < media_packet_slot; k++)
            result[subframe_slot][able_packet_status[j * 2 + 2] + k] |=\
              SET_X (whoisparent (setting, priority_num));

          change_usage_status (usage_list[subframe_slot], able_packet_status[j * 2 + 2],\
                               media_packet_slot, 16);
          parent_packet_num -= able_packet_num;
        }
        free (able_packet_status);
      }

      while (reply_packet_num != 0)
      {
        assert (subframe_slot < subframe);
        able_packet_status = wherecaniuse (usage_list[subframe_slot],\
                                           reply_packet_time, 16);
        if (*able_packet_status == 0)
        {
          subframe_slot++;
          free (able_packet_status);
          continue;
        }

        for (int j = 0; j < *able_packet_status; j++)
        {
          able_packet_num = MIN (able_packet_status[j * 2 + 1], reply_packet_num);
          media_packet_slot = PILOT(ceil((able_packet_num * reply_packet_time) /\
                                         ((double) SUBFRAME_SIZE / MINISLOT)));

          for (int k = 0; k < media_packet_slot; k++)
            result[subframe_slot][able_packet_status[j * 2 + 2] + k] |= SET_X (16);

          change_usage_status (usage_list[subframe_slot], able_packet_status[j * 2 + 2],\
                               media_packet_slot, 16);
          reply_packet_num -= able_packet_num;
        }
        free (able_packet_status);
      }

      while (parent_reply_packet_num != 0)
      {
        assert (subframe_slot < subframe);
        able_packet_status = wherecaniuse (usage_list[subframe_slot],\
                                           parent_packet_time, 16);
        if (*able_packet_status == 0)
        {
          subframe_slot++;
          free (able_packet_status);
          continue;
        }

        for (int j = 0; j < *able_packet_status; j++)
        {
          able_packet_num = MIN (able_packet_status[j * 2 + 1], parent_reply_packet_num);
          media_packet_slot = PILOT(ceil((able_packet_num * parent_packet_time) /\
                                         ((double) SUBFRAME_SIZE / MINISLOT)));

          for (int k = 0; k < media_packet_slot; k++)
            result[subframe_slot][able_packet_status[j * 2 + 2] + k] |=\
              SET_X (whoisparent (setting, priority_num));

          change_usage_status (usage_list[subframe_slot], able_packet_status[j * 2 + 2],\
                               media_packet_slot, 16);
          parent_reply_packet_num -= able_packet_num;
        }
        free (able_packet_status);
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
  setting->parent_num = 5;
  setting->parent[0] = 1;
  setting->parent[1] = 2;
  setting->parent[2] = 3;
  setting->parent[3] = 4;
  setting->parent[4] = 5;
  setting->children[1][0] = 6;
  setting->children[2][0] = 7;

  int rate = 11;
  setting->buffering = 1;

  setting->video [0] = true;
  setting->video [5] = true;
  setting->audio [1] = true;
  setting->audio [2] = true;
  setting->audio [3] = true;
  setting->audio [6] = true;

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
        printf ("%4x", (result[i][(55 * j) + k] & 0xffff));
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


