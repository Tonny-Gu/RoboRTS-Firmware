/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "dbus.h"

static void get_dr16_data(rc_device_t rc_dev, uint8_t *buff);
static void get_dr16_state(rc_device_t rc_dev);

int32_t rc_device_register(rc_device_t rc_dev, const char *name, uint16_t flags)
{
  if (rc_dev == NULL)
    return -RM_INVAL;

  if (device_find(name) != NULL)
    return -RM_EXISTED;

  ((device_t)rc_dev)->type = Device_Class_RC;

  rc_dev->get_data = get_dr16_data;
  rc_dev->get_state = get_dr16_state;

  device_register( &(rc_dev->parent), name, flags);

  return RM_OK;
}

int32_t rc_device_date_update(rc_device_t rc_dev, uint8_t *buff)
{ 
  if (rc_dev != NULL)
  {
    rc_dev->get_data(rc_dev, buff);
    rc_dev->get_state(rc_dev);
    return RM_OK;
  }
  return -RM_UNREGISTERED;
}

int32_t rc_device_get_state(rc_device_t rc_dev, uint16_t state)
{
  var_cpu_sr();
    
  enter_critical();

  if (rc_dev != NULL)
  {
    if((rc_dev->state & state) == state)
    {
      rc_dev->state &=(~(state & 0x00FF));
      exit_critical();
      return RM_OK;   
    }
    else
    {
      exit_critical();
      return -RM_NOSTATE;
    }
  }
  
  exit_critical();

  return -RM_UNREGISTERED;
}

rc_info_t rc_device_get_info(rc_device_t rc_dev)
{
  if (rc_dev == NULL)
  {
    return NULL;
  }

  return &(rc_dev->rc_info);
}

rc_device_t rc_device_find(const char *name)
{
  rc_device_t rc_dev;
  enum device_type type;

  rc_dev = (rc_device_t)device_find(name);
  
  if(rc_dev == NULL)
    return NULL;
  
  type = ((device_t)rc_dev)->type;
  
  if (type == Device_Class_RC)
  {
    return rc_dev;
  }
  else
  {
    return NULL;
  }
}
uint32_t sbus_ch[25];
static void get_dr16_data(rc_device_t rc_dev, uint8_t *buff)
{
  memcpy(&(rc_dev->last_rc_info), &rc_dev->rc_info, sizeof(struct rc_info));
	
	uint32_t i_buff=1, i_ch, buff_data = 0, buff_data_len = 0;
	for(i_ch=0; i_ch<25; ++i_ch)
	{
		while(buff_data_len<11)
		{
			buff_data |= buff[i_buff++]<<buff_data_len;
			buff_data_len += 8;
		}
		buff_data_len -= 11;
		sbus_ch[i_ch] = buff_data & 0x07FF;
		buff_data = buff_data>>11;
	}
	
  rc_info_t rc = &rc_dev->rc_info;
  
  rc->ch1 = sbus_ch[0] - 1024;
  rc->ch2 = sbus_ch[1] - 1024;
  rc->ch3 = sbus_ch[3] - 1024; // This is the remapped configuration of T8FB RC.
  rc->ch4 = sbus_ch[2] - 1024;
  
  /* prevent remote control zero deviation */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;
  
	switch(sbus_ch[6])
	{
		case 0x00C8: rc->sw1 = 3; break;
		//case 0x03E8: rc->sw1 = 3; break;
		case 0x0708: rc->sw1 = 2; break;
	}
	
	switch(sbus_ch[4])
	{
		case 0x00C8: rc->sw2 = 1; break;
		case 0x03E8: rc->sw2 = 3; break;
		case 0x0708: rc->sw2 = 2; break;
	}
       
  if ((abs(rc->ch1) > 780) || \
      (abs(rc->ch2) > 780) || \
      (abs(rc->ch3) > 780) || \
      (abs(rc->ch4) > 780))
  {
    memset(rc, 0, sizeof(struct rc_info));
    return ;
  }

  rc->mouse.x = 0; // No corresponding function
  rc->mouse.y = 0;
  rc->mouse.z = 0;

  rc->mouse.l = sbus_ch[7]; // Map to L/R Wheel
  rc->mouse.r = sbus_ch[5];

  rc->kb.key_code = 0;
  rc->wheel = 0;
}

static void get_dr16_state(rc_device_t rc_dev)
{

  if(rc_dev->rc_info.sw1 == 3)
  {
    rc_dev->state |= RC_S1_MID;
    rc_dev->state &= ~RC_S1_UP;
    rc_dev->state &= ~RC_S1_DOWN;
    if(rc_dev->last_rc_info.sw1 == 1)
    {
      rc_dev->state |= RC_S1_UP2MID;
    }
    else if(rc_dev->last_rc_info.sw1 == 2)
    {
      rc_dev->state |= RC_S1_DOWN2MID;
    }
  }
  else if(rc_dev->rc_info.sw1 == 1)
  {
    rc_dev->state &= ~RC_S1_MID;
    rc_dev->state |= RC_S1_UP;
    rc_dev->state &= ~RC_S1_DOWN;
    if(rc_dev->last_rc_info.sw1 == 3)
    {
      rc_dev->state |= RC_S1_MID2UP;
    }
  }
  else if(rc_dev->rc_info.sw1 == 2)
  {
    rc_dev->state &= ~RC_S1_MID;
    rc_dev->state &= ~RC_S1_UP;
    rc_dev->state |= RC_S1_DOWN;
    if(rc_dev->last_rc_info.sw1 == 3)
    {
      rc_dev->state |= RC_S1_MID2DOWN;
    }
  }
  
  if(rc_dev->rc_info.sw2 == 3)
  {
    rc_dev->state |= RC_S2_MID;
    rc_dev->state &= ~RC_S2_UP;
    rc_dev->state &= ~RC_S2_DOWN;
    if(rc_dev->last_rc_info.sw2 == 1)
    {
      rc_dev->state |= RC_S2_UP2MID;
    }
    else if(rc_dev->last_rc_info.sw2 == 2)
    {
      rc_dev->state |= RC_S2_DOWN2MID;
    }
  }
  else if(rc_dev->rc_info.sw2 == 1)
  {
    rc_dev->state &= ~RC_S2_MID;
    rc_dev->state |= RC_S2_UP;
    rc_dev->state &= ~RC_S2_DOWN;
    if(rc_dev->last_rc_info.sw2 == 3)
    {
      rc_dev->state |= RC_S2_MID2UP;
    }
  }
  else if(rc_dev->rc_info.sw2 == 2)
  {
    rc_dev->state &= ~RC_S2_MID;
    rc_dev->state &= ~RC_S2_UP;
    rc_dev->state |= RC_S2_DOWN;
    if(rc_dev->last_rc_info.sw2 == 3)
    {
      rc_dev->state |= RC_S2_MID2DOWN;
    }
  }
}
