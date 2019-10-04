#ifndef ONENET_H
#define ONENET_H



#define DBG_ENABLE
#define DBG_SECTION_NAME "onenet.sample"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>


void connect_onenet_http_device(void);
void post_data_stream_to_onenet(char *dev_id, char *api_key, char *datastream_id, int point_value);

#endif