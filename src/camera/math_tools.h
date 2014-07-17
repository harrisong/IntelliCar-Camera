#ifndef __MATH_TOOL_H
#define __MATH_TOOL_H

typedef struct moving_struct MOV_STRUCT;

struct moving_struct{
	int window_size;
	int window_count;
	float* window;
};

void moving_struct_init(MOV_STRUCT* moving_struct, float* buffer, int window_size);
void window_update(MOV_STRUCT* moving_struct, float data);
void window_clear(MOV_STRUCT* moving_struct);
float* moving_struct_get_window(MOV_STRUCT* moving_struct);
int moving_struct_get_window_size(MOV_STRUCT* moving_struct);
int moving_struct_get_window_count(MOV_STRUCT* moving_struct);
float avg(float* v, int length);
#endif		/* __MATH_TOOL_H  */
