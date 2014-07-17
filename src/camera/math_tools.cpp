#include "math_tools.h"

void moving_struct_init(MOV_STRUCT* moving_struct, float* buffer, int window_size){
	moving_struct->window = buffer;
	for(int i = 0; i < window_size; i++){
		moving_struct->window[i] = 0;
	}
	moving_struct->window_size = window_size;
	moving_struct->window_count = 0;
}

void window_update(MOV_STRUCT* moving_struct, float data){
	moving_struct->window[moving_struct->window_count++] = data;
	moving_struct->window_count = moving_struct->window_count == moving_struct->window_size ? 0 : moving_struct->window_count;
}

void window_clear(MOV_STRUCT* moving_struct){
	for(int i = 0; i < moving_struct->window_size; i++){
		moving_struct->window[i] = 0;
	}
	moving_struct->window_count = 0;
}

float* moving_struct_get_window(MOV_STRUCT* moving_struct){
	return moving_struct->window;
}

int moving_struct_get_window_size(MOV_STRUCT* moving_struct){
	return moving_struct->window_size;
}

int moving_struct_get_window_count(MOV_STRUCT* moving_struct){
	return moving_struct->window_count;
}
float avg(float* v, int length){
	float sum = 0;
	for(int i = 0; i < length; i++){
		sum += v[i];
	}
	return sum / length;
}
