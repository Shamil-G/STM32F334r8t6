/*
 * manage_pwm.c
 *
 *  Created on: Jan 17, 2024
 *  Author: Shamil Gusseynov
 */
#include "stdio.h"

struct manage_pwm_struct {

    int target_value;  // May be 400V
    int max_pwm_value; // May be 20000

    int prev_value;
    int curr_value;

    int prev_manag_value;
    int next_manag_value;
};

struct manage_pwm_struct manage_data;

int calculate_pwm(int curr_value) {
    int delta_1;
    int delta_2;

    float D;

    manage_data.curr_value=curr_value;

    delta_1 = manage_data.curr_value - manage_data.prev_value;
    delta_2 = manage_data.target_value - manage_data.curr_value;

    printf("\n******************************\nCURR_VALUE: %d\nDELTA_1: %d\nDELTA_2: %d ", curr_value, delta_1, delta_2);
    // For Start system
    if (delta_2==manage_data.target_value && curr_value==0){
    	manage_data.next_manag_value=manage_data.max_pwm_value/2;
        printf("\n1. curr_value==0");
    }
    else
    // System stability
    if(delta_1==0 && delta_2==0){
    	manage_data.next_manag_value=manage_data.prev_manag_value;
    	printf("\nSystem stability\ndelta_1==0. manage.next_manag_value: %d", manage_data.next_manag_value);
    }
    else
    // Идет рост
    if(delta_1>0 && delta_2>0){
    	D = (float)delta_2/(float)delta_1;
//    	printf("\n3.\nD: %fd", D);
    	manage_data.next_manag_value=manage_data.prev_manag_value*D;
    }
    else
    if(delta_1<0 && delta_2>0){
      D=1-(float)delta_2/(float)delta_1;
      printf("\ndelta_1<0 && delta_2>0 \n-----------------------");
    	manage_data.next_manag_value=manage_data.prev_manag_value*D;
    }
    else
    if(delta_1>0 && delta_2<0){
    	D = 1+(float)delta_2/(float)delta_1;
//    	printf("\ndelta_1>0 && delta_2<0. D: %f", D);
    	manage_data.next_manag_value=manage_data.prev_manag_value/D;
    }
    else
    if(delta_1<0 && delta_2<0){
      D=(float)delta_2/(float)delta_1 + 1;
      printf("\ndelta_1<0 && delta_2<0 \n-----------------------");
    	manage_data.next_manag_value=manage_data.prev_manag_value/D;
    }

    manage_data.next_manag_value=(manage_data.next_manag_value>manage_data.max_pwm_value)?manage_data.max_pwm_value:manage_data.next_manag_value;
    printf("\n4. NEXT_MANAG_VALUE: %d\n-----------------------", manage_data.next_manag_value);
    manage_data.prev_value=manage_data.curr_value;
    manage_data.prev_manag_value=manage_data.next_manag_value; // ШИМ  заполнение
    return manage_data.next_manag_value;
}



void init_manage_pwm(){
	manage_data.target_value=400;
	manage_data.max_pwm_value=20000;
	manage_data.prev_value=0;
	manage_data.curr_value=0;
}

int test_manage_pwm() {
    init_manage_pwm();
    printf("\nNEXT MANAGE_VALUE: %d",calculate_pwm(0));
    printf("\n******************************\n");
    printf("\nNEXT MANAGE_VALUE: %d",calculate_pwm(300));
    printf("\nNEXT MANAGE_VALUE: %d",calculate_pwm(390));
    printf("\nNEXT MANAGE_VALUE: %d",calculate_pwm(410));
    printf("\nNEXT MANAGE_VALUE: %d",calculate_pwm(395));
    printf("\nNEXT MANAGE_VALUE: %d",calculate_pwm(400));
    printf("\nNEXT MANAGE_VALUE: %d",calculate_pwm(50));
    printf("\nNEXT MANAGE_VALUE: %d",calculate_pwm(55));
    return 0;
}
