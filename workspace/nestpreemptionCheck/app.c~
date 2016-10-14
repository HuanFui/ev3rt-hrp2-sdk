/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h"
#include "app.h"

#include <time.h>

#define LOOP(time)	{	int loopEnd = time*425100;\
						int k = 0;\
						SYSTIM current_time;\
						for(int j=0; j<=loopEnd ;j++){\
							k |= k<<1;\
							get_tim(&current_time);\
						}\
					}

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif


int finished = 1;
int i = 0;
int j = 0;
int k = 0;
SYSTIM first_start;

// a cyclic handler to activate a task
void task_activator1(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK);
}

void task_activator2(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK);
}

void task_activator3(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK);
}

void haupt_task(intptr_t unused) {
	lcdfont_t font = EV3_FONT_SMALL;
    ev3_lcd_set_font(font);
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);	
	char lcdstr[100];
	SYSTIM start_time, end_time;

	
    /**
     * Main loop for the self-balance control algorithm
     */
    if(finished==1){
        get_tim(&start_time); 
		sprintf(lcdstr, "SH  %i  %i.",i,start_time-first_start);
  		ev3_lcd_draw_string(lcdstr, 0, fonth*1);
		LOOP(1.5);
		get_tim(&end_time);
		sprintf(lcdstr, "EH    %i.",end_time-first_start);
	    ev3_lcd_draw_string(lcdstr, 0, fonth*2);
		i +=1;
		//slp_tsk();
    }
}



void balance_task(intptr_t unused) {
	lcdfont_t font = EV3_FONT_SMALL;
    ev3_lcd_set_font(font);
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);	
	char lcdstr[100];
	SYSTIM start_time, end_time;

	
    /**
     * Main loop for the self-balance control algorithm
     */
    if(finished==1) {
        get_tim(&start_time); 
		sprintf(lcdstr, "SB  %i  %i.",j,start_time-first_start);
  		ev3_lcd_draw_string(lcdstr, 0, fonth*3);
		LOOP(2);
		get_tim(&end_time);
		sprintf(lcdstr, "EB    %i.",end_time-first_start);
	    ev3_lcd_draw_string(lcdstr, 0, fonth*4);
		j +=1;
		//slp_tsk();
    }
}


void idle_task(intptr_t unused) {
	lcdfont_t font = EV3_FONT_SMALL;
    ev3_lcd_set_font(font);
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);	
	char lcdstr[100];
	SYSTIM start_time, end_time;

    if(finished==1) {
    	get_tim(&start_time); 
		sprintf(lcdstr, "SI  %i  %i.",i,start_time-first_start);
  		ev3_lcd_draw_string(lcdstr, 0, fonth*5);
		LOOP(2);
		get_tim(&end_time);
		sprintf(lcdstr, "EI    %i.",end_time-first_start);
	    ev3_lcd_draw_string(lcdstr, 0, fonth*6);
		k +=1;
		//slp_tsk();
		if(i==2)
			finished = 0;
    }
}

void main_task(intptr_t unused) {
    // Draw information
    lcdfont_t font = EV3_FONT_SMALL;
    ev3_lcd_set_font(font);
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);
    char lcdstr[100];
    ev3_lcd_draw_string("App: Trial", 0, 0);
    int i =0;
	SYSTIM start_time, end_time;
    
	get_tim(&first_start); 
	sprintf(lcdstr, "StartApp %i.",first_start);
    ev3_lcd_draw_string(lcdstr, 0, fonth*7);

	ev3_sta_cyc(CYC_HAUPT_TSK);	
	ev3_sta_cyc(CYC_BALANCE_TSK);
	ev3_sta_cyc(CYC_IDLE_TSK);
	                                    
    // Start task for self-balancing
    //act_tsk(BALANCE_TASK);
    
    // Start task for printing message while idle
	//act_tsk(IDLE_TASK);

    while(finished) {
	/*
		get_tim(&start_time); 
		sprintf(lcdstr, "SM  %i  %i.",i,start_time-first_start);
  		ev3_lcd_draw_string(lcdstr, 0, fonth*1);
		LOOP(0.3);
		get_tim(&end_time);
		sprintf(lcdstr, "EM    %i.",end_time-first_start);
	    ev3_lcd_draw_string(lcdstr, 0, fonth*2);
		i +=1;*/
		tslp_tsk(30000);
	
    }
}





//Test:
//M	|xx       xx		|period=9
//B	|  xx    x  x		|period=8
//I	|    xx x    x		|period=7
//T |0123456789012
