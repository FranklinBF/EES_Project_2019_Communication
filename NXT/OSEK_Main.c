//
// Created by christos on 11.05.19.
//

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include <stddef.h>
#include "util.h"
#include "angle_Table.h"
#include "Intersection.h"
#include "Turn_Around.h"


const int line_timeout = 100 ; //time for changing direction due to line crossing
const int on_line_timeout = 100 ;

DeclareCounter(System_Counter) ;

DeclareEvent(Data);
DeclareEvent(No_Data);
DeclareEvent(Right_On_Line) ;
DeclareEvent(Right_Line);
DeclareEvent(Right_Clear);
DeclareEvent(Left_On_Line) ;
DeclareEvent(Left_Line);
DeclareEvent(Left_Clear);
DeclareEvent(Very_Close);
DeclareEvent(Close);
DeclareEvent(Getting_Closer);
DeclareEvent(No_Obstacle);
DeclareEvent(new_speed_angle);

DeclareEvent(Turn_Around_Event) ;
DeclareEvent(Turn_Left) ;
DeclareEvent(Turn_Right) ;


DeclareResource(received_data_resource) ;
DeclareResource(angle_speed_resource) ;

DeclareTask(Speed_Setter) ;
DeclareTask(Data_Receiver) ;
DeclareTask(Sync_with_Server) ;
DeclareTask(Decision_Maker) ;
DeclareTask(Turn_Around_Task) ;
DeclareTask(Intersection_Task) ;

float received_data ;
Data_Frame data_recv = {0.0F, 0, 0, 0, 0, 0, 0} ;


float angle_to_set = 0.0F ;
float speed_factor = 1.0F ;
float velocity_factor = 1.0F ;


int current_speed_right ;
int current_speed_left ;
float current_angle ;

unsigned int left_counter = 0, right_counter = 0 ;
unsigned int on_line_left_counter = 0, on_line_right_counter = 0 ;


float small_correction_angle_left_line ;
float small_correction_angle_right_line ;
float big_correction_angle_left_line ;
float big_correction_angle_right_line ;

char turning_around ; //flag from Turn_Around.h
char turning_intersection ; //flag from Intersection.h

unsigned int data_received_timeout = 0 ;

unsigned int sync_timeout = 0 ;

char offline_mode_active = 0 ;

void user_1ms_isr_type2(void) {
    SignalCounter(System_Counter) ;
    ++left_counter ;
    ++right_counter ;
    ++on_line_left_counter ;
    ++on_line_right_counter ;

    ++data_received_timeout ;
    ++sync_timeout ;
}

void ecrobot_device_initialize() {
    ecrobot_init_bt_slave("1234") ;
    ecrobot_set_light_sensor_active(Light_Sensor_Left);
    ecrobot_set_light_sensor_active(Light_Sensor_Right);
    ecrobot_init_sonar_sensor(UltraSonic) ;

    //cant be initialized when declared, so has to be done when nxt initiliazes
    small_correction_angle_left_line = deg_to_rad(-90.0f) ;
    small_correction_angle_right_line = deg_to_rad(90.0f) ;
    big_correction_angle_left_line = deg_to_rad(-120.0f) ;
    big_correction_angle_right_line = deg_to_rad(120.0f) ;
}

void ecrobot_device_terminate() {
    nxt_motor_set_speed(Motor_Left, 0, 1) ;
    nxt_motor_set_speed(Motor_Left, 0, 1) ;
    ecrobot_set_light_sensor_inactive(Light_Sensor_Left);
    ecrobot_set_light_sensor_inactive(Light_Sensor_Right);
    ecrobot_term_sonar_sensor(UltraSonic) ;
    ecrobot_term_bt_connection() ;
}

/**
 * gets the set angle and the speed after receiving the incoming event
 * then calculates the wheel speeds and sets them accordingly
 */
TASK(Speed_Setter) {
    WaitEvent(new_speed_angle) ;
    ClearEvent(new_speed_angle) ;
    GetResource(angle_speed_resource) ;
    float angle = angle_to_set ;
    float speed = speed_factor ;
    ReleaseResource(angle_speed_resource) ;

    current_angle = angle ;

    float speed_for_angle = get_Speed(angle) * velocity_factor ;
    if(offline_mode_active && speed_for_angle > 0.6F) {
    	speed_for_angle = 0.6F ;
    }
/*    if(velocity_factor < 1.0F) {
        speed_for_angle = 0.3F ;
    } */
    float vr = VR_Calc(angle, speed_for_angle) ;
    float vl = VL_Calc(angle, speed_for_angle) ;

    //vr setter
    int right_speed = round_float((vr / V_MAX_RAD) * 100) ;
    if(right_speed > 100) {
        right_speed = 100 ;
    }
    else if(right_speed < -100) {
        right_speed = -100 ;
    }

    //vl setter
    int left_speed = round_float((vl / V_MAX_RAD) * 100) ;
    if(left_speed > 100) {
        left_speed = 100 ;
    }
    else if(left_speed < -100) {
        left_speed = -100 ;
    }

    right_speed = right_speed * speed ;
    left_speed = left_speed * speed ;

    nxt_motor_set_speed(Motor_Right, right_speed, 1) ;
    current_speed_right = right_speed ;
    nxt_motor_set_speed(Motor_Left, left_speed, 1) ;
    current_speed_left = left_speed ;

    TerminateTask() ;
}

int data_read = 0 ;
enum {buffer_size =  32} ;
int frame_size = 8 ;
char receive_buffer[buffer_size] ;
char data_timed_out = 0 ;
StatusType status_data = 0 ;
StatusType status_no_data = 0 ;

/**
 * Task for receiving data from the server
 * whenever something comes in, notifies other task
 */
TASK(Data_Receiver) {
    U32 data_size = ecrobot_read_bt(receive_buffer, 0 + data_read, buffer_size - data_read) ;
    if(data_size > 0) {
        data_read += data_size ;
    }
    if(data_read >= frame_size) {
        data_received_timeout = 0 ;
        data_timed_out = 0 ;
        U16 header_bytes = 0 ;
        char flags = 0 ;
        float received_angle = 0.0f ;
        char crc = 0 ;
        memcpy(&header_bytes, receive_buffer, 2) ;
        memcpy(&flags, receive_buffer + 2, 1) ;
        memcpy(&received_angle, receive_buffer + 3, 4) ;
        memcpy(&crc, receive_buffer + 7, 1) ;
        data_read -= frame_size ;

        if(header_bytes == 0xFFFF) {
            GetResource(received_data_resource) ;
            //TODO missing crc calculation
            data_recv.angle = received_angle ;
            data_recv.obstacle = flags & 0x01 ;
            data_recv.turn_180 = (flags >> 1) & 0x01 ;
            data_recv.intersection = (flags >> 2) & 0x01 ;
            data_recv.left = (flags >> 3) & 0x01 ;
            data_recv.right = (flags >> 4) & 0x01 ;
            data_recv.straight = (flags >> 5) & 0x01 ;
            data_recv.slow_down = (flags >> 6) & 0x01 ;
            ReleaseResource(received_data_resource) ;
            status_data = SetEvent(Decision_Maker, Data) ;

 /*           display_clear(0) ;
            display_goto_xy(0, 0) ;
            if(status == E_OS_ID) {
                display_string("E_OS_ID") ;
            }
            else if(status == E_OS_ACCESS) {
                display_string("E_OS_ACCESS") ;
            }
            else if(status == E_OS_STATE) {
                display_string("E_OS_STATE") ;
            }
            else if(status == E_OK) {
                display_string("E_OK") ;
            }
            display_update() ;
            //TODO get a goto going when setting the event fails, might be the cause of the problems
            */
        }
    }
    if(data_received_timeout >= 500 && data_timed_out == 0) {
        data_timed_out = 1 ;
        status_no_data = SetEvent(Decision_Maker, No_Data) ;
    }
    if(status_no_data != E_OK) {
    	status_no_data = SetEvent(Decision_Maker, No_Data) ;
    }
    if(status_data != E_OK) {
    	status_data = SetEvent(Decision_Maker, Data) ;
    }
    TerminateTask() ;
}

/**
 * startup routine for syncing with server
 */
TASK(Sync_with_Server) {
	sync_timeout = 0 ;
    while(ecrobot_get_bt_status() != BT_STREAM && sync_timeout < 15000) {
        continue ;
    }
    if(sync_timeout < 15000) {
    	char buffer[] = "sync" ;
    	ecrobot_send_bt(buffer, 0, 4) ;
	}
/*    else {
        ecrobot_sound_tone(1000, 50, 50) ;
    } */
    TerminateTask() ;
}


char stop_obstacle_flag = 0, stop_line_flag = 0, line_right = 0, line_left = 0 ;
char on_line_right = 0, on_line_left = 0 ;
Data_Frame last_data_frame = {0.0F, 0, 0, 0, 0, 0, 0} ;

/**
 * brain of the entire system, basically handles everything that is going on
 * and then takes correct action, depending on input and state
 */
TASK(Decision_Maker) {
    EventMaskType mask = 0 ;
    WaitEvent(Data | No_Data
                   | Right_On_Line | Right_Line | Right_Clear | Left_On_Line | Left_Line | Left_Clear
                   | Very_Close | Close | Getting_Closer | No_Obstacle ) ;
    GetEvent(Decision_Maker, &mask) ;
    float speed_scale = 1.0F ;
    float velocity_scale = 1.0F ;
    float recv_angle = 0.0F ;
    char received_new_data = 0 ;
    char event_set = 0 ;


    if(data_timed_out == 1) {
        velocity_scale = 0.6F ;
        offline_mode_active = 1 ;
    }
    else {
        offline_mode_active = 0 ;
    }

    //check if data has timed out, which means no new packet in last 500ms
    if(mask & No_Data) {
        ClearEvent(No_Data) ;
//        last_data_frame = {0.0F, 0, 0, 0, 0, 0, 0} ;
        last_data_frame.angle = 0.0F ;
        last_data_frame.obstacle = 0 ;
        last_data_frame.turn_180 = 0 ;
        last_data_frame.intersection = 0 ;
        last_data_frame.left = 0 ;
        last_data_frame.right = 0 ;
        last_data_frame.straight = 0 ;
        recv_angle = last_data_frame.angle ;
        received_new_data = 1 ;

//        ecrobot_sound_tone(500, 50, 50) ;
    }
    //check if data has been received
    if(mask & Data) {
        ClearEvent(Data) ;
        GetResource(received_data_resource) ;
//        recv_angle = received_data ;
        last_data_frame.angle = data_recv.angle ;
        last_data_frame.obstacle = data_recv.obstacle ;
        last_data_frame.turn_180 = data_recv.turn_180 ;
        last_data_frame.intersection = data_recv.intersection ;
        last_data_frame.left = data_recv.left ;
        last_data_frame.right = data_recv.right ;
        last_data_frame.straight = data_recv.straight ;
        ReleaseResource(received_data_resource) ;
        recv_angle = last_data_frame.angle ;
        received_new_data = 1 ;
    }


    //check if left line is being crossed
    if(mask & Left_On_Line || on_line_left) {
        ClearEvent(Left_On_Line) ;
        line_left = 1 ;
        left_counter += line_timeout ; //if doesnt work correctly, remove this statement, same in line 329
        /* this caused the turning until other sensor triggers, cause it was never able
         * to set speed and angle when it cleared and it immediately went from on_line to clear
         * so this has to be = 1 and the problem should be fixed, same thing done for other sensor
         * also increase the left counter so that it will timeout instantly if we go from on_line to clear
         * */
        if(received_new_data && recv_angle < big_correction_angle_left_line) {
            velocity_scale = 0.75F ;
        }
        else {
            recv_angle = big_correction_angle_left_line ;
            velocity_scale = 0.75F ;
        }
        if(on_line_left == 0 || mask & Left_On_Line) {
            on_line_left_counter = 0 ;
        }
        on_line_left = 1 ;
    }
    if(on_line_left && on_line_left_counter > on_line_timeout) { //reset on line if expired timer
        on_line_left = 0 ;
    }
    if(mask & Left_Line || (line_left && !on_line_left)) {
        ClearEvent(Left_Line) ;
        on_line_left = 0 ;
        if(received_new_data && recv_angle < small_correction_angle_left_line) {
            velocity_scale = 0.75F ;
        }
        else {
            recv_angle = small_correction_angle_left_line ;
            velocity_scale = 0.75F ;
        }
        if(line_left != 1 || mask & Left_Line) { //look at other line sensor
            left_counter = 0 ;
        }
        line_left = 1 ;
    }
    if(mask & Left_Clear) {
        ClearEvent(Left_Clear) ;
        if(line_left && left_counter > line_timeout) {
            line_left = 0 ;
            if(line_right == 0) {
                velocity_scale = 1.0F ;
            }
            if(received_new_data == 0) {
                recv_angle = 0.0F ;
                received_new_data = 1 ;
            }
        }
    }


    //check if right line has been crossed
    if(mask & Right_On_Line || on_line_right) {
        ClearEvent(Right_On_Line) ;
        line_right = 1 ; //look at line 273 for explanation
        right_counter += line_timeout ;
        if(received_new_data && recv_angle > big_correction_angle_right_line) {
            velocity_scale = 0.75F ;
        }
        else {
            recv_angle = big_correction_angle_right_line ;
            velocity_scale = 0.75F ;
        }
        if(on_line_right == 0 || mask & Right_On_Line) {
            on_line_right_counter = 0 ;
        }
        on_line_right = 1 ;
    }
    if(on_line_right && on_line_right_counter > on_line_timeout) { //reset on line if expired timer
        on_line_right = 0 ;
    }
    if(mask & Right_Line || (line_right && !on_line_right)) {
        ClearEvent(Right_Line) ;
        on_line_right = 0 ;
        if(received_new_data && recv_angle > small_correction_angle_right_line) {
            velocity_scale = 0.75F ;
        }
        else {
            recv_angle = small_correction_angle_right_line ;
            velocity_scale = 0.75F ;
        }
        if(line_right != 1 || mask & Right_Line) { //this needs to be reset every time the event came in
            right_counter = 0 ;
        }
        line_right = 1 ;
    }
    if(mask & Right_Clear) {
        ClearEvent(Right_Clear) ;
        if(line_right == 1 && right_counter > line_timeout) {
            line_right = 0;
            if(line_left == 0) {
                velocity_scale = 1.0F ;
            }
            if(received_new_data == 0) {
                recv_angle = 0.0F ;
                received_new_data = 1 ;
            }
        }
    }


    if(last_data_frame.obstacle == 1) { //if server knows about obstacle, we ignore our sensor, server should handle it
        velocity_scale = 0.5F ;
    }
    else { //otherwise use our own sensor
        //check if obstacle is being too close
        if(mask & No_Obstacle) {
            ClearEvent(No_Obstacle) ;
            if(on_line_right == 0 && on_line_left == 0 && line_right == 0 && line_left == 0) {
                speed_scale = 1.0F ;
            }
            stop_obstacle_flag = 0 ;
            //remove speed multiplier set by being close to an obstacle
        }
        if(mask & Getting_Closer) {
            ClearEvent(Getting_Closer) ;
            if(on_line_right == 0 && on_line_left == 0 && line_right == 0 && line_left == 0) {
                speed_scale = 0.75F ;
            }
            stop_obstacle_flag = 0 ;
            //set speed multiplier accordingly
        }
        if(mask & Close) {
            ClearEvent(Close) ;
            speed_scale = 0.5F ;
            stop_obstacle_flag = 0 ;
            //set speed multiplier accordingly
        }
    }

    //we should keep the emergency situation still reachable, in case anything goes wrong, so we can stop
    if(mask & Very_Close || stop_obstacle_flag) {
        ClearEvent(Very_Close) ;
        speed_scale = 0.0F ;
        stop_obstacle_flag = 1 ;
        //set speed multiplier accordingly
    }

    //has high priority, so has to be later so it can take action properly
    if((line_left == 1 && line_right == 1) || (on_line_right == 1 && on_line_left == 1)
    	|| (line_right == 1 && on_line_left == 1) || (line_left == 1 && on_line_right == 1)) {
        stop_line_flag = 1 ;
        speed_scale = 0.0F ;
    }

    if(last_data_frame.turn_180 == 1 && turning_around == 0) {
        SetEvent(Turn_Around_Task, Turn_Around_Event) ;
        event_set = 1 ;
        last_data_frame.turn_180 = 0 ;
    }

    if(event_set == 0) {
        if(last_data_frame.left == 1 && turning_intersection == 0) {
            SetEvent(Intersection_Task, Turn_Left) ;
            event_set = 1 ;
            last_data_frame.left = 0 ;
        }
        else if(last_data_frame.right == 1 && turning_intersection == 0) {
            SetEvent(Intersection_Task, Turn_Right) ;
            event_set = 1 ;
            last_data_frame.right = 0 ;
        }
    }
/*    if(last_data_frame.straight == 1) {
        velocity_scale = 0.5F ;
    } */
    if(last_data_frame.slow_down == 1) { //new flag for slowing down, used with franklins code
    	velocity_scale = 0.5F ;
    }

    if(data_received_timeout > 120000) { //stop nxt after driving offline for 2min so it wont run forever
        speed_scale = 0.0F ;
    }

    //get resources and set speed, angle
    GetResource(angle_speed_resource) ;
    velocity_factor = velocity_scale ;
    speed_factor = speed_scale ;
    if(received_new_data || on_line_left || on_line_right || line_left || line_right) {
        angle_to_set = recv_angle ;
    }
    else {
        angle_to_set = current_angle ;
    }
    ReleaseResource(angle_speed_resource) ;
    if(event_set == 0 && turning_around == 0 && turning_intersection == 0) { //add flag for turning at intersection
        SetEvent(Speed_Setter, new_speed_angle) ;
    }
    TerminateTask() ;
}
