#include <stdio.h>
#include "i2c_slave.h"
#include "pid.h"
#include "h_bridge.h"
#include<unistd.h>


void app_main(void)
{
    init_gpio();
    init_pwm();

    while(1){
        update_motor(RIGHT, 5000);
    }



    // create_tasks();

    // init_gpio();
    // init_pwm();
    

    // int contador = 0;

    // //create_tasks();
    // while(1){
    //     update_motor(LEFT, contador);
    //     update_motor(RIGHT, contador);

    //     sleep(1);

    //     contador += 100;
    // }


}
