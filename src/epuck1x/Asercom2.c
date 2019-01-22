//#include "p30F6014A.h"
// Advanced Sercom B

#define FLOOR_SENSORS    // define to enable floor sensors
#define IR_RECEIVER

#define IMAGE_HEADER_SIZE 3 // mode, width, height
#define IMAGE_MAX_SIZE (BUFFER_SIZE-IMAGE_HEADER_SIZE)

#include <main.h>
#include "camera/dcmi_camera.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/ground.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "button.h"
#include "leds.h"
#include "sdio.h"

#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <motor_led/advance_one_timer/e_led.h>
#include <motor_led/advance_one_timer/e_motors.h>
#include <uart/e_uart_char.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <a_d/advance_ad_scan/e_acc.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <a_d/advance_ad_scan/e_micro.h>
#include <motor_led/advance_one_timer/e_agenda.h>
#include <camera/fast_2_timer/e_po8030d.h>
#include <camera/fast_2_timer/e_poxxxx.h>
#include <codec/e_sound.h>
#include <utility/utility.h>
#include <acc_gyro/e_lsm330.h>

#ifndef FLOOR_SENSORS
#define FLOOR_SENSORS
#endif
#ifdef FLOOR_SENSORS

#include <I2C/e_I2C_protocol.h>

#endif

#ifdef IR_RECEIVER

#include <motor_led/advance_one_timer/e_remote_control.h>
#include <../usbcfg.h>
#include <../../ChibiOS/os/hal/lib/streams/chprintf.h>
#include <../motors.h>
#include <math.h>
#include <../../ChibiOS_ext/os/hal/boards/epuck2/board.h>
#include <../../ChibiOS/os/rt/include/chvt.h>

#define SPEED_IR 600
#define USB_ACTIVE 4
#endif

#include "DataEEPROM.h"

#include "memory.h"

extern char buffer[BUFFER_SIZE];
//extern int e_mic_scan[3][MIC_SAMP_NB];
//extern unsigned int e_last_mic_scan_id;
extern int selector; //extern int selector;
extern char c;
//extern int e_ambient_ir[10];						// ambient light measurement
//extern int e_ambient_and_reflected_ir[10];		// light when led is on

#define uart1_send_static_text(msg) do { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); } while(0)
#define uart1_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)
#define uart2_send_static_text(msg) do { e_send_uart2_char(msg,sizeof(msg)-1); while(e_uart2_sending()); } while(0)
#define uart2_send_text(msg) do { e_send_uart2_char(msg,strlen(msg)); while(e_uart2_sending()); } while(0)



void Set_goal_coordinates(double X_goal_entered, double Y_goal_entered, double *X_goal_point, double *Y_goal_point) {
    //Устанавливает целевые координаты точки.
    *X_goal_point = X_goal_entered;
    *Y_goal_point = Y_goal_entered;
}
void Update_goal_point_distance(double *Goal_point_distance, double X_pos_actual, double Y_pos_actual, double X_goal_point, double Y_goal_point)  {
    //Обновляет значение расстояния до цели.
    if(X_pos_actual == X_goal_point && Y_pos_actual == Y_goal_point) //Если координаты робота и цели совпадают.
        *Goal_point_distance = 0; //[м]
    else //Иначе вычисление расстояния (гипотенузы по двум катетам).
        *Goal_point_distance = sqrt(pow(X_goal_point-X_pos_actual,2) + pow(Y_goal_point-Y_pos_actual,2)); //[м]
}
void Update_goal_point_angle(double *Goal_point_angle, double X_pos_actual, double Y_pos_actual, double X_goal_point, double Y_goal_point)  {
    /*Функция обновляет угол ориентации до цели для точки с роботом относительно Ox, [рад] на основе текущего положения робота.
    Т.е. это тот угол, на который робот должен повернуть, чтобы быть направленным к цели.
    Здесь не продуман случай, когда робот достиг целевой точки и угол уже никак нельзя
    вычислить*/

    int Quadrant; //Квадрант определяется для системы координат, центр которой совпадает с центром робота, а оси параллельны глобальной СК.
    if((X_goal_point-X_pos_actual)*(Y_goal_point-Y_pos_actual) == 0)
        Quadrant = 0; //Неопределенный квадрант (точка находится на оси).
    if((X_goal_point-X_pos_actual)*(Y_goal_point-Y_pos_actual) > 0) //Определение квадранта точки на плоскости.
        Quadrant = (((X_goal_point-X_pos_actual) > 0 && (Y_goal_point-Y_pos_actual) > 0) ? 1 : 3);
    if((X_goal_point-X_pos_actual)*(Y_goal_point-Y_pos_actual) < 0)
        Quadrant = (((X_goal_point-X_pos_actual) > 0 && (Y_goal_point-Y_pos_actual) < 0) ? 4 : 2);

    if((X_goal_point-X_pos_actual) != 0 || (Y_goal_point-Y_pos_actual) != 0)
        *Goal_point_angle = asin(fabs(Y_goal_point-Y_pos_actual)/sqrt(pow(X_goal_point-X_pos_actual,2)+pow(Y_goal_point-Y_pos_actual,2))); //Вычисление угла прямоугольного треугольника с катетами X_goal и Y_goal, [рад].
    else Quadrant = 0;

    switch(Quadrant) //Вычисление угла в локальной СК в зависимости от квадранта.
    {
        case 1: break;
        case 2: *Goal_point_angle = M_PI - *Goal_point_angle; break;
        case 3: *Goal_point_angle = M_PI + *Goal_point_angle ; break;
        case 4: *Goal_point_angle = 2*M_PI - *Goal_point_angle ; break;
        case 0:
            if((X_goal_point-X_pos_actual) == 0 && (Y_goal_point-Y_pos_actual) > 0) *Goal_point_angle = M_PI/2; //90 градусов.
            if((X_goal_point-X_pos_actual) == 0 && (Y_goal_point-Y_pos_actual) < 0) *Goal_point_angle = 1.5*M_PI; //270 градусов.
            if((X_goal_point-X_pos_actual) > 0  && (Y_goal_point-Y_pos_actual) == 0) *Goal_point_angle = 0; //0 градусов.
            if((X_goal_point-X_pos_actual) < 0  && (Y_goal_point-Y_pos_actual) == 0) *Goal_point_angle = M_PI; //180 градусов.
            if((X_goal_point-X_pos_actual) == 0 && (Y_goal_point-Y_pos_actual) == 0) *Goal_point_angle = M_PI/2; //Позиция робота совпадает с целевой точкой.
            break;
    }
}
void Update_angle_disagreement(double *Angle_disagreement, double Theta, double Goal_angle)  {
    //Обновление угла рассогласования с учетом рациональности поворота (чтобы робот поворачивал на угол, меньший или равный 180 град.
    //На входе два угла в [рад] - угол ориентации робота отн. оси Ох (Theta) и целевой угол (Goal_angle), так же отн. оси Ох.
    //На выходе - угол в [рад], обозначающий рассогласование.
    //При этом модуль |величина рассогласования| всегда <= 180 град.
    *Angle_disagreement = (Goal_angle - Theta);

    if((Goal_angle - Theta) > M_PI)
    {
        *Angle_disagreement = -(2*M_PI - (Goal_angle - Theta));
    }
    if((Goal_angle - Theta) < -M_PI)
    {
        *Angle_disagreement = 2*M_PI + (Goal_angle - Theta);
    }
}
void Update_odometer2(double *Left_encoder_old, double *Right_encoder_old, double *Theta, double *X_pos_actual, double *Y_pos_actual) {
    //Колесная одометрия.
    /*Функция обновляет положение робота и угол его ориентации
    на основе старых значений энкодеров и текущих координат и угла
    ориентации.*/

    double Left_encoder_actual = 0; //Количество шагов на левом колесе [шаги].
    double Right_encoder_actual = 0; //Количество шагов на правом колесе [шаги].
    double Left_length = 0; //Расстояние, пройденное левым колесом [м].
    double Right_length = 0; //Расстояние, пройденное правым колесом [м].
    double Alpha_odometer = 0; //Угол сегмента окружности при повороте [рад].
    double Radius_odometer = 0; //Радиус сегмента окружности при повороте [м].
    double Center_x_odometer = 0, Center_y_odometer = 0; //Координаты положения центра окружности поворота [м][м].

    Left_encoder_actual = (double)left_motor_get_pos(); //Обновляем данные энкодеров.
    Right_encoder_actual = (double)right_motor_get_pos(); //Получаем актуальные значения [шаги].

    Right_length = ((Right_encoder_actual - *Right_encoder_old)/1000)*0.1288; //Количество пройденных шагов делим на количество шагов в одном обороте колеса, получаем количество оборотов колеса, умножаем на длинну окружности колеса.
    Left_length = ((Left_encoder_actual - *Left_encoder_old)/1000)*0.1288; //[м].

    if(Right_length != Left_length)
    {
        Alpha_odometer = (Right_length - Left_length)/0.053; //В радианах. 0.053 - расстояние между колесами [м].
        Radius_odometer = Left_length/Alpha_odometer;

        Center_x_odometer = *X_pos_actual - (Radius_odometer+0.0265)*sin(*Theta); //0.0265 - полурасстояние между колесами [м].
        Center_y_odometer = *Y_pos_actual - (Radius_odometer+0.0265)*(-cos(*Theta));

        *Theta = fmod((*Theta+Alpha_odometer+2*M_PI), 2*M_PI);

        *X_pos_actual = Center_x_odometer + (Radius_odometer+0.0265)*sin(*Theta);
        *Y_pos_actual = Center_y_odometer + (Radius_odometer+0.0265)*(-cos(*Theta));
    }

    if(Right_length == Left_length)
    {
        *X_pos_actual = *X_pos_actual  + Left_length*cos(*Theta);
        *Y_pos_actual = *Y_pos_actual + Left_length*sin(*Theta);
    }

    *Left_encoder_old = (double)left_motor_get_pos(); //Обновляем данные энкодеров.
    *Right_encoder_old = (double)right_motor_get_pos(); //Получаем актуальные значения [шаги].
}
void Update_goal_speeds_values(double *Linear_speed_goal, double *Angular_speed_goal, double Goal_point_distance, double Dec_radius, double Stop_radius, double Linear_speed_max, double Angle_disagreement, double Angular_speed_max, double Dec_angle, uint8_t Potential_method_switch, double Moving_speed_value_potential_method)  {
    /*Обновляются целевые угловая и линейная скорость исходя из расстояния до цели и угла рассогласования
    (выдаваемая скорость не превышает максимального значения)*/
    //Вычисление целевой линейной скорости:
    switch (Potential_method_switch) {
        case 0:
            if(Goal_point_distance > Dec_radius) {
                *Linear_speed_goal = Linear_speed_max; //До радиуса торможения модуль линейной скорости максимален.
            } else {
                if(Goal_point_distance <= Dec_radius && Goal_point_distance > Stop_radius) {
                    *Linear_speed_goal = (Goal_point_distance / Dec_radius) * Linear_speed_max; //Внутри радиуса торможения модуль линейной скорости пропорционален расстоянию до цели.
                } else {
                    if(Goal_point_distance <= Stop_radius) {
                        *Linear_speed_goal = 0; //Внутри радиуса остановки - остановка.
                    }
                }
            }
            break;

        case 1: //Включен метод потенц. полей.
            *Linear_speed_goal = Moving_speed_value_potential_method;

            if(Goal_point_distance <= Stop_radius)
            {
                *Linear_speed_goal = 0; //Внутри радиуса остановки - остановка.
            }
            break;
    }

    //Вычисление целевой угловой скорости:
    if(fabs(Angle_disagreement) <= 0.015) { //Если рассогласование лежит в малых пределах, то угловая целевая скорость - ноль.
        *Angular_speed_goal = 0;
    } else {
        if(fabs(Angle_disagreement) <= Dec_angle) { //Если угол рассогласования меньше или равен углу торможения, то торможение.
            *Angular_speed_goal = (Angle_disagreement/Dec_angle) * Angular_speed_max;
        } else {
            *Angular_speed_goal = (fabs(Angle_disagreement)/(Angle_disagreement))*Angular_speed_max;
        }
    }
}
void Calculate_speeds(double *Linear_speed_real, double *Angular_speed_real, double Linear_acc, double Angular_acc, double Linear_speed_goal, double Angular_speed_goal)  {
    /*Обновляет реальную угловую и линейную скорости колес с учетом ускорения и текущей угловой и линейной скорости.
    Под "реальной" понимается та скорость, которая будет установлена на колеса.
    Это сделано для того, чтобы был эффект инерционности и скорость менялась не скачком, а с нарастанием (ограниченное приращение скорости).*/
    if((Linear_speed_goal - *Linear_speed_real) > 0) { //Анимация для ускорения (зеленый цвет), торможения (красный) и постоянной скорости (синий).
        set_rgb_led(0, 0, 10, 0);
        set_rgb_led(1, 0, 10, 0);
        set_rgb_led(2, 0, 10, 0);
        set_rgb_led(3, 0, 10, 0);
    } else {
        if((Linear_speed_goal - *Linear_speed_real) < 0) {
            set_rgb_led(0, 10, 0, 0);
            set_rgb_led(1, 10, 0, 0);
            set_rgb_led(2, 10, 0, 0);
            set_rgb_led(3, 10, 0, 0);
        } else {
            set_rgb_led(0, 0, 0, 10);
            set_rgb_led(1, 0, 0, 10);
            set_rgb_led(2, 0, 0, 10);
            set_rgb_led(3, 0, 0, 10);
        }
    }

    //Вычисление реальной линейной скорости:
    if(fabs(Linear_speed_goal - *Linear_speed_real) > Linear_acc) /*Если разница между целевой и реальной скоростью превышает величину возможного ускорения,
	то можно изменить скорость только на величину ускорения*/
    {
        *Linear_speed_real = *Linear_speed_real + (fabs(Linear_speed_goal - *Linear_speed_real)/(Linear_speed_goal - *Linear_speed_real))*Linear_acc;
    }
    else //Если разница между целевой и реальной скоростью не превышает величину возможного ускорения, то сразу переходим к целевой скорости.
    {
        *Linear_speed_real = Linear_speed_goal;
    }

    //Вычисление реальной угловой скорости (логика такая же, что и с линейной):
    if(fabs(Angular_speed_goal - *Angular_speed_real) > Angular_acc)
    {
        *Angular_speed_real = *Angular_speed_real + (fabs(Angular_speed_goal - *Angular_speed_real)/(Angular_speed_goal - *Angular_speed_real))*Angular_acc;
    }
    else
    {
        *Angular_speed_real = Angular_speed_goal;
    }
}
void Set_wheels_speeds(double Linear_speed_real, double Angular_speed_real, double h)  {
    /*Эта функция устанавливает скорости на колеса, переводя требуемую линейную и угловую скорости в [шаги/с] для каждого колеса.*/
    double Left_speed_ms, Right_speed_ms; //Скорость лев. и прав. колес в [м/с].
    int16_t Left_speed_steps, Right_speed_steps; //Скорость лев. и прав. колес [шаги/с].

    //Положительная угл. скорость - против часовой стрелки.
    Left_speed_ms = Linear_speed_real - h*Angular_speed_real;
    Right_speed_ms = Linear_speed_real + h*Angular_speed_real;

    //Из метров в секунду в шаги в секунду:
    Left_speed_steps =  (int16_t) 1200 * Left_speed_ms/(1.2*M_PI*0.041);
    Right_speed_steps = (int16_t) 1200 * Right_speed_ms/(1.2*M_PI*0.041);

    //Установка:
    left_motor_set_speed(Left_speed_steps);
    right_motor_set_speed(Right_speed_steps);
}
void Filter_and_update_proximity_values(int *Prox_0_sens_values, int *Prox_1_sens_values, int *Prox_2_sens_values, int *Prox_5_sens_values, int *Prox_6_sens_values, int *Prox_7_sens_values, int Size, int *Filtered_proximity_value)  {
    /*На вход подается 6 массивов размерности 5 с данными с каждого переднего ИК-дальномера.
    Применяется медианный фильтр (подходит под задачу, так как в последовательности данных с ИК-дальномера
    есть одиночные мощные импульсы, а остальные данные достаточно схожи). На выходе фильтра - массив размерности 6 с отфильтрованными
    значениями для каждого ИК-дальномера.*/
    int temp;

    //Занесение неотфильтрованных данных в массивы:
    for(int i = 0; i < Size; i++)
    {
        Prox_0_sens_values[i] = get_calibrated_prox(0);
        Prox_1_sens_values[i] = get_calibrated_prox(1);
        Prox_2_sens_values[i] = get_calibrated_prox(2);
        Prox_5_sens_values[i] = get_calibrated_prox(5);
        Prox_6_sens_values[i] = get_calibrated_prox(6);
        Prox_7_sens_values[i] = get_calibrated_prox(7);
        chThdSleepMilliseconds(5); //todo ask why u add sleep here?
    }

    //Сортировка по возрастанию для каждого массива:
    for(int i = 0; i < Size - 1; i++) {
        for(int j = 0; j < Size - i - 1; j++) {
            if(Prox_0_sens_values[j] > Prox_0_sens_values[j+1]) {
                temp = Prox_0_sens_values[j];
                Prox_0_sens_values[j] = Prox_0_sens_values[j+1];
                Prox_0_sens_values[j+1] = temp;
            }
            if(Prox_1_sens_values[j] > Prox_1_sens_values[j+1]) {
                temp = Prox_1_sens_values[j];
                Prox_1_sens_values[j] = Prox_1_sens_values[j+1];
                Prox_1_sens_values[j+1] = temp;
            }
            if(Prox_2_sens_values[j] > Prox_2_sens_values[j+1]) {
                temp = Prox_2_sens_values[j];
                Prox_2_sens_values[j] = Prox_2_sens_values[j+1];
                Prox_2_sens_values[j+1] = temp;
            }
            if(Prox_5_sens_values[j] > Prox_5_sens_values[j+1]) {
                temp = Prox_5_sens_values[j];
                Prox_5_sens_values[j] = Prox_5_sens_values[j+1];
                Prox_5_sens_values[j+1] = temp;
            }
            if(Prox_6_sens_values[j] > Prox_6_sens_values[j+1]) {
                temp = Prox_6_sens_values[j];
                Prox_6_sens_values[j] = Prox_6_sens_values[j+1];
                Prox_6_sens_values[j+1] = temp;
            }
            if(Prox_7_sens_values[j] > Prox_7_sens_values[j+1]) {
                temp = Prox_7_sens_values[j];
                Prox_7_sens_values[j] = Prox_7_sens_values[j+1];
                Prox_7_sens_values[j+1] = temp;
            }
        }
    }

    Filtered_proximity_value[0] = Prox_0_sens_values[2];
    Filtered_proximity_value[1] = Prox_1_sens_values[2];
    Filtered_proximity_value[2] = Prox_2_sens_values[2];
    Filtered_proximity_value[3] = Prox_5_sens_values[2];
    Filtered_proximity_value[4] = Prox_6_sens_values[2];
    Filtered_proximity_value[5] = Prox_7_sens_values[2]; //todo ask why [2]? because of median filter?
}
double Get_proximity_value_mm(int Data_from_sensor)  {
    /*Возвращает значение расстояния до препятствия в [мм].
    На основе значения данных с какого-либо ИК-дальномера.
    Входное значение - данные с сенсора. Используется аппроксимация четырьмя графиками.
    На вход подается отфильтрованное значение с одного ИК-датчика.*/
    if(Data_from_sensor >= 3116 && Data_from_sensor <= 3900)
        return sqrt((-Data_from_sensor+3900.0)/16.0);
    if(Data_from_sensor >= 2300 && Data_from_sensor < 3116)
        return ((-2.0*Data_from_sensor + 11944.0)/816.0);
    if(Data_from_sensor >=276 && Data_from_sensor < 2300)
        return (10000.0/(Data_from_sensor+200.0)+5);
    if(Data_from_sensor < 276)
        return ((-24.0*Data_from_sensor + 11460.0)/186.0);
    return 0;

    //todo ask don't understand the numbers here?
}
void Update_obstacle_repulsion_vector(double Proximity_values_mm[], double Theta, double Sensor_angles[], double *Vector_angle, double *Vector_length)  {
    /*На вход подается массив с отфильтрованными расстояниями в [мм] до препятствий от 6-ти передних ИК_дальномеров.
     Затем вычисляется направление (угол [рад]) и модуль результирующего вектора отталкивания относительно глобальной оси Ox.*/

    double X_repulse = 0.0; //Суммарный X всех векторов.
    double Y_repulse = 0.0; //Суммарный Y всех векторов.

    for(int i = 0; i < 6; i++) //Вычисление координат результирующего вектора.
    {
        if(Proximity_values_mm[i] >= 60.0) //Если больше 60 [мм], считаем, что препятствия нет (предел видимости ИК-дальномера).
        {
            continue;
        }
        X_repulse = X_repulse + (60.0 - Proximity_values_mm[i])/60.0*cos(Sensor_angles[i]);
        Y_repulse = Y_repulse + (60.0 - Proximity_values_mm[i])/60.0*sin(Sensor_angles[i]);
    }

    if(X_repulse == 0.0 && Y_repulse == 0.0) //Если препятствий нет, то модуль вектора отталкивания равен 0.
    {
        *Vector_length = 0;
        *Vector_angle = Theta;
    }
    else //Если препятствия есть, то нормируем модуль до единицы.
    {
        *Vector_angle = acos(X_repulse/sqrt(pow(X_repulse, 2)+pow(Y_repulse, 2))); //Угол отн. поперечной подвижнойоси, закрепленной на роботе (а нужен относительно глобальной оси Ox) [рад].
        *Vector_angle = fmod(*Vector_angle + (Theta - M_PI/2) + 2*M_PI + M_PI, 2*M_PI); //Угол отн. Ox [рад].
        *Vector_length = sqrt(pow(X_repulse, 2) + pow(Y_repulse, 2));
    }
}
void Update_moving_vector_for_potential_method(double Obstacle_repulsion_vector_angle, double Obstacle_repulsion_vector_length, double Goal_point_angle, double *Moving_speed_value_potential_method, double *Moving_angle_potential_method, double Linear_speed_max)  {
    /*На входе - угол и модуль вектора отталкивания от препятствий и угол до целевой точки.
     На выходе - направление и скорость те, которые вычислены для метода потенциалов.*/
    double X = 0.0;
    double Y = 0.0;
    double Vector_length = 0.0;

    if(Obstacle_repulsion_vector_length != 0.0) //Вектор отталкивания от препятствий не нулевой.
    {
        X = cos(Obstacle_repulsion_vector_angle) + cos(Goal_point_angle); //Считаем координаты результирующего вектора.
        Y = sin(Obstacle_repulsion_vector_angle) + sin(Goal_point_angle);
        Vector_length = sqrt(pow(X, 2)+ pow(Y, 2)); //Вычисляем модуль.

        *Moving_speed_value_potential_method = (Vector_length/2.0)*Linear_speed_max; //Значение скорости.
        *Moving_angle_potential_method = fmod(acos(X/Vector_length) + 2*M_PI, 2*M_PI); //Угол направления.
    }
    else //Вектор отталкивания от препятствий нулевой.
    {
        *Moving_speed_value_potential_method = Linear_speed_max; //Значение скорости.
        *Moving_angle_potential_method = Goal_point_angle; //Угол направления.
    }
}


void Update_odometer1(double *Left_encoder_old, double *Right_encoder_old, double *Theta, double *X_pos_actual,
                      double *Y_pos_actual) {
/*Функция обновляет координаты центра робота и угол его ориентации относительно Ox, используя старые значения счетчиков шагов энкодеров и текущих координат и угла ориентации робота.*/

    double Left_encoder_actual = 0; //Количество шагов на левом колесе [шаги].
    double Right_encoder_actual = 0; //Количество шагов на правом колесе [шаги].
    double Left_length = 0; //Расстояние, пройденное левым колесом [м].
    double Right_length = 0; //Расстояние, пройденное правым колесом [м].
    double Alpha_odometer = 0; //Угол сегмента окружности при повороте [рад].
    double Radius_odometer = 0; //Радиус сегмента окружности при повороте [м].
    double Center_x_odometer = 0, Center_y_odometer = 0; //Координаты центра окружности поворота [м][м].

    Left_encoder_actual = (double) left_motor_get_pos(); //Обновляем данные энкодеров.
    Right_encoder_actual = (double) right_motor_get_pos(); //Получаем актуальные значения [шаги].

    Right_length = ((Right_encoder_actual - *Right_encoder_old) / 1000) *
                   0.1288; //Количество пройденных шагов делим на количество шагов в одном обороте колеса, получаем количество оборотов колеса, умножаем на длину окружности колеса [м].
    Left_length = ((Left_encoder_actual - *Left_encoder_old) / 1000) * 0.1288;

    if (Right_length != Left_length) //Если левое и правое колеса прошли не одинаковые расстояния.
    {
        Alpha_odometer = (Right_length - Left_length) / 0.053; //В радианах. 0.053 - расстояние между колесами [м].
        Radius_odometer = Left_length / Alpha_odometer;

        Center_x_odometer =
                *X_pos_actual - (Radius_odometer + 0.0265) * sin(*Theta); //0.0265 - полурасстояние между колесами [м].
        Center_y_odometer = *Y_pos_actual - (Radius_odometer + 0.0265) * (-cos(*Theta));

        *Theta = fmod((*Theta + Alpha_odometer + 2 * M_PI), 2 * M_PI);

        *X_pos_actual = Center_x_odometer + (Radius_odometer + 0.0265) * sin(*Theta);
        *Y_pos_actual = Center_y_odometer + (Radius_odometer + 0.0265) * (-cos(*Theta));
    }

    if (Right_length == Left_length) //Если левое и правое колеса прошли  одинаковые расстояния.
    {
        *X_pos_actual = *X_pos_actual + Left_length * cos(*Theta);
        *Y_pos_actual = *Y_pos_actual + Left_length * sin(*Theta);
    }

    *Left_encoder_old = (double) left_motor_get_pos(); //Обновляем данные энкодеров.
    *Right_encoder_old = (double) right_motor_get_pos();
}

void printBufferToPort(void) {
    if (SDU1.config->usbp->state == 4) {
        chprintf((BaseSequentialStream *) &SDU1, "%s\n", buffer);
    }
}

void printNumberToPort(int number) {
    if (SDU1.config->usbp->state == 4) {
        chprintf((BaseSequentialStream *) &SDU1, "%d\n", number);
    }
}

void printStringToPort(char *string) {
    if (SDU1.config->usbp->state == 4) {
        chprintf((BaseSequentialStream *) &SDU1, "%s\n", string);
    }
}

int run_asercom2(void) {

    if (SDU1.config->usbp->state == 4) {
        chprintf((BaseSequentialStream *) &SDU1, "in asercom\n");
    }

    static char c1, c2, wait_cam = 0;
    static int i, j, n, speedr, speedl, positionr, positionl, LED_nbr, LED_action, accx, accy, accz, sound, gyrox, gyroy, gyroz;
    static int cam_mode, cam_width, cam_heigth, cam_zoom, cam_size, cam_x1, cam_y1;
    static char first = 0;
    static char rgb_value[12];
    char *ptr;
    static int mod, reg, val;
#ifdef IR_RECEIVER
    char ir_move = 0, ir_address = 0, ir_last_move = 0;
#endif
    static TypeAccSpheric accelero;
    //static TypeAccRaw accelero_raw;
    int use_bt = 0;
    unsigned int battValue = 0;
    uint8_t gumstix_connected = 0;
    uint16_t cam_start_index = 0;
    char cmd = 0;
    float tempf = 0.0;
    uint32_t tempi = 0;

    //e_init_port();    // configure port pins
    //e_start_agendas_processing();
    e_init_motors();
    //e_init_uart1();   // initialize UART to 115200 Kbaud
    //e_init_ad_scan();


    //Описание робота:
    double h = 0.0265; //Полурасстояние между колесами [м].
    double X_pos_actual = 0.0, Y_pos_actual = 0.0; //Реальные коорд-ты центра робота [м].
    double Theta = M_PI/2; //Угол ориентации относительно Ox [рад].
    double Linear_speed_real = 0.0; //Реальная линейная скорость [м/с].
    double Angular_speed_real = 0.0; //Реальная угловая скорость [рад/с].
    double Left_speed_ms = 0.0, Right_speed_ms = 0.0; //Скорость лев. и прав. колес в [м/с].
    int16_t Left_speed_steps = 0, Right_speed_steps = 0.0; //Скорость лев. и прав. колес в [шаги/с].
    double Max_wheel_speed_for_linear = 0.07; //Часть скорости колеса, используемая для линейной скорости центра [м/с].
    double Max_wheel_speed_for_angular = 1.2*0.041*M_PI - Max_wheel_speed_for_linear; //Часть скорости колеса, используемая для угловой скорости центра [м/с]. (1.2*0.041*M_PI - максимальная линейная скорость колеса).
    double Linear_speed_max = 0.5*Max_wheel_speed_for_linear; //Максимальная лин. скорость центра [м/с].
    double Angular_speed_max = Max_wheel_speed_for_angular/h; //Макс. угл. скорость робота [рад/с].
    double Linear_acc = 0.1*Linear_speed_max; //Линейное ускорение [м/с].
    double Angular_acc = 0.25*Angular_speed_max; //Угловое ускорение [рад/с].

    //Описание целевой точки:
    double X_goal_point = 1.0, Y_goal_point = 1.0; //Целевые коорд-ты центра робота [м].
    double Dec_radius = 0.07; //В этом радиусе до цели - торможение [м].
    double Stop_radius = 0.05; //В этом радиусе от цели - стоп [м].
    double Dec_angle = (45.0/180.0)*M_PI; //В этом угле начинается торможение [рад].
    double Goal_point_angle = 0.0; //Угол на цель отн. оси Ox [рад].
    double Goal_point_distance = 0.0; //Расстояние до цели [м].

    //Целевые параметры робота:
    double Linear_speed_goal = 0.0; //Целевая линейная скорость [м/с].
    double Angular_speed_goal = 0.0; //Целевая угловая скорость [рад/с].
    double Goal_angle = 0.0; //Это условный целевой угол отн. оси Ox, который будет все время меняться [рад].
    double Angle_disagreement = 0.0; //Рассогласование по углу [рад].

    //Переменные для работы колесной одометрии:
    double Left_encoder_old = (double)left_motor_get_pos(); //Количество шагов на левом колесе пердыдущее [шаги].
    double Right_encoder_old = (double)right_motor_get_pos(); //Количество шагов на правом колесе пердыдущее [шаги].

    //Данные для реализации метода потенциальных сил:
    //Данные для получения вектора отталкивания от препятствий:
    int Size = 5;
    int Prox_0_sens_values[Size]; //Значения, подаваемые в медианный фильтр для каждого сенсора.
    int Prox_1_sens_values[Size];
    int Prox_2_sens_values[Size];
    int Prox_5_sens_values[Size];
    int Prox_6_sens_values[Size];
    int Prox_7_sens_values[Size];
    int Filtered_proximity_value[6]; //Отфильтрованные значения.
    double Proximity_values_mm[6]; //Значения дальностей [мм].
    double Obstacle_repulsion_vector_length = 0;
    double Obstacle_repulsion_vector_angle = 0.0;
    double Sensor_angles[6] = {1.309, 0.785, 0.0, M_PI, 2.356, 1.885}; //Углы расположения ИК-датчиков на роботе (отн. поперечной оси) [рад].
    //todo ask I thought we have 8 IR sensors?


    //Данные для получения вектора отталкивания от других агентов:
    char n_robots = 2; //Общее количество агентов (включая текущего).
    double Agents_repulsion_vector_length = 0;
    double Agents_repulsion_vector_angle = 0;

    //Результирующий вектор движения:
    double Moving_speed_value_potential_method = 0.0;
    double Moving_angle_potential_method = 0.0;
    double initTheta = Theta;

    selector = getselector(); //SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
    printNumberToPort(selector);
    printStringToPort("selector");
    if (selector == 3 || selector == 15) {
        use_bt = 1; // new comments: selector == 3 for bt (at least in my case)
    } else {
        use_bt = 0;
    }
    if (selector == 10) {
        gumstix_connected = 1;
    } else {
        gumstix_connected = 0;
    }

#ifdef FLOOR_SENSORS
    if (gumstix_connected == 0) { // the I2C must remain disabled when using the gumstix extension
        e_i2cp_init();
    }
#endif

#ifdef IR_RECEIVER
    e_init_remote_control();
#endif
//    if (RCONbits.POR) { // reset if power on (some problem for few robots)
//        RCONbits.POR = 0;
//        RESET();
//    }
    /*read HW version from the eeprom (last word)*/
    static int HWversion = 0xFFFF;
    ReadEE(0x7F, 0xFFFE, &HWversion, 1);

    /*Cam default parameter*/
    cam_mode = RGB_565_MODE;
    //cam_mode=GREY_SCALE_MODE;
    cam_width = 40; // DEFAULT_WIDTH;
    cam_heigth = 40; // DEFAULT_HEIGHT;
    cam_zoom = 8;
    cam_size = cam_width * cam_heigth * 2;

    if (gumstix_connected == 0 && selector != 15) {
        e_poxxxx_init_cam();
        e_poxxxx_config_cam((ARRAY_WIDTH - cam_width * cam_zoom) / 2, (ARRAY_HEIGHT - cam_heigth * cam_zoom) / 2,
                            cam_width * cam_zoom, cam_heigth * cam_zoom, cam_zoom, cam_zoom, cam_mode);
        e_poxxxx_write_cam_registers();
    }

    if (gumstix_connected) { // Communicate with gumstix (i2c).
        // Send the following text through I2C to the gumstix.
        //uart1_send_static_text("\f\a"
        //        "WELCOME to the SerCom protocol on e-Puck\r\n"
        //        "the EPFL education robot type \"H\" for help\r\n");
    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
        e_acc_calibr();
        uart1_send_static_text("\f\a"
                               "WELCOME to the SerCom protocol on e-Puck\r\n"
                               "the EPFL education robot type \"H\" for help\r\n");
    } else { // Communicate with the pc (usb).
        e_acc_calibr();
        uart2_send_static_text("\f\a"
                               "WELCOME to the SerCom protocol on e-Puck\r\n"
                               "the EPFL education robot type \"H\" for help\r\n");
    }

    printStringToPort("before while");
    while (1) {
        if (gumstix_connected) { // Communicate with gumstix (i2c).
            printStringToPort("Communicate with gumstix (i2c).");
        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
            //printStringToPort("Communicate with ESP32 (uart) => BT.");
            //printStringToPort("e_getchar_uart1(&c)");
            if (SDU1.config->usbp->state == 4) {
                //chprintf((BaseSequentialStream *) &SDU1, "--->%c<----->%p<-----\n", c, &c);
            }
            //printStringToPort("e_getchar_uart1(&c)");

            while (e_getchar_uart1(&c) == 0)
#ifdef IR_RECEIVER
            {
                //printStringToPort(" while (e_getchar_uart1(&c) == 0)");
                ir_move = e_get_data();
                ir_address = e_get_address();
                if (((ir_address == 0) || (ir_address == 8)) && (ir_move != ir_last_move)) {
                    switch (ir_move) {
                        case 1:
                            speedr = SPEED_IR;
                            speedl = SPEED_IR / 2;
                            break;
                        case 2:
                            speedr = SPEED_IR;
                            speedl = SPEED_IR;
                            break;
                        case 3:
                            speedr = SPEED_IR / 2;
                            speedl = SPEED_IR;
                            break;
                        case 4:
                            speedr = SPEED_IR;
                            speedl = -SPEED_IR;
                            break;
                        case 5:
                            speedr = 0;
                            speedl = 0;
                            break;
                        case 6:
                            speedr = -SPEED_IR;
                            speedl = SPEED_IR;
                            break;
                        case 7:
                            speedr = -SPEED_IR;
                            speedl = -SPEED_IR / 2;
                            break;
                        case 8:
                            speedr = -SPEED_IR;
                            speedl = -SPEED_IR;
                            break;
                        case 9:
                            speedr = -SPEED_IR / 2;
                            speedl = -SPEED_IR;
                            break;
                        case 0:
                            if (first == 0) {
                                e_init_sound();
                                first = 1;
                            }
                            e_play_sound(11028, 8016);
                            break;
                        default:
                            speedr = speedl = 0;
                    }
                    ir_last_move = ir_move;
                    e_set_speed_left(speedl);
                    e_set_speed_right(speedr);
                }
            }
#else
            ;
#endif
        } else { // Communicate with the pc (usb).
            //printStringToPort("// Communicate with the pc (usb).");
            while (e_getchar_uart2(&c) == 0)
#ifdef IR_RECEIVER
            {
                ir_move = e_get_data();
                ir_address = e_get_address();
                if (((ir_address == 0) || (ir_address == 8)) && (ir_move != ir_last_move)) {
                    switch (ir_move) {
                        case 1:
                            speedr = SPEED_IR;
                            speedl = SPEED_IR / 2;
                            break;
                        case 2:
                            speedr = SPEED_IR;
                            speedl = SPEED_IR;
                            break;
                        case 3:
                            speedr = SPEED_IR / 2;
                            speedl = SPEED_IR;
                            break;
                        case 4:
                            speedr = SPEED_IR;
                            speedl = -SPEED_IR;
                            break;
                        case 5:
                            speedr = 0;
                            speedl = 0;
                            break;
                        case 6:
                            speedr = -SPEED_IR;
                            speedl = SPEED_IR;
                            break;
                        case 7:
                            speedr = -SPEED_IR;
                            speedl = -SPEED_IR / 2;
                            break;
                        case 8:
                            speedr = -SPEED_IR;
                            speedl = -SPEED_IR;
                            break;
                        case 9:
                            speedr = -SPEED_IR / 2;
                            speedl = -SPEED_IR;
                            break;
                        case 0:
                            if (first == 0) {
                                e_init_sound();
                                first = 1;
                            }
                            e_play_sound(11028, 8016);
                            break;
                        default:
                            speedr = speedl = 0;
                    }
                    ir_last_move = ir_move;
                    e_set_speed_left(speedl);
                    e_set_speed_right(speedr);
                }
            }
#else
            ;
#endif
        }

        if ((int8_t) c < 0) { // binary mode (big endian)
            //new comments - come here
            //printStringToPort("// binary mode (big endian)");
            i = 0;

            do {
                switch ((int8_t) -c) {
                    case 0x8: // Get all sensors.
                        printStringToPort("// Get all sensors.");

                        // Read accelerometer.
                        if (gumstix_connected == 0) {
                            accx = e_get_acc(0);
                            accy = e_get_acc(1);
                            accz = e_get_acc(2);
                        } else {
                            accx = 0;
                            accy = 0;
                            accz = 0;
                        }
                        buffer[i++] = accx & 0xff;
                        buffer[i++] = accx >> 8;
                        buffer[i++] = accy & 0xff;
                        buffer[i++] = accy >> 8;
                        buffer[i++] = accz & 0xff;
                        buffer[i++] = accz >> 8;

                        if (gumstix_connected == 0) {
                            accelero = e_read_acc_spheric();
                        } else {
                            accelero.acceleration = 0.0;
                            accelero.inclination = 0.0;
                            accelero.orientation = 0.0;
                        }
                        ptr = (char *) &accelero.acceleration;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);

                        ptr = (char *) &accelero.orientation;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);

                        ptr = (char *) &accelero.inclination;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);

                        // Read gyro.
                        if (gumstix_connected == 0) {
                            getAllAxesGyro(&gyrox, &gyroy, &gyroz);
                            buffer[i++] = gyrox & 0xFF;
                            buffer[i++] = gyrox >> 8;
                            buffer[i++] = gyroy & 0xFF;
                            buffer[i++] = gyroy >> 8;
                            buffer[i++] = gyroz & 0xFF;
                            buffer[i++] = gyroz >> 8;
                        } else {
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                        }

                        // Read magnetometer.
                        for (j = 0; j < 3; j++) {
                            tempf = get_magnetic_field(j);
                            tempi = *((uint32_t *) &tempf);
                            buffer[i++] = tempi & 0xff;
                            buffer[i++] = tempi >> 8;
                            buffer[i++] = tempi >> 16;
                            buffer[i++] = tempi >> 24;
                        }

                        // Read temperature.
                        if (gumstix_connected == 0) {
                            buffer[i++] = getTemperature();
                        } else {
                            buffer[i++] = 0;
                        }

                        // Read proximities.
                        for (j = 0; j < 8; j++) {
                            n = e_get_calibrated_prox(j); // or ? n=e_get_prox(j);
                            buffer[i++] = n & 0xff;
                            buffer[i++] = n >> 8;
                        }

                        // Read ambient light.
                        for (j = 0; j < 8; j++) {
                            n = e_get_ambient_light(j);
                            buffer[i++] = n & 0xff;
                            buffer[i++] = n >> 8;
                        }

                        // Read ToF.
                        if (gumstix_connected == 0) {
                            n = VL53L0X_get_dist_mm();
                            buffer[i++] = n & 0xff;
                            buffer[i++] = n >> 8;
                        } else {
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                        }

                        // Read microphones.
                        n = e_get_micro_volume(0);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;
                        n = e_get_micro_volume(1);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;
                        n = e_get_micro_volume(2);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;
                        n = e_get_micro_volume(3);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;

                        // Read encoders.
                        n = e_get_steps_left();
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;
                        n = e_get_steps_right();
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;

                        // Read battery.
                        battValue = getBatteryValueRaw();
                        buffer[i++] = battValue & 0xFF;
                        buffer[i++] = battValue >> 8;

                        // Read micro sd state.
                        if (sdio_is_present()) {
                            n = !sdio_connect();
                            buffer[i++] = n & 0xff;
                            sdio_disconnect();
                        } else {
                            buffer[i++] = 0;
                        }

                        // Read tv remote.
                        buffer[i++] = e_get_check();
                        buffer[i++] = e_get_address();
                        buffer[i++] = e_get_data();

                        // Read selector.
                        selector = SELECTOR0 + 2 * SELECTOR1 + 4 * SELECTOR2 + 8 * SELECTOR3;
                        buffer[i++] = selector;

                        // Read ground sensor proximity.
                        for (j = 0; j < 3; j++) {
                            n = get_ground_prox(j);
                            buffer[i++] = n & 0xff;
                            buffer[i++] = n >> 8;
                        }

                        // Read ground sensor ambient.
                        for (j = 0; j < 3; j++) {
                            n = get_ground_ambient_light(j);
                            buffer[i++] = n & 0xff;
                            buffer[i++] = n >> 8;
                        }

                        // Additional empty byte for future use.
                        buffer[i++] = 0;

                        break;

                    case 0x9: // Set all actuators.
                        printStringToPort("// Set all actuators");

                        // Handle behaviors and others commands.
                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&cmd) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&cmd) == 0);
                        }
                        if (cmd & 0x01) { // Calibrate proximity.
                            calibrate_ir();
                        }
                        if (cmd & 0x02) { // Enable obastacle avoidance.

                        } else { // Disable obstacle avoidance

                        }

                        // Set motor speed or motor position.
                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                            while (e_getchar_uart1(&c2) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                            while (e_getchar_uart2(&c2) == 0);
                        }
                        speedl = (unsigned char) c1 + ((unsigned int) c2 << 8);
                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                            while (e_getchar_uart1(&c2) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                            while (e_getchar_uart2(&c2) == 0);
                        }
                        speedr = (unsigned char) c1 + ((unsigned int) c2 << 8);
                        if (cmd & 0x04) { // Set steps.
                            e_set_steps_left(speedl);
                            e_set_steps_right(speedr);
                        } else { // Set speed.
                            e_set_speed_left(speedl);
                            e_set_speed_right(speedr);
                        }

                        // Set LEDs.
                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                        }
                        if (c1 & 0x01) {
                            set_led(LED1, 1);
                        } else {
                            set_led(LED1, 0);
                        }
                        if (c1 & 0x02) {
                            set_led(LED3, 1);
                        } else {
                            set_led(LED3, 0);
                        }
                        if (c1 & 0x04) {
                            set_led(LED5, 1);
                        } else {
                            set_led(LED5, 0);
                        }
                        if (c1 & 0x08) {
                            set_led(LED7, 1);
                        } else {
                            set_led(LED7, 0);
                        }
                        if (c1 & 0x10) {
                            set_body_led(1);
                        } else {
                            set_body_led(0);
                        }
                        if (c1 & 0x20) {
                            set_front_led(1);
                        } else {
                            set_front_led(0);
                        }

                        // RGBs setting.
                        for (j = 0; j < 12; j++) {
                            if (gumstix_connected) { // Communicate with gumstix (i2c).

                            } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                                while (e_getchar_uart1(&rgb_value[j]) == 0);
                            } else { // Communicate with the pc (usb).
                                while (e_getchar_uart2(&rgb_value[j]) == 0);
                            }
                        }
                        set_rgb_led(0, rgb_value[0], rgb_value[1], rgb_value[2]);
                        set_rgb_led(1, rgb_value[3], rgb_value[4], rgb_value[5]);
                        set_rgb_led(2, rgb_value[6], rgb_value[7], rgb_value[8]);
                        set_rgb_led(3, rgb_value[9], rgb_value[10], rgb_value[11]);

                        // Play sound.
                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                        }
                        if (c1 & 0x01) {
                            playMelody(MARIO, ML_FORCE_CHANGE, NULL);//e_play_sound(0, 2112);
                        }
                        if (c1 & 0x02) {
                            playMelody(UNDERWORLD, ML_FORCE_CHANGE, NULL);//e_play_sound(2116, 1760);
                        }
                        if (c1 & 0x04) {
                            playMelody(STARWARS, ML_FORCE_CHANGE, NULL);//e_play_sound(3878, 3412);
                        }
                        if (c1 & 0x08) {
                            e_play_sound(7294, 3732);
                        }
                        if (c1 & 0x10) {
                            e_play_sound(11028, 8016);
                        }
                        if (c1 & 0x20) {
                            e_close_sound();
                            stopCurrentMelody();
                        }

                        break;

                    case 0xA: // RGB setting => ESP32
                        printStringToPort("// RGB setting => ESP32");

                        for (j = 0; j < 12; j++) {
                            if (gumstix_connected) { // Communicate with gumstix (i2c).

                            } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                                while (e_getchar_uart1(&rgb_value[j]) == 0);
                            } else { // Communicate with the pc (usb).
                                while (e_getchar_uart2(&rgb_value[j]) == 0);
                            }
                        }
                        set_rgb_led(0, rgb_value[0], rgb_value[1], rgb_value[2]);
                        set_rgb_led(1, rgb_value[3], rgb_value[4], rgb_value[5]);
                        set_rgb_led(2, rgb_value[6], rgb_value[7], rgb_value[8]);
                        set_rgb_led(3, rgb_value[9], rgb_value[10], rgb_value[11]);
                        break;

                    case 0xB: // Button state => ESP32
                        printStringToPort("// Button state => ESP32");

                        buffer[i++] = button_get_state();
                        break;

                    case 0xC: // Get 4 microphones
                        printStringToPort("// Get 4 microphones");

                        n = e_get_micro_volume(0);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;

                        n = e_get_micro_volume(1);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;

                        n = e_get_micro_volume(2);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;

                        n = e_get_micro_volume(3);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;
                        break;

                    case 0xD: // Get distance sensor
                        printStringToPort("// Get distance sensor");

                        if (gumstix_connected == 0) {
                            n = VL53L0X_get_dist_mm();
                            buffer[i++] = n & 0xff;
                            buffer[i++] = n >> 8;
                        } else {
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                        }
                        break;

                    case 0xE: // Get SD state (0=not ok, 1=ok)
                        printStringToPort("// Get SD state (0=not ok, 1=ok)");

                        if (sdio_is_present()) {
                            n = !sdio_connect();
                            buffer[i++] = n & 0xff;
                            sdio_disconnect();
                        } else {
                            buffer[i++] = 0;
                        }
                        break;

                    case 'a': // Read acceleration sensors in a non filtered way, same as ASCII
                        printStringToPort("// Read acceleration sensors in a non filtered way, same as ASCII");

                        if (gumstix_connected == 0) {
                            accx = e_get_acc_filtered(0, 1);
                            accy = e_get_acc_filtered(1, 1);
                            accz = e_get_acc_filtered(2, 1);
                            //accx = e_get_acc(0);	//too much noisy
                            //accy = e_get_acc(1);
                            //accz = e_get_acc(2);
                        } else {
                            accx = 0;
                            accy = 0;
                            accz = 0;
                        }
                        buffer[i++] = accx & 0xff;
                        buffer[i++] = accx >> 8;
                        buffer[i++] = accy & 0xff;
                        buffer[i++] = accy >> 8;
                        buffer[i++] = accz & 0xff;
                        buffer[i++] = accz >> 8;
                        break;

                    case 'A': // read acceleration sensors
                        printStringToPort("// read acceleration sensors");

                        if (gumstix_connected == 0) {
                            accelero = e_read_acc_spheric();
                        } else {
                            accelero.acceleration = 0.0;
                            accelero.inclination = 0.0;
                            accelero.orientation = 0.0;
                        }
                        ptr = (char *) &accelero.acceleration;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);

                        ptr = (char *) &accelero.orientation;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);

                        ptr = (char *) &accelero.inclination;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        ptr++;
                        buffer[i++] = (*ptr);
                        break;

                    case 'b': // battery state
                        printStringToPort("// battery state");

                        //battValue = getBatteryValuePercentage();
                        battValue = getBatteryValueRaw();
                        buffer[i++] = battValue & 0xFF;
                        buffer[i++] = battValue >> 8;
                        break;

                    case 'D': // set motor speed
                        printStringToPort("// set motor speed");

                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                            while (e_getchar_uart1(&c2) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                            while (e_getchar_uart2(&c2) == 0);
                        }
                        speedl = (unsigned char) c1 + ((unsigned int) c2 << 8);
                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                            while (e_getchar_uart1(&c2) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                            while (e_getchar_uart2(&c2) == 0);
                        }
                        speedr = (unsigned char) c1 + ((unsigned int) c2 << 8);
                        e_set_speed_left(speedl);
                        e_set_speed_right(speedr);
                        break;
                    case 'E': // get motor speed
                        printStringToPort("// get motor speed");

                        buffer[i++] = speedl & 0xff;
                        buffer[i++] = speedl >> 8;
                        buffer[i++] = speedr & 0xff;
                        buffer[i++] = speedr >> 8;
                        break;

                    case 'g': // gyro rates
                        printStringToPort("// gyro rates");

                        if (gumstix_connected == 0) {
                            getAllAxesGyro(&gyrox, &gyroy, &gyroz);
                            buffer[i++] = gyrox & 0xFF;
                            buffer[i++] = gyrox >> 8;
                            buffer[i++] = gyroy & 0xFF;
                            buffer[i++] = gyroy >> 8;
                            buffer[i++] = gyroz & 0xFF;
                            buffer[i++] = gyroz >> 8;
                        } else {
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                            buffer[i++] = 0;
                        }
                        break;

                    case 'I': // get camera image
                        printStringToPort("// get camera image");

                        if (gumstix_connected == 0) {
                            e_poxxxx_launch_capture(&buffer[i + 3]);
                            wait_cam = 1;
                            buffer[i++] = (char) cam_mode & 0xff; //send image parameter
                            buffer[i++] = (char) cam_width & 0xff;
                            buffer[i++] = (char) cam_heigth & 0xff;
                            cam_start_index = i;
                            i += cam_size;
                        }
                        break;
                    case 'L': // set LED
                        printStringToPort("// set LED");

                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                            while (e_getchar_uart1(&c2) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                            while (e_getchar_uart2(&c2) == 0);
                        }
                        switch (c1) {
                            case 8:
                                e_set_body_led(c2);
                                break;
                            case 9:
                                e_set_front_led(c2);
                                break;
                            default:
                                e_set_led(c1, c2);
                                break;
                        }
                        break;
                    case 'M': // optional floor sensors
                        printStringToPort("// optional floor sensors");

#ifdef FLOOR_SENSORS
                        if (gumstix_connected == 0) {
                            for (j = 0; j < 3; j++) {
                                n = get_ground_prox(j);
                                buffer[i++] = n & 0xff;
                                buffer[i++] = n >> 8;
                            }
                        }
#else
                        for (j = 0; j < 6; j++) buffer[i++] = 0;
#endif
                        break;
                    case 'N': // read proximity sensors
                        printStringToPort("// read proximity sensors");

                        for (j = 0; j < 8; j++) {
                            n = e_get_calibrated_prox(j); // or ? n=e_get_prox(j);
                            buffer[i++] = n & 0xff;
                            buffer[i++] = n >> 8;
                        }
                        break;
                    case 'O': // read light sensors
                        printStringToPort("// read light sensors");

                        for (j = 0; j < 8; j++) {
                            n = e_get_ambient_light(j);
                            buffer[i++] = n & 0xff;
                            buffer[i++] = n >> 8;
                        }
                        break;

                    case 'P': // set motor position
                        printStringToPort("// set motor position");

                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                            while (e_getchar_uart1(&c2) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                            while (e_getchar_uart2(&c2) == 0);
                        }
                        positionl = (unsigned char) c1 + ((unsigned int) c2 << 8);
                        if (gumstix_connected) { // Communicate with gumstix (i2c).

                        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                            while (e_getchar_uart1(&c2) == 0);
                        } else { // Communicate with the pc (usb).
                            while (e_getchar_uart2(&c1) == 0);
                            while (e_getchar_uart2(&c2) == 0);
                        }
                        positionr = (unsigned char) c1 + ((unsigned int) c2 << 8);
                        e_set_steps_left(positionl);
                        e_set_steps_right(positionr);
                        break;
                    case 'Q': // read encoders
                        printStringToPort("// read encoders");

                        n = e_get_steps_left();
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;
                        n = e_get_steps_right();
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;
                        break;

                    case 't': // temperature
                        printStringToPort("// temperature");

                        if (gumstix_connected == 0) {
                            buffer[i++] = 0;
                        } else {
                            buffer[i++] = getTemperature();
                        }
                        break;

                    case 'u': // get last micro volumes
                        printStringToPort("// get last micro volumes");

                        n = e_get_micro_volume(0);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;

                        n = e_get_micro_volume(1);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;

                        n = e_get_micro_volume(2);
                        buffer[i++] = n & 0xff;
                        buffer[i++] = n >> 8;
                        break;
                    case 'U': // get micro buffer
                        printStringToPort(" // get micro buffer");

//                        ptr = (char *) e_mic_scan;
//                    	if(gumstix_connected) { // Communicate with gumstix (i2c).
//
//                    	} else if (use_bt) { // Communicate with ESP32 (uart) => BT.
//                            e_send_uart1_char(ptr, 600); //send sound buffer
//                        } else { // Communicate with the pc (usb).
//                            e_send_uart2_char(ptr, 600); //send sound buffer
//                        }
//                        n = e_last_mic_scan_id; //send last scan
//                        buffer[i++] = n & 0xff;
                        break;
                    case 'W':
                        printStringToPort(" case W");

                        if (gumstix_connected == 0) {
                            if (use_bt) { // Communicate with ESP32 (uart) => BT.
                                while (e_getchar_uart1((char *) &mod) == 0);
                                while (e_getchar_uart1((char *) &reg) == 0);
                                while (e_getchar_uart1((char *) &val) == 0);
                            } else {
                                while (e_getchar_uart2((char *) &mod) == 0);
                                while (e_getchar_uart2((char *) &reg) == 0);
                                while (e_getchar_uart2((char *) &val) == 0);
                            }
                            e_i2cp_enable();
                            e_i2cp_write((char) mod, (char) reg, (char) val);    // write I2C
                            e_i2cp_disable();
                        }
                        break;
                    case 'w':    // RGB-panel extension command: write 9-LEDs + 8 IRs setting through I2C (RGB-panel I2C address = 176)
                        printStringToPort("// RGB-panel extension command: write 9-LEDs ...");

                        if (gumstix_connected == 0) {
                            e_i2cp_enable();
                            for (j = 0; j < 24; j++) {
                                while (e_getchar_uart1(&buffer[j]) == 0);
                            }
                            for (j = 146; j < 149; j++) {
                                while (e_getchar_uart1(&buffer[j - 122]) == 0);
                            }
                            for (j = 164; j < 172; j++) {
                                while (e_getchar_uart1(&buffer[j - 137]) == 0);
                            }
                            for (j = 0; j < 24; j++) {
                                e_i2cp_write((unsigned char) 176, (unsigned char) j, (unsigned char) buffer[j]);
                            }
                            for (j = 146; j < 149; j++) {
                                e_i2cp_write((unsigned char) 176, (unsigned char) j, (unsigned char) buffer[j - 122]);
                            }
                            for (j = 164; j < 172; j++) {
                                e_i2cp_write((unsigned char) 176, (unsigned char) j, (unsigned char) buffer[j - 137]);
                            }
                            e_i2cp_write((unsigned char) 176, (unsigned char) 145, (unsigned char) 1);
                            e_i2cp_disable();
                        }
                        break;

                    case 'X':
                        ;
                        //Время для шага итерации:
                        systime_t T_nach; //Начало временного промежутка [мс].
                        //Флаг:
                        uint8_t Potential_method_switch = 0; //Включение и отключение метода потенциалов.




                        if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1(&c1) == 0);
                        }
                        //printStringToPort("in case X");

                        switch (c1) {
                            case 2 :
                                // printStringToPort("in case X get c1=0000");
                                left_motor_set_pos(0);
                                right_motor_set_pos(0);

                                Right_encoder_old = (double) right_motor_get_pos();
                                Left_encoder_old = (double) right_motor_get_pos();

                                Update_odometer1(&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                 &Y_pos_actual);

                                right_motor_set_speed(300);
                                left_motor_set_speed(-300);


                                initTheta = Theta;
                                while (fabs((Theta - initTheta) * 180 / M_PI) < 20) {
//                                printNumberToPort(left_motor_get_pos());
                                    // if (SDU1.config->usbp->state == 4) {

//                                    Right_encoder_old = (double) right_motor_get_pos();
//                                    Left_encoder_old = (double) right_motor_get_pos();

                                    Update_odometer1(&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                     &Y_pos_actual);


//                                    chprintf((BaseSequentialStream *) &SDU1,
//                                             "Theta = %8.4lf, X =  %8.4lf, Y =  %8.4lf, condition = %8.4lf\r\n",
//                                             Theta * 180 / M_PI,
//                                             X_pos_actual, Y_pos_actual, abs((Theta - initTheta) * 180 / M_PI));
                                    // }
                                }
                                right_motor_set_speed(0);
                                left_motor_set_speed(0);
                                buffer[i++] = 'r';
                                buffer[i++] = 'r';
                                buffer[i++] = 'r';
                                buffer[i++] = 'r';
                                printStringToPort("in case X send rrrrr");
                                break;
                            case 1:
                                printStringToPort("in case X get c1=1111");

                                left_motor_set_pos(0);
                                right_motor_set_pos(0);


                                Right_encoder_old = (double) right_motor_get_pos();
                                Left_encoder_old = (double) right_motor_get_pos();

                                Update_odometer2(&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                 &Y_pos_actual);

                                right_motor_set_speed(-300);
                                left_motor_set_speed(300);


                                initTheta = Theta;
                                while (fabs((Theta - initTheta) * 180 / M_PI) < 20) {
//                                printNumberToPort(left_motor_get_pos());
                                    // if (SDU1.config->usbp->state == 4) {

//                                    Right_encoder_old = (double) right_motor_get_pos();
//                                    Left_encoder_old = (double) right_motor_get_pos();

                                    Update_odometer2(&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                     &Y_pos_actual);


//                                    chprintf((BaseSequentialStream *) &SDU1,
//                                             "Theta = %8.4lf, X =  %8.4lf, Y =  %8.4lf, condition = %8.4lf\r\n",
//                                             Theta * 180 / M_PI,
//                                             X_pos_actual, Y_pos_actual, abs((Theta - initTheta) * 180 / M_PI));
                                    // }
                                }

                                right_motor_set_speed(0);
                                left_motor_set_speed(0);
                                buffer[i++] = 'l';
                                buffer[i++] = 'l';
                                buffer[i++] = 'l';
                                buffer[i++] = 'l';
                                printStringToPort("in case X send llll");
                                break;

                            case 4:
                                //Робот вычисляет вектор отталкивания и поворачивается в соответствии с ним.
                                Theta = M_PI/2;
                                while(1)
                                {
                                    Filter_and_update_proximity_values(Prox_0_sens_values, Prox_1_sens_values, Prox_2_sens_values, Prox_5_sens_values, Prox_6_sens_values, Prox_7_sens_values, Size, Filtered_proximity_value);
                                    for(i = 0; i < 6; i++)
                                    {
                                        Proximity_values_mm[i] = Get_proximity_value_mm(Filtered_proximity_value[i]);
                                    }
                                    Update_obstacle_repulsion_vector(Proximity_values_mm, Theta, Sensor_angles, &Obstacle_repulsion_vector_angle, &Obstacle_repulsion_vector_length);
                                    Goal_angle = Obstacle_repulsion_vector_angle;
                                    Goal_point_distance = 0;

                                    for(j = 0; j < 30; j++)
                                    {
                                        Update_odometer2(&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual, &Y_pos_actual);
                                        Update_angle_disagreement(&Angle_disagreement, Theta, Goal_angle);
                                        Update_goal_speeds_values(&Linear_speed_goal, &Angular_speed_goal, Goal_point_distance, Dec_radius, Stop_radius, Linear_speed_max, Angle_disagreement, Angular_speed_max, Dec_angle, Potential_method_switch, Moving_speed_value_potential_method);
                                        Calculate_speeds(&Linear_speed_real, &Angular_speed_real, Linear_acc, Angular_acc, Linear_speed_goal, Angular_speed_goal);
                                        Set_wheels_speeds(Linear_speed_real, Angular_speed_real, h);
                                        chThdSleepMilliseconds(95);

                                        if (SDU1.config->usbp->state == USB_ACTIVE)
                                        {
                                            chprintf((BaseSequentialStream *)&SDU1, "Theta = %lf, Obstacle repulse angle = %lf\r\n", Theta*180/M_PI, Obstacle_repulsion_vector_angle*180/M_PI);
                                        }
                                    }
                                }
                                break;
                            case 5:
                                //Тест - в порт выводится угол и модуль вектора отталкивания.
                                while(1)
                                {

                                    Filter_and_update_proximity_values(Prox_0_sens_values, Prox_1_sens_values, Prox_2_sens_values, Prox_5_sens_values, Prox_6_sens_values, Prox_7_sens_values, Size, Filtered_proximity_value);

                                    for(i = 0; i < 6; i++)
                                    {
                                        Proximity_values_mm[i] = Get_proximity_value_mm(Filtered_proximity_value[i]);
                                    }
                                    Update_obstacle_repulsion_vector(Proximity_values_mm, Theta, Sensor_angles, &Obstacle_repulsion_vector_angle, &Obstacle_repulsion_vector_length);
                                    chprintf((BaseSequentialStream *)&SDU1, "Angle = %lf, Module = %lf\r\n", Obstacle_repulsion_vector_angle*180/M_PI, Obstacle_repulsion_vector_length);

                                    chThdSleepMilliseconds(100);
                                }
                                break;
                            case 6:
                                //Движение к точке - БЕЗ метода потенциалов.
                                Potential_method_switch = 0;
                                Set_goal_coordinates(0.3, 0.2, &X_goal_point, &Y_goal_point); //Установка целевых координат [м][м].
                                while(1) //Рабочий цикл для движения к целевой точке.
                                {
                                    T_nach = chVTGetSystemTime(); //Начало временного промежутка.

                                    Update_odometer2(&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual, &Y_pos_actual); //Обновляет положение центра робота и его угол ориентации.
                                    Update_goal_point_distance(&Goal_point_distance, X_pos_actual, Y_pos_actual, X_goal_point, Y_goal_point);
                                    Update_goal_point_angle(&Goal_point_angle, X_pos_actual, Y_pos_actual, X_goal_point, Y_goal_point);
                                    Goal_angle = Goal_point_angle;
                                    Update_angle_disagreement(&Angle_disagreement, Theta, Goal_angle);
                                    Update_goal_speeds_values(&Linear_speed_goal, &Angular_speed_goal, Goal_point_distance, Dec_radius, Stop_radius, Linear_speed_max, Angle_disagreement, Angular_speed_max, Dec_angle, Potential_method_switch, Moving_speed_value_potential_method);
                                    Calculate_speeds(&Linear_speed_real, &Angular_speed_real, Linear_acc, Angular_acc, Linear_speed_goal, Angular_speed_goal);
                                    Set_wheels_speeds(Linear_speed_real, Angular_speed_real, h);

                                    if (SDU1.config->usbp->state == USB_ACTIVE)
                                    {
                                        chprintf((BaseSequentialStream *)&SDU1, "Theta = %8.3lf, X =  %8.3lf, Y =  %8.3lf\r\n", Theta*180/M_PI, X_pos_actual, Y_pos_actual);
                                    }
                                    while(chVTGetSystemTime() <  (T_nach + 100));
                                }
                                break;
                        }

                        e_send_uart1_char(buffer, i); // send answer
                        while (e_uart1_sending());

                        break;
                    default: // silently ignored
                        break;
                }
                if (gumstix_connected) { // Communicate with gumstix (i2c).

                } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                    while (e_getchar_uart1(&c) == 0); // get next command
                } else { // Communicate with the pc (usb).
                    while (e_getchar_uart2(&c) == 0); // get next command
                }
            } while (c);
            if (i != 0) {
                if (gumstix_connected == 0) {
                    if (wait_cam) {
                        wait_cam = 0;
                        e_poxxxx_wait_img_ready();
                        memcpy(&buffer[cam_start_index], dcmi_get_last_image_ptr(), cam_size);
                    }
                }

                if (gumstix_connected) { // Communicate with gumstix (i2c).

                } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                    e_send_uart1_char(buffer, i); // send answer
                    while (e_uart1_sending());
                } else { // Communicate with the pc (usb).
//                    e_send_uart2_char(buffer, i); // send answer
//                    while (e_uart2_sending());
                }
            }
            // **** ascii mode ****
        } else if (c > 0) { // ascii mode
            printStringToPort("// ascii mode");
            if (gumstix_connected) { // Communicate with gumstix (i2c).

            } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                while (c == '\n' || c == '\r') e_getchar_uart1(&c);
            } else { // Communicate with the pc (usb).
                while (c == '\n' || c == '\r') e_getchar_uart2(&c);
            }
            buffer[0] = c;
            i = 1;
            if (gumstix_connected) { // Communicate with gumstix (i2c).

            } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                do if (e_getchar_uart1(&c)) buffer[i++] = c;
                while (c != '\n' && c != '\r');
            } else { // Communicate with the pc (usb).
                do if (e_getchar_uart2(&c)) buffer[i++] = c;
                while (c != '\n' && c != '\r');
            }
            buffer[i++] = '\0';

            if ((buffer[0] != 'b') && (buffer[0] != 'g')) {
                buffer[0] = toupper(buffer[0]); // we also accept lowercase letters
            }
            switch (buffer[0]) {
                case 'A': // read accelerometer
                    if (gumstix_connected == 0) {
                        sprintf(buffer, "a,%d,%d,%d\r\n", e_get_acc_filtered(0, 1), e_get_acc_filtered(1, 1),
                                e_get_acc_filtered(2, 1));
                    } else {
                        sprintf(buffer, "a,0,0,0\r\n");
                    }
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(1);
                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(2);
                    }
                    break;
                case 'b': // battery state
                    sprintf(buffer, "b,%d (%d%%)\r\n", getBatteryValueRaw(), getBatteryValuePercentage());
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(3);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(4);

                    }
                    break;
                case 'B': // set body led
                    sscanf(buffer, "B,%d\r", &LED_action);
                    e_set_body_led(LED_action);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("b\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("b\r\n");
                    }
                    break;
                case 'C': // read selector position
                    selector = SELECTOR0 + 2 * SELECTOR1 + 4 * SELECTOR2 + 8 * SELECTOR3;
                    sprintf(buffer, "c,%d\r\n", selector);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(5);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(6);

                    }
                    break;
                case 'D': // set motor speed
                    sscanf(buffer, "D,%d,%d\r", &speedl, &speedr);
                    e_set_speed_left(speedl);
                    e_set_speed_right(speedr);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("d\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("d\r\n");
                    }
                    break;
                case 'E': // read motor speed
                    sprintf(buffer, "e,%d,%d\r\n", speedl, speedr);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(7);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(8);

                    }
                    break;
                case 'F': // set front led
                    sscanf(buffer, "F,%d\r", &LED_action);
                    e_set_front_led(LED_action);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("f\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("f\r\n");
                    }
                    break;
#ifdef IR_RECEIVER
                case 'G':
                    sprintf(buffer, "g IR check : 0x%x, address : 0x%x, data : 0x%x\r\n", e_get_check(),
                            e_get_address(), e_get_data());
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(9);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(10);

                    }
                    break;
#endif
                case 'g': // gyro rates
                    if (gumstix_connected == 0) {
                        sprintf(buffer, "g,%d,%d,%d\r\n", getXAxisGyro(), getYAxisGyro(), getZAxisGyro());
                    } else {
                        sprintf(buffer, "g,0,0,0\r\n");
                    }
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(11);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(12);

                    }
                    break;

                case 'H': // ask for help
                    if (gumstix_connected) { // Communicate with gumstix (i2c).
                        // when working with the gumstix extension some commands are no more available
//                        uart2_send_static_text("\n");
//                        if (isEpuckVersion1_3() == 0) {
//                            uart2_send_static_text("\"A\"       Accelerometer\r\n");
//                        }
//                        if (isEpuckVersion1_3()) {
//                            uart2_send_static_text("\"b\"       Battery value\r\n");
//                        } else {
//                            uart2_send_static_text("\"b\"       Battery state (1=ok, 0=low)\r\n");
//                        }
//                        //uart2_send_static_text("\"B,#\"       Body led 0=off 1=on 2=inverse\r\n");
//                        uart2_send_static_text("\"C\"       Selector position\r\n");
//                        uart2_send_static_text("\"D,#,#\"   Set motor speed left,right\r\n");
//                        uart2_send_static_text("\"E\"       Get motor speed left,right\r\n");
//                        //uart2_send_static_text("\"F,#\"       Front led 0=off 1=on 2=inverse\r\n");
//#ifdef IR_RECEIVER
//                        uart2_send_static_text("\"G\"       IR receiver\r\n");
//#endif
//                        uart2_send_static_text("\"H\"       Help\r\n");
//                        //uart2_send_static_text("\"I\"         Get camera parameter\r\n");
//                        //uart2_send_static_text("\"J,#,#,#,#,#,#\" Set camera parameter mode,width,heigth,zoom(1,4 or 8),x1,y1\r\n");
//                        uart2_send_static_text("\"K\"       Calibrate proximity sensors\r\n");
//                        uart2_send_static_text("\"L,#,#\"   Led number,0=off 1=on 2=inverse\r\n");
//#ifdef FLOOR_SENSORS
//                        uart2_send_static_text("\"M\"       Floor sensors\r\n");
//#endif
//                        uart2_send_static_text("\"N\"       Proximity\r\n");
//                        uart2_send_static_text("\"O\"       Light sensors\r\n");
//                        uart2_send_static_text("\"P,#,#\"   Set motor position left,right\r\n");
//                        uart2_send_static_text("\"Q\"       Get motor position left,right\r\n");
//                        uart2_send_static_text("\"R\"       Reset e-puck\r\n");
//                        uart2_send_static_text("\"S\"       Stop e-puck and turn off leds\r\n");
//                        //uart2_send_static_text("\"T,#\"       Play sound 1-5 else stop sound\r\n");
//                        uart2_send_static_text("\"U\"       Get microphone amplitude\r\n");
//                        uart2_send_static_text("\"V\"       Version of SerCom\r\n");
//                        //uart2_send_static_text("\"W\"         Write I2C (mod,reg,val)\r\n");
//                        //uart2_send_static_text("\"Y\"         Read I2C val=(mod,reg)\r\n");
                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("\n");
                        uart1_send_static_text("\"A\"               Accelerometer\r\n");
                        uart1_send_static_text("\"B,#\"             Body led 0=off 1=on 2=inverse\r\n");
                        if (isEpuckVersion1_3()) {
                            uart1_send_static_text("\"b\"               Battery value\r\n");
                        } else {
                            uart1_send_static_text("\"b\"               Battery state (1=ok, 0=low)\r\n");
                        }
                        uart1_send_static_text("\"C\"               Selector position\r\n");
                        uart1_send_static_text("\"D,#,#\"           Set motor speed left,right\r\n");
                        uart1_send_static_text("\"E\"               Get motor speed left,right\r\n");
                        uart1_send_static_text("\"F,#\"             Front led 0=off 1=on 2=inverse\r\n");
#ifdef IR_RECEIVER
                        uart1_send_static_text("\"G\"               IR receiver\r\n");
#endif
                        if (isEpuckVersion1_3()) {
                            uart1_send_static_text("\"g\"               Gyro\r\n");
                        }
                        uart1_send_static_text("\"H\"               Help\r\n");
                        uart1_send_static_text("\"I\"               Get camera parameter\r\n");
                        uart1_send_static_text(
                                "\"J,#,#,#,#,#,#\"   Set camera parameter mode,width,heigth,zoom(1,4 or 8),x1,y1\r\n");
                        uart1_send_static_text("\"K\"               Calibrate proximity sensors\r\n");
                        uart1_send_static_text("\"L,#,#\"           Led number,0=off 1=on 2=inverse\r\n");
#ifdef FLOOR_SENSORS
                        uart1_send_static_text("\"M\"               Floor sensors\r\n");
#endif
                        uart1_send_static_text("\"N\"               Proximity\r\n");
                        uart1_send_static_text("\"O\"               Light sensors\r\n");
                        uart1_send_static_text("\"P,#,#\"           Set motor position left,right\r\n");
                        uart1_send_static_text("\"Q\"               Get motor position left,right\r\n");
                        uart1_send_static_text("\"R\"               Reset e-puck\r\n");
                        uart1_send_static_text("\"S\"               Stop e-puck and turn off leds\r\n");
                        uart1_send_static_text("\"T,#\"             Play sound 1-5 else stop sound\r\n");
                        if (isEpuckVersion1_3()) {
                            uart1_send_static_text("\"t\"               Temperature\r\n");
                        }
                        uart1_send_static_text("\"U\"               Get microphone amplitude\r\n");
                        uart1_send_static_text("\"V\"               Version of SerCom\r\n");
                        uart1_send_static_text("\"W\"               Write I2C (mod,reg,val)\r\n");
                        uart1_send_static_text("\"Y\"               Read I2C val=(mod,reg)\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("\n");
                        if (isEpuckVersion1_3() == 0) {
                            uart2_send_static_text("\"A\"       Accelerometer\r\n");
                        }
                        if (isEpuckVersion1_3()) {
                            uart2_send_static_text("\"b\"       Battery value\r\n");
                        } else {
                            uart2_send_static_text("\"b\"       Battery state (1=ok, 0=low)\r\n");
                        }
                        uart2_send_static_text("\"B,#\"       Body led 0=off 1=on 2=inverse\r\n");
                        uart2_send_static_text("\"C\"       Selector position\r\n");
                        uart2_send_static_text("\"D,#,#\"   Set motor speed left,right\r\n");
                        uart2_send_static_text("\"E\"       Get motor speed left,right\r\n");
                        uart2_send_static_text("\"F,#\"       Front led 0=off 1=on 2=inverse\r\n");
#ifdef IR_RECEIVER
                        uart2_send_static_text("\"G\"       IR receiver\r\n");
#endif
                        uart2_send_static_text("\"H\"       Help\r\n");
                        uart2_send_static_text("\"I\"         Get camera parameter\r\n");
                        uart2_send_static_text(
                                "\"J,#,#,#,#,#,#\" Set camera parameter mode,width,heigth,zoom(1,4 or 8),x1,y1\r\n");
                        uart2_send_static_text("\"K\"       Calibrate proximity sensors\r\n");
                        uart2_send_static_text("\"L,#,#\"   Led number,0=off 1=on 2=inverse\r\n");
#ifdef FLOOR_SENSORS
                        uart2_send_static_text("\"M\"       Floor sensors\r\n");
#endif
                        uart2_send_static_text("\"N\"       Proximity\r\n");
                        uart2_send_static_text("\"O\"       Light sensors\r\n");
                        uart2_send_static_text("\"P,#,#\"   Set motor position left,right\r\n");
                        uart2_send_static_text("\"Q\"       Get motor position left,right\r\n");
                        uart2_send_static_text("\"R\"       Reset e-puck\r\n");
                        uart2_send_static_text("\"S\"       Stop e-puck and turn off leds\r\n");
                        uart2_send_static_text("\"T,#\"       Play sound 1-5 else stop sound\r\n");
                        uart2_send_static_text("\"U\"       Get microphone amplitude\r\n");
                        uart2_send_static_text("\"V\"       Version of SerCom\r\n");
                        uart2_send_static_text("\"W\"         Write I2C (mod,reg,val)\r\n");
                        uart2_send_static_text("\"Y\"         Read I2C val=(mod,reg)\r\n");
                    }
                    break;
                case 'I':
                    if (gumstix_connected == 0) {
                        sprintf(buffer, "i,%d,%d,%d,%d,%d\r\n", cam_mode, cam_width, cam_heigth, cam_zoom, cam_size);
                        if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            uart1_send_text(buffer);
                            printNumberToPort(13);

                        } else { // Communicate with the pc (usb).
                            uart2_send_text(buffer);
                            printNumberToPort(14);

                        }
                    }
                    break;
                case 'J'://set camera parameter see also cam library
                    if (gumstix_connected == 0) {
                        cam_x1 = -1;
                        cam_y1 = -1;
                        sscanf(buffer, "J,%d,%d,%d,%d,%d,%d\r", &cam_mode, &cam_width, &cam_heigth, &cam_zoom, &cam_x1,
                               &cam_y1);
                        if (cam_mode == GREY_SCALE_MODE)
                            cam_size = cam_width * cam_heigth;
                        else
                            cam_size = cam_width * cam_heigth * 2;
                        if (cam_size >
                            IMAGE_MAX_SIZE) { // if desired settings too demanding set to a reasonable default
                            cam_mode = RGB_565_MODE;
                            cam_width = 40; // DEFAULT_WIDTH;
                            cam_heigth = 40; // DEFAULT_HEIGHT;
                            cam_size = cam_width * cam_heigth * 2;
                        }
                        e_poxxxx_init_cam();
                        if (cam_x1 == -1) { // user did not specify: take default
                            cam_x1 = (ARRAY_WIDTH - cam_width * cam_zoom) / 2;
                        }
                        if (cam_y1 == -1) { // user did not specify: take default
                            cam_y1 = (ARRAY_HEIGHT - cam_heigth * cam_zoom) / 2;
                        }
                        e_poxxxx_config_cam(cam_x1, cam_y1, cam_width * cam_zoom, cam_heigth * cam_zoom, cam_zoom,
                                            cam_zoom, cam_mode);
                        e_poxxxx_write_cam_registers();
                        if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            uart1_send_static_text("j\r\n");
                        } else { // Communicate with the pc (usb).
                            uart2_send_static_text("j\r\n");
                        }
                    }
                    break;
                case 'K': // calibrate proximity sensors
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("k, Starting calibration - Remove any object in sensors range\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("k, Starting calibration - Remove any object in sensors range\r\n");
                    }
                    int long t;
                    e_set_led(8, 1);
                    for (t = 0; t < 1000000; ++t);
                    chThdSleepMilliseconds(400);
                    e_led_clear();
                    for (t = 0; t < 10000; ++t);
                    chThdSleepMilliseconds(100);
                    e_calibrate_ir();
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("k, Calibration finished\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("k, Calibration finished\r\n");
                    }
                    break;
                case 'L': // set led
                    sscanf(buffer, "L,%d,%d\r", &LED_nbr, &LED_action);
                    e_set_led(LED_nbr, LED_action);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("l\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("l\r\n");
                    }
                    break;
                case 'M': // read floor sensors (optional)
#ifdef FLOOR_SENSORS
                    if (gumstix_connected == 0) {
                        sprintf(buffer, "m,%d,%d,%d\r\n", get_ground_prox(0), get_ground_prox(1), get_ground_prox(2));
                        if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            uart1_send_text(buffer);
                            printNumberToPort(15);

                        } else { // Communicate with the pc (usb).
                            uart2_send_text(buffer);
                            printNumberToPort(16);

                        }
                    }
#else
                if (gumstix_connected) { // Communicate with gumstix (i2c).

                } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                    uart1_send_static_text("m,0,0,0\r\n");
                } else { // Communicate with the pc (usb).
                    uart2_send_static_text("m,0,0,0\r\n");
                }
#endif
                    break;
                case 'N': // read proximity sensors
                    sprintf(buffer, "n,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                            e_get_calibrated_prox(0), e_get_calibrated_prox(1), e_get_calibrated_prox(2),
                            e_get_calibrated_prox(3),
                            e_get_calibrated_prox(4), e_get_calibrated_prox(5), e_get_calibrated_prox(6),
                            e_get_calibrated_prox(7));
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(17);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(18);

                    }
                    break;
                case 'O': // read ambient light sensors
                    sprintf(buffer, "o,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                            e_get_ambient_light(0), e_get_ambient_light(1), e_get_ambient_light(2),
                            e_get_ambient_light(3),
                            e_get_ambient_light(4), e_get_ambient_light(5), e_get_ambient_light(6),
                            e_get_ambient_light(7));
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(19);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(20);

                    }
                    break;
                case 'P': // set motor position
                    sscanf(buffer, "P,%d,%d\r", &positionl, &positionr);
                    e_set_steps_left(positionl);
                    e_set_steps_right(positionr);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("p\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("p\r\n");
                    }
                    break;
                case 'Q': // read motor position
                    sprintf(buffer, "q,%d,%d\r\n", e_get_steps_left(), e_get_steps_right());
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                    }
                    break;
                case 'R': // reset
//                    if (use_bt) {
//                        uart1_send_static_text("r\r\n");
//                    } else {
//                        uart2_send_static_text("r\r\n");
//                    }
//                    RESET();
                    break;
                case 'S': // stop
                    e_set_speed_left(0);
                    e_set_speed_right(0);
                    e_set_led(8, 0);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("s\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("s\r\n");
                    }
                    break;
                case 'T': // stop
                    sscanf(buffer, "T,%d", &sound);
                    if (first == 0) {
                        e_init_sound();
                        first = 1;
                    }
                    switch (sound) {
                        case 1:
                            playMelody(MARIO, ML_FORCE_CHANGE, NULL);//e_play_sound(0, 2112);
                            break;
                        case 2:
                            playMelody(UNDERWORLD, ML_FORCE_CHANGE, NULL);//e_play_sound(2116, 1760);
                            break;
                        case 3:
                            playMelody(STARWARS, ML_FORCE_CHANGE, NULL);//e_play_sound(3878, 3412);
                            break;
                        case 4:
                            e_play_sound(7294, 3732);
                            break;
                        case 5:
                            e_play_sound(11028, 8016);
                            break;
                        default:
                            e_close_sound();
                            stopCurrentMelody();
                            first = 0;
                            break;
                    }
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("t\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("t\r\n");
                    }
                    break;

                case 't': // temperature
                    if (gumstix_connected == 0) {
                        sprintf(buffer, "t,%d\r\n", getTemperature());
                    } else {
                        sprintf(buffer, "t,0\r\n");
                    }
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(21);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(22);

                    }
                    break;

                case 'U':
                    sprintf(buffer, "u,%d,%d,%d\r\n", e_get_micro_volume(0), e_get_micro_volume(1),
                            e_get_micro_volume(2));
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(23);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(24);

                    }
                    break;
                case 'V': // get version information
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("v,Version 2.0 January 2018 GCtronic\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("v,Version 2.0 January 2018 GCtronic\r\n");
                    }
                    sprintf(buffer, "HW version: %X\r\n", HWversion);
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_text(buffer);
                        printNumberToPort(25);

                    } else { // Communicate with the pc (usb).
                        uart2_send_text(buffer);
                        printNumberToPort(26);

                    }
                    break;
                case 'W': // write I2C message
                    if (gumstix_connected == 0) {
                        sscanf(buffer, "W,%d,%d,%d\r", &mod, &reg, &val);
                        e_i2cp_enable();
                        e_i2cp_write((char) mod, (char) reg, (char) val); // write I2C
                        e_i2cp_disable();
                        if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            uart1_send_static_text("w\r\n");
                        } else { // Communicate with the pc (usb).
                            uart2_send_static_text("w\r\n");
                        }
                    }
                    break;
                case 'Y': // read I2C message
                    if (gumstix_connected == 0) {
                        sscanf(buffer, "Y,%d,%d\r", &mod, &reg);
                        sprintf(buffer, "y,%d,%d\r\n", mod, reg);
                        if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            uart1_send_text(buffer);
                            printNumberToPort(27);

                        } else { // Communicate with the pc (usb).
                            uart2_send_text(buffer);
                            printNumberToPort(28);

                        }
                        e_i2cp_enable();
                        val = e_i2cp_read((char) mod, (char) reg); // read I2C
                        e_i2cp_disable();
                        sprintf(buffer, "y,%d\r\n", val);
                        uart1_send_text(buffer);
                        printNumberToPort(29);

                        if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            uart1_send_text(buffer);
                            printNumberToPort(30);

                        } else { // Communicate with the pc (usb).
                            uart2_send_text(buffer);
                            printNumberToPort(31);

                        }
                    }
                    break;
                case 'Z': // scann I2C addresses
                    if (gumstix_connected == 0) {
                        for (j = 2; j < 255; j = j + 2) {
                            e_i2cp_enable();
                            val = e_i2cp_read((char) j, 0); // read I2C
                            e_i2cp_disable();
                            if (val >= 0) {
                                sprintf(buffer, "%d: %d\r\n", j, val);
                                if (use_bt) { // Communicate with ESP32 (uart) => BT.
                                    uart1_send_text(buffer);
                                    printNumberToPort(32);

                                } else { // Communicate with the pc (usb).
                                    uart2_send_text(buffer);
                                    printNumberToPort(33);

                                }
                            }
                        }
                    }
                    break;
                default:
                    if (gumstix_connected) { // Communicate with gumstix (i2c).

                    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                        uart1_send_static_text("z,Command not found\r\n");
                    } else { // Communicate with the pc (usb).
                        uart2_send_static_text("z,Command not found\r\n");
                    }
                    break;
            }
        }

        printStringToPort("end of loop");
    }
    printStringToPort("end");
}
