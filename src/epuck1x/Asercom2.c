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


void Set_goal_coordinates (double X_goal_entered, double Y_goal_entered, double *X_goal_point, double *Y_goal_point) {
    //Устанавливает целевые координаты точки.
    *X_goal_point = X_goal_entered;
    *Y_goal_point = Y_goal_entered;
}

void
Update_goal_point_distance (double *Goal_point_distance, double X_pos_actual, double Y_pos_actual, double X_goal_point,
                            double Y_goal_point) {
    //Обновляет значение расстояния до цели.
    if (X_pos_actual == X_goal_point && Y_pos_actual == Y_goal_point) //Если координаты робота и цели совпадают.
        *Goal_point_distance = 0; //[м]
    else //Иначе вычисление расстояния (гипотенузы по двум катетам).
        *Goal_point_distance = sqrt (pow (X_goal_point - X_pos_actual, 2) + pow (Y_goal_point - Y_pos_actual, 2)); //[м]
}

void Update_goal_point_angle (double *Goal_point_angle, double X_pos_actual, double Y_pos_actual, double X_goal_point,
                              double Y_goal_point) {
    /*Функция обновляет угол ориентации до цели для точки с роботом относительно Ox, [рад] на основе текущего положения робота.
    Т.е. это тот угол, на который робот должен повернуть, чтобы быть направленным к цели.
    Здесь не продуман случай, когда робот достиг целевой точки и угол уже никак нельзя
    вычислить*/

    int Quadrant; //Квадрант определяется для системы координат, центр которой совпадает с центром робота, а оси параллельны глобальной СК.
    if ((X_goal_point - X_pos_actual) * (Y_goal_point - Y_pos_actual) == 0)
        Quadrant = 0; //Неопределенный квадрант (точка находится на оси).
    if ((X_goal_point - X_pos_actual) * (Y_goal_point - Y_pos_actual) > 0) //Определение квадранта точки на плоскости.
        Quadrant = (((X_goal_point - X_pos_actual) > 0 && (Y_goal_point - Y_pos_actual) > 0) ? 1 : 3);
    if ((X_goal_point - X_pos_actual) * (Y_goal_point - Y_pos_actual) < 0)
        Quadrant = (((X_goal_point - X_pos_actual) > 0 && (Y_goal_point - Y_pos_actual) < 0) ? 4 : 2);

    if ((X_goal_point - X_pos_actual) != 0 || (Y_goal_point - Y_pos_actual) != 0)
        *Goal_point_angle = asin (fabs (Y_goal_point - Y_pos_actual) / sqrt (pow (X_goal_point - X_pos_actual, 2) +
                                                                             pow (Y_goal_point - Y_pos_actual,
                                                                                  2))); //Вычисление угла прямоугольного треугольника с катетами X_goal и Y_goal, [рад].
    else Quadrant = 0;

    switch (Quadrant) //Вычисление угла в локальной СК в зависимости от квадранта.
    {
        case 1:
            break;
        case 2:
            *Goal_point_angle = M_PI - *Goal_point_angle;
            break;
        case 3:
            *Goal_point_angle = M_PI + *Goal_point_angle;
            break;
        case 4:
            *Goal_point_angle = 2 * M_PI - *Goal_point_angle;
            break;
        case 0:
            if ((X_goal_point - X_pos_actual) == 0 && (Y_goal_point - Y_pos_actual) > 0)
                *Goal_point_angle = M_PI / 2; //90 градусов.
            if ((X_goal_point - X_pos_actual) == 0 && (Y_goal_point - Y_pos_actual) < 0)
                *Goal_point_angle = 1.5 * M_PI; //270 градусов.
            if ((X_goal_point - X_pos_actual) > 0 && (Y_goal_point - Y_pos_actual) == 0)
                *Goal_point_angle = 0; //0 градусов.
            if ((X_goal_point - X_pos_actual) < 0 && (Y_goal_point - Y_pos_actual) == 0)
                *Goal_point_angle = M_PI; //180 градусов.
            if ((X_goal_point - X_pos_actual) == 0 && (Y_goal_point - Y_pos_actual) == 0)
                *Goal_point_angle = M_PI / 2; //Позиция робота совпадает с целевой точкой.
            break;
    }
}

void Update_angle_disagreement (double *Angle_disagreement, double Theta, double Goal_angle) {
    //Обновление угла рассогласования с учетом рациональности поворота (чтобы робот поворачивал на угол, меньший или равный 180 град.
    //На входе два угла в [рад] - угол ориентации робота отн. оси Ох (Theta) и целевой угол (Goal_angle), так же отн. оси Ох.
    //На выходе - угол в [рад], обозначающий рассогласование.
    //При этом модуль |величина рассогласования| всегда <= 180 град.
    *Angle_disagreement = (Goal_angle - Theta);

    if ((Goal_angle - Theta) > M_PI) {
        *Angle_disagreement = -(2 * M_PI - (Goal_angle - Theta));
    }
    if ((Goal_angle - Theta) < -M_PI) {
        *Angle_disagreement = 2 * M_PI + (Goal_angle - Theta);
    }
}

void Update_odometer2 (double *Left_encoder_old, double *Right_encoder_old, double *Theta, double *X_pos_actual,
                       double *Y_pos_actual) {
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

    Left_encoder_actual = (double) left_motor_get_pos (); //Обновляем данные энкодеров.
    Right_encoder_actual = (double) right_motor_get_pos (); //Получаем актуальные значения [шаги].

    Right_length = ((Right_encoder_actual - *Right_encoder_old) / 1000) *
                   0.1288; //Количество пройденных шагов делим на количество шагов в одном обороте колеса, получаем количество оборотов колеса, умножаем на длинну окружности колеса.
    Left_length = ((Left_encoder_actual - *Left_encoder_old) / 1000) * 0.1288; //[м].

    if (Right_length != Left_length) {
        Alpha_odometer = (Right_length - Left_length) / 0.053; //В радианах. 0.053 - расстояние между колесами [м].
        Radius_odometer = Left_length / Alpha_odometer;

        Center_x_odometer =
                *X_pos_actual - (Radius_odometer + 0.0265) * sin (*Theta); //0.0265 - полурасстояние между колесами [м].
        Center_y_odometer = *Y_pos_actual - (Radius_odometer + 0.0265) * (-cos (*Theta));

        *Theta = fmod ((*Theta + Alpha_odometer + 2 * M_PI), 2 * M_PI);

        *X_pos_actual = Center_x_odometer + (Radius_odometer + 0.0265) * sin (*Theta);
        *Y_pos_actual = Center_y_odometer + (Radius_odometer + 0.0265) * (-cos (*Theta));
    }

    if (Right_length == Left_length) {
        *X_pos_actual = *X_pos_actual + Left_length * cos (*Theta);
        *Y_pos_actual = *Y_pos_actual + Left_length * sin (*Theta);
    }

    *Left_encoder_old = (double) left_motor_get_pos (); //Обновляем данные энкодеров.
    *Right_encoder_old = (double) right_motor_get_pos (); //Получаем актуальные значения [шаги].
}

void Update_goal_speeds_values (double *Linear_speed_goal, double *Angular_speed_goal, double Goal_point_distance,
                                double Dec_radius, double Stop_radius, double Linear_speed_max,
                                double Angle_disagreement, double Angular_speed_max, double Dec_angle,
                                uint8_t Potential_method_switch, double Moving_speed_value_potential_method) {
    /*Обновляются целевые угловая и линейная скорость исходя из расстояния до цели и угла рассогласования
    (выдаваемая скорость не превышает максимального значения)*/
    //Вычисление целевой линейной скорости:
    switch (Potential_method_switch) {
        case 0:
            if (Goal_point_distance > Dec_radius) {
                *Linear_speed_goal = Linear_speed_max; //До радиуса торможения модуль линейной скорости максимален.
            } else {
                if (Goal_point_distance <= Dec_radius && Goal_point_distance > Stop_radius) {
                    *Linear_speed_goal = (Goal_point_distance / Dec_radius) *
                                         Linear_speed_max; //Внутри радиуса торможения модуль линейной скорости пропорционален расстоянию до цели.
                } else {
                    if (Goal_point_distance <= Stop_radius) {
                        *Linear_speed_goal = 0; //Внутри радиуса остановки - остановка.
                    }
                }
            }
            break;

        case 1: //Включен метод потенц. полей.
            *Linear_speed_goal = Moving_speed_value_potential_method;

            if (Goal_point_distance <= Stop_radius) {
                *Linear_speed_goal = 0; //Внутри радиуса остановки - остановка.
            }
            break;
    }

    //Вычисление целевой угловой скорости:
    if (fabs (Angle_disagreement) <= 0.015) {
        //Если рассогласование лежит в малых пределах, то угловая целевая скорость - ноль.
        *Angular_speed_goal = 0;
    } else {
        if (fabs (Angle_disagreement) <= Dec_angle) {
            //Если угол рассогласования меньше или равен углу торможения, то торможение.
            *Angular_speed_goal = (Angle_disagreement / Dec_angle) * Angular_speed_max;
        } else {
            *Angular_speed_goal = (fabs (Angle_disagreement) / (Angle_disagreement)) * Angular_speed_max;
        }
    }
}

void Calculate_speeds (double *Linear_speed_real, double *Angular_speed_real, double Linear_acc, double Angular_acc,
                       double Linear_speed_goal, double Angular_speed_goal) {
    /*Обновляет реальную угловую и линейную скорости колес с учетом ускорения и текущей угловой и линейной скорости.
    Под "реальной" понимается та скорость, которая будет установлена на колеса.
    Это сделано для того, чтобы был эффект инерционности и скорость менялась не скачком, а с нарастанием (ограниченное приращение скорости).*/
    if ((Linear_speed_goal - *Linear_speed_real) >
        0) { //Анимация для ускорения (зеленый цвет), торможения (красный) и постоянной скорости (синий).
        set_rgb_led (0, 0, 10, 0);
        set_rgb_led (1, 0, 10, 0);
        set_rgb_led (2, 0, 10, 0);
        set_rgb_led (3, 0, 10, 0);
    } else {
        if ((Linear_speed_goal - *Linear_speed_real) < 0) {
            set_rgb_led (0, 10, 0, 0);
            set_rgb_led (1, 10, 0, 0);
            set_rgb_led (2, 10, 0, 0);
            set_rgb_led (3, 10, 0, 0);
        } else {
            set_rgb_led (0, 0, 0, 10);
            set_rgb_led (1, 0, 0, 10);
            set_rgb_led (2, 0, 0, 10);
            set_rgb_led (3, 0, 0, 10);
        }
    }

    //Вычисление реальной линейной скорости:
    if (fabs (Linear_speed_goal - *Linear_speed_real) > Linear_acc) /*Если разница между целевой и реальной скоростью превышает величину возможного ускорения,
	то можно изменить скорость только на величину ускорения*/
    {
        *Linear_speed_real = *Linear_speed_real + (fabs (Linear_speed_goal - *Linear_speed_real) /
                                                   (Linear_speed_goal - *Linear_speed_real)) * Linear_acc;
    } else //Если разница между целевой и реальной скоростью не превышает величину возможного ускорения, то сразу переходим к целевой скорости.
    {
        *Linear_speed_real = Linear_speed_goal;
    }

    //Вычисление реальной угловой скорости (логика такая же, что и с линейной):
    if (fabs (Angular_speed_goal - *Angular_speed_real) > Angular_acc) {
        *Angular_speed_real = *Angular_speed_real + (fabs (Angular_speed_goal - *Angular_speed_real) /
                                                     (Angular_speed_goal - *Angular_speed_real)) * Angular_acc;
    } else {
        *Angular_speed_real = Angular_speed_goal;
    }
}

void Set_wheels_speeds (double Linear_speed_real, double Angular_speed_real, double h) {
    /*Эта функция устанавливает скорости на колеса, переводя требуемую линейную и угловую скорости в [шаги/с] для каждого колеса.*/
    double Left_speed_ms, Right_speed_ms; //Скорость лев. и прав. колес в [м/с].
    int16_t Left_speed_steps, Right_speed_steps; //Скорость лев. и прав. колес [шаги/с].

    //Положительная угл. скорость - против часовой стрелки.
    Left_speed_ms = Linear_speed_real - h * Angular_speed_real;
    Right_speed_ms = Linear_speed_real + h * Angular_speed_real;

    //Из метров в секунду в шаги в секунду:
    Left_speed_steps = (int16_t) 1200 * Left_speed_ms / (1.2 * M_PI * 0.041);
    Right_speed_steps = (int16_t) 1200 * Right_speed_ms / (1.2 * M_PI * 0.041);

    //Установка:
    left_motor_set_speed (Left_speed_steps);
    right_motor_set_speed (Right_speed_steps);
}

void Filter_and_update_proximity_values (int *Prox_0_sens_values, int *Prox_1_sens_values, int *Prox_2_sens_values,
                                         int *Prox_5_sens_values, int *Prox_6_sens_values, int *Prox_7_sens_values,
                                         int Size, int *Filtered_proximity_value) {
    /*На вход подается 6 массивов размерности 5 с данными с каждого переднего ИК-дальномера.
    Применяется медианный фильтр (подходит под задачу, так как в последовательности данных с ИК-дальномера
    есть одиночные мощные импульсы, а остальные данные достаточно схожи). На выходе фильтра - массив размерности 6 с отфильтрованными
    значениями для каждого ИК-дальномера.*/
    int temp;

    //Занесение неотфильтрованных данных в массивы:
    for (int i = 0; i < Size; i++) {
        Prox_0_sens_values[i] = get_calibrated_prox (0);
        Prox_1_sens_values[i] = get_calibrated_prox (1);
        Prox_2_sens_values[i] = get_calibrated_prox (2);
        Prox_5_sens_values[i] = get_calibrated_prox (5);
        Prox_6_sens_values[i] = get_calibrated_prox (6);
        Prox_7_sens_values[i] = get_calibrated_prox (7);
        chThdSleepMilliseconds(5); //todo ask why u add sleep here?
    }

    //Сортировка по возрастанию для каждого массива:
    for (int i = 0; i < Size - 1; i++) {
        for (int j = 0; j < Size - i - 1; j++) {
            if (Prox_0_sens_values[j] > Prox_0_sens_values[j + 1]) {
                temp = Prox_0_sens_values[j];
                Prox_0_sens_values[j] = Prox_0_sens_values[j + 1];
                Prox_0_sens_values[j + 1] = temp;
            }
            if (Prox_1_sens_values[j] > Prox_1_sens_values[j + 1]) {
                temp = Prox_1_sens_values[j];
                Prox_1_sens_values[j] = Prox_1_sens_values[j + 1];
                Prox_1_sens_values[j + 1] = temp;
            }
            if (Prox_2_sens_values[j] > Prox_2_sens_values[j + 1]) {
                temp = Prox_2_sens_values[j];
                Prox_2_sens_values[j] = Prox_2_sens_values[j + 1];
                Prox_2_sens_values[j + 1] = temp;
            }
            if (Prox_5_sens_values[j] > Prox_5_sens_values[j + 1]) {
                temp = Prox_5_sens_values[j];
                Prox_5_sens_values[j] = Prox_5_sens_values[j + 1];
                Prox_5_sens_values[j + 1] = temp;
            }
            if (Prox_6_sens_values[j] > Prox_6_sens_values[j + 1]) {
                temp = Prox_6_sens_values[j];
                Prox_6_sens_values[j] = Prox_6_sens_values[j + 1];
                Prox_6_sens_values[j + 1] = temp;
            }
            if (Prox_7_sens_values[j] > Prox_7_sens_values[j + 1]) {
                temp = Prox_7_sens_values[j];
                Prox_7_sens_values[j] = Prox_7_sens_values[j + 1];
                Prox_7_sens_values[j + 1] = temp;
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

double Get_proximity_value_mm (int Data_from_sensor) {
    /*Возвращает значение расстояния до препятствия в [мм].
    На основе значения данных с какого-либо ИК-дальномера.
    Входное значение - данные с сенсора. Используется аппроксимация четырьмя графиками.
    На вход подается отфильтрованное значение с одного ИК-датчика.*/
    if (Data_from_sensor >= 3116 && Data_from_sensor <= 3900)
        return sqrt ((-Data_from_sensor + 3900.0) / 16.0);
    if (Data_from_sensor >= 2300 && Data_from_sensor < 3116)
        return ((-2.0 * Data_from_sensor + 11944.0) / 816.0);
    if (Data_from_sensor >= 276 && Data_from_sensor < 2300)
        return (10000.0 / (Data_from_sensor + 200.0) + 5);
    if (Data_from_sensor < 276)
        return ((-24.0 * Data_from_sensor + 11460.0) / 186.0);
    return 0;

    //todo ask don't understand the numbers here?
}

void Update_obstacle_repulsion_vector (double Proximity_values_mm[], double Theta, double Sensor_angles[],
                                       double *Vector_angle, double *Vector_length) {
    /*На вход подается массив с отфильтрованными расстояниями в [мм] до препятствий от 6-ти передних ИК_дальномеров.
     Затем вычисляется направление (угол [рад]) и модуль результирующего вектора отталкивания относительно глобальной оси Ox.*/

    double X_repulse = 0.0; //Суммарный X всех векторов.
    double Y_repulse = 0.0; //Суммарный Y всех векторов.

    for (int i = 0; i < 6; i++) //Вычисление координат результирующего вектора.
    {
        if (Proximity_values_mm[i] >=
            60.0) //Если больше 60 [мм], считаем, что препятствия нет (предел видимости ИК-дальномера).
        {
            continue;
        }
        X_repulse = X_repulse + (60.0 - Proximity_values_mm[i]) / 60.0 * cos (Sensor_angles[i]);
        Y_repulse = Y_repulse + (60.0 - Proximity_values_mm[i]) / 60.0 * sin (Sensor_angles[i]);
    }

    if (X_repulse == 0.0 && Y_repulse == 0.0) //Если препятствий нет, то модуль вектора отталкивания равен 0.
    {
        *Vector_length = 0;
        *Vector_angle = Theta;
    } else //Если препятствия есть, то нормируем модуль до единицы.
    {
        *Vector_angle = acos (X_repulse / sqrt (pow (X_repulse, 2) + pow (Y_repulse,
                                                                          2))); //Угол отн. поперечной подвижнойоси, закрепленной на роботе (а нужен относительно глобальной оси Ox) [рад].
        *Vector_angle = fmod (*Vector_angle + (Theta - M_PI / 2) + 2 * M_PI + M_PI, 2 * M_PI); //Угол отн. Ox [рад].
        *Vector_length = sqrt (pow (X_repulse, 2) + pow (Y_repulse, 2));
    }
}

void Update_moving_vector_for_potential_method (double Obstacle_repulsion_vector_angle,
                                                double Obstacle_repulsion_vector_length, double Goal_point_angle,
                                                double *Moving_speed_value_potential_method,
                                                double *Moving_angle_potential_method, double Linear_speed_max) {
    /*На входе - угол и модуль вектора отталкивания от препятствий и угол до целевой точки.
     На выходе - направление и скорость те, которые вычислены для метода потенциалов.*/
    double X = 0.0;
    double Y = 0.0;
    double Vector_length = 0.0;

    if (Obstacle_repulsion_vector_length != 0.0) //Вектор отталкивания от препятствий не нулевой.
    {
        X = cos (Obstacle_repulsion_vector_angle) +
            cos (Goal_point_angle); //Считаем координаты результирующего вектора.
        Y = sin (Obstacle_repulsion_vector_angle) + sin (Goal_point_angle);
        Vector_length = sqrt (pow (X, 2) + pow (Y, 2)); //Вычисляем модуль.

        *Moving_speed_value_potential_method = (Vector_length / 2.0) * Linear_speed_max; //Значение скорости.
        *Moving_angle_potential_method = fmod (acos (X / Vector_length) + 2 * M_PI, 2 * M_PI); //Угол направления.
    } else //Вектор отталкивания от препятствий нулевой.
    {
        *Moving_speed_value_potential_method = Linear_speed_max; //Значение скорости.
        *Moving_angle_potential_method = Goal_point_angle; //Угол направления.
    }
}


void Update_odometer1 (double *Left_encoder_old, double *Right_encoder_old, double *Theta, double *X_pos_actual,
                       double *Y_pos_actual) {
/*Функция обновляет координаты центра робота и угол его ориентации относительно Ox, используя старые значения счетчиков шагов энкодеров и текущих координат и угла ориентации робота.*/

    double Left_encoder_actual = 0; //Количество шагов на левом колесе [шаги].
    double Right_encoder_actual = 0; //Количество шагов на правом колесе [шаги].
    double Left_length = 0; //Расстояние, пройденное левым колесом [м].
    double Right_length = 0; //Расстояние, пройденное правым колесом [м].
    double Alpha_odometer = 0; //Угол сегмента окружности при повороте [рад].
    double Radius_odometer = 0; //Радиус сегмента окружности при повороте [м].
    double Center_x_odometer = 0, Center_y_odometer = 0; //Координаты центра окружности поворота [м][м].

    Left_encoder_actual = (double) left_motor_get_pos (); //Обновляем данные энкодеров.
    Right_encoder_actual = (double) right_motor_get_pos (); //Получаем актуальные значения [шаги].

    Right_length = ((Right_encoder_actual - *Right_encoder_old) / 1000) *
                   0.1288; //Количество пройденных шагов делим на количество шагов в одном обороте колеса, получаем количество оборотов колеса, умножаем на длину окружности колеса [м].
    Left_length = ((Left_encoder_actual - *Left_encoder_old) / 1000) * 0.1288;

    if (Right_length != Left_length) //Если левое и правое колеса прошли не одинаковые расстояния.
    {
        Alpha_odometer = (Right_length - Left_length) / 0.053; //В радианах. 0.053 - расстояние между колесами [м].
        Radius_odometer = Left_length / Alpha_odometer;

        Center_x_odometer =
                *X_pos_actual - (Radius_odometer + 0.0265) * sin (*Theta); //0.0265 - полурасстояние между колесами [м].
        Center_y_odometer = *Y_pos_actual - (Radius_odometer + 0.0265) * (-cos (*Theta));

        *Theta = fmod ((*Theta + Alpha_odometer + 2 * M_PI), 2 * M_PI);

        *X_pos_actual = Center_x_odometer + (Radius_odometer + 0.0265) * sin (*Theta);
        *Y_pos_actual = Center_y_odometer + (Radius_odometer + 0.0265) * (-cos (*Theta));
    }

    if (Right_length == Left_length) //Если левое и правое колеса прошли  одинаковые расстояния.
    {
        *X_pos_actual = *X_pos_actual + Left_length * cos (*Theta);
        *Y_pos_actual = *Y_pos_actual + Left_length * sin (*Theta);
    }

    *Left_encoder_old = (double) left_motor_get_pos (); //Обновляем данные энкодеров.
    *Right_encoder_old = (double) right_motor_get_pos ();
}

void printBufferToPort (void) {
    if (SDU1.config->usbp->state == 4) {
        chprintf ((BaseSequentialStream *) &SDU1, "%s\n", buffer);
    }
}

void printNumberToPort (int number) {
    if (SDU1.config->usbp->state == 4) {
        chprintf ((BaseSequentialStream *) &SDU1, "%d\n", number);
    }
}

void printStringToPort (char *string) {
    if (SDU1.config->usbp->state == 4) {
        chprintf ((BaseSequentialStream *) &SDU1, "%s\n", string);
    }
}

void debug_coord (int x, int y) {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
        chprintf ((BaseSequentialStream *) &SDU1, "in debug  x=%d, y=%d\r\n", x, y);
    }

    char buf[18];
    int buf_it = 0;

    buf[buf_it++] = '5';
    buf[buf_it++] = '5';

    //init debug mode
    buf[buf_it++] = '1';
    buf[buf_it++] = '1';
    buf[buf_it++] = '1';
    buf[buf_it++] = '1';

    //len = 8?
    buf[buf_it++] = '0';
    buf[buf_it++] = '0';
    buf[buf_it++] = '0';
    buf[buf_it++] = '8';

    int it;
    for (it = 0; it < 4; it++) {
        buf[buf_it++] = x / ((int) pow (10, 3 - it)) + '0';
        x -= (x / ((int) pow (10, 3 - it))) * ((int) pow (10, 3 - it));
    }
    for (it = 0; it < 4; it++) {
        buf[buf_it++] = y / ((int) pow (10, 3 - it)) + '0';
        y -= (y / ((int) pow (10, 3 - it))) * ((int) pow (10, 3 - it));

    }


    if (SDU1.config->usbp->state == USB_ACTIVE) {
        chprintf ((BaseSequentialStream *) &SDU1, "nearly end debug buf=%s, buf_it=%d, x=%d, y=%d\r\n", buf, buf_it, x,
                  y);
    }

    e_send_uart1_char (buf, buf_it); // send answer

    if (SDU1.config->usbp->state == USB_ACTIVE) {
        chprintf ((BaseSequentialStream *) &SDU1, "\nEND debug buf=%s, buf_it=%d, x=%d, y=%d\r\n", buf, buf_it, x, y);
    }

}

void send_end () {
    char buf[6];
    int buf_it = 0;

    buf[buf_it++] = '5';
    buf[buf_it++] = '5';

    //init debug mode
    buf[buf_it++] = '5';
    buf[buf_it++] = '5';
    buf[buf_it++] = '5';
    buf[buf_it++] = '5';

    if (SDU1.config->usbp->state == USB_ACTIVE) {
        chprintf ((BaseSequentialStream *) &SDU1,
                  "before send end\r\n");
    }

    e_send_uart1_char (buf, buf_it); // send answer

    if (SDU1.config->usbp->state == USB_ACTIVE) {
        chprintf ((BaseSequentialStream *) &SDU1,
                  "after send end buf=%s\r\n", buf);
    }

}

int run_asercom2 (void) {

    if (SDU1.config->usbp->state == 4) {
        chprintf ((BaseSequentialStream *) &SDU1, "in asercom\n");
    }

    static char c1, c2, wait_cam = 0;
    static int i, j, n, speedr, speedl, positionr, positionl, LED_nbr, LED_action, accx, accy, accz, sound, gyrox, gyroy, gyroz;
    static int buffer_itt = 0;
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
    e_init_motors ();
    //e_init_uart1();   // initialize UART to 115200 Kbaud
    //e_init_ad_scan();


    //Описание робота:
    double h = 0.0265; //Полурасстояние между колесами [м].
    double X_pos_actual = 0.0, Y_pos_actual = 0.0; //Реальные коорд-ты центра робота [м].
    double Theta = M_PI / 2; //Угол ориентации относительно Ox [рад].
    double Linear_speed_real = 0.0; //Реальная линейная скорость [м/с].
    double Angular_speed_real = 0.0; //Реальная угловая скорость [рад/с].
    double Left_speed_ms = 0.0, Right_speed_ms = 0.0; //Скорость лев. и прав. колес в [м/с].
    int16_t Left_speed_steps = 0, Right_speed_steps = 0.0; //Скорость лев. и прав. колес в [шаги/с].
    double Max_wheel_speed_for_linear = 0.07; //Часть скорости колеса, используемая для линейной скорости центра [м/с].
    double Max_wheel_speed_for_angular = 1.2 * 0.041 * M_PI -
                                         Max_wheel_speed_for_linear; //Часть скорости колеса, используемая для угловой скорости центра [м/с]. (1.2*0.041*M_PI - максимальная линейная скорость колеса).
    double Linear_speed_max = 0.5 * Max_wheel_speed_for_linear; //Максимальная лин. скорость центра [м/с].
    double Angular_speed_max = Max_wheel_speed_for_angular / h; //Макс. угл. скорость робота [рад/с].
    double Linear_acc = 0.1 * Linear_speed_max; //Линейное ускорение [м/с].
    double Angular_acc = 0.25 * Angular_speed_max; //Угловое ускорение [рад/с].

    //Описание целевой точки:
    double X_goal_point = 1.0, Y_goal_point = 1.0; //Целевые коорд-ты центра робота [м].
    double Dec_radius = 0.07; //В этом радиусе до цели - торможение [м].
    double Stop_radius = 0.05; //В этом радиусе от цели - стоп [м].
    double Dec_angle = (45.0 / 180.0) * M_PI; //В этом угле начинается торможение [рад].
    double Goal_point_angle = 0.0; //Угол на цель отн. оси Ox [рад].
    double Goal_point_distance = 100.0; //Расстояние до цели [м].

    //Целевые параметры робота:
    double Linear_speed_goal = 0.0; //Целевая линейная скорость [м/с].
    double Angular_speed_goal = 0.0; //Целевая угловая скорость [рад/с].
    double Goal_angle = 0.0; //Это условный целевой угол отн. оси Ox, который будет все время меняться [рад].
    double Angle_disagreement = 0.0; //Рассогласование по углу [рад].

    //Переменные для работы колесной одометрии:
    double Left_encoder_old = (double) left_motor_get_pos (); //Количество шагов на левом колесе пердыдущее [шаги].
    double Right_encoder_old = (double) right_motor_get_pos (); //Количество шагов на правом колесе пердыдущее [шаги].

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
    double Sensor_angles[6] = {1.309, 0.785, 0.0, M_PI, 2.356,
                               1.885}; //Углы расположения ИК-датчиков на роботе (отн. поперечной оси) [рад].
    //todo ask I thought we have 8 IR sensors?


    //Данные для получения вектора отталкивания от других агентов:
    char n_robots = 2; //Общее количество агентов (включая текущего).
    double Agents_repulsion_vector_length = 0;
    double Agents_repulsion_vector_angle = 0;

    //Результирующий вектор движения:
    double Moving_speed_value_potential_method = 0.0;
    double Moving_angle_potential_method = 0.0;
    double initTheta = Theta;

    selector = getselector (); //SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
    printNumberToPort (selector);
    printStringToPort ("selector");
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
        e_i2cp_init ();
    }
#endif

#ifdef IR_RECEIVER
    e_init_remote_control ();
#endif
//    if (RCONbits.POR) { // reset if power on (some problem for few robots)
//        RCONbits.POR = 0;
//        RESET();
//    }
    /*read HW version from the eeprom (last word)*/
    static int HWversion = 0xFFFF;
    ReadEE (0x7F, 0xFFFE, &HWversion, 1);

    /*Cam default parameter*/
    cam_mode = RGB_565_MODE;
    //cam_mode=GREY_SCALE_MODE;
    cam_width = 40; // DEFAULT_WIDTH;
    cam_heigth = 40; // DEFAULT_HEIGHT;
    cam_zoom = 8;
    cam_size = cam_width * cam_heigth * 2;

    if (gumstix_connected == 0 && selector != 15) {
        e_poxxxx_init_cam ();
        e_poxxxx_config_cam ((ARRAY_WIDTH - cam_width * cam_zoom) / 2, (ARRAY_HEIGHT - cam_heigth * cam_zoom) / 2,
                             cam_width * cam_zoom, cam_heigth * cam_zoom, cam_zoom, cam_zoom, cam_mode);
        e_poxxxx_write_cam_registers ();
    }

    if (gumstix_connected) { // Communicate with gumstix (i2c).
        // Send the following text through I2C to the gumstix.
        //uart1_send_static_text("\f\a"
        //        "WELCOME to the SerCom protocol on e-Puck\r\n"
        //        "the EPFL education robot type \"H\" for help\r\n");
    } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
        e_acc_calibr ();
//        uart1_send_static_text("\f\a"
//                               "WELCOME to the SerCom protocol on e-Puck\r\n"
//                               "the EPFL education robot type \"H\" for help\r\n");
    } else { // Communicate with the pc (usb).
        e_acc_calibr ();
        uart2_send_static_text("\f\a"
                               "WELCOME to the SerCom protocol on e-Puck\r\n"
                               "the EPFL education robot type \"H\" for help\r\n");
    }

    printStringToPort ("before while");
    while (1) {
        if (gumstix_connected) { // Communicate with gumstix (i2c).
            printStringToPort ("Communicate with gumstix (i2c).");
        } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
            //printStringToPort("Communicate with ESP32 (uart) => BT.");
            //printStringToPort("e_getchar_uart1(&c)");
            if (SDU1.config->usbp->state == 4) {
                //chprintf((BaseSequentialStream *) &SDU1, "--->%c<----->%p<-----\n", c, &c);
            }
            //printStringToPort("e_getchar_uart1(&c)");

            while (e_getchar_uart1 (&c) == 0)
#ifdef IR_RECEIVER
            {
                //printStringToPort(" while (e_getchar_uart1(&c) == 0)");
                ir_move = e_get_data ();
                ir_address = e_get_address ();
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
                                e_init_sound ();
                                first = 1;
                            }
                            e_play_sound (11028, 8016);
                            break;
                        default:
                            speedr = speedl = 0;
                    }
                    ir_last_move = ir_move;
                    e_set_speed_left (speedl);
                    e_set_speed_right (speedr);
                }
            }
#else
            ;
#endif
        } else { // Communicate with the pc (usb).
            //printStringToPort("// Communicate with the pc (usb).");
            while (e_getchar_uart2 (&c) == 0)
#ifdef IR_RECEIVER
            {
                ir_move = e_get_data ();
                ir_address = e_get_address ();
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
                                e_init_sound ();
                                first = 1;
                            }
                            e_play_sound (11028, 8016);
                            break;
                        default:
                            speedr = speedl = 0;
                    }
                    ir_last_move = ir_move;
                    e_set_speed_left (speedl);
                    e_set_speed_right (speedr);
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
                    case 'X':;
                        //Время для шага итерации:
                        systime_t T_nach; //Начало временного промежутка [мс].
                        //Флаг:
                        uint8_t Potential_method_switch = 0; //Включение и отключение метода потенциалов.




                        if (use_bt) { // Communicate with ESP32 (uart) => BT.
                            while (e_getchar_uart1 (&c1) == 0);
                        }
                        //printStringToPort("in case X");

                        switch (c1) {
                            case 2 :
                                // printStringToPort("in case X get c1=0000");
                                left_motor_set_pos (0);
                                right_motor_set_pos (0);

                                Right_encoder_old = (double) right_motor_get_pos ();
                                Left_encoder_old = (double) right_motor_get_pos ();

                                Update_odometer1 (&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                  &Y_pos_actual);

                                right_motor_set_speed (300);
                                left_motor_set_speed (-300);


                                initTheta = Theta;
                                while (fabs ((Theta - initTheta) * 180 / M_PI) < 20) {
//                                printNumberToPort(left_motor_get_pos());
                                    // if (SDU1.config->usbp->state == 4) {

//                                    Right_encoder_old = (double) right_motor_get_pos();
//                                    Left_encoder_old = (double) right_motor_get_pos();

                                    Update_odometer1 (&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                      &Y_pos_actual);


//                                    chprintf((BaseSequentialStream *) &SDU1,
//                                             "Theta = %8.4lf, X =  %8.4lf, Y =  %8.4lf, condition = %8.4lf\r\n",
//                                             Theta * 180 / M_PI,
//                                             X_pos_actual, Y_pos_actual, abs((Theta - initTheta) * 180 / M_PI));
                                    // }
                                }
                                right_motor_set_speed (0);
                                left_motor_set_speed (0);
                                buffer[buffer_itt++] = 'r';
                                buffer[buffer_itt++] = 'r';
                                buffer[buffer_itt++] = 'r';
                                buffer[buffer_itt++] = 'r';
                                printStringToPort ("in case X send rrrrr");
                                break;
                            case 1:
                                printStringToPort ("in case X get c1=1111");

                                left_motor_set_pos (0);
                                right_motor_set_pos (0);


                                Right_encoder_old = (double) right_motor_get_pos ();
                                Left_encoder_old = (double) right_motor_get_pos ();

                                Update_odometer2 (&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                  &Y_pos_actual);

                                right_motor_set_speed (-300);
                                left_motor_set_speed (300);


                                initTheta = Theta;
                                while (fabs ((Theta - initTheta) * 180 / M_PI) < 20) {
//                                printNumberToPort(left_motor_get_pos());
                                    // if (SDU1.config->usbp->state == 4) {

//                                    Right_encoder_old = (double) right_motor_get_pos();
//                                    Left_encoder_old = (double) right_motor_get_pos();

                                    Update_odometer2 (&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                      &Y_pos_actual);


//                                    chprintf((BaseSequentialStream *) &SDU1,
//                                             "Theta = %8.4lf, X =  %8.4lf, Y =  %8.4lf, condition = %8.4lf\r\n",
//                                             Theta * 180 / M_PI,
//                                             X_pos_actual, Y_pos_actual, abs((Theta - initTheta) * 180 / M_PI));
                                    // }
                                }

                                right_motor_set_speed (0);
                                left_motor_set_speed (0);
                                buffer[buffer_itt++] = 'l';
                                buffer[buffer_itt++] = 'l';
                                buffer[buffer_itt++] = 'l';
                                buffer[buffer_itt++] = 'l';
                                printStringToPort ("in case X send llll");
                                break;

                            case 4:
                                //Робот вычисляет вектор отталкивания и поворачивается в соответствии с ним.
                                Theta = M_PI / 2;
                                while (1) {
                                    Filter_and_update_proximity_values (Prox_0_sens_values, Prox_1_sens_values,
                                                                        Prox_2_sens_values, Prox_5_sens_values,
                                                                        Prox_6_sens_values, Prox_7_sens_values, Size,
                                                                        Filtered_proximity_value);
                                    for (i = 0; i < 6; i++) {
                                        Proximity_values_mm[i] = Get_proximity_value_mm (Filtered_proximity_value[i]);
                                    }
                                    Update_obstacle_repulsion_vector (Proximity_values_mm, Theta, Sensor_angles,
                                                                      &Obstacle_repulsion_vector_angle,
                                                                      &Obstacle_repulsion_vector_length);
                                    Goal_angle = Obstacle_repulsion_vector_angle;
                                    Goal_point_distance = 0;

                                    for (j = 0; j < 30; j++) {
                                        Update_odometer2 (&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                          &Y_pos_actual);
                                        Update_angle_disagreement (&Angle_disagreement, Theta, Goal_angle);
                                        Update_goal_speeds_values (&Linear_speed_goal, &Angular_speed_goal,
                                                                   Goal_point_distance, Dec_radius, Stop_radius,
                                                                   Linear_speed_max, Angle_disagreement,
                                                                   Angular_speed_max, Dec_angle,
                                                                   Potential_method_switch,
                                                                   Moving_speed_value_potential_method);
                                        Calculate_speeds (&Linear_speed_real, &Angular_speed_real, Linear_acc,
                                                          Angular_acc, Linear_speed_goal, Angular_speed_goal);
                                        Set_wheels_speeds (Linear_speed_real, Angular_speed_real, h);
                                        chThdSleepMilliseconds(95);

                                        if (SDU1.config->usbp->state == USB_ACTIVE) {
                                            chprintf ((BaseSequentialStream *) &SDU1,
                                                      "Theta = %lf, Obstacle repulse angle = %lf\r\n",
                                                      Theta * 180 / M_PI, Obstacle_repulsion_vector_angle * 180 / M_PI);
                                        }
                                    }
                                }
                                break;
                            case 5:
                                //Тест - в порт выводится угол и модуль вектора отталкивания.
                                while (1) {

                                    Filter_and_update_proximity_values (Prox_0_sens_values, Prox_1_sens_values,
                                                                        Prox_2_sens_values, Prox_5_sens_values,
                                                                        Prox_6_sens_values, Prox_7_sens_values, Size,
                                                                        Filtered_proximity_value);

                                    for (i = 0; i < 6; i++) {
                                        Proximity_values_mm[i] = Get_proximity_value_mm (Filtered_proximity_value[i]);
                                    }
                                    Update_obstacle_repulsion_vector (Proximity_values_mm, Theta, Sensor_angles,
                                                                      &Obstacle_repulsion_vector_angle,
                                                                      &Obstacle_repulsion_vector_length);
                                    chprintf ((BaseSequentialStream *) &SDU1, "Angle = %lf, Module = %lf\r\n",
                                              Obstacle_repulsion_vector_angle * 180 / M_PI,
                                              Obstacle_repulsion_vector_length);

                                    chThdSleepMilliseconds(100);
                                }
                                break;
                            case 6:
                                //Движение к точке - БЕЗ метода потенциалов.
                                Potential_method_switch = 0;
                                Set_goal_coordinates (0.2, 0.4, &X_goal_point,
                                                      &Y_goal_point); //Установка целевых координат [м][м].
                                while (1) //Рабочий цикл для движения к целевой точке.
                                {
                                    T_nach = chVTGetSystemTime (); //Начало временного промежутка.

                                    Update_odometer2 (&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                      &Y_pos_actual); //Обновляет положение центра робота и его угол ориентации.
                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "Left_encoder_old = %8.3lf, Right_encoder_old = %8.3lf, Theta =  %8.3lf, X_pos_actual =  %8.3lf, Y_pos_actual =  %8.3lf\r\n",
                                                  Left_encoder_old, Right_encoder_old, Theta * 180 / M_PI, X_pos_actual,
                                                  Y_pos_actual);
                                    }
                                    Update_goal_point_distance (&Goal_point_distance, X_pos_actual, Y_pos_actual,
                                                                X_goal_point, Y_goal_point);
                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "Goal_point_distance = %8.3lf, X_goal_point = %8.3lf, Y_goal_point =  %8.3lf, X_pos_actual =  %8.3lf, Y_pos_actual =  %8.3lf\r\n",
                                                  Goal_point_distance, X_goal_point, Y_goal_point, X_pos_actual,
                                                  Y_pos_actual);
                                    }

                                    Update_goal_point_angle (&Goal_point_angle, X_pos_actual, Y_pos_actual,
                                                             X_goal_point, Y_goal_point);
                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "Goal_point_angle = %8.3lf, X_goal_point = %8.3lf, Y_goal_point =  %8.3lf, X_pos_actual =  %8.3lf, Y_pos_actual =  %8.3lf\r\n",
                                                  Goal_point_angle * 180 / M_PI, X_goal_point, Y_goal_point,
                                                  X_pos_actual,
                                                  Y_pos_actual);
                                    }

                                    Goal_angle = Goal_point_angle;
                                    Update_angle_disagreement (&Angle_disagreement, Theta, Goal_angle);
                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "Angle_disagreement = %8.3lf, Theta = %8.3lf, Goal_angle =  %8.3lf\r\n",
                                                  Angle_disagreement * 180 / M_PI, Theta * 180 / M_PI,
                                                  Goal_angle * 180 / M_PI);
                                    }
                                    Update_goal_speeds_values (&Linear_speed_goal, &Angular_speed_goal,
                                                               Goal_point_distance, Dec_radius, Stop_radius,
                                                               Linear_speed_max, Angle_disagreement, Angular_speed_max,
                                                               Dec_angle, Potential_method_switch,
                                                               Moving_speed_value_potential_method);
                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "Linear_speed_goal = %8.3lf, Angular_speed_goal = %8.3lf, Goal_point_distance =  %8.3lf\r\n",
                                                  Linear_speed_goal, Angular_speed_goal, Goal_point_distance);
                                    }
                                    Calculate_speeds (&Linear_speed_real, &Angular_speed_real, Linear_acc, Angular_acc,
                                                      Linear_speed_goal, Angular_speed_goal);
                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "Linear_speed_real = %8.3lf, Angular_speed_real = %8.3lf, Linear_speed_goal = %8.3lf, Angular_speed_goal = %8.3lf, "
                                                  "Linear_acc = %8.3lf, Angular_acc = %8.3lf\r\n",
                                                  Linear_speed_real, Angular_speed_real, Linear_speed_goal,
                                                  Angular_speed_goal, Linear_acc, Angular_acc);
                                    }

                                    Set_wheels_speeds (Linear_speed_real, Angular_speed_real, h);
                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "Linear_speed_real = %8.3lf, Angular_speed_real = %8.3lf, h = %8.3lf\r\n",
                                                  Linear_speed_real, Angular_speed_real, h);
                                    }
                                    while (chVTGetSystemTime () < (T_nach + 100));
                                }
                                break;
                            case 7:;
                                int x = 0, y = 0;
                                for (j = 0; j < 4; j++) {
                                    if (use_bt) { // Communicate with ESP32 (uart) => BT.
                                        while (e_getchar_uart1 (&c2) == 0);
                                    }

                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "c2 = %c, \r\n", c2);
                                    }

                                    x = x * 10 + (c2 - '0');

                                }


                                for (j = 0; j < 4; j++) {
                                    if (use_bt) { // Communicate with ESP32 (uart) => BT.
                                        while (e_getchar_uart1 (&c2) == 0);
                                    }

                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "c2 = %c, \r\n", c2);
                                    }

                                    y = y * 10 + (c2 - '0');


                                }
                                if (SDU1.config->usbp->state == USB_ACTIVE) {
                                    chprintf ((BaseSequentialStream *) &SDU1,
                                              "x = %8.3lf, y = %8.3lf \r\n",
                                              (double) x / 100, (double) y / 100);
                                }

                                Potential_method_switch = 0;
                                Set_goal_coordinates ((double) x / 100, (double) y / 100, &X_goal_point, &Y_goal_point);
                                int x_pos_actual = 0, y_pos_actual = 0;
                                if (SDU1.config->usbp->state == USB_ACTIVE) {
                                    chprintf ((BaseSequentialStream *) &SDU1,
                                              "Goal_point_distance = %8.3lf\r\n",
                                              Goal_point_distance);
                                }
                                while (1 &&
                                       (Goal_point_distance > Stop_radius)) //Рабочий цикл для движения к целевой точке.
                                {
                                    T_nach = chVTGetSystemTime (); //Начало временного промежутка.

                                    Update_odometer2 (&Left_encoder_old, &Right_encoder_old, &Theta, &X_pos_actual,
                                                      &Y_pos_actual); //Обновляет положение центра робота и его угол ориентации.
                                    Update_goal_point_distance (&Goal_point_distance, X_pos_actual, Y_pos_actual,
                                                                X_goal_point, Y_goal_point);

                                    Update_goal_point_angle (&Goal_point_angle, X_pos_actual, Y_pos_actual,
                                                             X_goal_point, Y_goal_point);

                                    Goal_angle = Goal_point_angle;
                                    Update_angle_disagreement (&Angle_disagreement, Theta, Goal_angle);

                                    Update_goal_speeds_values (&Linear_speed_goal, &Angular_speed_goal,
                                                               Goal_point_distance, Dec_radius, Stop_radius,
                                                               Linear_speed_max, Angle_disagreement, Angular_speed_max,
                                                               Dec_angle, Potential_method_switch,
                                                               Moving_speed_value_potential_method);

                                    Calculate_speeds (&Linear_speed_real, &Angular_speed_real, Linear_acc, Angular_acc,
                                                      Linear_speed_goal, Angular_speed_goal);


                                    Set_wheels_speeds (Linear_speed_real, Angular_speed_real, h);

                                    x_pos_actual = (int) (X_pos_actual * 100);
                                    y_pos_actual = (int) (Y_pos_actual * 100);

                                    debug_coord (x_pos_actual, y_pos_actual);

                                    if (SDU1.config->usbp->state == USB_ACTIVE) {
                                        chprintf ((BaseSequentialStream *) &SDU1,
                                                  "Goal_point_distance = %8.3lf\r\n",
                                                  Goal_point_distance);
                                    }

                                    while (chVTGetSystemTime () < (T_nach + 100));
                                }

                                Set_wheels_speeds(0, 0, h);
                                send_end();
                                while (1);
                                break;
                        }

//                        e_send_uart1_char (buffer, buffer_itt); // send answer
//                        while (e_uart1_sending ());

                        break;
                    default: // silently ignored
                        break;
                }
                if (gumstix_connected) { // Communicate with gumstix (i2c).

                } else if (use_bt) { // Communicate with ESP32 (uart) => BT.
                    while (e_getchar_uart1 (&c) == 0); // get next command
                } else { // Communicate with the pc (usb).
                    while (e_getchar_uart2 (&c) == 0); // get next command
                }
            } while (c);

        }

        printStringToPort ("end of loop");
    }
    printStringToPort ("end");
}

