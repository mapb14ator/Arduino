#include "motors.h"

#define FILTER 41

#define BLACK 0
#define WHITE 1

#define FIRST_COLOR WHITE     //цвет кубиков, которые должны быть доставлены первыми
#define FIRST_COLOR_CUBES  4  //количество кубиков первого цвета 
#define SECOND_COLOR_CUBES 3  //количество кубиков второго цвета (если оба значения равны 0, робот будет в начале делать объезд трассы)

#define RUN_AFTER_CROSS             3     //проезд вперед после обнаружения перекрестка
#define L_RUN_AFTER_START           15    //левый поворот на прекрестке после зоны старта
#define R_RUN_AFTER_START           15    //правый поворот на прекрестке после зоны старта
#define TURN_LEFT_AFTER_RELOAD      13    //поворот налево после выгрузки кубиков
#define TURN_RIGHT_AFTER_RELOAD     13    //поворот направо после выгрузки
#define LEFT_REVERSE                11    //разворот на месте влево
#define RIGHT_REVERSE               11    //разворот на месте вправо
#define TURN_LEFT_RECT              5     //поворот на месте влево на прямом угле трассы
#define TURN_RIGHT_RECT             5     //поворот на месте вправо на прямом угле трассы
#define DELAY_AFTER_RELOAD_ONE_CUBE 1500  //задержка после выгрузки одного кубика для освобождения базы

#define NORMAL_LEFT_SPEED  255   //скорость вращения левого колеса (min 0... max 255)
#define NORMAL_RIGHT_SPEED 255   //скорость вращения правого колеса
#define TURN_LEFT_SPEED    55    //сниженная скорость левого колеса при повороте
#define TURN_RIGHT_SPEED   55    //сниженная скорость правого колеса при повороте 

#define BLACK_LEVEL 835 //пороговый уровень черного цвета

bool inversion = 0;

bool clockwide;

Manipulator mnp_left(12,90,0);
Manipulator mnp_right(11,90,180);

Motor leftMotor(4, 5);
Motor rightMotor(7, 6);

#include "robot_ai.h"

bool test_colors[9]={WHITE,WHITE,BLACK,WHITE,BLACK,BLACK,WHITE};

short test_it=0;

short test(){
  short color=test_colors[test_it];
  test_it++;
  return color;
}

void setup()
{
    mnp_left.init();
    mnp_right.init();
    mnp_left.up();
    mnp_right.up();
    leftMotor.init();
    rightMotor.init();
    leftMotor.setSpeed(NORMAL_LEFT_SPEED);
    rightMotor.setSpeed(NORMAL_RIGHT_SPEED);
    attachInterrupt(1, leftSumm, RISING);
    attachInterrupt(0, rightSumm, RISING);
    EIMSK&=~((1<<INT0)|(1<<INT1));
}

void loop()
{
  bool direct;
  short cube_counter;
  short first_color_cubes=FIRST_COLOR_CUBES;
  short second_color_cubes=SECOND_COLOR_CUBES;
  bool blocked=HIGH;
  short cb;
  leftEn = 0;
  rightEn = 0;
  short p=0;
  short color;
  
  clockwide=HIGH;
  
  leaveStartZone();//покинуть зону старта
  
  if ((first_color_cubes==0)&&(second_color_cubes==0)) {
    while(p!=CROSS_DETECTED){
      p=drive();
      if (p==CUBE_DETECTED){
        skipCube();
        stop();
        grabCube();
        color=getColor();
        if (color==FIRST_COLOR) {
          first_color_cubes++;
        } else {
           second_color_cubes++;
        }
      releaseCube();
      }
    }
    run(RUN_AFTER_CROSS);
  }
  cube_counter=0;
  direct=HIGH;
  while(first_color_cubes!=0){            //пока не доставлены все кубы первого цвета
    p=drive();
    if (p==CUBE_DETECTED){                //при обнаружении куба
      skipCube();
      if (direct==HIGH) {                 //если прямой ход
        cube_counter++;
        stop();
        grabCube();                       //опустить манипулятор и определить цвет
        color=test();
        if (color==FIRST_COLOR) {         //если цвет совпал
          if (cube_counter==1) {          //если это первый куб
            if (first_color_cubes==1){    //если куб первого цвета остался один
              if (clockwide==LOW){        //разворот на 180 град
                leftCenter(LEFT_REVERSE);
              } else {
                rightCenter(RIGHT_REVERSE);
              }
              clockwide=!clockwide;       //направление движения по кругу меняется на противоположное
              direct=LOW;                 //обратный ход
            } else {                      //если кубов осталось больше одного
              blocked=LOW;                //разрешаем забрать куб на обратном пути
              releaseCube();              //поднимаем манипулятор
            }
          } else {                        //если это не первый куб
            if (clockwide==LOW){          //забираем и разворачиваемся
              leftCenter(LEFT_REVERSE);
            } else {
              rightCenter(RIGHT_REVERSE);
            }
            clockwide=!clockwide;
            direct=LOW;
          }
        } else {                         //если цвет не совпал
          if (cube_counter==1) {         //если это первый куб
            blocked=HIGH;                //запрещаем забирать на обратном пути
          }
          releaseCube();                 //поднимаем манипулятор
        }
      } else {                           //обратный ход
        cube_counter--;
        if (blocked==LOW) {              //если разрешено забирать первый куб
          if (cube_counter==1) {
            stop();
            grabCube();
            cube_counter=0;
          } 
        } 
      }
    }
    if (p==CROSS_DETECTED) {                                   //при обнаружении перекрестка
      if ((mnp_right.get_state()==DOWN)||(mnp_left.get_state()==DOWN)){
        run(RUN_AFTER_CROSS);
        cb=reload_cubes();                                       //выгружаем кубики
        first_color_cubes=first_color_cubes-cb;                  //вычисляем оставшееся количество кубов
        if ((first_color_cubes==0)&&(second_color_cubes==0)) {   //если все кубики доставлены
          goHome();                                              //едем в зону старта
        } else {
          if (clockwide==HIGH){
            leftRun(TURN_LEFT_AFTER_RELOAD);
          } else {
            rightRun(TURN_RIGHT_AFTER_RELOAD);
          }
        }
        cube_counter=0;
        direct=HIGH;
      } else {
        run(RUN_AFTER_CROSS);
        if (clockwide==HIGH){
          rightCenter(TURN_RIGHT_RECT);
        } else {
          leftCenter(TURN_LEFT_RECT);
        }
      }
    }
  }
  while(second_color_cubes!=0){
    p=drive();
    if (p==CUBE_DETECTED){
      skipCube();
      if (direct==HIGH) {
        cube_counter++;
        if (cube_counter==1) {
          if (second_color_cubes==1){
            stop();
            grabCube();
            if (clockwide==LOW){
              leftCenter(LEFT_REVERSE);
            } else {
              rightCenter(RIGHT_REVERSE);
            }
            clockwide=!clockwide;
            direct=LOW;
          } 
        } else {
          stop();
          grabCube();
          if (clockwide==LOW){
            leftCenter(LEFT_REVERSE);
          } else {
            rightCenter(RIGHT_REVERSE);
          }
          clockwide=!clockwide;
          direct=LOW;
        }
      } else {
        cube_counter--;
        if (cube_counter==1) {
          stop();
          grabCube();
          cube_counter=0;
        }
      }
    }
    if (p==CROSS_DETECTED) {
      if ((mnp_right.get_state()==DOWN)||(mnp_left.get_state()==DOWN)){
        run(RUN_AFTER_CROSS);
        cb=reload_cubes();
        second_color_cubes=second_color_cubes-cb;
        if (second_color_cubes==0) {
          goHome();  
        } else {
          if (clockwide==HIGH){
            leftRun(TURN_LEFT_AFTER_RELOAD);
          } else {
            rightRun(TURN_RIGHT_AFTER_RELOAD);
          }
        }
        cube_counter=0;
        direct=HIGH;
      } else {
        if (clockwide==HIGH){
          rightCenter(TURN_RIGHT_RECT);
        } else {
          leftCenter(TURN_LEFT_RECT);
        }
      }
    }
  } //пока не привезли кубы второго цвета
}
