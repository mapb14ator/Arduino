#ifndef ROBOT_AI_H
#define ROBOT_AI_H

#define ALL_LINE_SNS    ((1<<PC0)|(1<<PC1)|(1<<PC2))
#define LEFT_LINE_SNS   (1<<PC2)
#define CENTER_LINE_SNS (1<<PC1)
#define RIGHT_LINE_SNS  (1<<PC0)
#define ALL_CUBE_SNS    ((1<<PC4)|(1<<PC5))
#define LEFT_CUBE_SNS   (1<<PC4)
#define RIGHT_CUBE_SNS  (1<<PC5)

#define LEFT_COLOR_SENSOR A5
#define RIGHT_COLOR_SENSOR A4

#define B_B_B B000
#define B_B_W B001
#define B_W_B B010
#define B_W_W B011
#define W_B_B B100
#define W_B_W B101
#define W_W_B B110
#define W_W_W B111

#define NONE           0
#define CROSS_DETECTED 1
#define CUBE_DETECTED  2

volatile int leftEn =  0;
volatile int rightEn = 0;

void leftSumm()
{
  leftEn++;
}

void rightSumm()
{
  rightEn++;
}

void stop()
{
  leftMotor.stop();
  rightMotor.stop();
}

void run(int distance=0)
{
  leftMotor.run();
  rightMotor.run();
  if(distance != 0)
  {
    EIMSK|=(1<<INT0);
    rightEn = 0;
    while(rightEn <= distance) {}
    EIMSK&=~(1<<INT0); 
  }
}

void back()
{
  leftMotor.back();
  rightMotor.back();
}

void rightRun(int distance=0)
{
  leftMotor.run();
  rightMotor.stop();
  if(distance != 0)
  {
    EIMSK|=(1<<INT1);
    leftEn = 0;
    while(leftEn <= distance) {}
    EIMSK&=~(1<<INT1);
  }
}

void leftRun(int distance=0)
{
  leftMotor.stop();
  rightMotor.run();
  if(distance != 0)
  {
    EIMSK|=(1<<INT0);
    rightEn = 0;
    while(rightEn <= distance) {}
    EIMSK&=~(1<<INT0);
  }
}

void leftCenter(int distance=0)
{
  leftMotor.back();
  rightMotor.run();
  if(distance != 0)
  {
    EIMSK|=(1<<INT0);
    rightEn = 0;
    while(rightEn <= distance) {}
    EIMSK&=~(1<<INT0);
  }
}

void rightCenter(int distance=0)
{
  leftMotor.run();
  rightMotor.back();
  if(distance != 0)
  {
    EIMSK|=(1<<INT1);
    leftEn = 0;
    while(leftEn <= distance) {}
    EIMSK&=~(1<<INT1);
  }
}

short getColor(){
  int signal;
  short result;
  if (clockwide==LOW){
    signal=analogRead(RIGHT_COLOR_SENSOR);
  } else {
    signal=analogRead(LEFT_COLOR_SENSOR);
  }
  if (signal>=BLACK_LEVEL){
    result=BLACK;
  } else {
    result=WHITE;
  }
  return result;
}

int scan()
{
  int counter, last_result;
  int result = (PINC&(ALL_LINE_SNS))|((PINB&((1<<PB0)|(1<<PB1)))<<4);
  if(FILTER > 0){
    counter = 0;
    while(counter < FILTER)
    {
      last_result = (PINC&(ALL_LINE_SNS))|((PINB&((1<<PB0)|(1<<PB1)))<<4);
      if(last_result  == result)
      {
        counter++;  
      }
      else
      {
        counter = 0;
        result = last_result;
      }
    }
    if(!inversion)
    {
      result = result ^ B111;
    }
    return result;
  }
}

void leaveStartZone(){
  short zone=1;
  int allDetectors=0;
  while(zone<3){
    allDetectors = scan()&ALL_LINE_SNS;
    switch (allDetectors) {
      case B_W_W:
        leftRun(); 
        break;
      case W_B_W:
        run();
        break;
      case W_W_B:
        rightRun();
        break;
      case W_W_W:
        run();
        break;
      default:
        switch(zone)
        {
          case 1:
            run(RUN_AFTER_CROSS);
            break;
          case 2:
            if (clockwide==LOW){
              leftRun(L_RUN_AFTER_START);
            } else {
              rightRun(R_RUN_AFTER_START);
            }
            break;
        }
        zone++;
    }
  }
}

short drive(){
  int _sensors=scan();
  short status=NONE;
  unsigned short tmp;
  switch (_sensors&ALL_LINE_SNS) {
    case B_B_W:
      while((scan()&RIGHT_LINE_SNS)!=0){
        leftRun();
      }
      break;
    case W_B_B:
      while((scan()&LEFT_LINE_SNS)!=0){
        rightRun();
      }
      break;
    case B_W_W:
      tmp=leftMotor.getSpeed();
      leftMotor.setSpeed(TURN_LEFT_SPEED);
      while((scan()&CENTER_LINE_SNS)!=0){
        run();   
      }
      leftMotor.setSpeed(tmp);
      break;
    case W_B_W:
      run();
      break;
    case W_W_B:
      tmp=rightMotor.getSpeed();
      rightMotor.setSpeed(TURN_RIGHT_SPEED);
      while((scan()&CENTER_LINE_SNS)!=0){
        run();
      }
      rightMotor.setSpeed(tmp);
      break;
    case W_W_W:
      run();
      break;
    default:
      status=CROSS_DETECTED;
      break;
  }
  if (clockwide==LOW)
  {
    if(((_sensors&RIGHT_CUBE_SNS)==0)&&(mnp_right.get_state()==UP)) {
      status=CUBE_DETECTED;
    }
  } else {
    if(((_sensors&LEFT_CUBE_SNS)==0)&&(mnp_left.get_state()==UP)) {
      status=CUBE_DETECTED;
    }
  }
  return status;
}

short reload_cubes(){
  short cubes_in_base=0;
    if (clockwide==LOW){
      rightCenter(5);
      run(3);
      rightCenter(5);
      stop();
      mnp_left.up();
      rightCenter(6);
      cubes_in_base++;
      if (mnp_right.get_state()==DOWN){
        stop();
        delay(DELAY_AFTER_RELOAD_ONE_CUBE);
        leftCenter(11);
        run(3);
        leftCenter(5);
        stop();
        mnp_right.up();
        leftCenter(6);
        run(3);
        cubes_in_base++;
      }
    } else {
      leftCenter(5);
      run(3);
      leftCenter(5);
      stop();
      mnp_right.up();
      leftCenter(6);
      cubes_in_base++;
      if (mnp_left.get_state()==DOWN){
        stop();
        delay(DELAY_AFTER_RELOAD_ONE_CUBE);
        rightCenter(11);
        run(3);
        rightCenter(5);
        stop();
        mnp_left.up();
        rightCenter(6);
        run(3);
        cubes_in_base++;
      }
    }
    return cubes_in_base;
}

void grabCube(){
  if (clockwide==LOW) {
    mnp_right.down();
  } else {
    mnp_left.down();
  }
}

void releaseCube(){
  if (clockwide==LOW) {
    mnp_right.up();
  } else {
    mnp_left.up();
  }
}

void skipCube(){
  short status;
  while (status==CUBE_DETECTED) {
    status=drive();
  }
}

void goHome(){
  short p=NONE;
  run(5);
  while (p!=CROSS_DETECTED){
    p=drive();
  }
  run(15);
  stop();
  while(1){}
}
#endif
