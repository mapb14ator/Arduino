#ifndef MOTORS_H
#define MOTORS_H

class Motor
{
  public:
    Motor(int Addr0, int Addr1);
    void init();
    void stop();
    void setSpeed(unsigned short spd);
    unsigned short getSpeed();
    void run();
    void back();
  private:
    int dir_pin;
    int speed_pin;
    int speed;
};

Motor::Motor(int Addr0, int Addr1)
{
    dir_pin = Addr0;
    speed_pin = Addr1;
}

void Motor::init()
{
    pinMode(dir_pin, OUTPUT);
    pinMode(speed_pin, OUTPUT);
}

void Motor::back()
{
  digitalWrite(dir_pin, LOW);
  analogWrite(speed_pin, 255-speed);
}

void Motor::stop()
{
    digitalWrite(dir_pin, LOW);
    analogWrite(speed_pin, 0);
}

void Motor::run()
{
    digitalWrite(dir_pin, HIGH);
    analogWrite(speed_pin, speed);
}

void Motor::setSpeed(unsigned short spd)
{
  speed = 255-spd;
}
unsigned short Motor::getSpeed()
{
  return (255-speed);
}

#define UP 1
#define DOWN 0

class Manipulator{
  private:
    short _state;
    int _up_angle;
    int _down_angle;
    short _pin;
  public:
    Manipulator(short pin,int up_angle,int down_angle);
    void init();
    short get_state();
    void up();
    void down();
};

Manipulator::Manipulator(short pin,int up_angle,int down_angle){
  _pin=pin;
  _up_angle=up_angle;
  _down_angle=down_angle;
}
void Manipulator::init(){
  pinMode(_pin,OUTPUT);  
}
short Manipulator::get_state(){
  return _state;  
}

void Manipulator::up(){
  _state=UP;
  int width=map(_up_angle,0,180,600,2400);
  for(int i=0;i<200;i++){
    digitalWrite(_pin,HIGH);
    delayMicroseconds(width);
    digitalWrite(_pin,LOW);
    delayMicroseconds(20000-width);  
  }  
}

void Manipulator::down(){
  _state=DOWN;
  int width=map(_down_angle,0,180,600,2400);
    for(int i=0;i<200;i++){
      digitalWrite(_pin,HIGH);
      delayMicroseconds(width);
      digitalWrite(_pin,LOW);
      delayMicroseconds(20000-width);  
  }
}
#endif
