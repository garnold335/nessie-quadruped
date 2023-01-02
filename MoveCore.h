/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef SPIDER_MOVES_H
#define SPIDER_MOVES_H
#include "Spider_Hardware.h"
enum Moves{
  MOV_INIT='I',
  MOV_STAND='O', //start !=0 to allow 0 being Abort-Move in String
  MOV_SIT='o',
  MOV_TURNLEFT='l',
  MOV_TURNRIGHT='m',
  MOV_STEPFORWARD='f',
  MOV_STEPBACK='p',
  MOV_BODY_LEFT='1',
  MOV_BODY_RIGHT='2',
  MOV_HANDWAVE='3',
  MOV_HANDSHAKE='4',
  MOV_HEADUP='5',
  MOV_HEADDOWN='6',
  MOV_NO='7',
  MOV_TRANSPORT='+',//e.g. Disconnect
  MOV_LASTMOV='0',
  MOV_GYRATE='8',
  MOV_BODY_SIDE_UPDOWN='9',
  MOV_BOW='A',
  MOV_TWIST='B',
  MOV_BODY_FWD_BKWD='C',
  MOV_BODY_SIDE2SIDE='D',
  MOV_TALL='E',
  MOV_BACK_BOW='F',
  MOV_STAGGER='H',
  MOV_RELAX_LEGS='J',
  MOV_WALK_LEGS='K',
  MOV_WALK_LEGS_OTHER='Q',
  MOV_SPRAWL='S',
  MOV_CHG_LEG_Z='T',
  MOV_CG='U',
  MOV_RETURN_CG='V'
};
/* JC Constants for movement ----------------------------------------------------*/
const float sprawl_leg = 95; // used for both X and Y on sprawl movement
const int WALK = 0;
const int RELAX = 1;
const int SPRAWL = 2;
const bool FALSE = 0;
const bool TRUE = 1;
const float half_step = 10; // forward and backward

//JC Variables for moves
const float z_bow_relative = 25;
const float z_tall = -100;
const float gyrate_speed =0.8;
const float bow_speed = 1;
const float cg_shift = 14; // distance to move CG to retain balance with leg raised

/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

  //Todo: change all this to Integer-math (not float)
  //we end up with counts between 100 and 600, so we should not waste
  //too much time for unused precision
  static const float spot_turn_speed = 4;
  static const float leg_move_speed = 8;
  static const float body_move_speed = 3;
  static const float stand_seat_speed = 1;
  /* variables for movement ----------------------------------------------------*/
extern volatile float site_now[NUMLEGS][NUMAXIS];    //real-time coordinates of the end of each leg
extern volatile float site_expect[NUMLEGS][NUMAXIS]; //expected coordinates of the end of each leg
extern volatile uint8_t ControlMode[NUMLEGS];
extern float temp_speed[NUMLEGS][NUMAXIS];   //each axis' speed, needs to be recalculated before each movement
extern float move_speed;     //movement speed  
extern float ServoNext[NUMSERVOS];
extern float ServoSpeed[NUMSERVOS];
extern float ServoDelta[NUMSERVOS];




class MoveCore{
  public:
  //functions' parameter
  static constexpr float KEEP = 255;
  static void Move(Moves MOV, int Parameter);
  static void SetSpeedFactor(int value);
  static void set_site(int leg, float x, float y, float z);
#define NUMLEGS 4
#define NUMAXIS 3
  static void servo_service(void);  
  static uint8_t ServoIdx(uint8_t Leg, uint8_t axis){
    return Leg*3+axis;
  }
  static void SetLegMode(uint8_t Leg, uint8_t Mode);
  static void SetAllLegMode(uint8_t Mode);
  private:
  /***********************
   * Section does the Math work
   */

  static void wait_all_reach(void);

//here the Moves declaration, having access to the variables
  static void No(uint8_t count);//added by Kasinator
  static void sit(void);
  static void transport(void);
  static void startposition(void);
  static void stand(void);
  static void turn_left(unsigned int step);
  static void turn_right(unsigned int step);
  static void step_forward(unsigned int step);
  static void step_back(unsigned int step);
// add by RegisHsu
  static void body_left(int i);
  static void body_right(int i);
  static void hand_wave(int i);
  static void hand_shake(int i);
  static void head_up(int i);
  static void head_down(int i);
//Math happens here:
  static void wait_reach(int leg);
  static void wait_reach_ca(int leg);
  static void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);
  static void cartesian_to_polar_cordic(volatile double &alpha, volatile double &beta, volatile double &gamma, volatile double x, volatile double y, volatile double z);
  //JC added moves
  static void gyrate(unsigned int number_of_times);
  static void body_side_up_side_down(unsigned int number_of_times);
  static void bow( unsigned int times);
  static void body_twist( unsigned int times);
  static void body_forward_backward( unsigned int times);
  static void body_side_to_side( unsigned int times);
  static void tall( int time);
  static void back_bow( unsigned int times);
  static void stagger(unsigned int times);
  static void relax_legs(void);
  static void walk_legs(void);
  static void walk_legs_other(void);
  static void sprawl(void);
  static void change_leg_z(int leg,float z);
  static void move_cg(int leg);
  static void return_cg(int leg);

  //     Head
  //Leg 1    3 Leg  Coxa(Alpha)   Thibia (Beta)  Femur (Gamma)
  //Leg 2    4 Leg
};
extern MoveCore Mover;
#endif
