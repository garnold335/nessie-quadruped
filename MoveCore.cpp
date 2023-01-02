/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#include "ServoHAL.h"
#include "MoveCore.h"
#include "Spider_Hardware.h"
#include "cordic.h"

MoveCore Mover;

//declared locally
void stand(void);
void sit(void);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void body_left(int i);
void body_right(int i);
void hand_wave(int i);
void hand_shake(int i);
void head_up(int i);
void head_down(int i);
void gyrate(unsigned int number_of_times);
void body_side_up_side_down(unsigned int number_of_times);
void bow( unsigned int times);
void body_twist( unsigned int times);
void body_forward_backward( unsigned int times);
void body_side_to_side( unsigned int times);
void tall( int time);
void back_bow( unsigned int times);
void stagger(unsigned int times);
void relax_legs(void);
void walk_legs(void);
void walk_legs_other(void);
void sprawl(void);
void change_leg_z(int leg,float z);
void move_cg(int leg);
void return_cg(int leg);

volatile float site_now[NUMLEGS][NUMAXIS];    //real-time coordinates of the end of each leg
volatile float site_expect[NUMLEGS][NUMAXIS]; //expected coordinates of the end of each leg
volatile uint8_t ControlMode[NUMLEGS]; //0 = Cartesian, 1 = Polar, 2 = Off
float temp_speed[NUMLEGS][NUMAXIS];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed  
float speed_multiple = 1;
float ServoNext[NUMSERVOS];
float ServoSpeed[NUMSERVOS];
float ServoDelta[NUMSERVOS];
int leg_position; // one of WALK, RELAX, SPRAWL
volatile bool capture = false;
volatile float site_cg[4][3]; // record current position to determine how to move cg
volatile int rest_counter;      //+1/0.02s, for automatic rest

void MoveCore::SetLegMode(uint8_t Leg, uint8_t Mode){
    if ((ControlMode[Leg]==0)&&(Mode == 1)){//Switch from cartesian to polar control mode
      ControlMode[Leg]=2; //Prevent accidential updates in ISR while updating variables
      float alpha, beta, gamma;
      cartesian_to_polar(alpha, beta, gamma, site_now[Leg][0], site_now[Leg][1], site_now[Leg][2]);
      site_now[Leg][0]=alpha;
      site_now[Leg][1]=beta;
      site_now[Leg][2]=gamma;
      //assume reached
      site_expect[Leg][0]=alpha;
      site_expect[Leg][1]=beta;
      site_expect[Leg][2]=gamma;
      temp_speed[Leg][0]=1;
      temp_speed[Leg][1]=1;
      temp_speed[Leg][2]=1;
    }else if ((ControlMode[Leg]!=Mode)&&(Mode == 0)){
      ControlMode[Leg]=0; //disable updates
      set_site(Leg, x_default , y_start , z_boot);
      for (int j = 0; j < 3; j++)
      {
        site_now[Leg][j] = site_expect[Leg][j];
      }
    }
    ControlMode[Leg]=Mode;
  }

void MoveCore::Move(Moves MOV, int Parameter){
//  Serial.print("Mover: ");
//  Serial.print(MOV);
//  Serial.print(' ');
//  Serial.println(Parameter);
  for (int leg = 0;leg<4;leg++){
    if (ControlMode[leg]!=0){
      startposition();
      SetLegMode(leg,0);
    }
  }
  switch(MOV){
    case MOV_INIT:
      set_site(0, x_default - x_offset, y_start + y_step, z_boot);
      set_site(1, x_default - x_offset, y_start + y_step, z_boot);
      set_site(2, x_default + x_offset, y_start, z_boot);
      set_site(3, x_default + x_offset, y_start, z_boot);
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          site_now[i][j] = site_expect[i][j];
        }
      };break;

      case MOV_STEPFORWARD: step_forward(1);break;
      case MOV_STEPBACK: step_back(1);break;
      case MOV_TURNLEFT: turn_left(1);break;
      case MOV_TURNRIGHT: turn_right(1);break;
      case MOV_STAND: stand();break;
      case MOV_SIT: sit();break;
      case MOV_TRANSPORT: transport(); break;
      case MOV_HANDWAVE: hand_wave(Parameter);break;
      case MOV_HANDSHAKE: hand_shake(Parameter);break;
      case MOV_HEADUP: head_up(Parameter);break;
      case MOV_HEADDOWN: head_down(Parameter);break;
      case MOV_NO: No(Parameter);break;
      case MOV_BODY_LEFT: body_left(Parameter); break;
      case MOV_BODY_RIGHT: body_right(Parameter); break;
      case MOV_GYRATE: gyrate(1); break;
      case MOV_TWIST: body_twist(1); break;
      case MOV_SPRAWL: sprawl(); break;
      case MOV_STAGGER: stagger(1); break;
      case MOV_TALL: tall(Parameter); break;
      case MOV_BODY_SIDE_UPDOWN: body_side_up_side_down(1); break;
      case MOV_BODY_FWD_BKWD: body_forward_backward(1); break;
      case MOV_BODY_SIDE2SIDE: body_side_to_side(1); break;
  }
}
/********************************+
 * Section deals with Movements
 */

void MoveCore::No(uint8_t count){
  Serial.print("No(");Serial.print(count);Serial.println(F(")"));
  long int p;
  int delta = (Srv.GetPath(HEADSERVO))/12;
  int waits = 20;
  while (count>0){
    for (p=Srv.GetPos(12);p>Srv.CalGet(HEADSERVO,SRV_MIN);p-=delta){
      Srv.write(HEADSERVO,p);
      delay(waits);
    }
    for (p=Srv.GetPos(12);p<Srv.CalGet(HEADSERVO,SRV_MAX);p+=delta){
      Srv.write(HEADSERVO,p);
      delay(waits);
    }
    count--;
  }
  for (p=Srv.GetPos(12);p>Srv.CalGet(HEADSERVO,SRV_REF90);p-=delta){
      Srv.write(HEADSERVO,p);
    delay(waits);
  }
  Srv.write(HEADSERVO,90);
  delay(100);
//  HeadServo.detach();
}

void MoveCore::SetAllLegMode(uint8_t mode){
  for (int leg = 0; leg < 4; leg++)
  {
    SetLegMode(leg,mode);//change to Polar Mode. Remembers current calculated positions and sets polar coordinates to reached.
    /*
    Serial.print(F("Leg "));
    Serial.print(leg);
    if (mode==1){
      Serial.print(F(" Tibia(alpha)="));
      Serial.print(site_now[leg][0]);
      Serial.print(F(" femur(beta)="));
      Serial.print(site_now[leg][1]);
      Serial.print(F(" coxa(gamma)="));
      Serial.println(site_now[leg][2]);
    }else if (mode==0){
      Serial.print(F(" x="));
      Serial.print(site_now[leg][0]);
      Serial.print(F(" y="));
      Serial.print(site_now[leg][1]);
      Serial.print(F(" z="));
      Serial.println(site_now[leg][2]);
    }
    */
  }
}

void MoveCore::startposition(){  
  const int8_t Final[4][3] = {{72,-50,-32},
                              {72,-50,-32},
                              {80,-61,0},
                              {80,-61,0}};
  for (int leg=0;leg<4;leg++){
    SetLegMode(leg,1);
    set_site(leg, Final[leg][0],Final[leg][1],Final[leg][2]);
    wait_reach(leg);    
  }
  SetAllLegMode(2);//no service until update
}

void MoveCore::transport(){
  const float Coords[][3]={{KEEP, KEEP, z_transport},
                            {KEEP,KEEP,90},
                            {70,KEEP,KEEP},
                            {KEEP,-90,KEEP},
                            {-20,KEEP,KEEP}};
  move_speed = 0.4;
  for (int substep=0;substep<5;substep++){
    for (int leg = 0; leg < 4; leg++)
    {
      set_site(leg, Coords[substep][0],Coords[substep][1],Coords[substep][2]);
    }
    wait_all_reach();
    SetAllLegMode(1);//noISR Service
  }
  SetAllLegMode(2);//noISR Service
  for (int i = 0; i<4; i++)
    for (int j = 0; j < 3; j++){
      Srv.writeCount(ServoIdx(i,j),0);
    }
  Serial.println(F("Robot is in Transport Position"));
}

void MoveCore::sit(void)
{
  Serial.println("sit()");
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::stand(void)
{
  Serial.println("stand()");
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void MoveCore::turn_left(unsigned int step)
{
  Serial.print("turn_left(");Serial.print(step);Serial.println(F(")"));
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void MoveCore::turn_right(unsigned int step)
{
  Serial.print("turn_right(");Serial.print(step);Serial.println(F(")"));
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void MoveCore::step_forward(unsigned int step)
{
  Serial.print("step_forward(");Serial.print(step);Serial.println(F(")"));
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void MoveCore::step_back(unsigned int step)
{
  Serial.print("step_back(");Serial.print(step);Serial.println(F(")"));
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

// add by RegisHsu

void MoveCore::body_left(int i)
{
  Serial.print("body_left(");Serial.print(i);Serial.println(F(")"));
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void MoveCore::body_right(int i)
{
  Serial.print("body_right(");Serial.print(i);Serial.println(F(")"));
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void MoveCore::hand_wave(int i)
{
  Serial.print("hand_wave(");Serial.print(i);Serial.println(F(")"));
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void MoveCore::hand_shake(int i)
{
  Serial.print("hand_shake(");Serial.print(i);Serial.println(F(")"));
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void MoveCore::head_up(int i)
{ float speedBackup=move_speed;
  Serial.print("head_up(");Serial.print(i);Serial.println(F(")"));
  move_speed = 0.5;
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
  move_speed = speedBackup;
}

void MoveCore::head_down(int i)
{
  Serial.print("head_down(");Serial.print(i);Serial.println(F(")"));
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void MoveCore::SetSpeedFactor(int Value){
  Serial.print(F("Setting Speed to:"));
  speed_multiple = Value/(float)100;  
  Serial.println(speed_multiple);
}

//void body_dance(int i)
//{
//  float x_tmp;
//  float y_tmp;
//  float z_tmp;
//  float body_dance_speed = 2;
//  sit();
//  move_speed = 1;
//  set_site(0, x_default, y_default, KEEP);
//  set_site(1, x_default, y_default, KEEP);
//  set_site(2, x_default, y_default, KEEP);
//  set_site(3, x_default, y_default, KEEP);
//  wait_all_reach();
//  //stand();
//  set_site(0, x_default, y_default, z_default - 20);
//  set_site(1, x_default, y_default, z_default - 20);
//  set_site(2, x_default, y_default, z_default - 20);
//  set_site(3, x_default, y_default, z_default - 20);
//  wait_all_reach();
//  move_speed = body_dance_speed;
//  head_up(30);
//  for (int j = 0; j < i; j++)
//  {
//    if (j > i / 4)
//      move_speed = body_dance_speed * 2;
//    if (j > i / 2)
//      move_speed = body_dance_speed * 3;
//    set_site(0, KEEP, y_default - 20, KEEP);
//    set_site(1, KEEP, y_default + 20, KEEP);
//    set_site(2, KEEP, y_default - 20, KEEP);
//    set_site(3, KEEP, y_default + 20, KEEP);
//    wait_all_reach();
//    set_site(0, KEEP, y_default + 20, KEEP);
//    set_site(1, KEEP, y_default - 20, KEEP);
//    set_site(2, KEEP, y_default + 20, KEEP);
//    set_site(3, KEEP, y_default - 20, KEEP);
//    wait_all_reach();
//  }
//  move_speed = body_dance_speed;
//  head_down(30);
//}
/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void MoveCore::servo_service(void)
{
  // sei(); //don't waste time for other Interrupts
  noInterrupts();
 // static double alpha, beta, gamma;
    static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    if (ControlMode[i]!=2){
      for (int j = 0; j < 3; j++)
      {
        if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
          site_now[i][j] += temp_speed[i][j];
        else
          site_now[i][j] = site_expect[i][j];
      }
    }
    
    if (ControlMode[i]==0){ //Kartesian to Polar Conversion
//      cartesian_to_polar_cordic(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
        cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
      if (ServoDebug==dbgPolarToServo){
        //print in ISR hangs with esp32
  //      Serial.print("ISRServos:");Serial.print(i);Serial.print(":");Serial.print(alpha); Serial.print(":"); Serial.print(beta);Serial.print(":"); Serial.println(gamma);
      }
    }else{ // Polar Mode
      alpha = site_now[i][0];
      beta = site_now[i][1];
      gamma = site_now[i][2];
    }
    if (ControlMode[i]!=2){
      //Direction Conversion was done already through calibration.
      Srv.write(ServoIdx(i,0), (float) gamma);//coxa (Body-Side)
      Srv.write(ServoIdx(i,1), (float) alpha);//Tibia (Middle)
      Srv.write(ServoIdx(i,2), (float) beta);//Femur (Tip)
    }

  }
  interrupts();
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}
/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::wait_reach_ca(int leg)
{ while (1){
  /*
    Serial.print(F("WaitReachCa, Leg: "));
    Serial.print(leg);
    for (int axis=0;axis<3;axis++){
      Serial.write(',');
      Serial.print(site_now[leg][axis]);
      Serial.write('-');
      Serial.print(site_expect[leg][axis]);
    }
    Serial.println();
    */
    if (abs(site_now[leg][0] - site_expect[leg][0])<=temp_speed[leg][0])
      if (abs(site_now[leg][1] - site_expect[leg][1])<=temp_speed[leg][1])
        if (abs(site_now[leg][2] - site_expect[leg][2])<=temp_speed[leg][2])
          break;
   }
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}



/*
  - trans site from cartesian to polar
  - mathematical model 2/2
  - 1 ------ 3
  -   |Body|
  - 2 ------ 4
  -Coxa: Alpha
  -Femur: Beta, Femur-Coxy=0°-->90° Beta
  -Tibia: Gamma, Femur-Tibia=90°-->0° Gamma
   ---------------------------------------------------------------------------*/

void MoveCore::cartesian_to_polar_cordic(volatile double &alpha, volatile double &beta, volatile double &gamma, volatile double x, volatile double y, volatile double z)
{
  //Black art for now, Change to Integer arithmetics.s
  //define PI for calculation
  // const float pi = 3.1415926;
  //calculate w-z degree
  double v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt_cordic(pow(x, 2) + pow(y, 2),20));
  v = w - length_c;    //          const              const             dyn       dyn            const               dyn         dyn
  alpha = arctan_cordic(z, v, 20) + arccos_cordic((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt_cordic(pow(v, 2) + pow(z, 2),20), 20);
  beta = arccos_cordic((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b, 10);
  //calculate x-y-z degree
  gamma = (w >= 0) ? arctan_cordic(y, x, 20) : arctan_cordic(-y, -x, 20);
  //trans degree pi->180
  alpha = multiply_cordic(alpha, 57.2957795130823210); //= alpha / pi * 180
  beta = multiply_cordic(beta, 57.295779513082321);  //= beta / pi * 180;
  gamma = multiply_cordic(gamma, 57.295779513082321); //= / pi * 180;
  beta = beta-90;
}

void MoveCore::cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //Black art for now, Change to Integer arithmetics.s
  //define PI for calculation
  const float pi = 3.1415926;
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;    //          const              const             dyn       dyn            const               dyn         dyn
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
  beta = beta-90;
}

/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::set_site(int leg, float x, float y, float z)
{
  if (ServoDebug==dbgSetSite){
    Serial.print("SetSite: Leg=");
    Serial.print(leg);
    Serial.print(" x=");
    Serial.print(x);
    Serial.print(" y=");
    Serial.print(y);
    Serial.print(" z=");
    Serial.print(z);
    Serial.println(';');
  }
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];
  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));
  if (ControlMode[leg]!=0){
    length=5;
  }
  //make sure we end up in all directions at final position
  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

// Start of added JC move routines

 /*
  - JC
  - Change_legs to for a X for a relaxed pose
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::relax_legs(void)
{
  move_speed = leg_move_speed;
  switch (leg_position)
  {  
    case WALK:
      if (site_now[3][1] == y_start)
      {
        move_cg(2);
        set_site(2, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(2, x_default - x_offset, y_start + y_step, z_up);
        wait_all_reach();
        return_cg(2);
        set_site(2, x_default - x_offset, y_start + y_step, z_default);
        wait_all_reach();
        move_cg(3);
        set_site(3, KEEP, KEEP , z_up);
        wait_all_reach();
        set_site(3, x_default - x_offset, y_start + y_step, z_up);
        wait_all_reach();
        return_cg(3);
        set_site(3, x_default - x_offset, y_start + y_step, z_default);
        wait_all_reach();
      }
      else
      {
        move_cg(0);
        set_site(0, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(0, x_default - x_offset, y_start + y_step, z_up);
        wait_all_reach();
        return_cg(0);
        set_site(0, x_default - x_offset, y_start + y_step, z_default);
        wait_all_reach();
        move_cg(1);
        set_site(1, KEEP, KEEP , z_up);
        wait_all_reach();
        set_site(1, x_default - x_offset, y_start + y_step, z_up);
        wait_all_reach();
        return_cg(1);
        set_site(1, x_default - x_offset, y_start + y_step, z_default);
        wait_all_reach();
      }
      break;
    
    case SPRAWL:
      set_site(1,KEEP, KEEP, z_up);
      wait_all_reach();
      set_site(1, x_default - x_offset, y_start + y_step, KEEP);
      wait_all_reach();
      
      set_site(3,KEEP, KEEP, z_up);
      wait_all_reach();
      set_site(3, x_default - x_offset, y_start + y_step, KEEP);
      wait_all_reach();
      
      set_site(0,KEEP, KEEP, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start + y_step, KEEP);
      wait_all_reach();
      
      set_site(2,KEEP, KEEP, z_up);
      wait_all_reach();
      set_site(2, x_default - x_offset, y_start + y_step, KEEP);
      wait_all_reach   ();
      
      for(int i=0; i<4; i++)
      { 
        set_site(i, x_default - x_offset, y_start + y_step, z_default);
      }
      wait_all_reach();
      break;
  }
  leg_position = RELAX;
}
/*
  - JC
  - body gyration movement
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::gyrate(unsigned int number_of_times)
{
  // set leg starting position
  relax_legs();
  move_speed = gyrate_speed;
  //capture = TRUE;
  set_site(0, KEEP, KEEP, z_default - z_bow_relative);
  set_site(3, KEEP, KEEP, z_default + z_bow_relative);
  wait_all_reach();
  
  while(number_of_times-- > 0)
  {
    set_site(2, KEEP, KEEP, z_default - z_bow_relative);
    set_site(1, KEEP, KEEP, z_default + z_bow_relative);
    set_site(0, KEEP, KEEP, z_default);
    set_site(3, KEEP, KEEP, z_default);
    wait_all_reach();
    
    set_site(3, KEEP, KEEP, z_default - z_bow_relative);
    set_site(0, KEEP, KEEP, z_default + z_bow_relative);
    set_site(2, KEEP, KEEP, z_default);
    set_site(1, KEEP, KEEP, z_default);
    wait_all_reach();
    
    set_site(1, KEEP, KEEP, z_default - z_bow_relative);
    set_site(2, KEEP, KEEP, z_default + z_bow_relative);
    set_site(3, KEEP, KEEP, z_default);
    set_site(0, KEEP, KEEP, z_default);
    wait_all_reach();
    
    set_site(0, KEEP, KEEP, z_default - z_bow_relative);
    set_site(3, KEEP, KEEP, z_default + z_bow_relative);
    set_site(1, KEEP, KEEP, z_default);
    set_site(2, KEEP, KEEP, z_default);
    wait_all_reach();    
  }
  
  // return legs to starting position
  set_site(1, KEEP, KEEP, z_default);
  set_site(3, KEEP, KEEP, z_default);
  capture = FALSE;
  wait_all_reach();
}
/*
  - JC
  - body gyration movement
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::body_side_up_side_down(unsigned int number_of_times)
{
  // set leg starting position
  relax_legs();
  move_speed = gyrate_speed;
  //capture = TRUE;
  set_site(0, KEEP, KEEP, z_default - z_bow_relative);
  set_site(1, KEEP, KEEP, z_default - z_bow_relative);
  set_site(2, KEEP, KEEP, z_default + z_bow_relative);
  set_site(3, KEEP, KEEP, z_default + z_bow_relative);
  wait_all_reach();
  
  while(number_of_times-- > 0)
  {
    set_site(2, KEEP, KEEP, z_default - z_bow_relative);
    set_site(3, KEEP, KEEP, z_default - z_bow_relative);
    set_site(0, KEEP, KEEP, z_default + z_bow_relative);
    set_site(1, KEEP, KEEP, z_default + z_bow_relative);
    wait_all_reach();
    
    set_site(0, KEEP, KEEP, z_default - z_bow_relative);
    set_site(1, KEEP, KEEP, z_default - z_bow_relative);
    set_site(2, KEEP, KEEP, z_default + z_bow_relative);
    set_site(3, KEEP, KEEP, z_default + z_bow_relative);
    wait_all_reach();
  }
  
  // return legs to starting position
  set_site(0, KEEP, KEEP, z_default);
  set_site(1, KEEP, KEEP, z_default);
  set_site(2, KEEP, KEEP, z_default);
  set_site(3, KEEP, KEEP, z_default);
  capture = FALSE;
  wait_all_reach();
}


/*
  -  JC
  - bow front legs
  - blocking function
*/
void MoveCore::bow( unsigned int times)
{
  relax_legs();
  move_speed = bow_speed;
  while(times-- > 0)
  {
    set_site(0, KEEP, KEEP, z_default + z_bow_relative);
    set_site(2, KEEP, KEEP, z_default + z_bow_relative);
    wait_all_reach();
    delay(300);
    set_site(0, KEEP, KEEP, z_default);
    set_site(2, KEEP, KEEP, z_default);
    wait_all_reach();
    delay(300);
  }
}
/*
  -  JC
  - twist body clockwise and counter clockwise
  - blocking function
*/
void MoveCore::body_twist( unsigned int times)
{
  relax_legs();
  
  move_cg(0);
  set_site(0, x_default - x_offset, y_start + y_step, z_up);
  wait_all_reach();
  set_site(0, turn_x1, turn_y1, z_up);
  wait_all_reach();
  return_cg(0);
  set_site(0, turn_x1, turn_y1, z_default);
  wait_all_reach();
  
  move_cg(1);
  set_site(1, x_default - x_offset, y_start + y_step, z_up);
  wait_all_reach();
  set_site(1, turn_x0, turn_y0, z_up);
  wait_all_reach();
  return_cg(1);
  set_site(1, turn_x0, turn_y0, z_default);
  wait_all_reach();
  
  move_cg(2);
  set_site(2, x_default - x_offset, y_start + y_step, z_up);
  wait_all_reach();
  set_site(2, turn_x0, turn_y0, z_up);
  wait_all_reach();
  return_cg(2);
  set_site(2, turn_x0, turn_y0, z_default);
  wait_all_reach();
  
  move_cg(3);
  set_site(3, x_default - x_offset, y_start + y_step, z_up);
  wait_all_reach();
  set_site(3, turn_x1, turn_y1, z_up);
  wait_all_reach();
  return_cg(3);
  set_site(3, turn_x1, turn_y1, z_default);
  wait_all_reach();
  
  move_speed = bow_speed;
  while(times-- > 0)
  {
    set_site(0, turn_x0, turn_y0, KEEP);
    set_site(1, turn_x1, turn_y1, KEEP);
    set_site(2, turn_x1, turn_y1, KEEP);
    set_site(3, turn_x0, turn_y0, KEEP);
    wait_all_reach();
    delay(300);
    set_site(0, turn_x1, turn_y1, KEEP);
    set_site(1, turn_x0, turn_y0, KEEP);
    set_site(2, turn_x0, turn_y0, KEEP);
    set_site(3, turn_x1, turn_y1, KEEP);
    wait_all_reach();
    delay(300);
  }
  move_speed = leg_move_speed;
  
  move_cg(0);
  set_site(0,KEEP, KEEP, z_up);
  wait_all_reach();
  set_site(0, x_default - x_offset, y_start + y_step, z_up);
  wait_all_reach();
  return_cg(0);
  set_site(0, x_default - x_offset, y_start + y_step, z_default);
  wait_all_reach();
  
  move_cg(1);
  set_site(1,KEEP, KEEP, z_up);
  wait_all_reach();
  set_site(1, x_default - x_offset, y_start + y_step, z_up);
  wait_all_reach();
  return_cg(1);
  set_site(1, x_default - x_offset, y_start + y_step, z_default);
  wait_all_reach();
  
  move_cg(2);
  set_site(2,KEEP, KEEP, z_up);
  wait_all_reach();
  set_site(2, x_default - x_offset, y_start + y_step, z_up);
  wait_all_reach();
  return_cg(2);
  set_site(2, x_default - x_offset, y_start + y_step, z_default);
  wait_all_reach();
  
  move_cg(3);
  set_site(3,KEEP, KEEP, z_up);
  wait_all_reach();
  set_site(3, x_default - x_offset, y_start + y_step, z_up);
  wait_all_reach();
  return_cg(3);
  set_site(3, x_default - x_offset, y_start + y_step, z_default);
  wait_all_reach();

}
/*
  -  JC
  - forward and backward body movement based on twist
  - blocking function
*/
void MoveCore::body_forward_backward( unsigned int times)
{
  relax_legs();
  move_speed = bow_speed;
  set_site(0, x_default - x_offset, y_start + y_step - half_step, z_default);
  set_site(1, x_default - x_offset, y_start + y_step + half_step, z_default);
  set_site(2, x_default - x_offset, y_start + y_step - half_step, z_default);
  set_site(3, x_default - x_offset, y_start + y_step + half_step, z_default);
  wait_all_reach();
  
  
  while(times-- > 0)
  {
    set_site(0, x_default - x_offset, y_start + y_step + 2*half_step, z_default);
    set_site(1, x_default - x_offset, y_start + y_step - 2*half_step, z_default);
    set_site(2, x_default - x_offset, y_start + y_step + 2*half_step, z_default);
    set_site(3, x_default - x_offset, y_start + y_step - 2*half_step, z_default);
    wait_all_reach();
    delay(300);
    set_site(0, x_default - x_offset, y_start + y_step - 2*half_step, z_default);
    set_site(1, x_default - x_offset, y_start + y_step + 2*half_step, z_default);
    set_site(2, x_default - x_offset, y_start + y_step - 2*half_step, z_default);
    set_site(3, x_default - x_offset, y_start + y_step + 2*half_step, z_default);
    wait_all_reach();
    delay(300);
  }
  set_site(0, x_default - x_offset, y_start + y_step, z_default);
  set_site(1, x_default - x_offset, y_start + y_step, z_default);
  set_site(2, x_default - x_offset, y_start + y_step, z_default);
  set_site(3, x_default - x_offset, y_start + y_step, z_default);
  wait_all_reach();
  move_speed = leg_move_speed;

}
/*
  -  JC
  - forward and backward body movement based on twist
  - blocking function
*/
void MoveCore::body_side_to_side( unsigned int times)
{
  relax_legs();
  move_speed = bow_speed;
  set_site(0, x_default - x_offset - half_step, y_start + y_step, z_default);
  set_site(1, x_default - x_offset - half_step, y_start + y_step, z_default);
  set_site(2, x_default - x_offset + half_step, y_start + y_step, z_default);
  set_site(3, x_default - x_offset + half_step, y_start + y_step, z_default);
  wait_all_reach();
  
  
  while(times-- > 0)
  {
    set_site(0, x_default - x_offset + 2*half_step, y_start + y_step, z_default);
    set_site(1, x_default - x_offset + 2*half_step, y_start + y_step, z_default);
    set_site(2, x_default - x_offset - 2*half_step, y_start + y_step, z_default);
    set_site(3, x_default - x_offset - 2*half_step, y_start + y_step, z_default);
    wait_all_reach();
    delay(300);
    set_site(0, x_default - x_offset - 2*half_step, y_start + y_step, z_default);
    set_site(1, x_default - x_offset - 2*half_step, y_start + y_step, z_default);
    set_site(2, x_default - x_offset + 2*half_step, y_start + y_step, z_default);
    set_site(3, x_default - x_offset + 2*half_step, y_start + y_step, z_default);
    wait_all_reach();
    delay(300);
  }
  set_site(0, x_default - x_offset, y_start + y_step, z_default);
  set_site(1, x_default - x_offset, y_start + y_step, z_default);
  set_site(2, x_default - x_offset, y_start + y_step, z_default);
  set_site(3, x_default - x_offset, y_start + y_step, z_default);
  wait_all_reach();
  move_speed = leg_move_speed;

}
/*
  -  JC
  - lift_up front legs
  - blocking function
*/
void MoveCore::tall( int time)
{
  relax_legs();
  move_speed = bow_speed;
  set_site(0, KEEP, KEEP, z_tall);
  set_site(2, KEEP, KEEP, z_tall);
  set_site(1, KEEP, KEEP, z_tall);
  set_site(3, KEEP, KEEP, z_tall);
  wait_all_reach();
  delay(time);
  set_site(0, KEEP, KEEP, z_default);
  set_site(2, KEEP, KEEP, z_default);
  set_site(1, KEEP, KEEP, z_default);
  set_site(3, KEEP, KEEP, z_default);
  wait_all_reach();
}
/*
  -  JC
  - bow back legs - looks a bit rude.
  - blocking function
*/
void MoveCore::back_bow( unsigned int times)
{
  relax_legs();
  move_speed = bow_speed;
  while(times-- > 0)
  {
    set_site(1, KEEP, KEEP, z_default - z_bow_relative);
    set_site(3, KEEP, KEEP, z_default - z_bow_relative);
    wait_all_reach();
    set_site(1, KEEP, KEEP, z_default);
    set_site(3, KEEP, KEEP, z_default);
    wait_all_reach();
  }
}
/*
  - JC
  - dance
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::stagger(unsigned int times)
{
  move_speed = leg_move_speed;
  while(times-- > 0)
  {
    walk_legs();
    relax_legs();
    walk_legs_other();
    relax_legs();
  }
  capture=FALSE;
}

 
/*
  - JC
  - Change_legs to for a start of walk pose V on one side = on other
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::walk_legs(void)
{
  move_speed = leg_move_speed;

  switch (leg_position)
  {  
    case RELAX:
        move_cg(2);
        set_site(2, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(2, x_default - x_offset, y_start, z_up);
        wait_all_reach();
        return_cg(2);
        set_site(2, x_default - x_offset, y_start, z_default);
        wait_all_reach();
        move_cg(3);
        set_site(3, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(3, x_default - x_offset, y_start, z_up);
        wait_all_reach();
        return_cg(3);
        set_site(3, x_default - x_offset, y_start, z_default);
        wait_all_reach();
      break;
    
    case SPRAWL:
      relax_legs();
      walk_legs();
      break;
  }
  leg_position = WALK;
}
/*
  - JC
  - Change_legs to for a start of walk pose V on one side = on other mirror image of walk_legs
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::walk_legs_other(void)
{
  move_speed = leg_move_speed;

  switch (leg_position)
  {  
    case RELAX:
        move_cg(0);
        set_site(0, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(0, x_default - x_offset, y_start, z_up);
        wait_all_reach();
        return_cg(0);
        set_site(0, x_default - x_offset, y_start, z_default);
        wait_all_reach();
        move_cg(1);
        set_site(1, KEEP, KEEP, z_up);
        wait_all_reach();
        set_site(1, x_default - x_offset, y_start, z_up);
        wait_all_reach();
        return_cg(1);
        set_site(1, x_default - x_offset, y_start, z_default);
        wait_all_reach();
      break;
    
    case SPRAWL:
      relax_legs();
      walk_legs_other();
      break;
  }
  leg_position = WALK;
}

/*
  - JC
  - Change_legs to for a X for a sprawl pose
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::sprawl(void)
{
  move_speed = leg_move_speed;

  relax_legs();
  for (int i=0; i<4; i++)
  {
    set_site(i,sprawl_leg,sprawl_leg,KEEP);
  }
  wait_all_reach();
  leg_position = SPRAWL;
}
/*
  - JC
  - Change_leg_z
  - leg to change
  - blocking function
   ---------------------------------------------------------------------------*/
void MoveCore::change_leg_z(int leg,float z)
{
  set_site(leg,KEEP,KEEP,z);
  wait_all_reach();
}
/*
  - JC
  - move_cg
  - leg to change
  - blocking function
  - move the body of the robot away from the leg that is going to be lifted
   ---------------------------------------------------------------------------*/
void MoveCore::move_cg(int leg)
{
  // reducing right leg and increasing left leg X moves right
  // reducing front leg and increasing back leg Y moves forward
  float temp_move_speed = move_speed;
  move_speed = body_move_speed;
  
  //site_now contains current leg position - remember this
  for (int i = 0; i <4; i++)
  {
    for (int j=0; j < 3; j++)
    {
      site_cg[i][j] = site_now[i][j];
    }
    if (capture)
    {
      Serial.println("CG, "+String(i)+", "+String(site_cg[i][0])+", "+String(site_cg[i][1])+", "+String(site_cg[i][2]));
    }
  }
 
  switch(leg)
  {
    case 0:  // front right leg - move body weight back and left
    set_site(0, site_cg[0][0] + cg_shift, site_cg[0][1] + cg_shift, site_cg[0][2]);
    set_site(1, site_cg[1][0] + cg_shift, site_cg[1][1] - cg_shift, site_cg[1][2]);
    set_site(2, site_cg[2][0] - cg_shift, site_cg[2][1] + cg_shift, site_cg[2][2]);
    set_site(3, site_cg[3][0] - cg_shift, site_cg[3][1] - cg_shift, site_cg[3][2]);
    break;
      
    case 1:  // back right leg - move body weight forward and left
    set_site(0, site_cg[0][0] + cg_shift, site_cg[0][1] - cg_shift, site_cg[0][2]);
    set_site(1, site_cg[1][0] + cg_shift, site_cg[1][1] + cg_shift, site_cg[1][2]);
    set_site(2, site_cg[2][0] - cg_shift, site_cg[2][1] - cg_shift, site_cg[2][2]);
    set_site(3, site_cg[3][0] - cg_shift, site_cg[3][1] + cg_shift, site_cg[3][2]);
    break;
      
    case 2:  // front left leg - move body weight back and right
    set_site(0, site_cg[0][0] - cg_shift, site_cg[0][1] + cg_shift, site_cg[0][2]);
    set_site(1, site_cg[1][0] - cg_shift, site_cg[1][1] - cg_shift, site_cg[1][2]);
    set_site(2, site_cg[2][0] + cg_shift, site_cg[2][1] + cg_shift, site_cg[2][2]);
    set_site(3, site_cg[3][0] + cg_shift, site_cg[3][1] - cg_shift, site_cg[3][2]);
    break;
      
    case 3:  // back left leg - move body weight forward and right
    set_site(0, site_cg[0][0] - cg_shift, site_cg[0][1] - cg_shift, site_cg[0][2]);
    set_site(1, site_cg[1][0] - cg_shift, site_cg[1][1] + cg_shift, site_cg[1][2]);
    set_site(2, site_cg[2][0] + cg_shift, site_cg[2][1] - cg_shift, site_cg[2][2]);
    set_site(3, site_cg[3][0] + cg_shift, site_cg[3][1] + cg_shift, site_cg[3][2]);
    break;
  } 
  wait_all_reach();
  move_speed = temp_move_speed;
  
}
/*
  - JC
  - return_cg
  - leg to change
  - blocking function
  - move the body of the robot back towards lifted leg
   ---------------------------------------------------------------------------*/
void MoveCore::return_cg(int leg)
{
  float temp_move_speed = move_speed;
  move_speed = body_move_speed;
  for (int i=0; i<4; i++)
  {
    if (i != leg)
    {
      set_site(i, site_cg[i][0], site_cg[i][1], site_cg[i][2]);
    }
  }

  move_speed = temp_move_speed;
  
}
// -- end of JC added move routines --

