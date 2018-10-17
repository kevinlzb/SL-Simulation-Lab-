
// system headers
#include "SL_system_headers.h"

/* SL includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

// defines

// local variables
static double      start_time = 0.0;
static SL_DJstate  target[N_DOFS+1];
static SL_Cstate   cog_target;
static SL_Cstate   cog_traj;
static SL_Cstate   cog_ref;
static double      delta_t = 0.01;
static double      duration = 5.0;
static double      time_to_go;
static int         action_state;
static int         control_mode;

// possible states of a state machine
enum ACTION_STATE {
    START,
    COG_TO_RIGHT_INIT,
    COG_TO_RIGHT,
    LEFT_LEG_UP,
    LEFT_LEG_STRETCH,
    COG_TO_LEFT,
    RIGHT_LEG_UP,
    RIGHT_LEG_STRETCH,
    TO_DEFAULT
};

enum CONTROL_MODE {
    CART,
    JOINT
};

// variables for COG control
static iMatrix     stat;
static Matrix      Jccogp;
static Matrix      NJccog;
static Matrix      fc;
static Vector      thd;
static Vector      cog_ref_xd;

// global functions
extern "C" void
add_balance_task( void );

// local functions
static int  init_balance_task(void);
static int  run_balance_task(void);
static int  change_balance_task(void);

static int
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
            double t_togo, double dt,
            double *x_next, double *xd_next, double *xdd_next);


void
add_balance_task( void )
{
  int i, j;

  addTask("Balance Task", init_balance_task,
      run_balance_task, change_balance_task);

}

static int
init_balance_task(void)
{
  int j, i;
  int ans;
  static int firsttime = TRUE;

  if (firsttime){
    firsttime = FALSE;

    // allocate memory
    stat   = my_imatrix(1,N_ENDEFFS,1,2*N_CART);
    Jccogp = my_matrix(1,N_DOFS,1,N_CART);
    NJccog = my_matrix(1,N_DOFS,1,N_DOFS+2*N_CART);
    fc     = my_matrix(1,N_ENDEFFS,1,2*N_CART);
    thd    = my_vector(1,N_DOFS);
    cog_ref_xd   = my_vector(1,N_CART);

    // this is an indicator which Cartesian components of the endeffectors are constraints
    // i.e., both feet are on the ground and cannot move in position or orientation
    stat[RIGHT_FOOT][1] = TRUE;
    stat[RIGHT_FOOT][2] = TRUE;
    stat[RIGHT_FOOT][3] = TRUE;
    stat[RIGHT_FOOT][4] = TRUE;
    stat[RIGHT_FOOT][5] = TRUE;
    stat[RIGHT_FOOT][6] = TRUE;

    stat[LEFT_FOOT][1] = TRUE;
    stat[LEFT_FOOT][2] = TRUE;
    stat[LEFT_FOOT][3] = TRUE;
    stat[LEFT_FOOT][4] = TRUE;
    stat[LEFT_FOOT][5] = TRUE;
    stat[LEFT_FOOT][6] = TRUE;

  }

  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (i=1; i<=N_DOFS; i++)
    target[i] = joint_default_state[i];

  target[2].th  = target[2].th - 0.4;
  target[8].th  = target[8].th - 0.4;

  // go to the target using inverse dynamics (ID)
  if (!go_target_wait_ID(target))
    return FALSE;

  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }

  // only go when user really types the right thing
  if (ans != 1)
    return FALSE;

  start_time = task_servo_time;
  printf("start time = %.3f, task_servo_time = %.3f\n",
     start_time, task_servo_time);

  // start data collection
  scd();

  // state machine starts at ASSIGN_COG_TARGET
  action_state = START;
  time_to_go = -1;

  return TRUE;
}

static int
run_balance_task(void)
{
  int j, i, n;

  double task_time;
  double kp = 0.1;

  // ******************************************
  // NOTE: all array indices start with 1 in SL
  // ******************************************

  task_time = task_servo_time - start_time;

  // the following code computes the contraint COG Jacobian
  // Jccogp is an N_DOFS x N_CART matrix
  // NJccog is an N_DOFS x N_DOF+2*N_CART matrix -- most likely this is not needed

  compute_cog_kinematics(stat, TRUE, FALSE, TRUE, Jccogp, NJccog);

  if(time_to_go <= 0){
      switch (action_state){
      case START:
          action_state = COG_TO_RIGHT_INIT; // next state
          break;

      case COG_TO_RIGHT_INIT:
          control_mode = CART; // current state's control mode
          time_to_go = duration; // current state's duration
          action_state = LEFT_LEG_UP; // next state

          // what is the target for the COG?
          bzero((void *)&cog_target,sizeof(cog_target));
          cog_target.x[_X_] = cart_des_state[RIGHT_FOOT].x[_X_] - 0.02;
          cog_target.x[_Y_] = cart_des_state[RIGHT_FOOT].x[_Y_] + 0.02;
          cog_target.x[_Z_] = cog_des.x[_Z_];

          // the structure cog_des has the current position of the COG computed from the
          // joint_des_state of the robot. cog_des should track cog_traj
          bzero((void *)&cog_traj,sizeof(cog_traj));
          for (i=1; i<=N_CART; ++i)
            cog_traj.x[i] = cog_des.x[i];

          break;

      case LEFT_LEG_UP:
          control_mode = JOINT;
          time_to_go = duration;
          action_state = COG_TO_LEFT;

          for (i=1; i<=N_DOFS; ++i)
            target[i] = joint_des_state[i];

          target[20].th = target[20].th + 0.3;
          target[21].th = target[21].th + 0.03;
          target[22].th = target[22].th + 0.3;

          break;

      case LEFT_LEG_STRETCH:
          control_mode = JOINT;
          time_to_go = duration;
          action_state = RIGHT_LEG_UP;

          for (i=1; i<=N_DOFS; ++i)
            target[i] = joint_des_state[i];

          target[20] = joint_default_state[20];
          target[22] = joint_default_state[22];
          target[23] = joint_default_state[23];
          target[24].th -= 0.1;
          target[18].th -= 0.08;

          break;

      case COG_TO_LEFT:
          control_mode = CART;
          time_to_go = duration;
          action_state = LEFT_LEG_STRETCH;

          // what is the target for the COG?
          bzero((void *)&cog_target,sizeof(cog_target));
          cog_target.x[_X_] = cart_des_state[LEFT_FOOT].x[_X_] + 0.06;
          cog_target.x[_Y_] = cart_des_state[LEFT_FOOT].x[_Y_] + 0.03;
          cog_target.x[_Z_] = cog_des.x[_Z_];
          // the structure cog_des has the current position of the COG computed from the
          // joint_des_state of the robot. cog_des should track cog_traj
          bzero((void *)&cog_traj,sizeof(cog_traj));
          for (i=1; i<=N_CART; ++i)
            cog_traj.x[i] = cog_des.x[i];

          break;

      case RIGHT_LEG_UP:
          control_mode = JOINT;
          time_to_go = duration;
          action_state = COG_TO_RIGHT;

          for (i=1; i<=N_DOFS; ++i)
            target[i] = joint_des_state[i];

          target[14].th = target[14].th + 0.12;
          target[15].th = target[15].th - 0.03;
          target[16].th = target[16].th + 0.08;
          target[17].th -= 0.25;

          break;

      case COG_TO_RIGHT:
          control_mode = CART; // current state's control mode
          time_to_go = duration; // current state's duration
          action_state = RIGHT_LEG_STRETCH; // next state

          // what is the target for the COG?
          bzero((void *)&cog_target,sizeof(cog_target));
          cog_target.x[_X_] = cart_des_state[RIGHT_FOOT].x[_X_] - 0.06;
          cog_target.x[_Y_] = cart_des_state[RIGHT_FOOT].x[_Y_] + 0.03;
          cog_target.x[_Z_] = cog_des.x[_Z_];

          // the structure cog_des has the current position of the COG computed from the
          // joint_des_state of the robot. cog_des should track cog_traj
          bzero((void *)&cog_traj,sizeof(cog_traj));
          for (i=1; i<=N_CART; ++i)
            cog_traj.x[i] = cog_des.x[i];

          break;

      case RIGHT_LEG_STRETCH:
          control_mode = JOINT;
          time_to_go = duration;
          action_state = TO_DEFAULT;

          for (i=1; i<=N_DOFS; ++i)
            target[i] = joint_des_state[i];

          target[14] = joint_default_state[14];
          target[16] = joint_default_state[16];
          target[17] = joint_default_state[17];
          target[18].th += 0.2;
          target[24].th += 0.2;

          break;

      case TO_DEFAULT:
          control_mode = JOINT;
          time_to_go = duration/5;
          action_state = COG_TO_RIGHT_INIT;

          for (i=1; i<=N_DOFS; ++i)
            target[i] = joint_default_state[i];

          target[2].th  = target[2].th - 0.4;
          target[8].th  = target[8].th - 0.4;

          break;

//      case SAMPLE_STATE:
//          control_mode = CART / JOINT;
//          time_to_go = duration;
//          action_state = SAMPLE_STATE;

//          // cart mode

//          // what is the target for the COG?
//          bzero((void *)&cog_target,sizeof(cog_target));
//          cog_target.x[_X_] = cart_des_state[endeffector].x[_X_];
//          cog_target.x[_Y_] = cart_des_state[endeffector].x[_Y_];
//          cog_target.x[_Z_] = cog_des.x[_Z_];

//          // the structure cog_des has the current position of the COG computed from the
//          // joint_des_state of the robot. cog_des should track cog_traj
//          bzero((void *)&cog_traj,sizeof(cog_traj));
//          for (i=1; i<=N_CART; ++i)
//            cog_traj.x[i] = cog_des.x[i];

//          // joint mode

//          for (i=1; i<=N_DOFS; ++i)
//            target[i] = joint_des_state[i];

//          target[joint].th +-= 0;

//          break;

      }
  }
  else{ // time_to_go > 0
      switch(control_mode){
      case CART:// this is for inverse kinematics control
          // plan the next step of cog with min jerk
          for (i=1; i<=N_CART; ++i) {
            min_jerk_next_step(cog_traj.x[i],
                   cog_traj.xd[i],
                   cog_traj.xdd[i],
                   cog_target.x[i],
                   cog_target.xd[i],
                   cog_target.xdd[i],
                   time_to_go,
                   delta_t,
                   &(cog_traj.x[i]),
                   &(cog_traj.xd[i]),
                   &(cog_traj.xdd[i]));
          }

          // inverse kinematics: we use a P controller to correct for tracking erros
          for (i=1; i<=N_CART; ++i)
            cog_ref.xd[i] = kp*(cog_traj.x[i] - cog_des.x[i]) + cog_traj.xd[i];

          // compute the transpose of cog_ref.xd
          for (i=1; i<=N_CART; ++i)
              cog_ref_xd[i] = cog_ref.xd[i];

          mat_vec_mult(Jccogp, cog_ref_xd, thd);

          // compute the joint_des_state[i].th and joint_des_state[i].thd
          for (i=1; i<=N_DOFS; ++i) {

            // intialize to zero
            joint_des_state[i].thd  = 0;
            joint_des_state[i].thdd = 0;
            joint_des_state[i].uff  = 0;

            joint_des_state[i].thd = thd[i];
            joint_des_state[i].th = joint_des_state[i].thd * delta_t
                                    + joint_des_state[i].th;
          }

          // decrement time to go
          time_to_go -= delta_t;
          break;

      case JOINT:
          // compute the update for the desired states
          for (i=1; i<=N_DOFS; ++i) {
            min_jerk_next_step(joint_des_state[i].th,
                   joint_des_state[i].thd,
                   joint_des_state[i].thdd,
                   target[i].th,
                   target[i].thd,
                   target[i].thdd,
                   time_to_go,
                   delta_t,
                   &(joint_des_state[i].th),
                   &(joint_des_state[i].thd),
                   &(joint_des_state[i].thdd));
          }

          // decrement time to go
          time_to_go -= delta_t;

          break;
      }
  }


  // this is a special inverse dynamics computation for a free standing robot
  //inverseDynamicsFloat(delta_t, stat, TRUE, joint_des_state, NULL, NULL, fc);


  return TRUE;
}

static int
change_balance_task(void)
{
  int    ivar;
  double dvar;

  get_int("This is how to enter an integer variable",ivar,&ivar);
  get_double("This is how to enter a double variable",dvar,&dvar);

  return TRUE;

}



static int
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
            double t_togo, double dt,
            double *x_next, double *xd_next, double *xdd_next)

{
  double t1,t2,t3,t4,t5;
  double tau,tau1,tau2,tau3,tau4,tau5;
  int    i,j;

  // a safety check
  if (dt > t_togo || dt <= 0) {
    return FALSE;
  }

  t1 = dt;
  t2 = t1 * dt;
  t3 = t2 * dt;
  t4 = t3 * dt;
  t5 = t4 * dt;

  tau = tau1 = t_togo;
  tau2 = tau1 * tau;
  tau3 = tau2 * tau;
  tau4 = tau3 * tau;
  tau5 = tau4 * tau;

  // calculate the constants
  const double dist   = t - x;
  const double p1     = t;
  const double p0     = x;
  const double a1t2   = tdd;
  const double a0t2   = xdd;
  const double v1t1   = td;
  const double v0t1   = xd;

  const double c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) -
    3.*(v0t1 + v1t1)/tau4;
  const double c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) +
    (8.*v0t1 + 7.*v1t1)/tau3;
  const double c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) -
    (6.*v0t1 + 4.*v1t1)/tau2;
  const double c4 = xdd/2.;
  const double c5 = xd;
  const double c6 = x;

  *x_next   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
  *xd_next  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
  *xdd_next = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;

  return TRUE;
}
