/*============================================================================
==============================================================================
                      
                              min_jerk_task.cpp
 
==============================================================================
Remarks:

      sekeleton to create the sample task

============================================================================*/

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
static double      delta_t = 0.01;
static double      duration = 1.0;
static double      time_to_go;
static int         nums=0;
static int         type=1;
// global functions 
extern "C" void
add_min_jerk_task( void );

// local functions
static int  init_min_jerk_task(void);
static int  run_min_jerk_task(void);
static int  change_min_jerk_task(void);

static int 
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
		    double t_togo, double dt,
		    double *x_next, double *xd_next, double *xdd_next);


/*****************************************************************************
******************************************************************************
Function Name	: add_min_jerk_task

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_min_jerk_task( void )
{
  int i, j;
  
  addTask("Min Jerk Task", init_min_jerk_task, 
	  run_min_jerk_task, change_min_jerk_task);

}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_min_jerk_task

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_min_jerk_task(void)
{
  int j, i;
  int ans;
  static int firsttime = TRUE;
  
  if (firsttime){
    firsttime = FALSE;
  }

  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (i=1; i<=N_DOFS; i++)
    target[i] = joint_default_state[i];

  // go to the target using inverse dynamics (ID)
  if (!go_target_wait_ID(target)) 
    return FALSE;

  // re-use the variable target for our min-jerk movement: only the right arm moves
//    target[R_SFE].th += 0.4;
//    target[R_SAA].th -= 0.4;
//   target[R_EB].th  -= 0.5;
  
      target[L_SFE].th += 0.8;
      
      
  

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

  // time to go
  time_to_go = duration;

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_min_jerk_task

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_min_jerk_task(void)
{
  int j, i;

  double task_time;

  // NOTE: all array indices start with 1 in SL

  task_time = task_servo_time - start_time;

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

  // compute inverse dynamics torques
  SL_InvDynNE(joint_state,joint_des_state,endeff,&base_state,&base_orient);
  
  // decrement time to go
  time_to_go -= delta_t;
  if (time_to_go <= 0){
       nums++;
        
        if(nums==1){
            target[L_SAA].th -= 0.6;
            
        }
        else if(nums==2){
           // target[L_SAA] +=0.6;
            target[L_SFE].th -= 0.6;
            
        }
        else if(nums==3){
            for (i=1; i<=N_DOFS; i++){
                target[i] = joint_default_state[i];
            }
        
        }
        else{
            freeze();
        }
        time_to_go=duration;
        run_min_jerk_task();
}
   

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_min_jerk_task

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_min_jerk_task(void)
{
  int    ivar;
  double dvar;

  get_int("This is how to enter an integer variable",ivar,&ivar);
  get_double("This is how to enter a double variable",dvar,&dvar);

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
\note  min_jerk_next_step
\date  April 2014
   
\remarks 

Given the time to go, the current state is updated to the next state
using min jerk splines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          x,xd,xdd : the current state, vel, acceleration
 \param[in]          t,td,tdd : the target state, vel, acceleration
 \param[in]          t_togo   : time to go until target is reached
 \param[in]          dt       : time increment
 \param[in]          x_next,xd_next,xdd_next : the next state after dt

 ******************************************************************************/
static int 
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
		    double t_togo, double dt,
		    double *x_next, double *xd_next, double *xdd_next)

{

 switch(type){
     case 1:{
   *x_next= x + xd*dt + 0.5*xdd*pow(dt,2) - ((20*x-20*t+12*xd*t_togo+8*td*t_togo+3*xdd*pow(t_togo,2)-tdd*pow(t_togo,2))/(2*pow(t_togo,3)))*pow(dt,3) + ((30*x-30*t+16*xd*t_togo+14*td*t_togo+3*xdd*pow(t_togo,2)-2*tdd*pow(t_togo,2))/(2*pow(t_togo,4)))*pow(dt,4) - ((12*x-12*t+6*xd*t_togo+6*td*t_togo+xdd*pow(t_togo,2)-tdd*pow(t_togo,2))/(2*pow(t_togo,5)))*pow(dt,5);
    
  *xd_next=xd + xdd*dt - 3*((20*x-20*t+12*xd*t_togo+8*td*t_togo+3 *xdd*pow(t_togo,2)-tdd*pow(t_togo,2))/(2*pow(t_togo,3)))*(pow(dt,2)) + 4*((30*x-30*t+16*xd*t_togo+14*td*t_togo+3*xdd*pow(t_togo,2)-2*tdd*pow(t_togo,2))/(2*pow(t_togo,4)))*pow(dt,3) - 5*((12*x-12*t+6*xd*t_togo+6*td*t_togo+xdd*pow(t_togo,2)-tdd*pow(t_togo,2))/(2*pow(t_togo,5)))*pow(dt,4);
    
  *xdd_next=xdd	 - 6*((20*x-20*t+12*xd*t_togo+8*td*t_togo+3 *xdd*pow(t_togo,2)-tdd*pow(t_togo,2))/(2*pow(t_togo,3)))*dt + 12*((30*x-30*t+16*xd*t_togo+14*td*t_togo+3*xdd*pow(t_togo,2)-2*tdd*pow(t_togo,2))/(2*pow(t_togo,4)))*pow(dt,2) - 20*((12*x-12*t+6*xd*t_togo+6*td*t_togo+xdd*pow(t_togo,2)-tdd*pow(t_togo,2))/(2*pow(t_togo,5)))*pow(dt,3);
   break;
           }
      case 2:{
         *x_next=x+dt*xd;
         *xd_next=xd+dt*xdd;
         *xdd_next=25*(6*(t-*x_next)-*xd_next);
    break; 
            }
 }
  return TRUE;
}

