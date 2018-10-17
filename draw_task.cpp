
// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_user.h"

/*! defines */

/* local variables */
static double      time_step;
static double     *cart;
static SL_Cstate  *ctarget;
static SL_Cstate  *cnext;
static int        *cstatus;
static SL_DJstate *target;
static int         firsttime = TRUE;
static double      movement_time = 1.0;
static double      tau;
static double      x_start;
static double      y_start;
static double      z_start;
static double      t = tau;


/* global functions */
extern "C" void
add_draw_task(void);

/* local functions */
static int  init_draw_task(void);
static int  run_draw_task(void);
static int  change_draw_task(void);
static void init_vars(void);
static int  calculate_min_jerk_next_step (SL_Cstate *curr_state,
					  SL_Cstate *des_state,
					  double tau,
					  double delta_t,
					  SL_Cstate *next_states);
 

void
add_draw_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  
  if (firsttime) {
    firsttime = FALSE;

    cart    = my_vector(1,n_endeffs*6);
    ctarget = (SL_Cstate *) my_calloc(n_endeffs+1,sizeof(SL_Cstate),MY_STOP);
    cnext   = (SL_Cstate *) my_calloc(n_endeffs+1,sizeof(SL_Cstate),MY_STOP);
    cstatus = my_ivector(1,n_endeffs*6);
    target  = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);


    addTask("Draw Task", init_draw_task, 
	    run_draw_task, change_draw_task);
  }

}    


static int 
init_draw_task(void)
{
  int    j, i;
  char   string[100];
  double max_range=0;
  int    ans;
  double aux;
  int    flag = FALSE;
  int    iaux;
  
  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Goto task can only be run if no other task is running!\n");
    return FALSE;
  }

  /* initialize some variables */
  init_vars();
  time_step = 1./(double)task_servo_rate;

  
  /* go to the default state */
  for (i=1; i<=n_dofs; ++i)
    target[i] = joint_default_state[i];

  target[R_SFE].th = -1.5;
  target[R_HR].th  = 1;
  target[R_EB].th  = 1.2;

  if (!go_target_wait_ID(target))
    return FALSE;

  movement_time = 2*M_PI;
  tau = movement_time;

  /* we move with the right hand */
  cstatus[(RIGHT_HAND-1)*6+_X_] = TRUE;
  cstatus[(RIGHT_HAND-1)*6+_Y_] = TRUE;
  cstatus[(RIGHT_HAND-1)*6+_Z_] = TRUE;

  /* choose as target 15 cm distance in x direction */
  //ctarget[RIGHT_HAND].x[_X_] = cart_des_state[RIGHT_HAND].x[_X_] + 0.15;
  //ctarget[RIGHT_HAND].x[_Y_] = cart_des_state[RIGHT_HAND].x[_Y_];
  //ctarget[RIGHT_HAND].x[_Z_] = cart_des_state[RIGHT_HAND].x[_Z_];
    x_start =cart_des_state[RIGHT_HAND].x[_X_];
    y_start =cart_des_state[RIGHT_HAND].x[_Y_];
    z_start =cart_des_state[RIGHT_HAND].x[_Z_];

  /* the cnext state is the desired state as seen form this program */
  for (i=1; i<=n_endeffs;++i) 
    cnext[i] = cart_des_state[i];

  /* ready to go */
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
    
  if (ans != 1) 
    return FALSE;
  
  scd();

  return TRUE;

}

static void
init_vars(void) 
{
  if (firsttime) {
    firsttime = FALSE;
    ivec_zero(cstatus);
    vec_zero(cart);
    bzero((char *)&(ctarget[1]),n_endeffs*sizeof(ctarget[1]));
  }
}


static int 
run_draw_task(void)
{
  int j, i;
  double sum=0;
  double aux;

  /* has the movement time expired? I intentially run 0.5 sec longer */
  if (tau <= -0.5*0 ) {
    freeze();
    return TRUE; 
  }

  /* progress by min jerk in cartesian space */
  //calculate_min_jerk_next_step(cnext,ctarget,tau,time_step,cnext);
    //version 1
    //cnext[RIGHT_HAND].x[_X_] = x_start + 0.06 * sin(t - tau);
    //cnext[RIGHT_HAND].x[_Z_] = z_start + 0.06 * sin(t - tau) * cos(t - tau);
    
    //version 2
    //cnext[RIGHT_HAND].x[_X_] = x_start + 0.06 * sin(M_PI/5*(t - tau));
    //cnext[RIGHT_HAND].x[_Z_] = z_start + 0.06 * sin(M_PI/5*(t - tau)) * cos(M_PI/5*(t - tau));

    // version 3
    //cnext[RIGHT_HAND].x[_X_] = cart_des_state[RIGHT_HAND].x[_X_] - 0.3 * sin((t-tau) + time_step) + 0.3 * sin(t-tau);
    //cnext[RIGHT_HAND].x[_Z_] = cart_des_state[RIGHT_HAND].x[_Z_] + 0.2 * sin((t-tau) + time_step)*cos((t-tau)+time_step) -0.2 * sin(t-tau)*cos(t-tau);

    
    // version 4
    //cnext[RIGHT_HAND].x[_X_] = cart_des_state[RIGHT_HAND].x[_X_] - 0.5 * cos((2*M_PI-tau) + time_step) + 0.5 * cos((2*M_PI-tau));
    //cnext[RIGHT_HAND].x[_Z_] = cart_des_state[RIGHT_HAND].x[_Z_] + 0.25 * sin(2*(2*M_PI-tau) + time_step) -0.25 * sin(2*(2*M_PI-tau));
    
    //versionn 5
    //cnext[RIGHT_HAND].x[_X_] = cart_des_state[RIGHT_HAND].x[_X_] + 0.3 * cos((t-tau) + time_step) - 0.3 * cos(t-tau);
    //cnext[RIGHT_HAND].x[_Z_] = cart_des_state[RIGHT_HAND].x[_Z_] + 0.2 * sin(2*(t-tau) + time_step) -0.2 * sin(2*(t-tau));
    
    // version 6
    //cnext[RIGHT_HAND].x[_X_] = cart_des_state[RIGHT_HAND].x[_X_] + 0.3 * sin((t-tau) + time_step) + 0.3 * sin(t-tau);
    //cnext[RIGHT_HAND].x[_Z_] = cart_des_state[RIGHT_HAND].x[_Z_] + 0.2 * sin(2*(t-tau) + time_step) -0.2 * sin(2*(t-tau));

    //best performance
     cnext[RIGHT_HAND].x[_X_] = cart_des_state[RIGHT_HAND].x[_X_] - 0.3 * cos((t-tau) + time_step) + 0.3 * cos(t-tau);
     cnext[RIGHT_HAND].x[_Z_] = cart_des_state[RIGHT_HAND].x[_Z_] + 0.3 * sin(2*(t-tau) + time_step) -0.3 * sin(2*(t-tau));
     tau -= time_step;
 
  /* shuffle the target for the inverse kinematics */
  for (i=1; i<=n_endeffs; ++i) {
    for (j=1; j<=N_CART; ++j) {
      aux  = cnext[i].x[j] - cart_des_state[i].x[j];
      cart[(i-1)*6+j] = cnext[i].xd[j] + 20.*aux;
    }
  }

  /* inverse kinematics */
  for (i=1; i<=n_dofs; ++i) {
    target[i].th = joint_des_state[i].th;
  }
  if (!inverseKinematics(target,endeff,joint_opt_state,
			 cart,cstatus,time_step)) {
    freeze();
    return FALSE;
  }


  // assign desired state
  for (i=1; i<=n_dofs; ++i) {

    joint_des_state[i].thd  = target[i].thd;
    joint_des_state[i].th   = target[i].th;

    // check range of motion violation
    if (joint_des_state[i].th > joint_range[i][MAX_THETA]) {
      joint_des_state[i].th = joint_range[i][MAX_THETA];
      joint_des_state[i].thd = 0.0;
    }
    if (joint_des_state[i].th < joint_range[i][MIN_THETA]) {
      joint_des_state[i].th = joint_range[i][MIN_THETA];
      joint_des_state[i].thd = 0.0;
    }
  }

  return TRUE;

}


static int 
change_draw_task(void)
{
  int j, i;

  return TRUE;

}

static int 
calculate_min_jerk_next_step (SL_Cstate *curr_state,
			      SL_Cstate *des_state,
			      double tau,
			      double delta_t,
			      SL_Cstate *next_state)

{
  double t1,t2,t3,t4,t5;
  double tau1,tau2,tau3,tau4,tau5;
  int    i,j;

  if (delta_t > tau || tau < 1./(double)task_servo_rate || delta_t <= 0) {
    return FALSE;
  }

  t1 = delta_t;
  t2 = t1 * delta_t;
  t3 = t2 * delta_t;
  t4 = t3 * delta_t;
  t5 = t4 * delta_t;

  tau1 = tau;
  tau2 = tau1 * tau;
  tau3 = tau2 * tau;
  tau4 = tau3 * tau;
  tau5 = tau4 * tau;

  for (j=1; j<=n_endeffs; ++j) {
    for (i=1; i<=N_CART; ++i) {

      if (cstatus[(j-1)*6+i]) {
	
	/* calculate the constants */
	
	const double dist   = des_state[j].x[i] - curr_state[j].x[i];
	const double p1     = des_state[j].x[i];
	const double p0     = curr_state[j].x[i];
	const double a1t2   = des_state[j].xdd[i];
	const double a0t2   = curr_state[j].xdd[i];
	const double v1t1   = des_state[j].xd[i];
	const double v0t1   = curr_state[j].xd[i];
	
	const double c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) - 
	  3.*(v0t1 + v1t1)/tau4;
	const double c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) +
	  (8.*v0t1 + 7.*v1t1)/tau3; 
	const double c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) -
	  (6.*v0t1 + 4.*v1t1)/tau2; 
	const double c4 = curr_state[j].xdd[i]/2.;
	const double c5 = curr_state[j].xd[i];
	const double c6 = curr_state[j].x[i];
	
	next_state[j].x[i]   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
	next_state[j].xd[i]  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
	next_state[j].xdd[i] = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;
	
      }
    }
  }
  
  return TRUE;
}

