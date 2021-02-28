
/******************************************IMPORTANT *************************************
******************************************************************************************
    in main code use (Distance_Two_sides) function first then (go to goal function)     
******************************************************************************************/

// el velocity awl point el ai hib3tli flag h5li el velocity be zero l8ait lma el angle ttzbt we b3d kda arg3 el velocity 0.5




#include "TM4C123GH6PM.h"
#include <stdint.h>
//#include "GPIO_interface.h"
#include "HighLevelConfig.h"
#include "HighLevelControl.h"
#include "math.h"
#include "stdlib.h"


// odemtry variables : 
  

	double  Distance_Right_Wheel =0 ;
	double  Distance_Left_Wheel =0 ;
	double  x_dt =0;
	double  y_dt =0;
	double  theta_dt =0;
	double  theta=0;
	double  x =0 ;
	double  y =0 ;
	double  New_x ;
	double  New_y  ;
	double  New_theta ;	
	double  Distance_Center =0;
	double  Phi =0 ;
	
// odemtry arrays :
  double  New_Current_Position [3] ={0};
	
	
// RObot PID Gains :
  float  Kp =0 ;
	float  Ki =0 ;
	float  Kd =0 ;
	
	
// Go To Goal variables :
	double Omega=0;
	double  u_x ;
	double  u_y ;
	double  theta_goal;
	double  e ;
	double  e_k ;
	double  e_P ;
	double  e_I ;
	double  e_D ;
	double  dt = 0.004 ;
	double  e_k_1 ;
	double  E_k ;
	
	
// Distance_Two_sides  variables 
  double Left_Average = 0 ;
	double Right_Average = 0 ;
	
// Description : function tell me where is the current postition of the robot now.
// Inputs      : The  average distance of the two sides of the robot
// Outputs     : the new  X , Y , Theta

void Update_Odemtry (void)
{

	Distance_Right_Wheel= Right_Average ; 
	Distance_Left_Wheel = Left_Average  ;
	
	Distance_Center =(Distance_Left_Wheel + Distance_Right_Wheel) / 2 ;
	Phi =( Distance_Right_Wheel - Distance_Left_Wheel) / Robot_Base_Length ;

	x_dt = Distance_Center * cos (theta);
 	y_dt = Distance_Center * sin (theta);
	theta_dt = Phi;
	
	New_x =  x + x_dt;
  New_y =  y + y_dt;
	New_theta = theta +Phi ;
	/*
	New_Current_Position[0] = New_x;
	New_Current_Position[1] = New_y;
	New_Current_Position[2] = atan2 (sin(New_theta) , cos(New_theta));*/
	
	theta = atan2 (sin(New_theta) , cos(New_theta)) ;
	x = New_x ;
	y = New_y ;
	
}


// Description : function takes the goal points and give us the needed W (omega)
// Inputs      : X goal and Y goal 
// Outputs     : W (omega) Needed

void Go_To_Goal (float xgoal , float ygoal )
{

	Update_Odemtry();
	
  // First Calculate the heading (angle) to the goal.
  // distance between goal and robot in x-direction
        u_x = xgoal - x ;     
                
  // distance between goal and robot in y-direction
        u_y = ygoal - y ;
                
  // angle from robot to goal. Hint: use atan to ensure that W is between [pi,-pi] 
        theta_goal = atan2( (u_y) , (u_x) ) ;
            
	// Second Calculate the heading error.
  // error between the goal angle and robot's angle
  // Hint: Use atan to make sure this stays in [-pi,pi].
        e =  theta_goal - theta; 
        e_k = atan2( sin (e) , cos (e) );      
	
	// Third Calculate PID for the steering angle 
            
  // error for the proportional term.
          e_P = e_k ;     
  // error for the integral term.
          e_I =  (e_k*dt) + E_k  ;
  // error for the derivative term.
          e_D =  e_k_1 - (e_k/dt) ;    
	// W (omega) calculations. 
	        Omega =  (Kp * e_P) + (Ki * e_I) + (Kd * e_D  );
					 
	// Fourth Save errors for next time step
          E_k = e_I;
          e_k_1 = e_k;
}
	


// Description : complementery function to odemetry function.
// Inputs      : The distance of four wheels
// Outputs     : the average of distance both sides.


void Distance_Two_sides ( double LB , double LF ,double RF ,double RB)
{
	 Left_Average  =  (LF+LB)/2;
	 Right_Average =  (RF+RB)/2;
	
}

