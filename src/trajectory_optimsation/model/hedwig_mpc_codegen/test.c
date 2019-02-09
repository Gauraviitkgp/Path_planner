
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

int main( )
{
	/* Some temporary variables. */
	int    i, iter,II;
	acado_timer t;

	float A[10]={0,0,3.0,0,0,0,1.0,0,0,0};


	for(int II=0;II<20;II++)
	{	
		/* Initialize the solver. */
		acado_initializeSolver();

		/* Initialize the states and controls. */
		for (i = 0; i < NX * (N + 1); ++i)
		{
		  acadoVariables.x[ i ] = 0.0; 		//States
		  if (i%NX==6)
		  	acadoVariables.x[ i ] = 1.0;
		}

		for (i = 0; i < NU * N; ++i)  
			acadoVariables.u[ i ] = 1.0;	//Controls

		/* Initialize the measurements/reference. */
		// printf("ACADO_NY %d,%d,%d\n",NY,NYN,N );
		for (i = 0; i < NY * (N); ++i)
		{
			acadoVariables.y[ i ] = 0.0;
			if (i%NY==6)
		  		acadoVariables.y[ i ] = 1.0;
			if (i%NY==2)
		  		acadoVariables.y[ i ] = 5.0;
		}

		for (i = 0; i < NYN; ++i)  
		{
			acadoVariables.yN[ i ] = 0.0;
			if (i%NY==6)
			{
				acadoVariables.yN[ i ] = 1.0;
			}
			if (i%NY==2)
			{
				acadoVariables.yN[ i ] = 5.0;
			}
		}

		/* MPC: initialize the current state feedback. */
		#if 1
			for (i = 0; i < NX; i++)
			{ 
				acadoVariables.x0[ i ] = A[i]; 		//States
			}
		#endif

		if( VERBOSE ) acado_printHeader();

		/* Prepare first step */
		acado_preparationStep();

		/* Get the time before start of the loop. */
		acado_tic( &t );

		/* The "real-time iterations" loop. */
		for(iter = 0; iter < NUM_STEPS; ++iter)
		{
	        /* Perform the feedback step. */
			acado_feedbackStep( );

			/* Apply the new control immediately to the process, first NU components. */

			if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

			/* Optional: shift the initialization (look at acado_common.h). */
	  		// acado_shiftStates(2, 0, 0);
			// acado_shiftControls( 0 ); 

			/* Prepare for the next step. */
			acado_preparationStep();
		}
		/* Read the elapsed time. */
		real_t te = acado_toc( &t );

		if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

		/* Eye-candy. */

		if( !VERBOSE )
		printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);
		acado_printDifferentialVariables();
		acado_printControlVariables();

		for (i = 0; i < NX; i++)
		{ 
			A[i]=acadoVariables.x[10+i]; 		//States
		}


	}
    return 0;
}
