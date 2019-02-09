#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>
#include <iostream>

int main( )
{
    USING_NAMESPACE_ACADO

    DifferentialState        px, py, pz, vx, vy, vz, qw, qx, qy, qz;    // the differential states
    Control                  c, wx, wy, wz         ;                    // the control input u
    DifferentialEquation     f;                                         // the differential equation

    double g = -9.8;    // Value of Gravity

    //Cost function is taken as
    //Int_{t=0}^{t=t} 200*(px_ref-px)^2+200*(py_ref-py)^2+500*(pz_ref-pz)^2+ ...+50*(qz_ref-qz)^2

    DMatrix Qd(10,1);
    Qd << 200, 200, 500, 10, 10, 10, 50, 50, 50, 50; //Scaling Factors of px,py,pz,vx ... qz in the cost function.
    DMatrix Q(10,10); //Coverting it into a matrix from vector
    Q = Qd.asDiagonal();

    Function h;
    h<<px; h<<py; h<<pz; h<<vx; h<<vy; h<<vz; h<<qw; h<<qx; h<<qy; h<<qz;
    //  -------------------------------------
    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;     //Time at which problem starts
    const double t_end   = 5.0;     //Time at which problem ends

    OCP ocp( t_start, t_end, 20 );      //20 is the number of steps taken or no of intervals between t_end and t_start
    ocp.minimizeLSQ( Q, h);         //Least Square Optimisation of Q,h,r
    ocp.minimizeLSQEndTerm( Q, h );

    f << dot(px) == vx;                         // Defining the differential Equations
    f << dot(py) == vy;
    f << dot(pz) == vz;
    f << dot(vx) == 2.0*(qx*qz - qw*qy)*c;
    f << dot(vy) == 2.0*(qy*qz + qw*qx)*c;
    f << dot(vz) == (1.0 - 2.0*qx*qx - 2.0*qy*qy)*c+g;
    f << dot(qw) == (-wx*qx-wy*qy-wz*qz)/2.0;
    f << dot(qx) == (wx*qw+wz*qy-wy*qz)/2.0;
    f << dot(qy) == (wy*qw-wz*qx+wx*qz)/2.0;
    f << dot(qz) == (wz*qw+wy*qx-wx*qy)/2.0;

    ocp.subjectTo( f                   );

    ocp.subjectTo( 2.0 <= c <=  20.0   );     
    ocp.subjectTo( -3.0 <= wx <=  3.0   );
    ocp.subjectTo( -3.0 <= wy <=  3.0   );
    ocp.subjectTo( -2.0 <= wz <=  2.0   );
//  -------------------------------------

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
    // RealTimeAlgorithm alg( ocp,0.05 );
    // alg.set( MAX_NUM_ITERATIONS, 2 );

    // alg.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    // alg.set(DISCRETIZATION_TYPE,       MULTIPLE_SHOOTING );
    // alg.set(SPARSE_QP_SOLUTION,        FULL_CONDENSING_N2  );
    // alg.set(INTEGRATOR_TYPE,           INT_IRK_GL2       );
    // alg.set(NUM_INTEGRATOR_STEPS,         20                  );
    // alg.set(QP_SOLVER,                 QP_QPOASES        );
    // alg.set(HOTSTART_QP,               NO                );
    // alg.set(LEVENBERG_MARQUARDT,          1e-10              );
    // alg.set(LINEAR_ALGEBRA_SOLVER,      GAUSS_LU         );
    // alg.set(IMPLICIT_INTEGRATOR_NUM_ITS,  2                  );
    // alg.set(CG_USE_OPENMP,              YES              );
    // alg.set(CG_HARDCODE_CONSTRAINT_VALUES,NO              );
    // alg.set(CG_USE_VARIABLE_WEIGHTING_MATRIX,NO           );


    // Export the code:
    OCPexport mpc(ocp);

    mpc.set( INTEGRATOR_TYPE , INT_RK4 );
    mpc.set( NUM_INTEGRATOR_STEPS , 30 );
    mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    mpc.set( GENERATE_TEST_FILE,NO );

    if (mpc.exportCode("hedwig_mpc_codegen") != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;


    return 0;
}
