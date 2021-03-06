#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>
#include <iostream>

int main( )
{

    USING_NAMESPACE_ACADO


    DifferentialState        px, py, pz, vx, vy, vz, qw, qx, qy, qz;     // the differential states
    Control                  c, wx, wy, wz         ;     // the control input u
    DifferentialEquation     f;     // the differential equation

    double g = -9.8;

    DMatrix Qd(10,1);
    Qd << 200, 200, 500, 10, 10, 10, 50, 50, 50, 50;
    DMatrix Q(10,10);
    Q = Qd.asDiagonal();

    Function h;
    h<<px; h<<py; h<<pz; h<<vx; h<<vy; h<<vz; h<<qw; h<<qx; h<<qy; h<<qz;
//  -------------------------------------
    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;
    const double t_end   = 5.0;

    OCP ocp( t_start, t_end, 20 );
    ocp.minimizeLSQ( Q, h );            // the time T should be optimized
    ocp.minimizeLSQEndTerm( Q, h );

    f << dot(px) == vx;                         // an implementation
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
    // ocp.subjectTo( AT_START, px ==  0.0 );
    // ocp.subjectTo( AT_START, py ==  0.0 );
    // ocp.subjectTo( AT_START, pz ==  1.0 );
    // ocp.subjectTo( AT_START, vx ==  0.0 );
    // ocp.subjectTo( AT_START, vy ==  0.0 );
    // ocp.subjectTo( AT_START, vz ==  0.0 );
    // ocp.subjectTo( AT_START, qw ==  1.0 );
    // ocp.subjectTo( AT_START, qx ==  0.0 );
    // ocp.subjectTo( AT_START, qy ==  0.0 );
    // ocp.subjectTo( AT_START, qz ==  0.0 );

    // ocp.subjectTo( AT_END, px ==  0.0 );
    // ocp.subjectTo( AT_END, py ==  0.0 );
    // ocp.subjectTo( AT_END, pz ==  1.5 );
    // ocp.subjectTo( AT_END, vx ==  0.0 );
    // ocp.subjectTo( AT_END, vy ==  0.0 );
    // ocp.subjectTo( AT_END, vz ==  0.0 );
    // ocp.subjectTo( AT_END, qw ==  1.0 );
    // ocp.subjectTo( AT_END, qx ==  0.0 );
    // ocp.subjectTo( AT_END, qy ==  0.0 );
    // ocp.subjectTo( AT_END, qz ==  0.0 );

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
    mpc.set( GENERATE_TEST_FILE,YES );

    if (mpc.exportCode("ark_mpc") != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );


    // DVector x0(10);
    // x0.setAll( 0.0 );
    // x0(2) = 1.0;
    // x0(6) = 1.0;    
    // StaticReferenceTrajectory zeroReference;

    // Controller controller( alg,zeroReference );


    // // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // // ----------------------------------------------------------
    // SimulationEnvironment sim( 0.0,3.0,process,controller );

    

    // if (sim.init( x0 ) != SUCCESSFUL_RETURN)
    //     exit( EXIT_FAILURE );
    // if (sim.run( ) != SUCCESSFUL_RETURN)
    //     exit( EXIT_FAILURE );

    // // ...AND PLOT THE RESULTS
    // // ----------------------------------------------------------
    // VariablesGrid sampledProcessOutput;
    // sim.getSampledProcessOutput( sampledProcessOutput );

    // VariablesGrid feedbackControl;
    // sim.getFeedbackControl( feedbackControl );

    // GnuplotWindow window;
    // window.addSubplot( sampledProcessOutput(0), "px" );
    // window.addSubplot( sampledProcessOutput(1), "py" );
    // window.addSubplot( sampledProcessOutput(2), "pz" );
    // window.addSubplot( sampledProcessOutput(5), "vz" );
    // window.addSubplot( feedbackControl(1),      "wx" );
    // window.addSubplot( feedbackControl(0),      "c" );
    // window.plot( );

    return EXIT_SUCCESS;


    return 0;
}
