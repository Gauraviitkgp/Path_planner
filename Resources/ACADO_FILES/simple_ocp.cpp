#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado/variables_grid/matrix_variables_grid.hpp>
#include <iostream>
#include <string>

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
    DVector r(10);
    r.setAll( 0.0 ); //R is the final state we wish the quad should achieve
    r(0) = 8.2;
    r(1) = 8.2;
    r(2) = 5.0;
    r(6) = 0.8660254;
    r(9) = -0.5;

    Function h;
    h<<px; h<<py; h<<pz; h<<vx; h<<vy; h<<vz; h<<qw; h<<qx; h<<qy; h<<qz;
//  -------------------------------------
    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;     //Time at which problem starts
    const double t_end   = 1.0;     //Time at which problem ends

    OCP ocp( t_start, t_end, 20 );      //20 is the number of steps taken or no of intervals between t_end and t_start
    ocp.minimizeLSQ( Q, h, r );         //Least Square Optimisation of Q,h,r

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

    ocp.subjectTo( f                   );       //These are the constraint Equations
    // ocp.subjectTo( AT_START, px ==  1.0 );   //Initial Conditions
    // ocp.subjectTo( AT_START, py ==  0.0 );
    // ocp.subjectTo( AT_START, pz ==  1.0 );
    // ocp.subjectTo( AT_START, vx ==  0.0 );
    // ocp.subjectTo( AT_START, vy ==  0.0 );
    // ocp.subjectTo( AT_START, vz ==  0.0 );
    // ocp.subjectTo( AT_START, qw ==  1.0 );
    // ocp.subjectTo( AT_START, qx ==  0.0 );
    // ocp.subjectTo( AT_START, qy ==  0.0 );
    // ocp.subjectTo( AT_START, qz ==  0.0 );

    // ocp.subjectTo( AT_END, px ==  0.0 );     //Final Conditions
    // ocp.subjectTo( AT_END, py ==  1.0 );
    // ocp.subjectTo( AT_END, pz ==  1.0 );
    // ocp.subjectTo( AT_END, vx ==  0.0 );
    // ocp.subjectTo( AT_END, vy ==  0.0 );
    // ocp.subjectTo( AT_END, vz ==  0.0 );
    // ocp.subjectTo( AT_END, qw ==  1.0 );
    // ocp.subjectTo( AT_END, qx ==  0.0 );
    // ocp.subjectTo( AT_END, qy ==  0.0 );
    // ocp.subjectTo( AT_END, qz ==  0.0 );

    double wxy_max = 3.0;
    double wz_max = 2.0;
    double c_min = 0.2;
    double c_max = 20.0;
    ocp.subjectTo( c_min <= c <=  c_max   );       //Inequality constraints
    ocp.subjectTo( -wxy_max <= wx <=  wxy_max  );
    ocp.subjectTo( -wxy_max <= wy <=  wxy_max   );
    ocp.subjectTo( -wz_max <= wz <=  wz_max   );
//  -------------------------------------

    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
    OutputFcn identity;
    DynamicSystem dynamicSystem( f,identity );

    Process process( dynamicSystem,INT_RK45 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
    RealTimeAlgorithm alg( ocp,0.05 );
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

    DVector x0(10);
    x0.setAll( 0.0 );
    x0(2) = 5.0;
    x0(6) = 1.0;

    // alg.init(x0);
    // alg.solve();

    
    StaticReferenceTrajectory zeroReference;

    Controller controller( alg,zeroReference );


    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
    SimulationEnvironment sim( 0.0,3.0,process,controller );

    

    if (sim.init( x0 ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );
    if (sim.run( ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
    VariablesGrid sampledProcessOutput;
    sim.getSampledProcessOutput( sampledProcessOutput );

    VariablesGrid feedbackControl;
    sim.getFeedbackControl( feedbackControl );

    GnuplotWindow window;
    window.addSubplot( sampledProcessOutput(0), "px" );
    window.addSubplot( sampledProcessOutput(1), "py" );
    window.addSubplot( sampledProcessOutput(2), "pz" );
    window.addSubplot( sampledProcessOutput(3), "vx" );
    window.addSubplot( sampledProcessOutput(4), "vy" );
    // window.addSubplot( sampledProcessOutput(5), "vz" );
    // window.addSubplot( sampledProcessOutput(6), "qw" );
    // window.addSubplot( sampledProcessOutput(7), "qx" );
    // window.addSubplot( sampledProcessOutput(8), "qy" );
    // window.addSubplot( sampledProcessOutput(9), "qz" );
    window.addSubplot( feedbackControl(0),      "c" );
    window.addSubplot( feedbackControl(1),      "wx" );
    window.addSubplot( feedbackControl(2),      "wy" );
    window.addSubplot( feedbackControl(3),      "wz" );
    sampledProcessOutput.print();
    window.plot( );

    //alg.getDifferentialStates("states.txt"    );error: ‘class ACADO::VariablesGrid’ has no member named ‘printToString’

    //alg.getParameters        ("parameters.txt");
    //std::string msg;
    //sampledProcessOutput.printToString(msg);
    return EXIT_SUCCESS;


    return 0;
}
