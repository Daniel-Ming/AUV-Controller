#include <ct/optcon/optcon.h>
#include <iostream>
#include <matio.h>
#include "../include/AUV_model.h"

using namespace std;
using namespace ct::core;
using namespace ct::optcon;
int main(int argc, char** argv){

    const size_t state_dim = my_sys::auv_model::AUV_Model::STATE_DIM ;
    const size_t control_dim = my_sys::auv_model::AUV_Model::CONTROL_DIM;
    ct::core::Time timeHorizon = 3.0;
    StateVector<state_dim> x0;
    StateVector<state_dim> x_final;
    ControlVector<control_dim> u0;
    u0.setZero();
    ControlVector<control_dim> u_lb;
    ControlVector<control_dim> u_hb;

//    x0 << 0,0,0,0,0,0,0,0;
    x_final << 0,0,0,0,0,0,0,0;
    u_lb << -1000,-1000,-1000,-1000,0,-10,-10, 1;
    u_hb << 1000, 1000, 1000, 1000,0,10, 10, 1;

    mat_t *Amat;
    mat_t *Bmat;
    matvar_t *Amat_var;
    matvar_t *Bmat_var;
    Amat = Mat_Open("../UAV_nolag_Model/A_c.mat", MAT_ACC_RDONLY);
    Bmat = Mat_Open("../UAV_nolag_Model/B_c.mat", MAT_ACC_RDONLY);
    Amat_var = Mat_VarRead(Amat, "A_s_c");
    Bmat_var = Mat_VarRead(Bmat, "B_c_c");
    const double *Adata = static_cast<const double*>(Amat_var->data);
    const double *Bdata = static_cast<const double*>(Bmat_var->data);
    Eigen::Matrix<double,8,8> A;
    Eigen::Matrix<double,8,8> B;
    for(int col=0,i=0;col<8;col++){
        for (int row=0;row<8;row++){
            A(row,col) = Adata[i++];
        }
    }
    for(int col=0,i=0;col<8;col++){
        for (int row=0;row<8;row++){
            B(row,col) = Bdata[i++];
        }
    }
    StateVector<8> A1, B1; // 8*1
    StateVector<8> A2, B2;
    StateVector<8> A3, B3;
    StateVector<8> A4, B4;
    StateVector<8> A5, B5;
    StateVector<8> A6, B6;
    StateVector<8> A7, B7;
    StateVector<8> A8, B8;
    A1 << A(0,0),A(0,1),A(0,2),A(0,3),A(0,4),A(0,5),A(0,6),A(0,7);
    A2 << A(1,0),A(1,1),A(1,2),A(1,3),A(1,4),A(1,5),A(1,6),A(1,7);
    A3 << A(2,0),A(2,1),A(2,2),A(2,3),A(2,4),A(2,5),A(2,6),A(2,7);
    A4 << A(3,0),A(3,1),A(3,2),A(3,3),A(3,4),A(3,5),A(3,6),A(3,7);
    A5 << A(4,0),A(4,1),A(4,2),A(4,3),A(4,4),A(4,5),A(4,6),A(4,7);
    A6 << A(5,0),A(5,1),A(5,2),A(5,3),A(5,4),A(5,5),A(5,6),A(5,7);
    A7 << A(6,0),A(6,1),A(6,2),A(6,3),A(6,4),A(6,5),A(6,6),A(6,7);
    A8 << A(7,0),A(7,1),A(7,2),A(7,3),A(7,4),A(7,5),A(7,6),A(7,7);

    B1 << B(0,0),B(0,1),B(0,2),B(0,3),B(0,4),B(0,5),B(0,6),B(0,7);
    B2 << B(1,0),B(1,1),B(1,2),B(1,3),B(1,4),B(1,5),B(1,6),B(1,7);
    B3 << B(2,0),B(2,1),B(2,2),B(2,3),B(2,4),B(2,5),B(2,6),B(2,7);
    B4 << B(3,0),B(3,1),B(3,2),B(3,3),B(3,4),B(3,5),B(3,6),B(3,7);
    B5 << B(4,0),B(4,1),B(4,2),B(4,3),B(4,4),B(4,5),B(4,6),B(4,7);
    B6 << B(5,0),B(5,1),B(5,2),B(5,3),B(5,4),B(5,5),B(5,6),B(5,7);
    B7 << B(6,0),B(6,1),B(6,2),B(6,3),B(6,4),B(6,5),B(6,6),B(6,7);
    B8 << B(7,0),B(7,1),B(7,2),B(7,3),B(7,4),B(7,5),B(7,6),B(7,7);

    x0.setRandom();
    u0.setZero();
    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> AUV_Dynamics(new my_sys::auv_model::AUV_Model(A1,A2,A3,A4,A5,A6,A7,A8,B1,B2,B3,B4,B5,B6,B7,B8));
    /*************** Configure the costFunction and load intermediateCost to set Q and R ***************/
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(new ct::optcon::TermQuadratic<state_dim, control_dim>());
    intermediateCost->loadConfigFile("../include/AUVCost.info", "intermediateCost", true);
    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(new CostFunctionAnalytical<state_dim, control_dim>());
    costFunction->addIntermediateTerm(intermediateCost);
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(timeHorizon, x0,AUV_Dynamics, costFunction);
    /***************************************************************************************************/

    NLOptConSettings ilqr_settings;
    ilqr_settings.dt = 0.01;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 10;
    ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
    ilqr_settings.printSummary = true;

    size_t K = ilqr_settings.computeK(timeHorizon); // K is the number of discrete time steps for an arbitrary input time interval
    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
    NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

    // STEP 2-C: create an NLOptConSolver instance
    NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);
    // set the initial guess
    iLQR.setInitialGuess(initController);
    // we solve the optimal control problem and retrieve the solution
    iLQR.solve();
    ct::core::StateFeedbackController<state_dim, control_dim> initialSolution = iLQR.getSolution();


    /*  MPC-EXAMPLE
     * we store the initial solution obtained from solving the initial optimal control problem,
     * and re-use it to initialize the MPC solver in the following. */
    /* STEP 1: first, we set up an MPC instance for the iLQR solver and configure it. Since the MPC
     * class is wrapped around normal Optimal Control Solvers, we need to different kind of settings,
     * those for the optimal control solver, and those specific to MPC: */
    // 1) settings for the iLQR instance used in MPC. Of course, we use the same settings
    // as for solving the initial problem ...
    NLOptConSettings ilqr_settings_mpc = ilqr_settings;
    // ... however, in MPC-mode, it makes sense to limit the overall number of iLQR iterations (real-time iteration scheme)
    ilqr_settings_mpc.max_iterations = 1;
    // and we limited the printouts, too.
    ilqr_settings_mpc.printSummary = false;

    // 2) settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
    ct::optcon::mpc_settings mpc_settings;
    mpc_settings.stateForwardIntegration_ = true;
    mpc_settings.postTruncation_ = true;
    mpc_settings.measureDelay_ = true;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
    mpc_settings.coldStart_ = false;

    // STEP 2 : Create the iLQR-MPC object, based on the optimal control problem and the selected settings.
    MPC<NLOptConSolver<state_dim, control_dim>> ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);
    // initialize it using the previously computed initial controller
    ilqr_mpc.setInitialGuess(initialSolution);


    /* STEP 3: running MPC
     * Here, we run the MPC loop. Note that the general underlying idea is that you receive a state-estimate
     * together with a time-stamp from your robot or system. MPC needs to receive both that time information and
     * the state from your control system. Here, "simulate" the time measurement using std::chrono and wrap
     * everything into a for-loop.
     * The basic idea of operation is that after receiving time and state information, one executes the finishIteration() method of MPC.
     */
    auto start_time = std::chrono::high_resolution_clock::now();
    // limit the maximum number of runs in this example
    size_t maxNumRuns = 50;
    std::cout << "Starting to run MPC" << std::endl;
    for (size_t i = 0; i < maxNumRuns; i++){
        // let's for simplicity, assume that the "measured" state is the first state from the optimal trajectory plus some noise
//        if (i > 0)
//            x0 = 0.1 * StateVector<state_dim>::Random();

        // time which has passed since start of MPC
        auto current_time = std::chrono::high_resolution_clock::now();
        ct::core::Time t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

        // prepare mpc iteration
        ilqr_mpc.prepareIteration(t);
        // new optimal policy
        ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;
        // timestamp of the new optimal policy
        ct::core::Time ts_newPolicy;

        current_time = std::chrono::high_resolution_clock::now();
        t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

        bool success = ilqr_mpc.finishIteration(x0, t, newPolicy, ts_newPolicy);
        newPolicy.computeControl(x0,t,u0);
        cout << u0 << endl;

        // we break the loop in case the time horizon is reached or solve() failed
        if (ilqr_mpc.timeHorizonReached() | !success)
            break;
    }
    // the summary contains some statistical data about time delays, etc.
    ilqr_mpc.printMpcSummary();


    /********* Put constraint on control input and Setting final desired state through constraint ***********/
//    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> inputConstraints(new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
//    std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlInConstraint(new ControlInputConstraint<state_dim, control_dim>(u_lb, u_hb));
//    controlInConstraint->setName("InputConstraint");
//    inputConstraints->addIntermediateConstraint(controlInConstraint, true);
//    inputConstraints->initialize();
//
//    optConProblem.setGeneralConstraints(inputConstraints);
//
//    // we include the desired terminal state as a hard constraint
//    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> finalConstraints(new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
//    std::shared_ptr<TerminalConstraint<state_dim, control_dim>> terminalConstraint(new TerminalConstraint<state_dim, control_dim>(x_final));
//    terminalConstraint->setName("TerminalConstraint");
//    finalConstraints->addTerminalConstraint(terminalConstraint, true);
//    finalConstraints->initialize();
//
//    optConProblem.setInitialState(x0);
//    optConProblem.setGeneralConstraints(finalConstraints);
    /**********************************************************************************************************/

    /*************** COnfigure ilqr settings and MPC settings ***********/
//    NLOptConSettings ilqr_settings_mpc;
//    ilqr_settings_mpc.dt = 0.01;  // the control discretization in [sec]
//    ilqr_settings_mpc.integrator = ct::core::IntegrationType::EULERCT;
//    ilqr_settings_mpc.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
//    ilqr_settings_mpc.max_iterations = 1;
//    ilqr_settings_mpc.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
//    ilqr_settings_mpc.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
//    ilqr_settings_mpc.printSummary = true;
//
//
//    size_t K = ilqr_settings_mpc.computeK(timeHorizon); // K is the number of discrete time steps for an arbitrary input time interval
//    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
//    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
//    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
//    NLOptConSolver<state_dim, control_dim>::Policy_t initSolutionGuess(x_ref_init, u0_ff, u0_fb, ilqr_settings_mpc.dt);
//
//    ct::optcon::mpc_settings mpc_settings;
//    mpc_settings.stateForwardIntegration_ = true;
//    mpc_settings.postTruncation_ = true;
//    mpc_settings.measureDelay_ = true;
//    mpc_settings.delayMeasurementMultiplier_ = 1.0;
//    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
//    mpc_settings.coldStart_ = false;
//    /************************************************************************************/
//
//    MPC<NLOptConSolver<state_dim, control_dim>> ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);
//    ilqr_mpc.setInitialGuess(initSolutionGuess);
//
//    auto start_time = std::chrono::high_resolution_clock::now();
//    size_t maxNumRuns = 100;
//    std::cout << "Starting to run MPC" << std::endl;
//    for (size_t i = 0; i < maxNumRuns; i++){
//        // time which has passed since start of MPC
//        auto current_time = std::chrono::high_resolution_clock::now();
//        ct::core::Time t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
//
//        // prepare mpc iteration
//        ilqr_mpc.prepareIteration(t);
//        // new optimal policy
//        ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;
//        // timestamp of the new optimal policy
//        ct::core::Time ts_newPolicy;
//
//        current_time = std::chrono::high_resolution_clock::now();
//        t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
//
//        bool success = ilqr_mpc.finishIteration(x0, t, newPolicy, ts_newPolicy);
//        newPolicy.computeControl(x0,t,u0);
//        cout << success << endl;
//
//        // we break the loop in case the time horizon is reached or solve() failed
//        if (ilqr_mpc.timeHorizonReached() | !success)
//            break;
//    }
//    ilqr_mpc.printMpcSummary();
}
