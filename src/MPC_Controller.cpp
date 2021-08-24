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
    x0.setRandom();
    u0.setZero();
    ControlVector<control_dim> u_lb;
    ControlVector<control_dim> u_hb;

    mat_t *Amat;
    mat_t *A_lagmat;
    mat_t *Bmat;
    mat_t *B_lagmat;
    matvar_t *Amat_var;
    matvar_t *Alagmat_var;
    matvar_t *Bmat_var;
    matvar_t *Blagmat_var;
    Amat = Mat_Open("../UAV_nolag_Model/A_c.mat", MAT_ACC_RDONLY);
    Bmat = Mat_Open("../UAV_nolag_Model/B_c.mat", MAT_ACC_RDONLY);
    A_lagmat = Mat_Open("../UAV_nolag_Model/A.mat", MAT_ACC_RDONLY);
    B_lagmat = Mat_Open("../UAV_nolag_Model/B.mat", MAT_ACC_RDONLY);
    Amat_var = Mat_VarRead(Amat, "A_s_c");
    Bmat_var = Mat_VarRead(Bmat, "B_c_c");
    Alagmat_var = Mat_VarRead(A_lagmat, "A");
    Blagmat_var = Mat_VarRead(B_lagmat, "B");
    const double *Adata = static_cast<const double*>(Amat_var->data);
    const double *Bdata = static_cast<const double*>(Bmat_var->data);
    const double *A_lagdata = static_cast<const double*>(Alagmat_var->data);
    const double *B_lagdata = static_cast<const double*>(Blagmat_var->data);
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

//    u_des
//    {
//        (0,0) 12
//                (1,0) 12
//                (2,0) 12
//                (3,0) 12
//                (4,0) 12
//                (5,0) 12
//                (6,0) 12
//                (7,0) 1
//    }

    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> AUV_Dynamics(new my_sys::auv_model::AUV_Model(A,B));
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
        cout << "Control Input: " << x0 << endl;
        // we break the loop in case the time horizon is reached or solve() failed
        if (ilqr_mpc.timeHorizonReached() | !success)
            break;
    }
    // the summary contains some statistical data about time delays, etc.
    ilqr_mpc.printMpcSummary();
}
