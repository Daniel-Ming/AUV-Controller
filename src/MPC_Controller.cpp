#include <ct/optcon/optcon.h>
using namespace ct::core;
using namespace ct::optcon;
int main(int argc, char** argv){

    const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
    const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;
    ct::core::Time timeHorizon = 3.0;
    StateVector<state_dim> x0;
    StateVector<state_dim> x_final;
    ControlVector<control_dim> u_lb;
    ControlVector<control_dim> u_hb;
    x0 << 0,0;
    x_final << 1.5,1.5;
    u_lb << 0;
    u_hb << 1;
//    x0 << 0,0,0,0,0,0,0,0,0,0,0,0;
//    x_final << 0,0,0,0,0,0,0,0,0,5,5,5;
//    u_lb << -1000,-1000,-1000,-1000,0,-10,-10;
//    u_hb << 1000, 1000, 1000, 1000,0, 10, 10;

    std::shared_ptr<SecondOrderSystem> oscillator(new SecondOrderSystem(1,1));

    /*************** Configure the costFunction and load intermediateCost to set Q and R ***************/
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(new ct::optcon::TermQuadratic<state_dim, control_dim>());
    intermediateCost->loadConfigFile("../include/dmsCost.info", "intermediateCost", true);
    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(new CostFunctionAnalytical<state_dim, control_dim>());
    costFunction->addIntermediateTerm(intermediateCost);
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(oscillator, costFunction);
    /***************************************************************************************************/

    /********* Put constraint on control input and Setting final desired state through constraint ***********/
    // we put constraint on the control input
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> inputConstraints(new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
    std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlInConstraint(new ControlInputConstraint<state_dim, control_dim>(u_lb, u_hb));
    controlInConstraint->setName("InputConstraint");
    inputConstraints->addIntermediateConstraint(controlInConstraint, true);
    inputConstraints->initialize();

    optConProblem.setGeneralConstraints(inputConstraints);

    // we include the desired terminal state as a hard constraint
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> finalConstraints(new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
    std::shared_ptr<TerminalConstraint<state_dim, control_dim>> terminalConstraint(new TerminalConstraint<state_dim, control_dim>(x_final));
    terminalConstraint->setName("TerminalConstraint");
    finalConstraints->addTerminalConstraint(terminalConstraint, true);
    finalConstraints->initialize();

    optConProblem.setInitialState(x0);
    optConProblem.setGeneralConstraints(finalConstraints);
    /**********************************************************************************************************/

    /*************** COnfigure ilqr settings and MPC settings ***********/
    NLOptConSettings ilqr_settings_mpc;
    ilqr_settings_mpc.dt = 0.01;  // the control discretization in [sec]
    ilqr_settings_mpc.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings_mpc.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings_mpc.max_iterations = 1;
    ilqr_settings_mpc.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS;
    ilqr_settings_mpc.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
    ilqr_settings_mpc.printSummary = false;


    size_t K = ilqr_settings_mpc.computeK(timeHorizon); // K is the number of discrete time steps for an arbitrary input time interval
    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
    NLOptConSolver<state_dim, control_dim>::Policy_t initSolutionGuess(x_ref_init, u0_ff, u0_fb, ilqr_settings_mpc.dt);

    ct::optcon::mpc_settings mpc_settings;
    mpc_settings.stateForwardIntegration_ = true;
    mpc_settings.postTruncation_ = true;
    mpc_settings.measureDelay_ = true;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
    mpc_settings.coldStart_ = false;
    /************************************************************************************/

    MPC<NLOptConSolver<state_dim, control_dim>> ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);
    ilqr_mpc.setInitialGuess(initSolutionGuess);

    auto start_time = std::chrono::high_resolution_clock::now();
    size_t maxNumRuns = 100;
    std::cout << "Starting to run MPC" << std::endl;
    for (size_t i = 0; i < maxNumRuns; i++){
        // let's for simplicity, assume that the "measured" state is the first state from the optimal trajectory plus some noise
        if (i > 0)
            x0 = 0.1 * StateVector<state_dim>::Random();

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
        // we break the loop in case the time horizon is reached or solve() failed
        if (ilqr_mpc.timeHorizonReached() | !success)
            break;
    }
    ilqr_mpc.printMpcSummary();
}
