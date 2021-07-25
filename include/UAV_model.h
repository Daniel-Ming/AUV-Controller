#ifndef MPC_CONTROLLER_UAV_MODEL_H
#define MPC_CONTROLLER_UAV_MODEL_H

#pragma once

#include <ct/core/core.h>
#include <matio.h>
#include <cmath>
#include <memory>
#include <iostream>

namespace my_sys {
    namespace uav_model {
        template<typename SCALAR>
        class UAV_Model : ct::core::ControlledSystem<8, 7, SCALAR> {
        public:
            static const Eigen::Matrix<SCALAR, 8, 225> STATE_DIM = 8;
            static const size_t CONTROL_DIM = 7;

            void computeControlledDynamics(const ct::core::StateVector <STATE_DIM, SCALAR> &state,
                                           const SCALAR &t,
                                           const ct::core::ControlVector <CONTROL_DIM, SCALAR> &control,
                                           ct::core::StateVector <STATE_DIM, SCALAR> &derivative) override
            {
                mat_t *Amat;
                mat_t *Bmat;
                matvar_t *Amat_var;
                matvar_t *Bmat_var;
                Amat = Mat_Open("../model_data/A.mat", MAT_ACC_RDONLY);
                Bmat = Mat_Open("../model_data/B.mat", MAT_ACC_RDONLY);
                Amat_var = Mat_VarRead(Amat, "A");
                Bmat_var = Mat_VarRead(Bmat, "B");
                const double *Adata = static_cast<const double*>(Amat_var->data);
                const double *Bdata = static_cast<const double*>(Bmat_var->data);
                this->B << Bdata[0],Bdata[1],Bdata[2],Bdata[3],Bdata[4],Bdata[5],Bdata[6],Bdata[7];
                for(int col=0,i=0;col<225;col++){
                    for (int row=0;row<8;row++){
                        this->A(row,col) = Adata[i++];
                    }
                }


            }

        private:
            Eigen::Matrix<SCALAR, 8, 225> A;
            Eigen::Matrix<SCALAR, 8, 1> B;
        };
    }
    using UAV_Model = uav_model::UAV_Model<double>; // this line ensures backwards compatibility of dependent code
} // namespace uav_model

#endif //MPC_CONTROLLER_UAV_MODEL_H
