#pragma once

#include <cmath>
#include <memory>
#include <iostream>
#include "ct/core/systems/continuous_time/ControlledSystem.h"

using namespace ct::core;

namespace my_sys {
    namespace auv_model {

        namespace tpl {

            template <typename SCALAR>
            class AUV_Model : public ct::core::ControlledSystem<8, 8, SCALAR>
            {
                public:
                    static const size_t STATE_DIM = 8;
                    static const size_t CONTROL_DIM = 8;
                    typedef ct::core::ControlledSystem<8, 8, SCALAR> Base;
                    typedef typename Base::time_t time_t;

                    AUV_Model() = default;

                    AUV_Model(Eigen::Matrix<SCALAR, 8, 8> A,
                              Eigen::Matrix<SCALAR, 8, 8> B,
                              std::shared_ptr<ct::core::Controller<8, 8, SCALAR>> controller = nullptr)
                              : ct::core::ControlledSystem<8, 8, SCALAR>(controller, ct::core::SYSTEM_TYPE::GENERAL),
                                A_(A),
                                B_(B)
                    {
                    }

                    //! copy constructor
                    AUV_Model(const AUV_Model& arg)
                            : ct::core::ControlledSystem<8, 8, SCALAR>(arg),
                              A_(arg.A_),
                              B_(arg.B_)
                    {
                    }

                    //! destructor
                    virtual ~AUV_Model() {}
                    //! deep copy
                    AUV_Model* clone() const override { return new AUV_Model(*this); }

                    void setDynamics(Eigen::Matrix<SCALAR, 8, 8> A,
                                     Eigen::Matrix<SCALAR, 8, 8> B)
                    {
                        A_ = A;
                        B_ = B;
                    }
                    virtual void computeControlledDynamics(const ct::core::StateVector <STATE_DIM, SCALAR> &state,
                                                           const time_t &t,
                                                           const ct::core::ControlVector <CONTROL_DIM, SCALAR> &control,
                                                           ct::core::StateVector <STATE_DIM, SCALAR> &derivative) override
                    {
                        Eigen::Matrix<SCALAR, 8, 1> res;
                        res = this->A_ * state + this->B_ * control;
                        derivative(0) = res(0);
                        derivative(1) = res(1);
                        derivative(2) = res(2);
                        derivative(3) = res(3);
                        derivative(4) = res(4);
                        derivative(5) = res(5);
                        derivative(6) = res(6);
                        derivative(7) = res(7);
                    }

                private:
                Eigen::Matrix<SCALAR, 8, 8> A_;
                Eigen::Matrix<SCALAR, 8, 8> B_;
            };

        }  // namespace tpl

        typedef tpl::AUV_Model<double> AUV_Model;  // set all SCALAR type as double

    }  // namespace core
}  // namespace ct
