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
        class UAV_Model : ct::core::ControlledSystem<8, 8, SCALAR>{
        public:
            static const size_t STATE_DIM = 8;
            static const size_t CONTROL_DIM = 8;
            typedef ct::core::ControlledSystem<8, 8, SCALAR> Base;
            typedef typename Base::time_t time_t;
            UAV_Model() = delete;

            UAV_Model(SCALAR A,
                      SCALAR B,
                      std::shared_ptr<ct::core::Controller<8, 8, SCALAR>> controller = nullptr)
                    : ct::core::ControlledSystem<8, 8, SCALAR>(controller, ct::core::SYSTEM_TYPE::GENERAL),
                      A_(A),
                      B_(B)
            {
            }

            //! copy constructor
            UAV_Model(const UAV_Model& arg)
                    : ct::core::ControlledSystem<8, 8, SCALAR>(arg),
                      A_(arg.A_),
                      B_(arg.B_)
            {
            }

//                UAV_Model(SCALAR A, SCALAR B, std::shared_ptr<ct::core::Controller<8, 8>> controller = nullptr)
//                    : ct::core::ControlledSystem<8, 8>(controller),
//                        A_(A),
//                        B_(B)
//                {
//                }
            //! deep copy
            UAV_Model* clone() const override { return new UAV_Model(*this); }
            //! destructor
            virtual ~UAV_Model() {}

            void setDynamics(SCALAR A, SCALAR B)
            {
                A_ = A;
                B_ = B;
            }
            virtual void computeControlledDynamics(const ct::core::StateVector <STATE_DIM, SCALAR> &state,
                                                   const time_t &t,
                                                   const ct::core::ControlVector <CONTROL_DIM, SCALAR> &control,
                                                   ct::core::StateVector <STATE_DIM, SCALAR> &derivative) override
            {
                derivative = this->A_ * state + this->B_ * control;
            }

        private:
            Eigen::Matrix<SCALAR, 8, 8> A_;
            Eigen::Matrix<SCALAR, 8, 8> B_;
        };
    }
    using UAV_Model = uav_model::UAV_Model<double>; // this line ensures backwards compatibility of dependent code
} // namespace uav_model

#endif //MPC_CONTROLLER_UAV_MODEL_H

/*********** Latest feasible implementation ***********/

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

                AUV_Model(StateVector<8,SCALAR> A1,
                          StateVector<8,SCALAR> A2,
                          StateVector<8,SCALAR> A3,
                          StateVector<8,SCALAR> A4,
                          StateVector<8,SCALAR> A5,
                          StateVector<8,SCALAR> A6,
                          StateVector<8,SCALAR> A7,
                          StateVector<8,SCALAR> A8,
                          StateVector<8,SCALAR> B1,
                          StateVector<8,SCALAR> B2,
                          StateVector<8,SCALAR> B3,
                          StateVector<8,SCALAR> B4,
                          StateVector<8,SCALAR> B5,
                          StateVector<8,SCALAR> B6,
                          StateVector<8,SCALAR> B7,
                          StateVector<8,SCALAR> B8,
                          std::shared_ptr<ct::core::Controller<8, 8, SCALAR>> controller = nullptr)
                        : ct::core::ControlledSystem<8, 8, SCALAR>(controller, ct::core::SYSTEM_TYPE::GENERAL),
                          A1_(A1),
                          A2_(A2),
                          A3_(A3),
                          A4_(A4),
                          A5_(A5),
                          A6_(A6),
                          A7_(A7),
                          A8_(A8),
                          B1_(B1),
                          B2_(B2),
                          B3_(B3),
                          B4_(B4),
                          B5_(B5),
                          B6_(B6),
                          B7_(B7),
                          B8_(B8)
                {
                }

                //! copy constructor
                AUV_Model(const AUV_Model& arg)
                        : ct::core::ControlledSystem<8, 8, SCALAR>(arg),
                          A1_(arg.A1_),
                          A2_(arg.A2_),
                          A3_(arg.A3_),
                          A4_(arg.A4_),
                          A5_(arg.A5_),
                          A6_(arg.A6_),
                          A7_(arg.A7_),
                          A8_(arg.A8_),
                          B1_(arg.B1_),
                          B2_(arg.B2_),
                          B3_(arg.B3_),
                          B4_(arg.B4_),
                          B5_(arg.B5_),
                          B6_(arg.B6_),
                          B7_(arg.B7_),
                          B8_(arg.B8_)
                {
                }

//                    AUV_Model(Eigen::Matrix<double,8,8> A, Eigen::Matrix<double,8,8> B, std::shared_ptr<ct::core::Controller<8, 8>> controller = nullptr)
//                        : ct::core::ControlledSystem<8, 8>(controller),
//                            A_(A),
//                            B_(B)
//                    {
//                    }
                //! destructor
                virtual ~AUV_Model() {}
                //! deep copy
                AUV_Model* clone() const override { return new AUV_Model(*this); }

                void setDynamics(StateVector<8,SCALAR> A1,
                                 StateVector<8,SCALAR> A2,
                                 StateVector<8,SCALAR> A3,
                                 StateVector<8,SCALAR> A4,
                                 StateVector<8,SCALAR> A5,
                                 StateVector<8,SCALAR> A6,
                                 StateVector<8,SCALAR> A7,
                                 StateVector<8,SCALAR> A8,
                                 StateVector<8,SCALAR> B1,
                                 StateVector<8,SCALAR> B2,
                                 StateVector<8,SCALAR> B3,
                                 StateVector<8,SCALAR> B4,
                                 StateVector<8,SCALAR> B5,
                                 StateVector<8,SCALAR> B6,
                                 StateVector<8,SCALAR> B7,
                                 StateVector<8,SCALAR> B8)
                {
                    A1_ = A1;
                    A2_ = A2;
                    A3_ = A3;
                    A4_ = A4;
                    A5_ = A5;
                    A6_ = A6;
                    A7_ = A7;
                    A8_ = A8;
                    B1_ = B1;
                    B2_ = B2;
                    B3_ = B3;
                    B4_ = B4;
                    B5_ = B5;
                    B6_ = B6;
                    B7_ = B7;
                    B8_ = B8;
                }
                virtual void computeControlledDynamics(const ct::core::StateVector <STATE_DIM, SCALAR> &state,
                                                       const time_t &t,
                                                       const ct::core::ControlVector <CONTROL_DIM, SCALAR> &control,
                                                       ct::core::StateVector <STATE_DIM, SCALAR> &derivative) override
                {
                    double res1=0;
                    double res2=0;
                    double res3=0;
                    double res4=0;
                    double res5=0;
                    double res6=0;
                    double res7=0;
                    double res8=0;
                    for (int i=0; i<8; i++){
                        res1 += A1_(i) * state(i) + B1_(i) * control(i);
                        res2 += A2_(i) * state(i) + B2_(i) * control(i);
                        res3 += A3_(i) * state(i) + B3_(i) * control(i);
                        res4 += A4_(i) * state(i) + B4_(i) * control(i);
                        res5 += A5_(i) * state(i) + B5_(i) * control(i);
                        res6 += A6_(i) * state(i) + B6_(i) * control(i);
                        res7 += A7_(i) * state(i) + B7_(i) * control(i);
                        res8 += A8_(i) * state(i) + B8_(i) * control(i);
                    }
                    // Because we are doing SCALAR Template cast and set all SCALAR equal to double
                    // thus, the following code, A1_ ...etc, can not be transfered to double type
                    // because they are matrix.
//                        double res1 = A1_.transpose() * state + B1_.transpose() * control;
//                        double res2 = A2_.transpose() * state + B2_.transpose() * control;
//                        double res3 = A3_.transpose() * state + B3_.transpose() * control;
//                        double res4 = A4_.transpose() * state + B4_.transpose() * control;
//                        double res5 = A5_.transpose() * state + B5_.transpose() * control;
//                        double res6 = A6_.transpose() * state + B6_.transpose() * control;
//                        double res7 = A7_.transpose() * state + B7_.transpose() * control;
//                        double res8 = A8_.transpose() * state + B8_.transpose() * control;
                    derivative(0) = res1;
                    derivative(1) = res2;
                    derivative(2) = res3;
                    derivative(3) = res4;
                    derivative(4) = res5;
                    derivative(5) = res6;
                    derivative(6) = res7;
                    derivative(7) = res8;
                }

            private:
                ct::core::StateVector<8,SCALAR> A1_; // 8*1
                ct::core::StateVector<8,SCALAR> A2_;
                ct::core::StateVector<8,SCALAR> A3_;
                ct::core::StateVector<8,SCALAR> A4_;
                ct::core::StateVector<8,SCALAR> A5_;
                ct::core::StateVector<8,SCALAR> A6_;
                ct::core::StateVector<8,SCALAR> A7_;
                ct::core::StateVector<8,SCALAR> A8_;
                ct::core::StateVector<8,SCALAR> B1_;
                ct::core::StateVector<8,SCALAR> B2_;
                ct::core::StateVector<8,SCALAR> B3_;
                ct::core::StateVector<8,SCALAR> B4_;
                ct::core::StateVector<8,SCALAR> B5_;
                ct::core::StateVector<8,SCALAR> B6_;
                ct::core::StateVector<8,SCALAR> B7_;
                ct::core::StateVector<8,SCALAR> B8_;
            };

        }  // namespace tpl

        typedef tpl::AUV_Model<double> AUV_Model;  // set all SCALAR type as double

    }  // namespace core
}  // namespace ct
