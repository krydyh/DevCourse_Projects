#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cstdint>
#include <memory>

namespace Xycar {
/**
 * @brief PID Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class PIDController
{
public:
    using Ptr=std::unique_ptr<PIDController>;
    
    PIDController(PREC Kp, PREC Ki, PREC Kd)
        : mKp(Kp), mKi(Ki), mKd(Kd), mPrievError(0.0f), mIntegral(0.0f)
    {
    }
    PREC getControlOutput(int32_t errorFromMid);
    PREC getControlOutput_straight(int32_t errorFromMid);

private:
    const PREC mKp;
    const PREC mKi;
    const PREC mKd;
    PREC mPrievError;
    PREC mIntegral;
  
};
} // namespace Xycar
#endif // PID_CONTROLLER_HPP_
