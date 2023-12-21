#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {
template class PIDController<float>;
template class PIDController<double>;

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(int32_t errorFromMid)
{
    PREC mproportional = (mKp+0.04) * static_cast<PREC>(errorFromMid);

    mIntegral += static_cast<PREC>(errorFromMid);
    PREC mintegral_result = mKi * mIntegral;

    PREC mderivative = mKd * (static_cast<PREC>(errorFromMid) - mPrievError);
    mPrievError = static_cast<PREC>(errorFromMid);

    return mproportional + mintegral_result + mderivative;
}
template <typename PREC>
PREC PIDController<PREC>::getControlOutput_straight(int32_t errorFromMid)
{
    PREC mproportional = mKp/10 * static_cast<PREC>(errorFromMid);

    mIntegral += static_cast<PREC>(errorFromMid);
    PREC mintegral_result = mKi * mIntegral;

    PREC mderivative = mKd/10 * (static_cast<PREC>(errorFromMid) - mPrievError);
    mPrievError = static_cast<PREC>(errorFromMid);

    return mproportional + mintegral_result + mderivative;
}


} // namespace Xycar
