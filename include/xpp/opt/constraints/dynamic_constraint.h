/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_
#define XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_

#include <memory>

#include <xpp/opt/linear_inverted_pendulum.h>
#include <xpp/time_discretization_constraint.h>

namespace xpp {
namespace opt {

class BaseMotion;
class EndeffectorLoad;
class EndeffectorsMotion;
class CenterOfPressure; // zmp_ remove

class DynamicConstraint : public TimeDiscretizationConstraint {
public:
  using BaseMotionPtr = std::shared_ptr<BaseMotion>;
  using EEMotionPtr   = std::shared_ptr<EndeffectorsMotion>;
  using EELoadPtr     = std::shared_ptr<EndeffectorLoad>;

  // zmp_ remove
  using CopPtr        = std::shared_ptr<CenterOfPressure>;

  DynamicConstraint (const OptVarsPtr& opt_vars, double T, double dt);
  virtual ~DynamicConstraint ();

private:
  BaseMotionPtr com_motion_;
  EEMotionPtr ee_motion_;
  EELoadPtr ee_load_;

  CopPtr cop_;
  LinearInvertedPendulum model_;

  int GetRow(int node, int dimension) const;

  virtual void UpdateConstraintAtInstance(double t, int k) override;
  virtual void UpdateBoundsAtInstance(double t, int k) override;
  virtual void UpdateJacobianAtInstance(double t, int k) override;

  double kHeight_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_ */
