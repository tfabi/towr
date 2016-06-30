/**
 @file    nlp_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Declares the class NlpStructure
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_

#include <Eigen/Dense>

#include <memory>
#include <map>

namespace xpp {
namespace zmp {

class VariableSet;

/** @brief Holds the optimization variables.
  *
  * This class is responsible for holding the current values of the optimization
  * variables and giving semantic information what each variable represents. It
  * returns only exactly those values.
  */
class NlpStructure {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef double Number;
  typedef std::unique_ptr<VariableSet> VariableSetPtr;
  typedef std::map<std::string, VariableSetPtr> VariableSetMap;

  NlpStructure();
  virtual ~NlpStructure();

  void AddVariableSet(std::string name, int n_variables);

  VectorXd GetOptimizationVariables() const;
  int GetOptimizationVariableCount() const;

  void SetAllVariables(const VectorXd& x_all);
  void SetAllVariables(const Number* x_all);

  void SetVariables(std::string set_name, const VectorXd& values);
  VectorXd GetVariables(std::string set_name) const;


private:
  VariableSetMap variable_sets_;
  int n_variables_;
  VectorXd ConvertToEigen(const Number* x) const;
};


class VariableSet {
public:
  typedef Eigen::VectorXd VectorXd;
  VariableSet(int n_variables);
  virtual ~VariableSet();

  VectorXd GetVariables() const;
  void SetVariables(const VectorXd& x);
private:
  VectorXd x_;
};

} // namespace zmp
} // namespace xpp

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_NLP_STRUCTURE_H_ */
