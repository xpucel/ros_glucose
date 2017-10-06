#include <cmath>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

#include "core/SolverTypes.h"
#include "simp/SimpSolver.h"

#include "ros/ros.h"
#include "ros_glucose/AddClause.h"
#include "ros_glucose/AddClauses.h"
#include "ros_glucose/AddClausesTab.h"
#include "ros_glucose/AddClausesStr.h"
#include "ros_glucose/Clause.h"
#include "ros_glucose/Init.h"
#include "ros_glucose/Reset.h"
#include "ros_glucose/Solve.h"
#include "ros_glucose/SolveAsmpt.h"
#include "ros_glucose/SetFrozen.h"

Glucose::SimpSolver* solver;
Glucose::vec<Glucose::Lit> tmpvec;

/**
 * Converts a dimacs-style literal (int) to a Lit object
 * NB: dimacs-style integers start at (+/-) 1, while minisat Literal objects
 * start at variable id 0.
 */
Glucose::Lit intToLit(int32_t i) {
  if (i == 0)
    throw std::invalid_argument("Invalid variable id: 0 in literal");
  return Glucose::mkLit(std::abs(i) - 1, i < 0);
}

void checkSolver() {
  if (solver == NULL)
    throw std::logic_error("Solver is not initialized yet. Call the /init service first.");
}

/**
 * Ensures the solver has declared enough variables to encode this dimacs style
 * literal.
 */
void ensureVar(int32_t i) {
  i = std::abs(i);
  while(solver->nVars() < i)
    solver->newVar();
}

/**
 * ROS Service /init callback
 * Initializes the solver. Fails if the solver is already initialized.
 */
bool init(ros_glucose::Init::Request& req,
          ros_glucose::Init::Response& res) {
  if (solver != NULL)
    throw std::logic_error("Solver is already initialized. Call the /reset service.");
  solver = new Glucose::SimpSolver();
  return true;
}

/**
 * ROS Service /reset callback
 * Resets the solver. Fails if the solver is not initialized.
 */
bool reset(ros_glucose::Reset::Request& req,
           ros_glucose::Reset::Response& res) {
  checkSolver();
  delete solver;
  solver = new Glucose::SimpSolver();
  return true;
}

void addClauseVec(const std::vector<int32_t>& clause) {
  std::for_each(clause.begin(), clause.end(), ensureVar);
  tmpvec.clear();
  tmpvec.growTo(clause.size());
  std::transform(clause.begin(), clause.end(), &(*tmpvec), intToLit);    
  solver->addClause(tmpvec);
}

/**
 * ROS Service /addClause callback
 * Adds a clause to the solver. Fails if the solver is not initialized.
 * Clause should be specified in dimacs-style.
 * Invocation:
 *    rosservice call /glucose_solver/addClause "{clause:[1,-2]}"
 */
bool addClause(ros_glucose::AddClause::Request& req,
               ros_glucose::AddClause::Response& res) {
  checkSolver();
  addClauseVec(req.clause);
  return true;
}

void addClauseMsg(const ros_glucose::Clause& clause) {
  addClauseVec(clause.clause);
}

/**
 * ROS Service /addClauses callback
 * Adds several clauses to the solver. Fails if the solver is not initialized.
 * Clauses should be specified in dimacs-style without the trailing zero.
 * Invocation:
 *    rosservice call /glucose_solver/addClauses "[{clause:[1,2]},{clause:[-1,-2]},{clause:[-1]}]"
 */
bool addClauses(ros_glucose::AddClauses::Request& req,
                ros_glucose::AddClauses::Response& res) {
  checkSolver();
  std::for_each(req.clauses.begin(), req.clauses.end(), addClauseMsg);
  return true;
}

void addClausesTabImpl(const std::vector<int32_t>& clauses) {
  auto minmax = std::minmax_element(clauses.begin(), clauses.end());
  int32_t max = std::max(std::abs(*minmax.first), std::abs(*minmax.second));
  ensureVar(max);
  
  tmpvec.clear();
  for (auto it = clauses.begin(); it != clauses.end(); ++it) {
    if (*it == 0) {
      if (tmpvec.size() > 0) {
        solver->addClause(tmpvec);
        tmpvec.clear();
      }
    } else
      tmpvec.push(intToLit(*it));
  }
  if (tmpvec.size() > 0)
    solver->addClause(tmpvec);
}

/**
 * ROS Service /addClausesTab callback
 * Adds several clauses to the solver. Fails if the solver is not initialized.
 * Clauses should be specified in dimacs-style.
 * Invocation:
 *    rosservice call /glucose_solver/addClausesTab "[1,2,0,2,-3,0,-2,3,0,-1,-2,0,1,4,3]"
 */
bool addClausesTab(ros_glucose::AddClausesTab::Request& req,
                   ros_glucose::AddClausesTab::Response& res) {
  checkSolver();
  addClausesTabImpl(req.clauses);    
  return true;
}

/**
 * ROS Service /addClausesStr callback
 * Adds several clauses to the solver. Fails if the solver is not initialized.
 * Clauses should be specified in dimacs-style. All whitespace (including
 * `"\n"`) is treated equally.
 * Invocation:
 *    rosservice call /glucose_solver/addClausesStr "1 2 0 2 -3 0 -2 3 0 -1 -2 0 1 4 3"
 */
bool addClausesStr(ros_glucose::AddClausesStr::Request& req,
                   ros_glucose::AddClausesStr::Response& res) {
  checkSolver();

  std::vector<int32_t> intclauses;
  std::istringstream input(req.clauses);
  
  int lit;
  while(input >> lit) 
    intclauses.emplace_back(std::move(lit));

  addClausesTabImpl(intclauses);
    
  return true;
}

/**
 * Implementation of the solve methods.
 */
void extractModel(std::vector<int32_t>& model) {
  model.reserve(solver->nAssigns());
  for (int i = 0; i < solver->model.size(); ++i) {
    if (solver->model[i] == l_True)
      model.push_back(i+1);
    else if (solver->model[i] == l_False)
      model.push_back(-i-1);
  }
}

/**
 * ROS Service /solve callback
 */
bool solve(ros_glucose::Solve::Request& req,
           ros_glucose::Solve::Response& res) {
  checkSolver();
  res.result = solver->solve(false, true);
  if (res.result)
    extractModel(res.model);
  return true;
}

/**
 * ROS Service /solveAsmpt callback
 * Invocation:
 *    rosservice call /glucose_solver/solveAsmpt  "[-1,2]"
 */
bool solveAsmpt(ros_glucose::SolveAsmpt::Request& req,
                ros_glucose::SolveAsmpt::Response& res) {
  checkSolver();

  tmpvec.clear();
  tmpvec.growTo(req.assumptions.size());
  std::transform(req.assumptions.begin(), req.assumptions.end(),
                 &(*tmpvec), intToLit);
  for (int i = 0; i < tmpvec.size(); ++i) {
    if (solver->isEliminated(Glucose::var(tmpvec[i])))
      throw std::invalid_argument("Assumption variable has been eliminated : "
                                  + std::to_string(Glucose::var(tmpvec[i])+1));
  }
//  res.result = solver->solve(tmpvec, false, true);
  res.result = solver->solve(tmpvec);

  if (res.result)
    extractModel(res.model);
  return true;
}

/**
 * ROS Service /setFrozen callback.
 * This prevents the glucose solver to eliminate the specified variable.
 * If you intend to call the /solveAsmpt service, you should first freeze the
 * assumptions variables.
 * Invocation:
 *    rosservice call /glucose_solver/setFrozen "{var:1, frozen:true}"
 */
bool setFrozen(ros_glucose::SetFrozen::Request& req,
               ros_glucose::SetFrozen::Response& rep) {
  checkSolver();
  Glucose::Var v = Glucose::var(intToLit(req.var));
  if (v >= solver->nVars())
    throw std::out_of_range("Variable " + std::to_string(v) +
                            " not defined in the solver.");
  if (solver->isEliminated(v))
    throw std::logic_error("Variable " +  std::to_string(v) +
                           " is already eliminated. Variables must be frozen" +
                           " before any call to /solve");
  solver->setFrozen(v, req.frozen);
  return true;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "glucose_solver");
  ros::NodeHandle n;
  
  auto name = ros::this_node::getName();
  auto init_srv  = n.advertiseService(name + "/init", init);
  auto reset_srv = n.advertiseService(name + "/reset", reset);
  auto solve_srv = n.advertiseService(name + "/solve", solve);
  auto sa_srv    = n.advertiseService(name + "/solveAsmpt", solveAsmpt);
  auto sf_srv    = n.advertiseService(name + "/setFrozen", setFrozen);
  auto ac_srv    = n.advertiseService(name + "/addClause", addClause);
  auto acs_srv   = n.advertiseService(name + "/addClauses", addClauses);
  auto act_srv   = n.advertiseService(name + "/addClausesTab", addClausesTab);
  auto acd_srv   = n.advertiseService(name + "/addClausesStr", addClausesStr);
  
  ros::spin();
  
  return 0;
}
