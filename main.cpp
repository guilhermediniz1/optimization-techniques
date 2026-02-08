#include "cplex_solver.hpp"
#include "grasp_solver.hpp"
#include "gurobi_solver.hpp"
#include "utils.hpp"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {
  string nomeInstancia;
  string solver = "gurobi";
  GRASPParams graspParams;
  bool metaMode = false;

  for (int i = 1; i < argc; i++) {
    string arg = argv[i];
    if (arg == "--gurobi") {
      solver = "gurobi";
    } else if (arg == "--cplex") {
      solver = "cplex";
    } else if (arg == "--grasp") {
      solver = "grasp";
    } else if (arg == "--meta") {
      metaMode = true;
    } else if (arg.rfind("--seed=", 0) == 0) {
      graspParams.seed = atoi(arg.substr(7).c_str());
    } else if (arg.rfind("--alpha=", 0) == 0) {
      graspParams.alpha = atof(arg.substr(8).c_str());
    } else if (arg.rfind("--max-iter=", 0) == 0) {
      graspParams.max_iter = atoi(arg.substr(11).c_str());
    } else if (arg.rfind("--tempo-limite=", 0) == 0) {
      graspParams.tempo_limite = atof(arg.substr(15).c_str());
    } else if (argv[i][0] != '-') {
      nomeInstancia = argv[i];
    } else {
      cerr << "Unknown option: " << argv[i] << endl;
      return 1;
    }
  }

  if (nomeInstancia.empty()) {
    cerr << "Error: instance name is required" << endl;
    return 1;
  }

  if (metaMode) {
    graspParams.verbose = false;
  }

  InstanciaEVRP instancia;

  if (carregarInstancia(nomeInstancia, instancia)) {
    if (solver == "grasp") {
      if (graspParams.verbose) {
        cout << "Solver: GRASP" << endl;
      }
      resolverEVRPGRASP(instancia, nomeInstancia, graspParams);
    } else if (solver == "gurobi") {
      cout << "Solver: GUROBI" << endl;
      resolverEVRPGurobi(instancia, nomeInstancia);
    } else {
      cout << "Solver: CPLEX" << endl;
      resolverEVRP(instancia, nomeInstancia);
    }
  }

  return 0;
}
