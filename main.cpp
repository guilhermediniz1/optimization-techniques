#include "cplex_solver.hpp"
#include "gurobi_solver.hpp"
#include "utils.hpp"
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {
  string nomeInstancia;
  string solver = "gurobi";

  for (int i = 1; i < argc; i++) {
    string arg = argv[i];
    if (arg == "--gurobi") {
      solver = "gurobi";
    } else if (arg == "--cplex") {
      solver = "cplex";
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

  InstanciaEVRP instancia;

  if (carregarInstancia(nomeInstancia, instancia)) {
    if (solver == "gurobi") {
      cout << "Solver: GUROBI" << endl;
      resolverEVRPGurobi(instancia, nomeInstancia);
    } else {
      cout << "Solver: CPLEX" << endl;
      resolverEVRP(instancia, nomeInstancia);
    }
  }

  return 0;
}
