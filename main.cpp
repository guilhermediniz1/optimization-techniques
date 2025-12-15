#include "utils.hpp"
#include "gurobi_solver.hpp"
#include "cplex_solver.hpp"
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {
  string nomeInstancia;
  string solver = "gurobi";

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--gurobi") == 0) {
      solver = "gurobi";
    } else if (strcmp(argv[i], "--cplex") == 0) {
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
    cout << "Solver: " << (solver == "gurobi" ? "GUROBI" : "CPLEX") << endl;

    if (solver == "gurobi") {
      resolverEVRPGurobi(instancia, nomeInstancia);
    } else {
      resolverEVRP(instancia, nomeInstancia);
    }
  }

  return 0;
}
