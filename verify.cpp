#include "utils.hpp"
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {
  string nomeInstancia;
  string solver = "cplex";

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--gurobi") == 0) {
      solver = "gurobi";
    } else if (strcmp(argv[i], "--cplex") == 0) {
      solver = "cplex";
    } else if (argv[i][0] != '-') {
      nomeInstancia = argv[i];
    } else {
      cerr << "Unknown option: " << argv[i] << endl;
      cerr << "Usage: ./verify <instance_name> [--cplex|--gurobi]" << endl;
      return 1;
    }
  }

  if (nomeInstancia.empty()) {
    cerr << "Error: instance name is required" << endl;
    cerr << "Usage: ./verify <instance_name> [--cplex|--gurobi]" << endl;
    cerr << "Example: ./verify E-n22-k4 --cplex" << endl;
    return 1;
  }

  InstanciaEVRP instancia;

  if (!carregarInstancia(nomeInstancia, instancia)) {
    cerr << "Error: could not load instance" << endl;
    return 1;
  }

  bool valido = verificarSolucaoArquivo(instancia, nomeInstancia, solver);
  return valido ? 0 : 1;
}