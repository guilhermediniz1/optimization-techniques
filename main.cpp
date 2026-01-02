#include "cplex_solver.hpp"
#include "gurobi_solver.hpp"
#include "sa_solver.hpp"
#include "utils.hpp"
#include <cstring>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {
  string nomeInstancia;
  string solver = "gurobi";
  bool meta = false;
  SAParams saParams;

  for (int i = 1; i < argc; i++) {
    string arg = argv[i];
    if (arg == "--gurobi") {
      solver = "gurobi";
    } else if (arg == "--cplex") {
      solver = "cplex";
    } else if (arg == "--sa") {
      solver = "sa";
    } else if (arg == "--meta") {
      meta = true;
      solver = "sa";
    } else if (arg.find("--seed=") == 0) {
      saParams.seed = stoi(arg.substr(7));
    } else if (arg.find("--temp-inicial=") == 0) {
      saParams.temp_inicial = stod(arg.substr(15));
    } else if (arg.find("--temp-final=") == 0) {
      saParams.temp_final = stod(arg.substr(13));
    } else if (arg.find("--taxa-resfriamento=") == 0) {
      saParams.taxa_resfriamento = stod(arg.substr(20));
    } else if (arg.find("--iter-temp=") == 0) {
      saParams.iter_temp = stoi(arg.substr(12));
    } else if (arg.find("--tempo-limite=") == 0) {
      saParams.tempo_limite = stod(arg.substr(15));
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
    if (solver == "sa") {
      saParams.verbose = !meta;
      if (!meta)
        cout << "Solver: Simulated Annealing" << endl;
      double custo = resolverEVRPSA(instancia, nomeInstancia, saParams);
      if (meta) {
        cout << custo << endl;
      }
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