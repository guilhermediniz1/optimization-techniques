#ifndef GRASP_SOLVER_HPP
#define GRASP_SOLVER_HPP

#include "utils.hpp"
#include <string>

using namespace std;

struct GRASPParams {
  double alpha = 0.3;
  int max_iter = 100;
  int seed = -1;
  double tempo_limite = -1;
  bool verbose = true;
  int run_number = -1; // -1 = no suffix, >= 0 appends _runN to filename
};

double resolverEVRPGRASP(const InstanciaEVRP &instancia,
                         const string &nomeArquivo,
                         const GRASPParams &params = GRASPParams());

#endif
