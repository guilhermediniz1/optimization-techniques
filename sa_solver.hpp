#ifndef SA_SOLVER_HPP
#define SA_SOLVER_HPP

#include "utils.hpp"

struct SAParams {
  double temp_inicial = 1500.0;
  double temp_final = 0.01;
  double taxa_resfriamento = 0.99;
  int iter_temp = 50;
  int seed = -1;
  double tempo_limite = -1;
  bool verbose = true;
};

double resolverEVRPSA(const InstanciaEVRP &instancia, const string &nomeArquivo,
                      const SAParams &params = SAParams());

#endif
