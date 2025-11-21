#include "utils.hpp"
#include <string>

using namespace std;

int main(int argc, char *argv[]) {

  InstanciaEVRP instancia;

  string nomeInstancia = argv[1];

  if (carregarInstancia(nomeInstancia, instancia)) {
    resolverEVRP(instancia, nomeInstancia);
  }

  return 0;
}
