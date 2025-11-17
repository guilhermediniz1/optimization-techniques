#include "utils.hpp"
#include <iostream>
#include <string>

using namespace std;

int main() {
  string nomeArquivo;
  cout << "Digite o nome do arquivo (ex: E-n22-k4.evrp): ";
  cin >> nomeArquivo;

  InstanciaEVRP instancia;

  if (carregarInstancia(nomeArquivo, instancia)) {
    resolverEVRP(instancia);
    cout << "" << endl;
  } else {
    cerr << "Falha ao processar o arquivo." << endl;
  }

  return 0;
}
