#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <vector>

using namespace std;

struct No {
  int id;
  double x, y;
};

struct DemandaNo {
  int id;
  int demanda;
};

struct InstanciaEVRP {
  string nome;
  string comentario;
  string tipo;
  double valorOtimo;
  int veiculos;
  int dimensao;
  int estacoesTotal;
  int capacidade;
  double capacidadeEnergia;
  double consumoEnergia;
  string formatoBorda;

  vector<No> nos;
  vector<DemandaNo> demandas;
  vector<int> idEstacoes;
  int idDeposito;
};

bool carregarInstancia(const string &nomeArquivo, InstanciaEVRP &instancia);

void resolverEVRP(const InstanciaEVRP &instancia);
#endif
