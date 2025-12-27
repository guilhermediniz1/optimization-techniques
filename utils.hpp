#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <vector>

using namespace std;

struct No {
  int id;
  double x;
  double y;
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
  int estacoes;
  int estacoesTotal;
  int capacidade;
  int capacidadeEnergia;
  double consumoEnergia;
  string formatoBorda;
  int idDeposito;

  vector<No> nos;
  vector<DemandaNo> demandas;
  vector<int> idEstacoes;
};

void imprimirNo(const No &n);
void imprimirDemandaNo(const DemandaNo &d);
void imprimirInstanciaEVRP(const InstanciaEVRP &instancia);
bool carregarInstancia(const string &nomeArquivo, InstanciaEVRP &instancia);
double calcularDistancia(const No &a, const No &b);
No getNoByIndex(const InstanciaEVRP &instancia, int idx);
void exportEVRPtoLP(const InstanciaEVRP &instancia, const string &nomeArquivo);
int getDemandaByNodeId(const InstanciaEVRP &instancia, int nodeId);

#endif
