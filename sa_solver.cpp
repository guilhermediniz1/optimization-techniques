#include "sa_solver.hpp"
#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

using namespace std;

namespace {

void limparRotasVazias(vector<vector<int>> &rotas) {
  for (int i = rotas.size() - 1; i >= 0; i--) {
    if (rotas[i].size() <= 2) {
      rotas.erase(rotas.begin() + i);
    }
  }
}

double calcularCustoRota(const vector<int> &rota,
                         const vector<vector<double>> &dist) {
  double custo = 0.0;
  if (rota.empty())
    return 0.0;
  for (size_t i = 0; i < rota.size() - 1; i++) {
    custo += dist[rota[i]][rota[i + 1]];
  }
  return custo;
}

double calcularCustoSolucao(const vector<vector<int>> &rotas,
                            const vector<vector<double>> &dist) {
  double total = 0.0;
  for (const auto &r : rotas) {
    total += calcularCustoRota(r, dist);
  }
  return total;
}

void removerEstacoes(vector<int> &rota, int dimensao) {
  vector<int> novaRota;
  novaRota.reserve(rota.size());
  if (rota.empty()) {
    rota = {0, 0};
    return;
  }

  novaRota.push_back(0);
  for (int no : rota) {
    if (no != 0 && no < dimensao) {
      novaRota.push_back(no);
    }
  }
  novaRota.push_back(0);
  rota = novaRota;
}

bool inserirEstacoes(const InstanciaEVRP &instancia, vector<int> &rota,
                     const vector<vector<double>> &dist) {
  double energia = instancia.capacidadeEnergia;
  double h = instancia.consumoEnergia;
  vector<int> novaRota;
  novaRota.push_back(rota[0]);

  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;

  for (size_t i = 0; i < rota.size() - 1; i++) {
    int atual = novaRota.back();
    int proximo = rota[i + 1];
    double consumo = h * dist[atual][proximo];

    if (energia - consumo >= 0) {
      energia -= consumo;
      novaRota.push_back(proximo);
      if (isEstacao(instancia, proximo)) {
        energia = instancia.capacidadeEnergia;
      }
    } else {
      int melhorEstacao = -1;
      double menorAcrecimo = numeric_limits<double>::max();

      for (int s = 0; s < m; s++) {
        int idxEstacao = n + s;
        double consAteEst = h * dist[atual][idxEstacao];
        double consDeEst = h * dist[idxEstacao][proximo];

        if (energia - consAteEst >= 0 &&
            instancia.capacidadeEnergia - consDeEst >= 0) {
          double acrescimo = dist[atual][idxEstacao] +
                             dist[idxEstacao][proximo] - dist[atual][proximo];
          if (acrescimo < menorAcrecimo) {
            menorAcrecimo = acrescimo;
            melhorEstacao = idxEstacao;
          }
        }
      }

      if (melhorEstacao != -1) {
        novaRota.push_back(melhorEstacao);
        energia = instancia.capacidadeEnergia;
        energia -= h * dist[melhorEstacao][proximo];
        novaRota.push_back(proximo);
        if (isEstacao(instancia, proximo)) {
          energia = instancia.capacidadeEnergia;
        }
      } else {
        return false;
      }
    }
  }
  rota = novaRota;
  return true;
}

bool validarCapacidade(const InstanciaEVRP &instancia,
                       const vector<int> &rota) {
  int carga = 0;
  for (int no : rota) {
    if (no < instancia.dimensao && no > 0) {
      carga += getDemandaByNodeId(instancia, instancia.nos[no].id);
    }
    if (carga > instancia.capacidade) {
      return false;
    }
  }
  return true;
}

vector<vector<int>> gerarSolucaoInicial(const InstanciaEVRP &instancia,
                                        const vector<vector<double>> &dist) {
  vector<vector<int>> rotas;
  int numClientes = instancia.dimensao - 1;
  vector<bool> visitado(instancia.dimensao, false);
  visitado[0] = true;
  int clientesVisitados = 0;

  while (clientesVisitados < numClientes) {
    vector<int> rota;
    rota.push_back(0);
    int atual = 0;
    int cargaAtual = 0;

    bool adicionouAlgo = false;
    while (true) {
      int melhorCliente = -1;
      double menorDist = numeric_limits<double>::max();

      for (int i = 1; i <= numClientes; i++) {
        if (!visitado[i]) {
          int dem = getDemandaByNodeId(instancia, instancia.nos[i].id);
          if (cargaAtual + dem <= instancia.capacidade) {
            if (dist[atual][i] < menorDist) {
              menorDist = dist[atual][i];
              melhorCliente = i;
            }
          }
        }
      }

      if (melhorCliente != -1) {
        rota.push_back(melhorCliente);
        visitado[melhorCliente] = true;
        cargaAtual +=
            getDemandaByNodeId(instancia, instancia.nos[melhorCliente].id);
        atual = melhorCliente;
        clientesVisitados++;
        adicionouAlgo = true;
      } else {
        break;
      }
    }
    rota.push_back(0);

    if (!inserirEstacoes(instancia, rota, dist)) {
      rotas.clear();
      for (int i = 1; i <= numClientes; i++) {
        vector<int> r = {0, i, 0};
        if (!inserirEstacoes(instancia, r, dist)) {
          cerr << "Erro Fatal: Cliente " << i
               << " inalcancavel mesmo com bateria cheia." << endl;
        }
        rotas.push_back(r);
      }
      return rotas;
    }

    if (adicionouAlgo) {
      rotas.push_back(rota);
    } else if (clientesVisitados < numClientes) {
      for (int i = 1; i <= numClientes; i++) {
        if (!visitado[i]) {
          vector<int> r = {0, i, 0};
          inserirEstacoes(instancia, r, dist);
          rotas.push_back(r);
          visitado[i] = true;
          clientesVisitados++;
          break;
        }
      }
    }
  }

  limparRotasVazias(rotas);
  return rotas;
}
} // namespace

double resolverEVRPSA(const InstanciaEVRP &instancia, const string &nomeArquivo,
                      const SAParams &params) {
  if (params.seed != -1) {
    srand(params.seed);
  } else {
    srand(time(NULL));
  }
  if (params.verbose)
    imprimirInstanciaEVRP(instancia);

  string nomeBase = nomeArquivo;
  size_t posSlash = nomeBase.rfind('/');
  if (posSlash != string::npos)
    nomeBase = nomeBase.substr(posSlash + 1);
  size_t posExt = nomeBase.find(".evrp");
  if (posExt != string::npos)
    nomeBase = nomeBase.substr(0, posExt);

  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;
  int totalNos = n + m;

  vector<vector<double>> dist(totalNos, vector<double>(totalNos, 0.0));
  for (int i = 0; i < totalNos; i++) {
    for (int j = 0; j < totalNos; j++) {
      No noI = getNoByIndex(instancia, i);
      No noJ = getNoByIndex(instancia, j);
      dist[i][j] = calcularDistancia(noI, noJ);
    }
  }

  clock_t inicio = clock();

  vector<vector<int>> solucaoAtual = gerarSolucaoInicial(instancia, dist);
  double custoAtual = calcularCustoSolucao(solucaoAtual, dist);

  vector<vector<int>> melhorSolucao = solucaoAtual;
  double melhorCusto = custoAtual;

  double T = params.temp_inicial;
  double alpha = params.taxa_resfriamento;
  double T_min = params.temp_final;
  int iterMax = (n * params.iter_temp);

  while (T > T_min) {
    if (params.tempo_limite > 0) {
      double tempoDecorrido = (double)(clock() - inicio) / CLOCKS_PER_SEC;
      if (tempoDecorrido >= params.tempo_limite)
        break;
    }

    for (int i = 0; i < iterMax; i++) {
      vector<vector<int>> novaSolucao = solucaoAtual;

      if (novaSolucao.empty()) {
        novaSolucao = gerarSolucaoInicial(instancia, dist);
      }

      int op = rand() % 3;
      bool movimentoValido = false;

      if (op == 0 && novaSolucao.size() >= 1) {
        int r1 = rand() % novaSolucao.size();
        int r2 = rand() % novaSolucao.size();

        removerEstacoes(novaSolucao[r1], n);
        if (r1 != r2)
          removerEstacoes(novaSolucao[r2], n);

        if (novaSolucao[r1].size() > 2) {
          int idx1 = 1 + rand() % (novaSolucao[r1].size() - 2);
          int cliente = novaSolucao[r1][idx1];

          novaSolucao[r1].erase(novaSolucao[r1].begin() + idx1);

          int idx2 = 1;
          if (novaSolucao[r2].size() > 1) {
            idx2 = 1 + rand() % (novaSolucao[r2].size() - 1);
          }
          novaSolucao[r2].insert(novaSolucao[r2].begin() + idx2, cliente);

          if (validarCapacidade(instancia, novaSolucao[r2]) &&
              inserirEstacoes(instancia, novaSolucao[r1], dist) &&
              inserirEstacoes(instancia, novaSolucao[r2], dist)) {
            movimentoValido = true;
          }
        } else {
          inserirEstacoes(instancia, novaSolucao[r1], dist);
          if (r1 != r2)
            inserirEstacoes(instancia, novaSolucao[r2], dist);
        }
      } else if (op == 1 && !novaSolucao.empty()) {
        int r = rand() % novaSolucao.size();
        removerEstacoes(novaSolucao[r], n);

        if (novaSolucao[r].size() > 3) {
          int i1 = 1 + rand() % (novaSolucao[r].size() - 2);
          int i2 = 1 + rand() % (novaSolucao[r].size() - 2);
          while (i1 == i2)
            i2 = 1 + rand() % (novaSolucao[r].size() - 2);

          swap(novaSolucao[r][i1], novaSolucao[r][i2]);

          if (inserirEstacoes(instancia, novaSolucao[r], dist)) {
            movimentoValido = true;
          }
        } else {
          inserirEstacoes(instancia, novaSolucao[r], dist);
        }
      } else if (op == 2 && novaSolucao.size() >= 2) {
        int r1 = rand() % novaSolucao.size();
        int r2 = rand() % novaSolucao.size();
        while (r1 == r2)
          r2 = rand() % novaSolucao.size();

        removerEstacoes(novaSolucao[r1], n);
        removerEstacoes(novaSolucao[r2], n);

        if (novaSolucao[r1].size() > 2 && novaSolucao[r2].size() > 2) {
          int idx1 = 1 + rand() % (novaSolucao[r1].size() - 2);
          int idx2 = 1 + rand() % (novaSolucao[r2].size() - 2);

          swap(novaSolucao[r1][idx1], novaSolucao[r2][idx2]);

          if (validarCapacidade(instancia, novaSolucao[r1]) &&
              validarCapacidade(instancia, novaSolucao[r2]) &&
              inserirEstacoes(instancia, novaSolucao[r1], dist) &&
              inserirEstacoes(instancia, novaSolucao[r2], dist)) {
            movimentoValido = true;
          }
        } else {
          inserirEstacoes(instancia, novaSolucao[r1], dist);
          inserirEstacoes(instancia, novaSolucao[r2], dist);
        }
      }

      if (movimentoValido) {
        limparRotasVazias(novaSolucao);

        double novoCusto = calcularCustoSolucao(novaSolucao, dist);
        double delta = novoCusto - custoAtual;

        if (delta < 0 || (exp(-delta / T) > ((double)rand() / RAND_MAX))) {
          solucaoAtual = novaSolucao;
          custoAtual = novoCusto;

          if (custoAtual < melhorCusto) {
            melhorSolucao = solucaoAtual;
            melhorCusto = custoAtual;
          }
        }
      }
    }
    T *= alpha;
  }

  double tempoTotal = (double)(clock() - inicio) / CLOCKS_PER_SEC;

  string solucaoArquivo = "solucoes/" + nomeBase + "_SA.txt";
  ofstream solFile(solucaoArquivo);

  solFile << "Instancia: " << nomeBase << endl;
  solFile << fixed << setprecision(6);
  solFile << "\nUB (Upper Bound): " << melhorCusto << endl;
  solFile << "TEMPO (seg): " << tempoTotal << endl;

  solFile << "\nRotas:" << endl;
  int numRota = 1;
  for (size_t i = 0; i < melhorSolucao.size(); i++) {
    if (melhorSolucao[i].size() <= 2)
      continue;

    solFile << "Rota " << numRota++ << ": ";
    double distRota = 0;
    double cargaRota = 0;

    for (size_t k = 0; k < melhorSolucao[i].size(); k++) {
      solFile << melhorSolucao[i][k];
      if (k < melhorSolucao[i].size() - 1)
        solFile << " ";

      if (k > 0)
        distRota += dist[melhorSolucao[i][k - 1]][melhorSolucao[i][k]];
      if (melhorSolucao[i][k] < n && melhorSolucao[i][k] > 0)
        cargaRota += getDemandaByNodeId(instancia,
                                        instancia.nos[melhorSolucao[i][k]].id);
    }
    solFile << endl;
    solFile << "  Distancia: " << distRota << endl;
    solFile << "  Carga: " << cargaRota << endl;
    solFile << endl;
  }

  solFile << "Numero de rotas: " << (numRota - 1) << endl;
  solFile << "Distancia total: " << melhorCusto << endl;

  solFile.close();
  if (params.verbose)
    cout << "\nSolucao SA salva em: " << solucaoArquivo << endl;

  return melhorCusto;
}
