#include "grasp_solver.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace std;

static void construirMatrizDistancia(const InstanciaEVRP &instancia,
                                     vector<vector<double>> &dist) {
  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;
  int totalNos = n + m;

  dist.assign(totalNos, vector<double>(totalNos, 0.0));

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      dist[i][j] = calcularDistancia(instancia.nos[i], instancia.nos[j]);
    }
  }

  for (int i = 0; i < n; i++) {
    for (int s = 0; s < m; s++) {
      int idEstacaoFisica = s % instancia.estacoes;
      int idEstacao = instancia.idEstacoes[idEstacaoFisica] - 1;
      int indiceEstacao = n + s;
      double d = calcularDistancia(instancia.nos[i], instancia.nos[idEstacao]);
      dist[i][indiceEstacao] = d;
      dist[indiceEstacao][i] = d;
    }
  }

  for (int s1 = 0; s1 < m; s1++) {
    for (int s2 = 0; s2 < m; s2++) {
      int idEstacaoFisica1 = s1 % instancia.estacoes;
      int idEstacaoFisica2 = s2 % instancia.estacoes;
      int idEstacao1 = instancia.idEstacoes[idEstacaoFisica1] - 1;
      int idEstacao2 = instancia.idEstacoes[idEstacaoFisica2] - 1;
      int idx1 = n + s1;
      int idx2 = n + s2;
      dist[idx1][idx2] = calcularDistancia(instancia.nos[idEstacao1],
                                           instancia.nos[idEstacao2]);
    }
  }
}

static int encontrarMelhorEstacao(const InstanciaEVRP &instancia,
                                  const vector<vector<double>> &dist, int atual,
                                  int proximo, double energiaAtual,
                                  const vector<bool> &estacaoUsada) {
  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;
  double h = instancia.consumoEnergia;
  double Q = instancia.capacidadeEnergia;
  int melhor = -1;
  double melhorCusto = 1e18;

  for (int s = 0; s < m; s++) {
    int idxEstacao = n + s;
    if (estacaoUsada[s])
      continue;

    double distAtualEstacao = dist[atual][idxEstacao];
    double distEstacaoProximo = dist[idxEstacao][proximo];
    double consumoIda = h * distAtualEstacao;

    if (consumoIda > energiaAtual + 0.0001)
      continue;

    double energiaAposRecarga = Q;
    double consumoVolta = h * distEstacaoProximo;
    if (consumoVolta > energiaAposRecarga + 0.0001)
      continue;

    double custo = distAtualEstacao + distEstacaoProximo - dist[atual][proximo];
    if (custo < melhorCusto) {
      melhorCusto = custo;
      melhor = s;
    }
  }

  return melhor;
}

static bool inserirEstacoesRota(const InstanciaEVRP &instancia,
                                const vector<vector<double>> &dist,
                                vector<int> &rota, vector<bool> &estacaoUsada) {
  double h = instancia.consumoEnergia;
  double Q = instancia.capacidadeEnergia;
  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;

  bool mudou = true;
  while (mudou) {
    mudou = false;
    double energia = Q;

    for (size_t i = 0; i < rota.size() - 1; i++) {
      int de = rota[i];
      int para = rota[i + 1];
      double consumo = h * dist[de][para];

      bool precisaEstacao = false;

      // Check 1: Can't reach next node
      if (energia - consumo < -0.0001) {
        precisaEstacao = true;
      }

      // Check 2: Can reach next node but would be stuck there
      // (per paper Algorithm 4, line 12)
      if (!precisaEstacao && !isEstacao(instancia, para)) {
        double energiaApos = energia - consumo;
        double minConsParaEstacao = 1e18;
        bool temEstacaoDisponivel = false;
        for (int s = 0; s < m; s++) {
          if (!estacaoUsada[s]) {
            double c = h * dist[para][n + s];
            if (c < minConsParaEstacao) {
              minConsParaEstacao = c;
              temEstacaoDisponivel = true;
            }
          }
        }
        // Also consider depot (index 0) as recharging point
        double consumoParaDeposito = h * dist[para][0];
        if (consumoParaDeposito < minConsParaEstacao) {
          minConsParaEstacao = consumoParaDeposito;
          temEstacaoDisponivel = true;
        }
        if (temEstacaoDisponivel && energiaApos < minConsParaEstacao - 0.0001) {
          precisaEstacao = true;
        }
      }

      if (precisaEstacao) {
        int s = encontrarMelhorEstacao(instancia, dist, de, para, energia,
                                       estacaoUsada);
        if (s == -1)
          return false;

        int idxEstacao = n + s;
        estacaoUsada[s] = true;
        rota.insert(rota.begin() + i + 1, idxEstacao);
        mudou = true;
        break;
      }

      energia -= consumo;

      if (isEstacao(instancia, para)) {
        energia = Q;
      }
    }
  }

  return true;
}

static double calcularCustoRota(const vector<int> &rota,
                                const vector<vector<double>> &dist) {
  double custo = 0.0;
  for (size_t i = 0; i < rota.size() - 1; i++) {
    custo += dist[rota[i]][rota[i + 1]];
  }
  return custo;
}

static double calcularCustoTotal(const vector<vector<int>> &rotas,
                                 const vector<vector<double>> &dist) {
  double total = 0.0;
  for (const auto &rota : rotas) {
    total += calcularCustoRota(rota, dist);
  }
  return total;
}

static vector<int> removerEstacoes(const InstanciaEVRP &instancia,
                                   const vector<int> &rota) {
  int n = instancia.dimensao;
  vector<int> limpa;
  for (int no : rota) {
    if (no == 0 || (no >= 1 && no < n && !isEstacao(instancia, no))) {
      limpa.push_back(no);
    }
  }
  return limpa;
}

struct Solucao {
  vector<vector<int>> rotas;
  double custo;
};

static Solucao construirSolucao(const InstanciaEVRP &instancia,
                                const vector<vector<double>> &dist,
                                double alpha, mt19937 &rng) {
  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;
  int numClientes = n - 1;
  double C = instancia.capacidade;

  vector<bool> visitado(numClientes + 1, false);
  vector<vector<int>> rotas;
  int clientesRestantes = numClientes;

  while (clientesRestantes > 0) {
    vector<int> rota = {0};
    double cargaAtual = 0;
    int atual = 0;

    while (clientesRestantes > 0) {
      // Candidatos viáveis por capacidade
      vector<pair<double, int>> candidatos;
      for (int c = 1; c <= numClientes; c++) {
        if (visitado[c])
          continue;
        No noC = getNoByIndex(instancia, c);
        double demanda = getDemandaByNodeId(instancia, noC.id);
        if (cargaAtual + demanda > C + 0.0001)
          continue;
        double custo = dist[atual][c];
        candidatos.push_back({custo, c});
      }

      if (candidatos.empty())
        break;

      // RCL
      double cMin = candidatos[0].first;
      double cMax = candidatos[0].first;
      for (const auto &p : candidatos) {
        cMin = min(cMin, p.first);
        cMax = max(cMax, p.first);
      }

      double limite = cMin + alpha * (cMax - cMin);
      vector<int> rcl;
      for (const auto &p : candidatos) {
        if (p.first <= limite + 0.0001) {
          rcl.push_back(p.second);
        }
      }

      uniform_int_distribution<int> escolha(0, rcl.size() - 1);
      int escolhido = rcl[escolha(rng)];

      rota.push_back(escolhido);
      visitado[escolhido] = true;
      clientesRestantes--;

      No noEsc = getNoByIndex(instancia, escolhido);
      cargaAtual += getDemandaByNodeId(instancia, noEsc.id);
      atual = escolhido;
    }

    rota.push_back(0);

    // Inserir estações de recarga
    vector<bool> estacaoUsada(m, false);
    // Marcar estações já usadas em rotas anteriores
    for (const auto &r : rotas) {
      for (int no : r) {
        if (no >= n) {
          estacaoUsada[no - n] = true;
        }
      }
    }

    if (!inserirEstacoesRota(instancia, dist, rota, estacaoUsada)) {
      // Fallback: rota não viável de energia, ainda assim a mantemos
      // A busca local pode corrigi-la
    }

    rotas.push_back(rota);
  }

  Solucao sol;
  sol.rotas = rotas;
  sol.custo = calcularCustoTotal(rotas, dist);
  return sol;
}

static bool buscaLocalRelocate(
    const InstanciaEVRP &instancia, const vector<vector<double>> &dist,
    Solucao &sol,
    chrono::high_resolution_clock::time_point deadline = {}) {
  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;
  double C = instancia.capacidade;

  for (size_t r1 = 0; r1 < sol.rotas.size(); r1++) {
    vector<int> limpa1 = removerEstacoes(instancia, sol.rotas[r1]);
    for (size_t i = 1; i < limpa1.size() - 1; i++) {
      if (deadline.time_since_epoch().count() > 0 &&
          chrono::high_resolution_clock::now() >= deadline)
        return false;

      int cliente = limpa1[i];
      No noCliente = getNoByIndex(instancia, cliente);
      double demCliente = getDemandaByNodeId(instancia, noCliente.id);

      for (size_t r2 = 0; r2 < sol.rotas.size(); r2++) {
        if (r1 == r2)
          continue;

        vector<int> limpa2 = removerEstacoes(instancia, sol.rotas[r2]);

        // Verificar capacidade da rota destino
        double carga2 = 0;
        for (int no : limpa2) {
          if (no >= 1 && no < n) {
            No noN = getNoByIndex(instancia, no);
            carga2 += getDemandaByNodeId(instancia, noN.id);
          }
        }
        if (carga2 + demCliente > C + 0.0001)
          continue;

        for (size_t j = 1; j < limpa2.size(); j++) {
          // Tentar inserir cliente na posição j da rota2
          vector<int> novaR1;
          for (int no : limpa1) {
            if (no != cliente)
              novaR1.push_back(no);
          }
          if (novaR1.size() <= 2)
            continue; // rota ficaria vazia

          vector<int> novaR2 = limpa2;
          novaR2.insert(novaR2.begin() + j, cliente);

          // Inserir estações
          vector<bool> estacaoUsada(m, false);
          for (size_t rx = 0; rx < sol.rotas.size(); rx++) {
            if (rx == r1 || rx == r2)
              continue;
            for (int no : sol.rotas[rx]) {
              if (no >= n)
                estacaoUsada[no - n] = true;
            }
          }

          vector<bool> eu1 = estacaoUsada;
          if (!inserirEstacoesRota(instancia, dist, novaR1, eu1))
            continue;

          vector<bool> eu2 = eu1;
          if (!inserirEstacoesRota(instancia, dist, novaR2, eu2))
            continue;

          double custoAntigo = calcularCustoRota(sol.rotas[r1], dist) +
                               calcularCustoRota(sol.rotas[r2], dist);
          double custoNovo =
              calcularCustoRota(novaR1, dist) + calcularCustoRota(novaR2, dist);

          if (custoNovo < custoAntigo - 0.0001) {
            sol.rotas[r1] = novaR1;
            sol.rotas[r2] = novaR2;
            sol.custo = calcularCustoTotal(sol.rotas, dist);
            return true;
          }
        }
      }
    }
  }
  return false;
}

static bool buscaLocal2Opt(
    const InstanciaEVRP &instancia, const vector<vector<double>> &dist,
    Solucao &sol,
    chrono::high_resolution_clock::time_point deadline = {}) {
  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;

  for (size_t r = 0; r < sol.rotas.size(); r++) {
    vector<int> limpa = removerEstacoes(instancia, sol.rotas[r]);
    if (limpa.size() < 4)
      continue;

    for (size_t i = 1; i < limpa.size() - 2; i++) {
      if (deadline.time_since_epoch().count() > 0 &&
          chrono::high_resolution_clock::now() >= deadline)
        return false;
      for (size_t j = i + 1; j < limpa.size() - 1; j++) {
        vector<int> nova = limpa;
        reverse(nova.begin() + i, nova.begin() + j + 1);

        vector<bool> estacaoUsada(m, false);
        for (size_t rx = 0; rx < sol.rotas.size(); rx++) {
          if (rx == r)
            continue;
          for (int no : sol.rotas[rx]) {
            if (no >= n)
              estacaoUsada[no - n] = true;
          }
        }

        if (!inserirEstacoesRota(instancia, dist, nova, estacaoUsada))
          continue;

        double custoAntigo = calcularCustoRota(sol.rotas[r], dist);
        double custoNovo = calcularCustoRota(nova, dist);

        if (custoNovo < custoAntigo - 0.0001) {
          sol.rotas[r] = nova;
          sol.custo = calcularCustoTotal(sol.rotas, dist);
          return true;
        }
      }
    }
  }
  return false;
}

static bool buscaLocalExchange(
    const InstanciaEVRP &instancia, const vector<vector<double>> &dist,
    Solucao &sol,
    chrono::high_resolution_clock::time_point deadline = {}) {
  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;
  double C = instancia.capacidade;

  for (size_t r1 = 0; r1 < sol.rotas.size(); r1++) {
    vector<int> limpa1 = removerEstacoes(instancia, sol.rotas[r1]);
    for (size_t i = 1; i < limpa1.size() - 1; i++) {
      if (deadline.time_since_epoch().count() > 0 &&
          chrono::high_resolution_clock::now() >= deadline)
        return false;
      for (size_t r2 = r1 + 1; r2 < sol.rotas.size(); r2++) {
        vector<int> limpa2 = removerEstacoes(instancia, sol.rotas[r2]);
        for (size_t j = 1; j < limpa2.size() - 1; j++) {
          int c1 = limpa1[i];
          int c2 = limpa2[j];

          No no1 = getNoByIndex(instancia, c1);
          No no2 = getNoByIndex(instancia, c2);
          double dem1 = getDemandaByNodeId(instancia, no1.id);
          double dem2 = getDemandaByNodeId(instancia, no2.id);

          // Verificar capacidades após troca
          double carga1 = 0, carga2 = 0;
          for (int no : limpa1) {
            if (no >= 1 && no < n) {
              No noN = getNoByIndex(instancia, no);
              carga1 += getDemandaByNodeId(instancia, noN.id);
            }
          }
          for (int no : limpa2) {
            if (no >= 1 && no < n) {
              No noN = getNoByIndex(instancia, no);
              carga2 += getDemandaByNodeId(instancia, noN.id);
            }
          }

          if (carga1 - dem1 + dem2 > C + 0.0001)
            continue;
          if (carga2 - dem2 + dem1 > C + 0.0001)
            continue;

          vector<int> novaR1 = limpa1;
          vector<int> novaR2 = limpa2;
          novaR1[i] = c2;
          novaR2[j] = c1;

          vector<bool> estacaoUsada(m, false);
          for (size_t rx = 0; rx < sol.rotas.size(); rx++) {
            if (rx == r1 || rx == r2)
              continue;
            for (int no : sol.rotas[rx]) {
              if (no >= n)
                estacaoUsada[no - n] = true;
            }
          }

          vector<bool> eu1 = estacaoUsada;
          if (!inserirEstacoesRota(instancia, dist, novaR1, eu1))
            continue;

          vector<bool> eu2 = eu1;
          if (!inserirEstacoesRota(instancia, dist, novaR2, eu2))
            continue;

          double custoAntigo = calcularCustoRota(sol.rotas[r1], dist) +
                               calcularCustoRota(sol.rotas[r2], dist);
          double custoNovo =
              calcularCustoRota(novaR1, dist) + calcularCustoRota(novaR2, dist);

          if (custoNovo < custoAntigo - 0.0001) {
            sol.rotas[r1] = novaR1;
            sol.rotas[r2] = novaR2;
            sol.custo = calcularCustoTotal(sol.rotas, dist);
            return true;
          }
        }
      }
    }
  }
  return false;
}

static void buscaLocal(
    const InstanciaEVRP &instancia, const vector<vector<double>> &dist,
    Solucao &sol,
    chrono::high_resolution_clock::time_point deadline = {}) {
  bool melhorou = true;
  while (melhorou) {
    if (deadline.time_since_epoch().count() > 0 &&
        chrono::high_resolution_clock::now() >= deadline)
      break;
    melhorou = false;
    if (buscaLocal2Opt(instancia, dist, sol, deadline)) {
      melhorou = true;
      continue;
    }
    if (buscaLocalRelocate(instancia, dist, sol, deadline)) {
      melhorou = true;
      continue;
    }
    if (buscaLocalExchange(instancia, dist, sol, deadline)) {
      melhorou = true;
      continue;
    }
  }
}

double resolverEVRPGRASP(const InstanciaEVRP &instancia,
                         const string &nomeArquivo, const GRASPParams &params) {
  if (params.verbose) {
    imprimirInstanciaEVRP(instancia);
  }

  string nomeBase = nomeArquivo;
  size_t posSlash = nomeBase.rfind('/');
  if (posSlash != string::npos) {
    nomeBase = nomeBase.substr(posSlash + 1);
  }
  size_t posExt = nomeBase.find(".evrp");
  if (posExt != string::npos) {
    nomeBase = nomeBase.substr(0, posExt);
  }

  vector<vector<double>> dist;
  construirMatrizDistancia(instancia, dist);

  unsigned int semente =
      (params.seed >= 0)
          ? static_cast<unsigned int>(params.seed)
          : static_cast<unsigned int>(
                chrono::system_clock::now().time_since_epoch().count());
  mt19937 rng(semente);

  auto inicio = chrono::high_resolution_clock::now();
  auto deadline = chrono::high_resolution_clock::time_point{};
  if (params.tempo_limite > 0) {
    deadline = inicio + chrono::duration_cast<chrono::high_resolution_clock::duration>(
                            chrono::duration<double>(params.tempo_limite));
  }

  Solucao melhorSolucao;
  melhorSolucao.custo = 1e18;
  double tempoMelhor = 0.0;

  for (int iter = 0; iter < params.max_iter; iter++) {
    if (deadline.time_since_epoch().count() > 0 &&
        chrono::high_resolution_clock::now() >= deadline)
      break;

    Solucao sol = construirSolucao(instancia, dist, params.alpha, rng);

    // Aceitar solução construída antes da busca local se for válida
    if (sol.custo < melhorSolucao.custo &&
        validarSolucao(instancia, sol.rotas, dist, false)) {
      melhorSolucao = sol;
      auto agora = chrono::high_resolution_clock::now();
      tempoMelhor = chrono::duration<double>(agora - inicio).count();
      if (params.verbose) {
        cout << "Iteracao " << (iter + 1) << " (construcao): custo = " << fixed
             << setprecision(6) << melhorSolucao.custo << endl;
      }
    }

    buscaLocal(instancia, dist, sol, deadline);

    if (sol.custo < melhorSolucao.custo &&
        validarSolucao(instancia, sol.rotas, dist, false)) {
      melhorSolucao = sol;
      auto agora = chrono::high_resolution_clock::now();
      tempoMelhor = chrono::duration<double>(agora - inicio).count();
      if (params.verbose) {
        cout << "Iteracao " << (iter + 1) << ": melhor custo = " << fixed
             << setprecision(6) << melhorSolucao.custo << endl;
      }
    }
  }

  auto fim = chrono::high_resolution_clock::now();
  double tempoTotal = chrono::duration<double>(fim - inicio).count();

  // Salvar solução
  string solucaoArquivo = "solucoes/" + nomeBase + "_GRASP";
  if (params.seed >= 0) {
    solucaoArquivo += "_seed" + to_string(params.seed);
  }
  if (params.run_number >= 0) {
    solucaoArquivo += "_run" + to_string(params.run_number + 1);
  }
  solucaoArquivo += ".txt";

  if (!params.verbose) {
    cout << fixed << setprecision(6) << melhorSolucao.custo << " " << tempoMelhor
         << endl;
  }
  ofstream solFile(solucaoArquivo);
  solFile << "Instancia: " << nomeBase << endl;
  solFile << fixed << setprecision(6);
  solFile << "\nFO (Funcao Objetivo): " << melhorSolucao.custo << endl;
  solFile << "TEMPO (seg): " << tempoTotal << endl;
  solFile << "TEMPO_MELHOR (seg): " << tempoMelhor << endl;

  solFile << "\nRotas:" << endl;
  double distTotal = 0.0;
  for (size_t r = 0; r < melhorSolucao.rotas.size(); r++) {
    const auto &rota = melhorSolucao.rotas[r];
    double distRota = calcularCustoRota(rota, dist);
    distTotal += distRota;

    double carga = 0;
    int n = instancia.dimensao;
    for (int no : rota) {
      if (no >= 1 && no < n && !isEstacao(instancia, no)) {
        No noN = getNoByIndex(instancia, no);
        carga += getDemandaByNodeId(instancia, noN.id);
      }
    }

    solFile << "Rota " << (r + 1) << ": ";
    for (size_t i = 0; i < rota.size(); i++) {
      solFile << rota[i];
      if (i < rota.size() - 1)
        solFile << " ";
    }
    solFile << endl;
    solFile << "  Distancia: " << distRota << endl;
    solFile << "  Carga: " << carga << endl;
    solFile << endl;
  }

  solFile << "Numero de rotas: " << melhorSolucao.rotas.size() << endl;
  solFile << "Distancia total: " << distTotal << endl;
  solFile.close();

  if (params.verbose) {
    cout << "\nSolucao salva em: " << solucaoArquivo << endl;
    cout << "Custo: " << melhorSolucao.custo << endl;
    cout << "Tempo: " << tempoTotal << " seg" << endl;
    cout << "Tempo melhor: " << tempoMelhor << " seg" << endl;

    validarSolucao(instancia, melhorSolucao.rotas, dist, params.verbose);
  }

  return melhorSolucao.custo;
}
