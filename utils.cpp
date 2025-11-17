#include "utils.hpp"
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace std;

void imprimirNo(const No &n) {
  cout << "No(ID: " << n.id << ", X: " << n.x << ", Y: " << n.y << ")";
}

void imprimirDemandaNo(const DemandaNo &d) {
  cout << "Demanda(ID: " << d.id << ", Q: " << d.demanda << ")";
}

void imprimirInstanciaEVRP(const InstanciaEVRP &instancia) {
  cout << "\n--- Resumo da Instancia ---" << endl;
  cout << "Nome: " << instancia.nome << endl;
  cout << "Tipo: " << instancia.tipo << endl;
  cout << "Veiculos: " << instancia.veiculos << endl;
  cout << "Dimensao: " << instancia.dimensao << endl;
  cout << "Estacoes: " << instancia.estacoesTotal << endl;
  cout << "Capacidade: " << instancia.capacidade << endl;
  cout << "Deposito: " << instancia.idDeposito << endl;

  cout << "\nTotal de Nos lidos: " << instancia.nos.size() << endl;
  if (!instancia.nos.empty()) {
    cout << "  -> Primeiro no: ";
    imprimirNo(instancia.nos[0]);
    cout << endl;
  }

  cout << "\nTotal de Demandas lidas: " << instancia.demandas.size() << endl;
  if (instancia.demandas.size() > 1) {
    cout << "  -> Segunda demanda: ";
    imprimirDemandaNo(instancia.demandas[1]);
    cout << endl;
  }

  cout << "\nTotal de Estacoes (IDs): " << instancia.idEstacoes.size() << endl;
  if (!instancia.idEstacoes.empty()) {
    cout << "  -> ID da primeira estacao: " << instancia.idEstacoes[0] << endl;
  }
  cout << "--------------------------------" << endl;
}

bool carregarInstancia(const string &nomeArquivo, InstanciaEVRP &instancia) {
  string caminhoCompleto = "dataset/" + nomeArquivo;
  ifstream arquivo(caminhoCompleto);

  if (!arquivo.is_open()) {
    cerr << "Erro: Nao foi possivel abrir o arquivo " << caminhoCompleto
         << endl;
    return false;
  }

  string linha;
  string secaoAtual = "";

  while (getline(arquivo, linha)) {

    if (linha.find_first_not_of(" \t") == string::npos) {
      continue;
    }

    if (linha.find("NODE_COORD_SECTION") != string::npos) {
      secaoAtual = "NODE_COORD_SECTION";
      continue;
    } else if (linha.find("DEMAND_SECTION") != string::npos) {
      secaoAtual = "DEMAND_SECTION";
      continue;
    } else if (linha.find("STATIONS_COORD_SECTION") != string::npos) {
      secaoAtual = "STATIONS_COORD_SECTION";
      continue;
    } else if (linha.find("DEPOT_SECTION") != string::npos) {
      secaoAtual = "DEPOT_SECTION";
      continue;
    } else if (linha.find("EOF") != string::npos) {
      break;
    }

    istringstream issLinha(linha);

    if (secaoAtual == "") {
      size_t divisorPos = linha.find(":");
      if (divisorPos == string::npos)
        continue;

      string chave = linha.substr(0, divisorPos);
      size_t ultimoCaractereChave = chave.find_last_not_of(" \t");
      if (ultimoCaractereChave != string::npos) {
        chave = chave.substr(0, ultimoCaractereChave + 1);
      }

      string valorStr = linha.substr(divisorPos + 1);
      size_t primeiroCaractereValor = valorStr.find_first_not_of(" \t");
      if (primeiroCaractereValor == string::npos)
        continue;
      valorStr = valorStr.substr(primeiroCaractereValor);

      stringstream ssValor(valorStr);

      if (chave == "Name") {
        instancia.nome = valorStr;
      } else if (chave == "COMMENT") {
        instancia.comentario = valorStr;
      } else if (chave == "TYPE") {
        instancia.tipo = valorStr;
      } else if (chave == "OPTIMAL_VALUE") {
        ssValor >> instancia.valorOtimo;
      } else if (chave == "VEHICLES") {
        ssValor >> instancia.veiculos;
      } else if (chave == "DIMENSION") {
        ssValor >> instancia.dimensao;
      } else if (chave == "STATIONS") {
        ssValor >> instancia.estacoesTotal;
      } else if (chave == "CAPACITY") {
        ssValor >> instancia.capacidade;
      } else if (chave == "ENERGY_CAPACITY") {
        ssValor >> instancia.capacidadeEnergia;
      } else if (chave == "ENERGY_CONSUMPTION") {
        ssValor >> instancia.consumoEnergia;
      } else if (chave == "EDGE_WEIGHT_FORMAT") {
        instancia.formatoBorda = valorStr;
      }
    } else if (secaoAtual == "NODE_COORD_SECTION") {
      int id;
      double x, y;
      if (issLinha >> id >> x >> y) {
        instancia.nos.push_back({id, x, y});
      }
    } else if (secaoAtual == "DEMAND_SECTION") {
      int id, demanda;
      if (issLinha >> id >> demanda) {
        instancia.demandas.push_back({id, demanda});
      }
    } else if (secaoAtual == "STATIONS_COORD_SECTION") {
      int id;
      if (issLinha >> id) {
        instancia.idEstacoes.push_back(id);
      }
    } else if (secaoAtual == "DEPOT_SECTION") {
      int id;
      if (issLinha >> id) {
        if (id != -1) {
          instancia.idDeposito = id;
        }
      }
    }
  }

  arquivo.close();
  return true;
}

double calcularDistancia(const No &a, const No &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return sqrt(dx * dx + dy * dy);
}

void resolverEVRP(const InstanciaEVRP &instancia, const string &nomeArquivo) {
  imprimirInstanciaEVRP(instancia);

  try {
    IloEnv env;
    IloModel model(env);

    int n = instancia.dimensao;
    int m = instancia.estacoesTotal;
    int numClientes = n - 1;
    int totalNos = n + m;

    vector<vector<double>> dist(totalNos, vector<double>(totalNos, 0.0));

    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        dist[i][j] = calcularDistancia(instancia.nos[i], instancia.nos[j]);
      }
    }

    for (int i = 0; i < n; i++) {
      for (int s = 0; s < m; s++) {
        int idEstacao = instancia.idEstacoes[s] - 1;
        int indiceEstacao = n + s;
        double d = calcularDistancia(instancia.nos[i], instancia.nos[idEstacao]);
        dist[i][indiceEstacao] = d;
        dist[indiceEstacao][i] = d;
      }
    }

    for (int s1 = 0; s1 < m; s1++) {
      for (int s2 = 0; s2 < m; s2++) {
        int idEstacao1 = instancia.idEstacoes[s1] - 1;
        int idEstacao2 = instancia.idEstacoes[s2] - 1;
        int idx1 = n + s1;
        int idx2 = n + s2;
        double d = calcularDistancia(instancia.nos[idEstacao1], instancia.nos[idEstacao2]);
        dist[idx1][idx2] = d;
      }
    }

    IloArray<IloNumVarArray> x(env, totalNos);
    for (int i = 0; i < totalNos; i++) {
      x[i] = IloNumVarArray(env, totalNos, 0, 1, ILOBOOL);
    }

    IloNumVarArray y(env, totalNos, 0, instancia.capacidadeEnergia);
    IloNumVarArray u(env, totalNos, 0, instancia.capacidade);

    double h = instancia.consumoEnergia;
    double Q = instancia.capacidadeEnergia;
    double C = instancia.capacidade;

    // Função objetivo (1)
    IloExpr objExpr(env);
    for (int i = 0; i < totalNos; i++) {
      for (int j = 0; j < totalNos; j++) {
        if (i != j) {
          objExpr += dist[i][j] * x[i][j];
        }
      }
    }
    model.add(IloMinimize(env, objExpr));
    objExpr.end();

    // Restrição (2): Cada cliente tem exatamente 1 sucessor
    for (int i = 1; i <= numClientes; i++) {
      IloExpr expr(env);
      for (int j = 0; j < totalNos; j++) {
        if (i != j) {
          expr += x[i][j];
        }
      }
      model.add(expr == 1);
      expr.end();
    }

    // Restrição (3): Cada estação tem no máximo 1 sucessor
    for (int s = 0; s < m; s++) {
      int idx = n + s;
      IloExpr expr(env);
      for (int j = 0; j < totalNos; j++) {
        if (idx != j) {
          expr += x[idx][j];
        }
      }
      model.add(expr <= 1);
      expr.end();
    }

    // Restrição (4): Conservação de fluxo
    for (int j = 1; j < totalNos; j++) {
      IloExpr exprIn(env);
      IloExpr exprOut(env);
      for (int i = 0; i < totalNos; i++) {
        if (i != j) {
          exprIn += x[i][j];
          exprOut += x[j][i];
        }
      }
      model.add(exprIn - exprOut == 0);
      exprIn.end();
      exprOut.end();
    }

    double BigM_battery = 2.0 * Q;
    double BigM_capacity = 2.0 * C;

    // Restrição (5): Bateria diminui entre clientes
    for (int i = 1; i <= numClientes; i++) {
      for (int j = 1; j < totalNos; j++) {
        if (i != j) {
          model.add(y[j] <= y[i] - h * dist[i][j] + BigM_battery * (1 - x[i][j]));
          model.add(y[j] >= y[i] - h * dist[i][j] - BigM_battery * (1 - x[i][j]));
        }
      }
    }

    // Restrição (6): Bateria cheia ao sair de depósito/estações
    for (int j = 1; j < totalNos; j++) {
      model.add(y[j] <= Q - h * dist[0][j] + BigM_battery * (1 - x[0][j]));
      model.add(y[j] >= Q - h * dist[0][j] - BigM_battery * (1 - x[0][j]));
    }
    
    for (int s = 0; s < m; s++) {
      int i = n + s;
      for (int j = 1; j < totalNos; j++) {
        if (i != j) {
          model.add(y[j] <= Q - h * dist[i][j] + BigM_battery * (1 - x[i][j]));
          model.add(y[j] >= Q - h * dist[i][j] - BigM_battery * (1 - x[i][j]));
        }
      }
    }

    // Restrição (7): Capacidade diminui com entregas
    for (int i = 0; i < totalNos; i++) {
      for (int j = 1; j <= numClientes; j++) {
        if (i != j) {
          double demandaJ = instancia.demandas[j].demanda;
          model.add(u[j] <= u[i] - demandaJ + BigM_capacity * (1 - x[i][j]));
          model.add(u[j] >= u[i] - demandaJ - BigM_capacity * (1 - x[i][j]));
        }
      }
    }

    for (int i = 0; i < totalNos; i++) {
      for (int s = 0; s < m; s++) {
        int j = n + s;
        if (i != j) {
          model.add(u[j] <= u[i] + BigM_capacity * (1 - x[i][j]));
          model.add(u[j] >= u[i] - BigM_capacity * (1 - x[i][j]));
        }
      }
    }

    // Restrição (8)
    model.add(u[0] >= 0);
    model.add(u[0] <= C);
    model.add(y[0] == Q);

    // Limitar número de veículos
    IloExpr exprVeiculos(env);
    for (int j = 1; j < totalNos; j++) {
      exprVeiculos += x[0][j];
    }
    model.add(exprVeiculos <= instancia.veiculos);
    exprVeiculos.end();

    // Conservação no depósito
    IloExpr exprDepotOut(env);
    IloExpr exprDepotIn(env);
    for (int j = 1; j < totalNos; j++) {
      exprDepotOut += x[0][j];
      exprDepotIn += x[j][0];
    }
    model.add(exprDepotOut == exprDepotIn);
    exprDepotOut.end();
    exprDepotIn.end();

    // Evitar self-loops
    for (int i = 0; i < totalNos; i++) {
      model.add(x[i][i] == 0);
    }

    for (int j = 0; j < totalNos; j++) {
      model.add(y[j] >= 0);
      model.add(u[j] <= C);
    }

    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::TimeLimit, 3600);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0);
    
    double tempoInicio = cplex.getCplexTime();
    bool solved = cplex.solve();
    double tempoTotal = cplex.getCplexTime() - tempoInicio;

    string nomeBase = nomeArquivo;
    size_t posExt = nomeBase.find(".evrp");
    if (posExt != string::npos) {
      nomeBase = nomeBase.substr(0, posExt);
    }

    string solucaoArquivo = "solucoes/" + nomeBase + "_CPLEX.txt";
    ofstream solFile(solucaoArquivo);
    
    solFile << "Instancia: " << nomeBase << endl;
    solFile << fixed << setprecision(6);

    double lb = 0, ub = 0, gap = 0;

    if (solved) {
      IloAlgorithm::Status status = cplex.getStatus();
      ub = cplex.getObjValue();
      
      try {
        lb = cplex.getBestObjValue();
      } catch (...) {
        lb = ub;
      }

      if (ub > 0.0001) {
        gap = ((ub - lb) / ub) * 100.0;
      }

      solFile << "\nLB (Lower Bound): " << lb << endl;
      solFile << "UB (Upper Bound): " << ub << endl;
      solFile << "GAP (%): " << gap << endl;
      solFile << "TEMPO (seg): " << tempoTotal << endl;
      solFile << "Status: " << status << endl;

      if (instancia.valorOtimo > 0) {
        double gapOtimo = ((ub - instancia.valorOtimo) / instancia.valorOtimo) * 100.0;
        solFile << "Valor Otimo Conhecido: " << instancia.valorOtimo << endl;
        solFile << "GAP vs Otimo (%): " << gapOtimo << endl;
        cout << "  GAP vs Otimo: " << gapOtimo << "%" << endl;
      }

      int numRota = 1;

      for (int start = 1; start < totalNos; start++) {
        if (cplex.getValue(x[0][start]) > 0.5) {
          vector<int> rota;
          rota.push_back(0);

          int atual = start;
          double distRota = dist[0][start];
          double cargaRota = 0;
          
          int maxIter = totalNos * 2;
          int iter = 0;

          while (atual != 0 && iter < maxIter) {
            rota.push_back(atual);

            if (atual >= 1 && atual <= numClientes) {
              cargaRota += instancia.demandas[atual].demanda;
            }

            int proximo = -1;
            for (int j = 0; j < totalNos; j++) {
              if (cplex.getValue(x[atual][j]) > 0.5) {
                proximo = j;
                distRota += dist[atual][j];
                break;
              }
            }

            if (proximo == -1) {
              distRota += dist[atual][0];
              rota.push_back(0);
              break;
            }
            
            if (proximo == 0) {
              rota.push_back(0);
              break;
            }
            
            atual = proximo;
            iter++;
          }

          for (size_t i = 0; i < rota.size(); i++) {
            int no = rota[i];
            
            if (no == 0) {
              solFile << "0";
            } else if (no <= numClientes) {
              solFile << no;
            } else {
              solFile << "E" << (no - n + 1);
            }
            
            if (i < rota.size() - 1) {
              solFile << " ";
            }
          }
          solFile << endl;
          solFile << "  Distancia: " << distRota << endl;
          solFile << "  Carga: " << cargaRota << endl;
          solFile << endl;

          numRota++;
        }
      }

      solFile << "Numero de rotas: " << (numRota - 1) << endl;
      solFile << "Distancia total: " << ub << endl;

    } else {
      solFile << "\nStatus: Sem solucao no tempo limite" << endl;
      solFile << "TEMPO (seg): " << tempoTotal << endl;
      
      try {
        lb = cplex.getBestObjValue();
        solFile << "LB (Lower Bound): " << lb << endl;
      } catch (...) {
        solFile << "LB: N/A" << endl;
      }

      cout << "\nCPLEX nao encontrou solucao no tempo limite." << endl;
      cout << "  TEMPO: " << tempoTotal << " seg" << endl;
    }

    solFile.close();
    
    cout << "\nSolucao salva em: " << solucaoArquivo << endl;

    env.end();

  } catch (IloException &e) {
    cerr << "Erro no CPLEX: " << e << endl;
  } catch (exception &e) {
    cerr << "Erro: " << e.what() << endl;
  }
}
