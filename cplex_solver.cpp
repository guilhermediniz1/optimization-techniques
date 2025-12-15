#include "cplex_solver.hpp"
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

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
        double d =
            calcularDistancia(instancia.nos[i], instancia.nos[idEstacao]);
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
        double d = calcularDistancia(instancia.nos[idEstacao1],
                                     instancia.nos[idEstacao2]);
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

    // debugando matiz x 
    for (int i = 0; i < totalNos; i++) {
      for (int j = 0; j < totalNos; j++) {
        cout << i << "->" << j << " = " << x[i][j];
      }
    }

    // FO (1)
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

    // Constraint (2): Cada cliente tem somente 1 sucessor
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

    // Constraint (3): Cada estacao tem max 1 sucessor
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

    // Constraint (4): Garantir o fluxo (entrada/saida)
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

    // Constraint (5): Desgaste da bateria entre clientes
    for (int i = 1; i <= numClientes; i++) {
      for (int j = 1; j < totalNos; j++) {
        if (i != j) {
          model.add(y[j] <=
                    y[i] - h * dist[i][j] + BigM_battery * (1 - x[i][j]));
          model.add(y[j] >=
                    y[i] - h * dist[i][j] - BigM_battery * (1 - x[i][j]));
        }
      }
    }

    // Constraint (6): Recarga da bateria ao deixar um posto
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

    // Constraint (7): Reducao da capacidade
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

    // Constraint (8)
    model.add(u[0] >= 0);
    model.add(u[0] <= C);
    model.add(y[0] == Q);

    // Limita o número de veículos
    IloExpr exprVeiculos(env);
    for (int j = 1; j < totalNos; j++) {
      exprVeiculos += x[0][j];
    }
    model.add(exprVeiculos <= instancia.veiculos);
    exprVeiculos.end();

    // Garantia de fluxo do depósito
    IloExpr exprDepotOut(env);
    IloExpr exprDepotIn(env);
    for (int j = 1; j < totalNos; j++) {
      exprDepotOut += x[0][j];
      exprDepotIn += x[j][0];
    }
    model.add(exprDepotOut == exprDepotIn);
    exprDepotOut.end();
    exprDepotIn.end();

    // Previne Self-loops
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
        double gapOtimo =
            ((ub - instancia.valorOtimo) / instancia.valorOtimo) * 100.0;
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
