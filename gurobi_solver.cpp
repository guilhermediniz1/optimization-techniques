#include "gurobi_solver.hpp"
#include "gurobi_c++.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

void resolverEVRPGurobi(const InstanciaEVRP &instancia, const string &nomeArquivo) {
  imprimirInstanciaEVRP(instancia);

  try {
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", "gurobi.log");
    env.start();

    GRBModel model = GRBModel(env);

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

    // Variáveis de decisao x[i][j]
    vector<vector<GRBVar>> x(totalNos, vector<GRBVar>(totalNos));
    for (int i = 0; i < totalNos; i++) {
      for (int j = 0; j < totalNos; j++) {
        string name = "x_" + to_string(i) + "_" + to_string(j);
        x[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, name);
      }
    }

    // y[j] - bateria 
    vector<GRBVar> y(totalNos);
    for (int j = 0; j < totalNos; j++) {
      string name = "y_" + to_string(j);
      y[j] = model.addVar(0.0, instancia.capacidadeEnergia, 0.0, GRB_CONTINUOUS, name);
    }

    // u[j] - carga
    vector<GRBVar> u(totalNos);
    for (int j = 0; j < totalNos; j++) {
      string name = "u_" + to_string(j);
      u[j] = model.addVar(0.0, instancia.capacidade, 0.0, GRB_CONTINUOUS, name);
    }

    double h = instancia.consumoEnergia;
    double Q = instancia.capacidadeEnergia;
    double C = instancia.capacidade;

    // FO (1)
    GRBLinExpr objExpr = 0;
    for (int i = 0; i < totalNos; i++) {
      for (int j = 0; j < totalNos; j++) {
        if (i != j) {
          objExpr += dist[i][j] * x[i][j];
        }
      }
    }
    model.setObjective(objExpr, GRB_MINIMIZE);

    // Constraint (2): Cada cliente tem somente 1 sucessor
    for (int i = 1; i <= numClientes; i++) {
      GRBLinExpr expr = 0;
      for (int j = 0; j < totalNos; j++) {
        if (i != j) {
          expr += x[i][j];
        }
      }
      model.addConstr(expr == 1, "client_out_" + to_string(i));
    }

    // Constraint (3): Cada estacao tem max 1 sucessor
    for (int s = 0; s < m; s++) {
      int idx = n + s;
      GRBLinExpr expr = 0;
      for (int j = 0; j < totalNos; j++) {
        if (idx != j) {
          expr += x[idx][j];
        }
      }
      model.addConstr(expr <= 1, "station_out_" + to_string(s));
    }

    // Constraint (4): Garantir o fluxo (entrada/saida)
    for (int j = 1; j < totalNos; j++) {
      GRBLinExpr exprIn = 0;
      GRBLinExpr exprOut = 0;
      for (int i = 0; i < totalNos; i++) {
        if (i != j) {
          exprIn += x[i][j];
          exprOut += x[j][i];
        }
      }
      model.addConstr(exprIn - exprOut == 0, "flow_" + to_string(j));
    }

    double BigM_battery = 2.0 * Q;
    double BigM_capacity = 2.0 * C;

    // Constraint (5): Desgaste da bateria entre clientes
    for (int i = 1; i <= numClientes; i++) {
      for (int j = 1; j < totalNos; j++) {
        if (i != j) {
          model.addConstr(y[j] <= y[i] - h * dist[i][j] + BigM_battery * (1 - x[i][j]),
                          "bat_dec_ub_" + to_string(i) + "_" + to_string(j));
          model.addConstr(y[j] >= y[i] - h * dist[i][j] - BigM_battery * (1 - x[i][j]),
                          "bat_dec_lb_" + to_string(i) + "_" + to_string(j));
        }
      }
    }

    // Constraint (6): Recarga da bateria ao deixar um posto
    for (int j = 1; j < totalNos; j++) {
      model.addConstr(y[j] <= Q - h * dist[0][j] + BigM_battery * (1 - x[0][j]),
                      "bat_depot_ub_" + to_string(j));
      model.addConstr(y[j] >= Q - h * dist[0][j] - BigM_battery * (1 - x[0][j]),
                      "bat_depot_lb_" + to_string(j));
    }

    for (int s = 0; s < m; s++) {
      int i = n + s;
      for (int j = 1; j < totalNos; j++) {
        if (i != j) {
          model.addConstr(y[j] <= Q - h * dist[i][j] + BigM_battery * (1 - x[i][j]),
                          "bat_station_ub_" + to_string(i) + "_" + to_string(j));
          model.addConstr(y[j] >= Q - h * dist[i][j] - BigM_battery * (1 - x[i][j]),
                          "bat_station_lb_" + to_string(i) + "_" + to_string(j));
        }
      }
    }

    // Constraint (7): Reducao da capacidade 
    for (int i = 0; i < totalNos; i++) {
      for (int j = 1; j <= numClientes; j++) {
        if (i != j) {
          double demandaJ = instancia.demandas[j].demanda;
          model.addConstr(u[j] <= u[i] - demandaJ + BigM_capacity * (1 - x[i][j]),
                          "cap_dec_ub_" + to_string(i) + "_" + to_string(j));
          model.addConstr(u[j] >= u[i] - demandaJ - BigM_capacity * (1 - x[i][j]),
                          "cap_dec_lb_" + to_string(i) + "_" + to_string(j));
        }
      }
    }

    for (int i = 0; i < totalNos; i++) {
      for (int s = 0; s < m; s++) {
        int j = n + s;
        if (i != j) {
          model.addConstr(u[j] <= u[i] + BigM_capacity * (1 - x[i][j]),
                          "cap_station_ub_" + to_string(i) + "_" + to_string(j));
          model.addConstr(u[j] >= u[i] - BigM_capacity * (1 - x[i][j]),
                          "cap_station_lb_" + to_string(i) + "_" + to_string(j));
        }
      }
    }

    // Constraint (8)
    model.addConstr(u[0] >= 0, "depot_cap_lb");
    model.addConstr(u[0] <= C, "depot_cap_ub");
    model.addConstr(y[0] == Q, "depot_bat");

    // Limita o número de veículos
    GRBLinExpr exprVeiculos = 0;
    for (int j = 1; j < totalNos; j++) {
      exprVeiculos += x[0][j];
    }
    model.addConstr(exprVeiculos <= instancia.veiculos, "max_vehicles");

    // Garantia de fluxo do depósito
    GRBLinExpr exprDepotOut = 0;
    GRBLinExpr exprDepotIn = 0;
    for (int j = 1; j < totalNos; j++) {
      exprDepotOut += x[0][j];
      exprDepotIn += x[j][0];
    }
    model.addConstr(exprDepotOut == exprDepotIn, "depot_flow");

    // Previne Self-loops
    for (int i = 0; i < totalNos; i++) {
      model.addConstr(x[i][i] == 0, "no_self_loop_" + to_string(i));
    }

    for (int j = 0; j < totalNos; j++) {
      model.addConstr(y[j] >= 0, "bat_lb_" + to_string(j));
      model.addConstr(u[j] <= C, "cap_ub_" + to_string(j));
    }

    // Parametros do Solver
    model.set(GRB_DoubleParam_TimeLimit, 3600.0);
    model.set(GRB_DoubleParam_MIPGap, 0.0);

    double tempoInicio = model.get(GRB_DoubleAttr_Runtime);
    model.optimize();
    double tempoTotal = model.get(GRB_DoubleAttr_Runtime);

    string nomeBase = nomeArquivo;
    size_t posExt = nomeBase.find(".evrp");
    if (posExt != string::npos) {
      nomeBase = nomeBase.substr(0, posExt);
    }

    string solucaoArquivo = "solucoes/" + nomeBase + "_GUROBI.txt";
    ofstream solFile(solucaoArquivo);

    solFile << "Instancia: " << nomeBase << endl;
    solFile << fixed << setprecision(6);

    double lb = 0, ub = 0, gap = 0;

    int status = model.get(GRB_IntAttr_Status);

    if (status == GRB_OPTIMAL || status == GRB_TIME_LIMIT) {
      int solCount = model.get(GRB_IntAttr_SolCount);

      if (solCount > 0) {
        ub = model.get(GRB_DoubleAttr_ObjVal);
        lb = model.get(GRB_DoubleAttr_ObjBound);

        if (ub > 0.0001) {
          gap = ((ub - lb) / ub) * 100.0;
        }

        solFile << "\nLB (Lower Bound): " << lb << endl;
        solFile << "UB (Upper Bound): " << ub << endl;
        solFile << "GAP (%): " << gap << endl;
        solFile << "TEMPO (seg): " << tempoTotal << endl;
        solFile << "Status: " << status << " (2=Optimal, 9=TimeLimit)" << endl;

        if (instancia.valorOtimo > 0) {
          double gapOtimo = ((ub - instancia.valorOtimo) / instancia.valorOtimo) * 100.0;
          solFile << "Valor Otimo Conhecido: " << instancia.valorOtimo << endl;
          solFile << "GAP vs Otimo (%): " << gapOtimo << endl;
          cout << "  GAP vs Otimo: " << gapOtimo << "%" << endl;
        }

        int numRota = 1;

        for (int start = 1; start < totalNos; start++) {
          if (x[0][start].get(GRB_DoubleAttr_X) > 0.5) {
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
                if (x[atual][j].get(GRB_DoubleAttr_X) > 0.5) {
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
        solFile << "\nStatus: Sem solucao encontrada" << endl;
        solFile << "TEMPO (seg): " << tempoTotal << endl;

        try {
          lb = model.get(GRB_DoubleAttr_ObjBound);
          solFile << "LB (Lower Bound): " << lb << endl;
        } catch (...) {
          solFile << "LB: N/A" << endl;
        }

        cout << "\nGurobi nao encontrou solucao no tempo limite." << endl;
        cout << "  TEMPO: " << tempoTotal << " seg" << endl;
      }
    } else {
      solFile << "\nStatus: " << status << " (Infeasible ou outro)" << endl;
      solFile << "TEMPO (seg): " << tempoTotal << endl;
      cout << "\nGurobi terminou com status: " << status << endl;
    }

    solFile.close();

    cout << "\nSolucao salva em: " << solucaoArquivo << endl;

  } catch (GRBException &e) {
    cerr << "Erro no Gurobi: " << e.getMessage() << " (Code: " << e.getErrorCode() << ")" << endl;
  } catch (exception &e) {
    cerr << "Erro: " << e.what() << endl;
  }
}
