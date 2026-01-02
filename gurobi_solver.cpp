#include "gurobi_solver.hpp"
#include "gurobi_c++.h"
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

void resolverEVRPGurobi(const InstanciaEVRP &instancia,
                        const string &nomeArquivo) {
  imprimirInstanciaEVRP(instancia);

  // Extrai apenas o nome do arquivo (sem diretorio)
  string nomeBase = nomeArquivo;
  size_t posSlash = nomeBase.rfind('/');
  if (posSlash != string::npos) {
    nomeBase = nomeBase.substr(posSlash + 1);
  }
  size_t posExt = nomeBase.find(".evrp");
  if (posExt != string::npos) {
    nomeBase = nomeBase.substr(0, posExt);
  }

  // Gera o arquivo LP no diretorio lp/
  string lpFilename = "lp/" + nomeBase + ".lp";
  exportEVRPtoLP(instancia, "lp/" + nomeBase);

  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;
  int numClientes = n - 1;
  int totalNos = n + m;

  // Calcula distancias para extrair rotas
  vector<vector<double>> dist(totalNos, vector<double>(totalNos, 0.0));
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      dist[i][j] = calcularDistancia(instancia.nos[i], instancia.nos[j]);
    }
  }
  for (int i = 0; i < n; i++) {
    for (int s = 0; s < m; s++) {
      // Each physical station has 2 dummies, so map s to physical station index
      int idEstacaoFisica = s / 2;
      int idEstacao = instancia.idEstacoes[idEstacaoFisica] - 1;
      int indiceEstacao = n + s;
      double d = calcularDistancia(instancia.nos[i], instancia.nos[idEstacao]);
      dist[i][indiceEstacao] = d;
      dist[indiceEstacao][i] = d;
    }
  }
  for (int s1 = 0; s1 < m; s1++) {
    for (int s2 = 0; s2 < m; s2++) {
      // Each physical station has 2 dummies, so map to physical station index
      int idEstacaoFisica1 = s1 / 2;
      int idEstacaoFisica2 = s2 / 2;
      int idEstacao1 = instancia.idEstacoes[idEstacaoFisica1] - 1;
      int idEstacao2 = instancia.idEstacoes[idEstacaoFisica2] - 1;
      int idx1 = n + s1;
      int idx2 = n + s2;
      dist[idx1][idx2] = calcularDistancia(instancia.nos[idEstacao1],
                                           instancia.nos[idEstacao2]);
    }
  }

  try {
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", "gurobi.log");
    env.start();

    // Le o modelo do arquivo LP
    GRBModel model = GRBModel(env, lpFilename);

    // Parametros do Solver
    model.set(GRB_DoubleParam_TimeLimit, 3600.0);
    model.set(GRB_DoubleParam_MIPGap, 0.0);

    cout << "\nIniciando otimizacao com Gurobi..." << endl;

    model.optimize();
    double tempoTotal = model.get(GRB_DoubleAttr_Runtime);

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

        // Extrai valores das variaveis x usando getVarByName
        vector<vector<double>> xVal(totalNos, vector<double>(totalNos, 0.0));
        for (int i = 0; i < totalNos; i++) {
          for (int j = 0; j < totalNos; j++) {
            string varName = "x_" + to_string(i) + "_" + to_string(j);
            try {
              GRBVar var = model.getVarByName(varName);
              xVal[i][j] = var.get(GRB_DoubleAttr_X);
            } catch (...) {
              xVal[i][j] = 0.0;
            }
          }
        }

        // Extrai rotas
        int numRota = 1;
        solFile << "\nRotas:" << endl;
        vector<vector<int>> todasRotas;

        for (int start = 1; start < totalNos; start++) {
          if (xVal[0][start] > 0.5) {
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
                cargaRota +=
                    getDemandaByNodeId(instancia, instancia.nos[atual].id);
              }

              int proximo = -1;
              for (int j = 0; j < totalNos; j++) {
                if (xVal[atual][j] > 0.5) {
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

            solFile << "Rota " << numRota << ": ";
            for (size_t i = 0; i < rota.size(); i++) {
              solFile << rota[i];
              if (i < rota.size() - 1) {
                solFile << " ";
              }
            }
            solFile << endl;
            solFile << "  Distancia: " << distRota << endl;
            solFile << "  Carga: " << cargaRota << endl;
            solFile << endl;

            todasRotas.push_back(rota);
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
    cerr << "Erro no Gurobi: " << e.getMessage()
         << " (Code: " << e.getErrorCode() << ")" << endl;
  } catch (exception &e) {
    cerr << "Erro: " << e.what() << endl;
  }
}
