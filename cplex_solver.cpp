#include "cplex_solver.hpp"
#include <ilcplex/ilocplex.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

void resolverEVRP(const InstanciaEVRP &instancia, const string &nomeArquivo) {
    imprimirInstanciaEVRP(instancia);

    string nomeBase = nomeArquivo;
    size_t posSlash = nomeBase.rfind('/');
    if (posSlash != string::npos) {
        nomeBase = nomeBase.substr(posSlash + 1);
    }
    size_t posExt = nomeBase.find(".evrp");
    if (posExt != string::npos) {
        nomeBase = nomeBase.substr(0, posExt);
    }

    string lpFilename = "lp/" + nomeBase + ".lp";
    exportEVRPtoLP(instancia, "lp/" + nomeBase);

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
            dist[idx1][idx2] = calcularDistancia(instancia.nos[idEstacao1], instancia.nos[idEstacao2]);
        }
    }

    IloEnv env;
    try {
        IloModel model(env);
        IloCplex cplex(env);

        // Importa o modelo do arquivo LP
        IloObjective obj(env);
        IloNumVarArray vars(env);
        IloRangeArray rngs(env);

        cplex.importModel(model, lpFilename.c_str(), obj, vars, rngs);
        cplex.extract(model);

        // Parametros do solver
        cplex.setParam(IloCplex::Param::TimeLimit, 3600.0);
        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0);

        cout << "\nIniciando otimizacao com CPLEX..." << endl;

        double tempoInicio = cplex.getCplexTime();
        bool solved = cplex.solve();
        double tempoTotal = cplex.getCplexTime() - tempoInicio;

        string solucaoArquivo = "solucoes/" + nomeBase + "_CPLEX.txt";
        ofstream solFile(solucaoArquivo);

        solFile << "Instancia: " << nomeBase << endl;
        solFile << fixed << setprecision(6);

        double lb = 0, ub = 0, gap = 0;
        IloCplex::CplexStatus status = cplex.getCplexStatus();

        if (solved) {
            ub = cplex.getObjValue();
            lb = cplex.getBestObjValue();

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

            // Extrai valores das variaveis x para reconstruir rotas
            IloNumArray vals(env);
            cplex.getValues(vals, vars);

            // Mapeia variaveis x_i_j para valores
            vector<vector<double>> xVal(totalNos, vector<double>(totalNos, 0.0));
            for (IloInt v = 0; v < vars.getSize(); v++) {
                string varName = vars[v].getName();
                if (varName.substr(0, 2) == "x_") {
                    size_t pos1 = varName.find('_', 2);
                    if (pos1 != string::npos) {
                        int i = stoi(varName.substr(2, pos1 - 2));
                        int j = stoi(varName.substr(pos1 + 1));
                        if (i < totalNos && j < totalNos) {
                            xVal[i][j] = vals[v];
                        }
                    }
                }
            }

            // Extrai rotas
            int numRota = 1;
            solFile << "\nRotas:" << endl;

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
                            cargaRota += getDemandaByNodeId(instancia, instancia.nos[atual].id);
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

                    numRota++;
                }
            }

            solFile << "Numero de rotas: " << (numRota - 1) << endl;
            solFile << "Distancia total: " << ub << endl;

            vals.end();
        } else {
            solFile << "\nStatus: Sem solucao encontrada" << endl;
            solFile << "TEMPO (seg): " << tempoTotal << endl;
            solFile << "Status CPLEX: " << status << endl;

            cout << "\nCPLEX nao encontrou solucao." << endl;
            cout << "  TEMPO: " << tempoTotal << " seg" << endl;
        }

        solFile.close();
        cout << "\nSolucao salva em: " << solucaoArquivo << endl;
    } catch (IloException &e) {
        cerr << "Erro no CPLEX: " << e.getMessage() << endl;
    } catch (exception &e) {
        cerr << "Erro: " << e.what() << endl;
    }

    env.end();
}
