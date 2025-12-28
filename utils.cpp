#include "utils.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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
  cout << "Estacoes: " << instancia.estacoes << endl;
  cout << "Capacidade: " << instancia.capacidade << endl;
  cout << "Deposito: " << instancia.idDeposito << endl;
  cout << "--------------------------------" << endl;
}

bool carregarInstancia(const string &nomeArquivo, InstanciaEVRP &instancia) {
  string caminhoCompleto = "dataset/" + nomeArquivo + ".evrp";
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
        ssValor >> instancia.estacoes;
        instancia.estacoesTotal = instancia.estacoes * 2;
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

No getNoByIndex(const InstanciaEVRP &instancia, int idx) {
  int n = instancia.dimensao;
  int estacoes = instancia.estacoes;

  if (idx < n) {
    return instancia.nos[idx];
  }

  int offset = idx - n;

  int estacaoOriginalIdx = offset % estacoes;
  int idEstacaoOriginal = instancia.idEstacoes[estacaoOriginalIdx];

  for (const auto &no : instancia.nos) {
    if (no.id == idEstacaoOriginal) {
      return no;
    }
  }

  cerr << "ERRO: Estacao com ID " << idEstacaoOriginal << " nao encontrada nos nos!" << endl;
  throw runtime_error("Estacao nao encontrada");
}

int getDemandaByNodeId(const InstanciaEVRP &instancia, int nodeId) {
  for (const auto &d : instancia.demandas) {
    if (d.id == nodeId) {
      return d.demanda;
    }
  }
  return 0;
}

void exportEVRPtoLP(const InstanciaEVRP &instancia, const string &nomeArquivo) {
  string lpFilename = nomeArquivo + ".lp";
  ofstream lpFile(lpFilename);

  if (!lpFile.is_open()) {
    cerr << "Erro ao criar arquivo .lp" << endl;
    return;
  }

  cout << "Gerando arquivo LP: " << lpFilename << "..." << endl;

  int n = instancia.dimensao;
  int m = instancia.estacoesTotal;
  int numClientes = n - 1;
  int totalNos = n + m;

  double h = instancia.consumoEnergia;
  double Q = instancia.capacidadeEnergia;
  double C = instancia.capacidade;

  vector<vector<double>> dist(totalNos, vector<double>(totalNos, 0.0));
  for (int i = 0; i < totalNos; i++) {
    for (int j = 0; j < totalNos; j++) {
      No noI = getNoByIndex(instancia, i);
      No noJ = getNoByIndex(instancia, j);
      dist[i][j] = calcularDistancia(noI, noJ);
    }
  }

  lpFile << fixed << setprecision(4);

  lpFile << "Minimize" << endl;
  lpFile << " obj: ";

  bool first = true;
  for (int i = 0; i < totalNos; i++) {
    for (int j = 0; j < totalNos; j++) {
      if (i != j) {
        if (!first)
          lpFile << " + ";
        lpFile << dist[i][j] << " x_" << i << "_" << j;
        first = false;
      }
    }
  }
  lpFile << endl;

  lpFile << "Subject To" << endl;

  for (int i = 1; i <= numClientes; i++) {
    first = true;
    lpFile << " c2_" << i << ": ";
    for (int j = 0; j < totalNos; j++) {
      if (i != j) {
        if (!first)
          lpFile << " + ";
        lpFile << "x_" << i << "_" << j;
        first = false;
      }
    }
    lpFile << " = 1" << endl;
  }

  lpFile << endl;

  for (int s = 0; s < m; s++) {
    int idx = n + s;
    first = true;
    lpFile << " c3_" << s + 1 << ": ";
    for (int j = 0; j < totalNos; j++) {
      if (idx != j) {
        if (!first)
          lpFile << " + ";
        lpFile << "x_" << idx << "_" << j;
        first = false;
      }
    }
    lpFile << " <= 1" << endl;
  }

  lpFile << endl;

  for (int j = 1; j < totalNos; j++) {
    bool first = true;
    lpFile << " c4_" << j << ": ";
    for (int i = 0; i < totalNos; i++) {
      if (i != j) {
        if (!first) lpFile << " +";
        lpFile << " x_" << j << "_" << i;
        first = false;
      }
    }

    for (int i = 0; i < totalNos; i++) {
      if (i != j) {
        lpFile << " - x_" << i << "_" << j;
      }
    }

    lpFile << " = 0" << endl;
  }

  lpFile << endl;

  // ==========================================
  // Constraint (5): Desgaste da bateria (Cliente -> Cliente/Estacao)
  // Logic: y_j <= y_i - h*dist + Q(1-x)
  // Applies where source 'i' is a Customer (Index 1 to numClientes)
  // ==========================================
  for (int i = 1; i <= numClientes; i++) {
    for (int j = 1; j < totalNos; j++) {
      if (i != j) {
        double coef = h * dist[i][j] + Q;

        lpFile << " c5_" << i << "_" << j << "a: y_" << j << " >= 0" << endl;
        lpFile << " c5_" << i << "_" << j << "b: y_" << j << " - y_" << i << " + " << coef << " x_" << i << "_" << j << " <= " << Q << endl;
      }
    }
  }

  lpFile << endl;

  // ==========================================
  // Constraint (6): Bateria (Origem = Deposito OU Estacao)
  // Logic: y_j <= Q - h*dist*x
  // Applies where source 'i' is Depot (0) OR Station
  // ==========================================
  
  // Create list of recharging sources: Depot + Stations
  vector<int> rechargingSources;
  rechargingSources.push_back(0); // Add Depot
  for (int k = totalNos - m; k < totalNos; k++) {
      rechargingSources.push_back(k); // Add Stations
  }

  for (int j = 1; j < totalNos; j++) {
    lpFile << " c6_" << j << "_a: y_" << j << " >= 0" << endl;

    for (int i : rechargingSources) {
      if (i != j) {
        double custo = h * dist[i][j];
        
        lpFile << " c6_" << j << "_" << i << "_b: y_" << j 
               << " + " << custo << " x_" << i << "_" << j 
               << " <= " << Q << endl;
      }
    }
  }

  lpFile << endl;

  // ==========================================
  // Constraint (7): Capacidade / Carga (Todo 'i' para todo 'j')
  // Logic: u_j <= u_i - q_j*x + C(1-x)
  // Applies to ALL nodes 'i' (Depot, Customers, Stations)
  // ==========================================
  for (int j = 1; j < totalNos; j++) {
    lpFile << " c7_" << j << "_a: u_" << j << " >= 0" << endl;
    
    // FIXED: i loops from 0 to totalNos (includes Depot, Customers, Stations)
    for (int i = 0; i < totalNos; i++) {
      if (i != j) {
        
        // Get demand of the DESTINATION j
        No noJ = getNoByIndex(instancia, j);
        double q_dest = getDemandaByNodeId(instancia, noJ.id);
        
        // Standard VRP Formula: u_j - u_i + (C + q_j) * x_ij <= C
        double x_coefficient = C + q_dest;

        lpFile << " c7_" << j << "_" << i << "_b: ";
        lpFile << "u_" << j << " - u_" << i << " + " 
               << x_coefficient << " x_" << i << "_" << j 
               << " <= " << C << endl;
      }
    }
  }

  lpFile << endl;

  // Constraint (8): Boundary conditions
  lpFile << " c8a: u_0 >= 0" << endl;
  lpFile << " c8b: u_0 <= " << C << endl;

  // ---------------------------------------------------------
  // 3. BOUNDS
  // ---------------------------------------------------------
  lpFile << "Bounds" << endl;

  // Variaveis continuas (Y e U)
  for (int i = 0; i < totalNos; i++) {
    lpFile << " 0 <= y_" << i << " <= " << Q << endl;
    lpFile << " 0 <= u_" << i << " <= " << C << endl;
  }

  // ---------------------------------------------------------
  // 4. BINARIES
  // ---------------------------------------------------------
  lpFile << "Binary" << endl;
  for (int i = 0; i < totalNos; i++) {
    for (int j = 0; j < totalNos; j++) {
      if (i != j) {
        lpFile << " x_" << i << "_" << j << endl;
      }
    }
  }

  lpFile << "End" << endl;
  lpFile.close();
  cout << "Arquivo LP gerado com sucesso." << endl;
}
