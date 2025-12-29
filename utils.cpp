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

bool isEstacao(const InstanciaEVRP &instancia, int idx) {
  int n = instancia.dimensao;

  // O deposito (idx 0) tambem recarrega a bateria
  if (idx == 0) {
    return true;
  }

  // Indices >= n sao estacoes de recarga (copias)
  if (idx >= n) {
    return true;
  }

  // Verifica se o no original eh uma estacao
  int nodeId = instancia.nos[idx].id;
  for (int idEstacao : instancia.idEstacoes) {
    if (nodeId == idEstacao) {
      return true;
    }
  }

  return false;
}

bool validarRota(const InstanciaEVRP &instancia, const vector<int> &rota,
                 const vector<vector<double>> &dist, bool verbose) {
  if (rota.size() < 2) {
    if (verbose) {
      cerr << "Erro: Rota muito curta (menos de 2 nos)" << endl;
    }
    return false;
  }

  // Verifica se a rota comeca e termina no deposito
  if (rota.front() != 0 || rota.back() != 0) {
    if (verbose) {
      cerr << "Erro: Rota deve comecar e terminar no deposito (0)" << endl;
      cerr << "  Inicio: " << rota.front() << ", Fim: " << rota.back() << endl;
    }
    return false;
  }

  double energia = instancia.capacidadeEnergia;
  double capacidade = instancia.capacidade;
  double distanciaTotal = 0.0;
  double h = instancia.consumoEnergia;

  bool valido = true;

  for (size_t i = 0; i < rota.size() - 1; i++) {
    int de = rota[i];
    int para = rota[i + 1];

    // Calcula consumo de energia para este trecho
    double consumoEnergia = h * dist[de][para];
    energia -= consumoEnergia;
    distanciaTotal += dist[de][para];

    // Verifica energia antes de chegar ao destino
    if (energia < -0.0001) {
      if (verbose) {
        cerr << "Erro: Energia abaixo de 0 no trecho " << de << " -> " << para << endl;
        cerr << "  Energia restante: " << energia << endl;
        cerr << "  Consumo do trecho: " << consumoEnergia << endl;
        cerr << "  Distancia do trecho: " << dist[de][para] << endl;
      }
      valido = false;
    }

    // Subtrai demanda do destino (se for cliente)
    No noDestino = getNoByIndex(instancia, para);
    int demanda = getDemandaByNodeId(instancia, noDestino.id);
    capacidade -= demanda;

    // Verifica capacidade
    if (capacidade < -0.0001) {
      if (verbose) {
        cerr << "Erro: Capacidade excedida ao visitar no " << para << endl;
        cerr << "  Capacidade restante: " << capacidade << endl;
        cerr << "  Demanda do no: " << demanda << endl;
      }
      valido = false;
    }

    // Recarrega bateria se destino eh estacao ou deposito
    if (isEstacao(instancia, para)) {
      energia = instancia.capacidadeEnergia;
    }

    // Recarrega capacidade se retornar ao deposito
    if (para == 0) {
      capacidade = instancia.capacidade;
    }
  }

  if (verbose && valido) {
    cout << "  Rota valida! Distancia: " << distanciaTotal << endl;
  }

  return valido;
}

bool validarSolucao(const InstanciaEVRP &instancia, const vector<vector<int>> &rotas,
                    const vector<vector<double>> &dist, bool verbose) {
  if (rotas.empty()) {
    if (verbose) {
      cerr << "Erro: Solucao sem rotas" << endl;
    }
    return false;
  }

  bool todasValidas = true;
  double distanciaTotal = 0.0;
  int numClientes = instancia.dimensao - 1;
  vector<bool> clienteVisitado(numClientes + 1, false);

  if (verbose) {
    cout << "\n=== Validando Solucao ===" << endl;
    cout << "Numero de rotas: " << rotas.size() << endl;
  }

  for (size_t r = 0; r < rotas.size(); r++) {
    if (verbose) {
      cout << "\nValidando Rota " << (r + 1) << ": ";
      for (int no : rotas[r]) {
        cout << no << " ";
      }
      cout << endl;
    }

    if (!validarRota(instancia, rotas[r], dist, verbose)) {
      todasValidas = false;
    }

    // Calcula distancia e marca clientes visitados
    for (size_t i = 0; i < rotas[r].size() - 1; i++) {
      distanciaTotal += dist[rotas[r][i]][rotas[r][i + 1]];
    }

    // Marca clientes visitados (indices 1 a numClientes)
    for (int no : rotas[r]) {
      if (no >= 1 && no <= numClientes) {
        if (clienteVisitado[no]) {
          if (verbose) {
            cerr << "Erro: Cliente " << no << " visitado mais de uma vez!" << endl;
          }
          todasValidas = false;
        }
        clienteVisitado[no] = true;
      }
    }
  }

  // Verifica se todos os clientes foram visitados
  for (int i = 1; i <= numClientes; i++) {
    if (!clienteVisitado[i]) {
      if (verbose) {
        cerr << "Erro: Cliente " << i << " nao foi visitado!" << endl;
      }
      todasValidas = false;
    }
  }

  if (verbose) {
    cout << "\n=== Resumo da Validacao ===" << endl;
    cout << "Distancia total: " << distanciaTotal << endl;
    cout << "Status: " << (todasValidas ? "VALIDO" : "INVALIDO") << endl;
  }

  return todasValidas;
}

bool carregarSolucao(const string &nomeArquivo, vector<vector<int>> &rotas) {
  ifstream arquivo(nomeArquivo);

  if (!arquivo.is_open()) {
    cerr << "Erro: Nao foi possivel abrir o arquivo de solucao " << nomeArquivo << endl;
    return false;
  }

  rotas.clear();
  string linha;
  bool secaoRotas = false;

  while (getline(arquivo, linha)) {
    // Procura pelo inicio da secao de rotas
    if (linha.find("Rotas:") != string::npos) {
      secaoRotas = true;
      continue;
    }

    // Para ao encontrar linhas de resumo ou fim das rotas
    if (secaoRotas && (linha.find("Numero de rotas:") != string::npos ||
                       linha.find("Distancia total:") != string::npos)) {
      break;
    }

    // Ignora linhas de detalhes (Distancia:, Carga:)
    if (linha.find("Distancia:") != string::npos ||
        linha.find("Carga:") != string::npos) {
      continue;
    }

    // Parse das rotas: "Rota X: 0 1 2 3 0"
    if (secaoRotas && linha.find("Rota") != string::npos) {
      size_t pos = linha.find(":");
      if (pos != string::npos) {
        string nosStr = linha.substr(pos + 1);
        istringstream iss(nosStr);
        vector<int> rota;
        int no;
        while (iss >> no) {
          rota.push_back(no);
        }
        if (!rota.empty()) {
          rotas.push_back(rota);
        }
      }
    }
  }

  arquivo.close();
  return !rotas.empty();
}

bool verificarSolucaoArquivo(const InstanciaEVRP &instancia, const string &nomeInstancia,
                              const string &solver) {
  string solverUpper = (solver == "gurobi") ? "GUROBI" : "CPLEX";
  string arquivoSolucao = "solucoes/" + nomeInstancia + "_" + solverUpper + ".txt";

  cout << "=== Verificacao de Solucao ===" << endl;
  cout << "Instancia: " << nomeInstancia << endl;
  cout << "Solver: " << solverUpper << endl;
  cout << "Arquivo: " << arquivoSolucao << endl;
  cout << endl;

  vector<vector<int>> rotas;
  if (!carregarSolucao(arquivoSolucao, rotas)) {
    cerr << "Erro: Nao foi possivel carregar a solucao do arquivo" << endl;
    return false;
  }

  cout << "Rotas carregadas: " << rotas.size() << endl;
  for (size_t i = 0; i < rotas.size(); i++) {
    cout << "Rota " << (i + 1) << ": ";
    for (int no : rotas[i]) {
      cout << no << " ";
    }
    cout << endl;
  }
  cout << endl;

  // Calcula matriz de distancias
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

  return validarSolucao(instancia, rotas, dist, true);
}
