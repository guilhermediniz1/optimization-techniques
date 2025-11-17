#include "utils.hpp"
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <fstream>
#include <iostream>
#include <sstream>
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
    // Pega o segundo item (indice 1) pois o primeiro (deposito) tem demanda 0
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
// --- FIM DA CORRECAO 2 ---

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

    // Ignora linhas em branco
    if (linha.find_first_not_of(" \t") == string::npos) {
      continue;
    }

    // Identifica secoes
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

    // Secao de Cabecalho (Chave: Valor)
    if (secaoAtual == "") {
      size_t divisorPos = linha.find(":");
      if (divisorPos == string::npos)
        continue;

      // Limpa a chave
      string chave = linha.substr(0, divisorPos);
      size_t ultimoCaractereChave = chave.find_last_not_of(" \t");
      if (ultimoCaractereChave != string::npos) {
        chave = chave.substr(0, ultimoCaractereChave + 1);
      }

      // Limpa o valor
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
