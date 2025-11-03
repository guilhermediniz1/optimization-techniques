#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdio>
#include "utils.h"

#define TAMANHO_BUFFER 100
#define TAMANHO_MAX_CAMINHO 64

using namespace std;

enum FuncaoLeitura
{
    PARAMETROS,
    COORDENADAS,
    DEMANDAS,
    PONTOS_RECARGA,
    DEPOSITO
};

void ler_instancia(char *arquivo)
{
    ifstream leitor;
    const char *caminho_base = "dataset";

    char caminho_arquivo[TAMANHO_MAX_CAMINHO];

    snprintf(caminho_arquivo, TAMANHO_MAX_CAMINHO, "%s\\%s", caminho_base, arquivo);

    leitor.open(caminho_arquivo);

    if (leitor.is_open())
    {
        int capacidade_carga, capacidade_bateria, veiculos, dimensao, estacoes;
        float taxa_consumo_bateria;
        FuncaoLeitura funcaoAtual = PARAMETROS;

        char buffer[TAMANHO_BUFFER];

        while (leitor.getline(buffer, TAMANHO_BUFFER)) {
            if (strncmp(buffer, "DIMENSION", 8) == 0) {
                if (sscanf(buffer, "DIMENSION: %d", &dimensao) == 1) {
                    cout << "Dimensao: " << dimensao << endl;
                }
                continue;

            } else if (strncmp(buffer, "STATIONS", 8) == 0) {
                if (sscanf(buffer, "STATIONS: %d", &estacoes) == 1) {
                    cout << "Estacoes de carregamento: " << estacoes << endl;
                }
                continue;
            } 
            
            // Inicializando matriz de coordenadas
            int coordenadas[dimensao + estacoes][2];
            
            
            if (strncmp(buffer, "CAPACITY", 8) == 0) {
                if (sscanf(buffer, "CAPACITY: %d", &capacidade_carga) == 1) {
                    cout << "Capacidade: " << capacidade_carga << endl;
                }
                continue;
            } else if (strncmp(buffer, "ENERGY_CAPACITY", 15) == 0) {
                if (sscanf(buffer, "ENERGY_CAPACITY: %d", &capacidade_bateria) == 1) {
                    cout << "Capacidade de Energia: " << capacidade_bateria << endl;
                }
                continue;
            } else if (strncmp(buffer, "ENERGY_CONSUMPTION", 18) == 0) {
                if (sscanf(buffer, "ENERGY_CONSUMPTION: %f", &taxa_consumo_bateria) == 1) {
                    cout << "Consumo de bateria: " << taxa_consumo_bateria << endl;
                }
                continue;
            } else if (strncmp(buffer, "NODE_COORD_SECTION", 18) == 0) {
                funcaoAtual = COORDENADAS;
                continue;
            } else if (strncmp(buffer, "DEMAND_SECTION", 14) == 0) {
                funcaoAtual = DEMANDAS;
                continue;
            } else if (strncmp(buffer, "STATIONS_COORD_SECTION", 14) == 0) {
                funcaoAtual = PONTOS_RECARGA;
                continue;
            } else if (strncmp(buffer, "DEPOT_SECTION", 8) == 0) {
                funcaoAtual = DEPOSITO;
                continue;
            } else if (strncmp(buffer, "EOF", 3) == 0) {
                break;
            }

            switch (funcaoAtual) {
            case COORDENADAS:
                cout << "Coord: " << buffer << endl;
                break;

            case DEMANDAS:
                cout << "Demanda: " << buffer << endl;
                break;

            case PONTOS_RECARGA:
                cout << "Pontos de recarga: " << buffer << endl;
                break;

            case DEPOSITO:
                cout << "Deposito: " << buffer << endl;
                break;

            default:
                break;
            }
        }

        leitor.close();
    }
    else
    {
        cout << "Erro: Nao foi possivel abrir o arquivo de instancia: '" << caminho_arquivo << "'" << endl;
    }
}
