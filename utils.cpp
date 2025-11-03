#include <iostream>
#include <fstream>
#include "utils.h"

#define TAMANHO_BUFFER 100
#define TAMANHO_MAX_CAMINHO 64

using namespace std;

void ler_instancia(char* arquivo) {
    ifstream leitor;
    const char* caminho_base = "dataset";

    char caminho_arquivo[TAMANHO_MAX_CAMINHO];

    snprintf(caminho_arquivo, TAMANHO_MAX_CAMINHO, "%s\\%s", caminho_base, arquivo);
    
    leitor.open(caminho_arquivo);

    if (leitor.is_open()) {
        cout << "Lendo instancia: '" << arquivo << "':\n\n";

        char buffer[TAMANHO_BUFFER];

        while (leitor.getline(buffer, TAMANHO_BUFFER)) {
            cout << buffer << endl;
        }

        leitor.close();

    } else {
        cout << "Erro: Nao foi possivel abrir o arquivo de instancia: '" << caminho_arquivo << "'" << endl;
    }
}
