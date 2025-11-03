#include <iostream>
#include <fstream>
#include "utils.h"

using namespace std;

int main (int argc, char* argv[]) {
    char* arquivo = argv[1];
    ler_instancia(arquivo);

    return 0;
}
