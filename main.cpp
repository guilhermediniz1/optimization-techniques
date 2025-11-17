#include "utils.hpp"
#include <iostream>
#include <vector>
#include <string>

using namespace std;

int main() {
    vector<string> instancias = {
        "E-n22-k4.evrp",
        "E-n23-k3.evrp",
        "E-n30-k3.evrp",
        "E-n33-k4.evrp",
        "E-n51-k5.evrp",
        "E-n76-k7.evrp"
    };

    InstanciaEVRP instancia;

    if (carregarInstancia(instancias[0], instancia)) {
      resolverEVRP(instancia, instancias[0]);
    }

    return 0;
}
