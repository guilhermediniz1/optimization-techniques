CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

GUROBI_HOME = /opt/gurobi1203/linux64
CPLEX_HOME = /opt/ibm/ILOG/CPLEX_Studio2211

GUROBI_INC = -I$(GUROBI_HOME)/include
GUROBI_LIB = -L$(GUROBI_HOME)/lib -lgurobi_c++ -lgurobi120 -lm

CPLEX_INC = -I$(CPLEX_HOME)/cplex/include -I$(CPLEX_HOME)/concert/include
CPLEX_LIB = -L$(CPLEX_HOME)/cplex/lib/x86-64_linux/static_pic \
            -L$(CPLEX_HOME)/concert/lib/x86-64_linux/static_pic \
            -lilocplex -lcplex -lconcert -lm -lpthread -ldl

SOURCES = main.cpp utils.cpp cplex_solver.cpp gurobi_solver.cpp meta_heuristica.cpp
TARGET = main

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(GUROBI_INC) $(CPLEX_INC) -o $(TARGET) $(SOURCES) $(GUROBI_LIB) $(CPLEX_LIB)

clean:
	rm -f $(TARGET)

.PHONY: all clean
