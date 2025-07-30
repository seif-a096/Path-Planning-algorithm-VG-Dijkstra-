#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
using namespace std;
#include "IOHandler.h"
#include "algorithm.h"

void clear16_end(fstream& file , const string& name ){
    vector<string> lines;
    string line;
    int line_num = 0;
    ifstream infile(name);
    while (getline(infile, line) && line_num < 13) {
        lines.push_back(line);
        line_num++;
    }
    infile.close();

    // Overwrite file with only first 15 lines
    ofstream outfile( name, ios::out);  
    for (const auto& l : lines) {
        outfile << l << endl;
    }
    outfile.close();
}

int main() {
    //////////////////////////////////////// clear files before running each time -not to do it manually-///////////////
    ofstream file1("./CSV/edges.csv", ios::out );
    ofstream file2("./CSV/path.csv", ios::out );
    fstream file3("./CSV/nodes.csv");
    clear16_end(file3 ,"./CSV/nodes.csv" );

    //////////////////////////////////////// DO NOT Change this code ///////////////////////////////////////////////////
    vector<vector<float>> obstacles = readCSV("./CSV/obstacles.csv"); // obstacles.csv
    vector<vector<float>> nodes = readCSV("./CSV/nodes.csv"); // nodes.csv
    vector<vector<float>> treeNodes;
    vector<vector<int>> edges;
    vector<int> path = YourChosenAlgorithm(obstacles, nodes, treeNodes, edges);
    cout << "Path: "<< path.size() << endl;
    for (int node : path) {
        std::cout << node << "->";
    }
    writePath(path, "./CSV/path.csv");
    appendNodes("./CSV/nodes.csv", treeNodes);
    writeEdges("./CSV/edges.csv", edges);
    
    //////////////////////////////////////// DO NOT Change this code ///////////////////////////////////////////////////


     // Rrun python file by default
    system("python -u visualiser.py");



    return 0;
}