#include "Debug.h"
#include <fstream>
#include <iostream>
#include <string>
using namespace std;


dbg::dbg(std::string y = "/home/lvuser/DEBUG.txt") {
    file.open(y);
    iterations = 0;
    if(!file.is_open()) {
        std::cout << "ERROR DEBUG FILE FAILED TO OPEN";
    }
}
void dbg::out(string x) {
    iterations++;
    file << "[" << iterations << "]" << x << endl;
}
bool dbg::end() {
    file << "[ROBOT SHUT OFF]";
    file.close();
    if(!file.is_open()) {
        return true;
    } else {
        return false;
    }
}
 dbg::~dbg() {
    this->end();
}