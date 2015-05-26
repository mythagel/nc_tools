#include "rs274_model.h"
#include "rs274ngc_return.hh"

#include <iostream>

int main() {
    rs274_model modeler;

    std::string line;
    while(std::getline(std::cin, line)) {
        int status;

        status = modeler.read(line.c_str());
        if(status != RS274NGC_OK) {
            if(status != RS274NGC_EXECUTE_FINISH) {
                std::cerr << "Error reading line!: \n";
                std::cerr << line <<"\n";
                return status;
            }
        }
        
        status = modeler.execute();
        if(status != RS274NGC_OK)
            return status;
    }

    return 0;
}
