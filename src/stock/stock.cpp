#include "geom/polyhedron.h"
#include "geom/primitives.h"
#include <iostream>

int main(int argc, char* argv[]) {
    auto stock = geom::make_box({x:0, y:0, z:0}, {x:50, y:50, z:10});
    std::cout << geom::format::off << stock;
    return 0;
}
