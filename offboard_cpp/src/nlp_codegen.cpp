#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <casadi/casadi.hpp>
using namespace casadi;


int main()
{
    MX x = MX::sym("x", 2);
    MX f = x(0)*x(0) + x(1)*x(1);
    MX g = x(0)+x(1)-10;
    Function solver = nlpsol("solver", "ipopt", {{"x", x}, {"f", f}, {"g", g}});
    solver.generate_dependencies("nlp.c");


    bool jit = false;
    if (jit) 
    {
        solver = nlpsol("solver", "ipopt", "nlp.c");
    } 
    else 
    {
        int flag = system("gcc -fPIC -shared -O3 nlp.c -o nlp.so");
        casadi_assert(flag==0, "Compilation failed");

        solver = nlpsol("solver", "ipopt", "nlp.so");
    }

    std::map<std::string, DM> arg, res;
    arg["lbx"] = -DM::inf();
    arg["ubx"] =  DM::inf();
    arg["lbg"] =  0;
    arg["ubg"] =  0;
    arg["x0"] = 0;

    res = solver(arg);

    std::cout << "-----" << std::endl;
    std::cout << "objective at solution = " << res.at("f") << std::endl;
    std::cout << "primal solution = " << res.at("x") << std::endl;
    std::cout << "dual solution (x) = " << res.at("lam_x") << std::endl;
    std::cout << "dual solution (g) = " << res.at("lam_g") << std::endl;

    return 0;
}