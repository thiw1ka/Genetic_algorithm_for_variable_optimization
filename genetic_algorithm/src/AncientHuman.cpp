#include "genetic_algorithm/AncientHuman.hpp"


void print_chromosomes(const AncientHuman& g) {
            std::cout << "[chromosome] ";
            std::cout << "fitness (" << g.fitness << ") ";
            for (auto const& v : g.chromosome) {
                std::cout << v.first << "(" << v.second << ") ";
            }

            std::cout << std::endl;
};

std::vector<std::tuple<std::string, double> > AncientHuman::getVariablesInTuple() {
    std::vector<std::tuple<std::string, double> > v;
    for (auto const& var : chromosome) {
        v.push_back(std::make_tuple(var.first, var.second));
    }
    return std::move(v);
}

std::string AncientHuman::getVariableInString () {
    std::string result = "";
    for (auto const& var : chromosome) {
        result+= "\'" + var.first + "\'" + ",";
        result+= "\'" + std::to_string(var.second) + "\'"  + ",";
    }
    result.pop_back();
    return result;
}

std::string AncientHuman::getVariableToSave () {
    //tempreture,0.000000,ps,0.000000,pd,0.000000,wind_speed,0.000000,wind_heading,0.000000,fitness,0.560769
    std::string result = "";
    for (auto const& var : chromosome) {
        result+= var.first + ",";
        result+= std::to_string(var.second)+ ",";
    }
    result.pop_back();
    result += ",fitness," + std::to_string(fitness);
    return result;
}

// void AncientHuman::setAChoromosome (std::string name, auto val) {
//     // chromosome.at(name) = val;

// }

// AncientHuman AncientHuman::operator= (AncientHuman other) {
//     std::cout << "[AncientHuman] equal operator \n";
//     std::copy(other.chromosome.begin(), other.chromosome.end(), chromosome.begin());
//     std::cout << "[AncientHuman] equal operator1 \n";
//     fitness = other.fitness;
//     std::cout << "[AncientHuman] equal operator2 \n";
// }