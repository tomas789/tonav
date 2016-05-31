//
// Created by Tomas Krejci on 5/11/16.
//

#ifndef TONAV_NAVIGATOR_H
#define TONAV_NAVIGATOR_H

#include <boost/program_options.hpp>

class Navigator {
public:
    int run(int argc, const char* argv[]);

private:
    void parseOptions(int argc, const char* argv[]);
    void printHelp(bool to_stderr = false);

    boost::program_options::options_description options_description_;
    boost::program_options::variables_map vmap_;
};

#endif //TONAV_NAVIGATOR_H
