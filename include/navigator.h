//
// Created by Tomas Krejci on 5/11/16.
//

#ifndef TONAV_NAVIGATOR_H
#define TONAV_NAVIGATOR_H

#include <boost/program_options.hpp>

/**
 * @brief Perform navigation from dataset.
 *
 * @deprecated This is deprecated. Use Tonav or it's convenience
 *             wrapper TonavRos instead.
 */
class Navigator {
public:
    /**
     * @brief Run navigation node from dataset.
     *
     * @deprecated This is deprecated. Use Tonav or it's convenience
     *             wrapper TonavRos instead.
     *
     * @param argc Number of command line arguments.
     * @param argv List of command line arguments.
     */
    int run(int argc, const char* argv[]);

private:
    void parseOptions(int argc, const char* argv[]);
    void printHelp(bool to_stderr = false);

    boost::program_options::options_description options_description_;
    boost::program_options::variables_map vmap_;
};

#endif //TONAV_NAVIGATOR_H
