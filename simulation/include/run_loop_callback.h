//
// Created by Tomas Krejci on 10/14/17.
//

#ifndef TONAV_RUN_LOOP_CALLBACK_H
#define TONAV_RUN_LOOP_CALLBACK_H

class RunLoopCallback {
public:
    virtual void runLoopCallback(double time) = 0;
};

#endif //TONAV_RUN_LOOP_CALLBACK_H
