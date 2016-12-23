//
//  BezierCurve.h
//
//  Created by Michael Shelley on 18/04/14.
//  Copyright (c) 2014 Michael Shelley. All rights reserved.
//
//  Taken from this paper:
//  https://www.cs.tcd.ie/publications/tech-reports/reports.94/TCD-CS-94-18.pdf
//
//  And quaternion slerp from here:
//  http://www.engr.colostate.edu/ECE555/reading/article_8.pdf
//

#ifndef __MSCKF__BezierCurve__
#define __MSCKF__BezierCurve__

#include <iostream>
#include <Eigen/Dense>
#include "BezierNode.h"

template <class TYPE>
class BezierCurve {
    unsigned long N;

    std::vector<float> ST;
    std::vector<float> PR;
    std::vector<BezierNode<TYPE> > C[6];
    std::vector<BezierNode<TYPE> > V[5];
    std::vector<BezierNode<TYPE> > A[4];

    void Differentiate(const std::vector<BezierNode<TYPE> > x[], std::vector<BezierNode<TYPE> > y[], const int dim) {
        for (int d = 0; d < dim; d++) {
            y[d].resize(N);
            for (int i = 0; i < N; i++) {
                y[d][i] = (x[d+1][i] - x[d][i]);
            }
        }
    }

    void GenerateControlPoints(const std::vector<BezierNode<TYPE> > &K) {
        N = K.size();
        BezierNode<TYPE> I;

        // R and T control points
        std::vector<BezierNode<TYPE> > R(N);
        std::vector<BezierNode<TYPE> > T(N);

        R[0] = K[0];
        T[N-1] = K[N-1];
        for (int i = 1; i < N; i++) {
            R[i] = BezierNode<TYPE>::Interpolate(K[i-1], K[i], 1 + PR[i]);
        }
        for (int i = 0; i < N-1; i++) {
            T[i] = BezierNode<TYPE>::Interpolate(R[i], K[i+1], 0.5);
        }

        // Cubic inner control points (used for calculating accelerations)
        std::vector<BezierNode<TYPE> > x(N);
        std::vector<BezierNode<TYPE> > y(N);

        for (int i = 0; i < N; i++) {
            x[i] = BezierNode<TYPE>::Interpolate(K[i], T[i], 1./3.);
        }
        for (int i = 0; i < N-1; i++) {
            y[i] = BezierNode<TYPE>::Interpolate(K[i+1], x[i+1], -1./PR[i+1]);
        }

        // zero initial and final velocity
        x[0] = K[0];
        y[N-1] = K[N-1];

        // Cubic accelerations (NOT continuous)
        std::vector<BezierNode<TYPE> > A30(N); // in
        std::vector<BezierNode<TYPE> > A31(N); // out

        for (int i = 0; i < N; i++) {
            A30[i] = (y[i] - x[i]) - (x[i] - K[i]);
        }
        for (int i = 0; i < N-1; i++) {
            A31[i] = (x[i] - y[i]) - (y[i] - K[i+1]);
        }
        A31[N-1] = (x[N-1] - y[N-1]);

        // Quintic accelerations (continuous)
        std::vector<BezierNode<TYPE> > A50(N); // in
        std::vector<BezierNode<TYPE> > A51(N); // out

        A50[0] = BezierNode<TYPE>::Interpolate(I * PR[0], A30[0], 0.5);
        for (int i = 1; i < N; i++) {
            A50[i] = BezierNode<TYPE>::Interpolate(A31[i-1], A30[i], 0.5);
        }
        for (int i = 0; i < N-1; i++) {
            A51[i] = BezierNode<TYPE>::Interpolate(A31[i], A30[i+1] * (1./PR[i+1]), 0.5);
        }
        A51[N-1] = BezierNode<TYPE>::Interpolate(A31[N-1], I, 0.5);

        // Control Points
        for (int i = 0; i < 6; i++) {
            C[i] = std::vector<BezierNode<TYPE> >(N);
        }
        for (int i = 0; i < N-1; i++) {
            C[0][i] = K[i];
            C[5][i] = K[i+1];
        }
        for (int i = 0; i < N-1; i++) {
            C[1][i] = BezierNode<TYPE>::Interpolate(K[i], T[i], 1./5.);
        }
        C[1][N-1] = K[N-1];

        for (int i = 0; i < N-1; i++) {
            C[4][i] = BezierNode<TYPE>::Interpolate(K[i+1], C[1][i+1], -1./PR[i+1]);
        }
        for (int i = 0; i < N-1; i++) {
            C[2][i] = C[1][i] + ((C[1][i] - K[i]) + A50[i]);
        }
        for (int i = 0; i < N-1; i++) {
            C[3][i] = C[4][i] + ((C[4][i] - K[i+1]) + A51[i]);
        }

        // initial velocity = 0
        C[1][0] = K[0];
        C[5][N-1] = K[N-1];

        // Calculate velocity and acceleration nodes
        Differentiate(C, V, 5);
        Differentiate(V, A, 4);
    }

    // Interpolate between all nodes recursively
    // For Example:
    //
    // C1 C2 C3 C4 C5
    //   I1 I2 I3 I4
    //    J1 J2 J3
    //     K1 K2       <-- base recursive case
    //       V
    //
    BezierNode<TYPE> evaluateAtTime(const std::vector<BezierNode<TYPE> > &x, const float t) const {
        if (x.size() == 2) {
            return BezierNode<TYPE>::Interpolate(x[0], x[1], t);
        }

        std::vector<BezierNode<TYPE> > y(x.size() - 1);
        for (int i = 0; i < y.size(); i++) {
            y[i] = BezierNode<TYPE>::Interpolate(x[i], x[i+1], t);
        }

        return evaluateAtTime(y, t);
    }

    // turn t into [0, 1) between nodes index and index+1
    float normalizedTime(const float time, unsigned long &index) const {
        // TODO: make variable
        float cum_time = 0;
        for (index = 0; index < N-2; index++) {
            if (time < cum_time + ST[index]) {
                break;
            }
            cum_time += ST[index];
        }

        return (time - cum_time) / ST[index];
    }

    const BezierNode<TYPE> evaluate(const std::vector<BezierNode<TYPE> > x[],
                                    const int order,
                                    const float time,
                                    unsigned long &index) const {
        if (time <= 0) {
            return x[0][0].value();
        }

        float t = normalizedTime(time, index);

        if (index >= N - 1 || t > 1) {
            index = N - 1;
            return x[order-1][N-1].value();
        }

        std::vector<BezierNode<TYPE> > y(order);
        for (int i = 0; i < order; i++) {
            y[i] = x[i][index];
        }

        return evaluateAtTime(y, t);
    }

public:
    /* Constructor */
    BezierCurve(const std::vector<TYPE> &waypoints,
                const std::vector<float> &durations) {

        N = waypoints.size();

        assert(durations.size() == N - 1);
        assert(N > 2);

        ST = durations;
        PR = std::vector<float>(N, 1.);
        PR[0] = ST[1] / ST[0];

        float t[3] = { 0, ST[0], ST[0] + ST[1] };
        for (int i = 1; i < N-2; i++) {
            PR[i] = (t[2] - t[1]) / (t[1] - t[0]);
            t[0] = t[1];
            t[1] = t[2];
            t[2] = t[2] + ST[i+1];
        }

        std::vector<BezierNode<TYPE> > K;
        for (int i = 0; i < N; i++) {
            BezierNode<TYPE> node(waypoints[i]);
            K.push_back(node);
        }

        GenerateControlPoints(K);
    }

    BezierCurve(const std::vector<TYPE> &waypoints,
                const float segmentDuration=1.) {

        N = waypoints.size();
        assert(N > 2);

        ST = std::vector<float>(N-1, segmentDuration);

        std::vector<BezierNode<TYPE> > K;
        for (int i = 0; i < N; i++) {
            BezierNode<TYPE> node(waypoints[i]);
            K.push_back(node);
        }

        PR = std::vector<float>(N, 1.);
        GenerateControlPoints(K);
    }

    Vector3f positionAtTime(const float time) const {
        unsigned long index = 0;
        return evaluate(C, 6, time, index).value();
    }

    Vector3f velocityAtTime(const float time) const {
        unsigned long index = 0;
        Vector3f v = evaluate(V, 5, time, index).value();
        return  v * (5. / ST[index]);
    }

    Vector3f accelerationAtTime(const float time) const {
        unsigned long index = 0;
        Vector3f a = evaluate(A, 4, time, index).value();
        return a * (20. / (ST[index] * ST[index]));
    }

    Quaternionf quaternionAtTime(const float time) {
        unsigned long index = 0;
        return evaluate(C, 6, time, index).value();
    }

    Vector3f angularVelocityAtTime(const float time) const {
        unsigned long index = 0;
        Quaternionf dq = evaluate(V, 5, time, index).value();
        return Vector3f(dq.coeffs().head(3) * 2. * 5. / ST[index]);
    }
};

#endif /* defined(__MSCKF__BezierCurve__) */
