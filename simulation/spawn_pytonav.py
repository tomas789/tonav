#!/usr/bin/python3

import multiprocessing
import subprocess
import importlib.util
import numpy as np
import json
import os
import shutil

pytonav_path_candidates = [
    "../build/simulation/pytonavsimulation.cpython-35m-x86_64-linux-gnu.so",
    "../build/simulation/Debug/pytonavsimulation.cpython-36m-darwin.so"
]
for candidate in pytonav_path_candidates:
    if os.path.exists(candidate):
        pytonav_path = candidate
        break
spec = importlib.util.spec_from_file_location("pytonavsimulation", pytonav_path)
pytonavsimulation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pytonavsimulation)

def gen_param(size, order_min, order_max):
    magnitude = np.random.randint(order_min, order_max)
    values = np.power(10.0, float(magnitude)) * np.random.random(size)
    return values


def work(process_rank):
    print(process_rank)
    experiment_dir = "experiment_{}".format(process_rank)
    if os.path.exists(experiment_dir):
        shutil.rmtree(experiment_dir)
    os.mkdir(experiment_dir)
    os.mkdir(os.path.join(experiment_dir, 'results'))
    os.chdir(experiment_dir)

    i = -1
    while True:
        i += 1

        sim_params = {
            "image_noise_variance": gen_param(1, -2, 3),
            "accelerometer_variance": gen_param(1, -6, 2),
            "gyroscope_variance": gen_param(1, -6, 2),
            "accelerometer_random_walk_variance": gen_param(1, -6, 2),
            "gyroscope_random_walk_variance": gen_param(1, -6, 2),
            "orientation_noise": gen_param(3, -6, 0),
            "position_noise": gen_param(3, -6, 0),
            "velocity_noise": gen_param(3, -6, 0),
            "gyroscope_bias_noise": gen_param(3, -6, 2),
            "accelerometer_bias_noise": gen_param(3, -6, 2),
            "gyroscope_acceleration_sensitivity_noise": gen_param((3, 3), -6, -3),
            "gyroscope_shape_matrix_noise": gen_param((3, 3), -6, -3),
            "accelerometer_shape_matrix_noise": gen_param((3, 3), -6, -3),
            "position_of_body_in_camera_frame_noise": gen_param(3, -6, 0),
            "focal_length_noise": gen_param(2, -3, 2),
            "optical_center_noise": gen_param(2, -3, 2),
            "radial_distortion_noise": gen_param(3, -6, -2),
            "tangential_distortion_noise": gen_param(2, -6, -2),
            "camera_delay_time_noise": gen_param(1, -6, -2),
            "camera_readout_time_noise": gen_param(1, -6, -2)
        }

        for end_time in [1, 2, 8, 20]:
            output_path = "results/tonav_output_{}_{}.json".format(i, end_time)
            pytonavsimulation.DebugLogger.getInstance().set_output_file(output_path)
            sim_setup = pytonavsimulation.SimSetup("../../examples/sim_setup_tonav.json")
            calibration = sim_setup.getOdometry().get_tonav_calibration()
            for k, v in sim_params.items():
                setattr(calibration, k, v)

            sim = pytonavsimulation.VioSimulation()
            sim.set_headless()
            sim.set_simulation_length(end_time)
            sim.run(sim_setup)
            with open('results/tonav_params_{}_{}.json'.format(i, end_time), 'w') as o:
                json.dump({k:(v[0] if len(v) == 1 else v.tolist()) for k, v in sim_params.items()}, o, indent=4, sort_keys=True)
            pytonavsimulation.DebugLogger.getInstance().write_and_clear()
            os.remove(output_path)
            del sim
            del sim_setup
            del calibration
            with open("gt_eval.json", "r") as o:
                gt_eval = json.load(o)
            last_key = sorted(gt_eval.keys())[-1]
            print("last_key: {}".format(last_key))
            position_error = gt_eval[last_key]["p_B_G"]["error_norm"]
            os.rename("gt_eval.json", "results/gt_eval_{}_{}.json".format(i, end_time))
            if position_error >= 1:
                break
    pytonavsimulation.DebugLogger.getInstance().set_output_file("tonav_output_empty.json")
    return 0


if __name__ == '__main__':
    count = multiprocessing.cpu_count()
    pool = multiprocessing.Pool(processes=count)
    print(pool.map(work, list(range(count))))
