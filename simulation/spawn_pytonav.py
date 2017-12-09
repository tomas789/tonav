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
    np.random.seed(process_rank)
    experiment_dir = "experiment_{}".format(process_rank)
    if not os.path.exists(experiment_dir):
        os.mkdir(experiment_dir)
        os.mkdir(os.path.join(experiment_dir, 'results'))
    os.chdir(experiment_dir)

    items = [int(d.split("_")[2]) for d in os.listdir("results") if d.startswith("gt_eval_")]
    if len(items) == 0:
        i = -1
    else:
        i = max(items) - 1

    while True:
        i += 1

        sim_params = {
            "image_noise_variance": gen_param(1, 0, 2),
            "accelerometer_variance": gen_param(1, -8, 6),
            "gyroscope_variance": gen_param(1, -6, -5),
            "accelerometer_random_walk_variance": gen_param(1, -7, -5),
            "gyroscope_random_walk_variance": gen_param(1, -7, 5),
            "orientation_noise": gen_param(3, -7, -5),
            "position_noise": gen_param(3, -5, -2),
            "velocity_noise": gen_param(3, -6, -2),
            "gyroscope_bias_noise": gen_param(3, -8, -5),
            "accelerometer_bias_noise": gen_param(3, -6, -4),
            "gyroscope_acceleration_sensitivity_noise": gen_param((3, 3), -8, -5),
            "gyroscope_shape_matrix_noise": gen_param((3, 3), -7, -5),
            "accelerometer_shape_matrix_noise": gen_param((3, 3), -6, -5),
            "position_of_body_in_camera_frame_noise": gen_param(3, -6, -4),
            "focal_length_noise": gen_param(2, -3, 2),
            "optical_center_noise": gen_param(2, -4, 1),
            "radial_distortion_noise": gen_param(3, -8, -5),
            "tangential_distortion_noise": gen_param(2, -8, -4),
            "camera_delay_time_noise": gen_param(1, -6, -2),
            "camera_readout_time_noise": gen_param(1, -6, -3)
        }

        pytonavsimulation.DebugLogger.getInstance().set_output_file("results/tonav_output_{}.json".format(i))
        print(os.path.exists("../../examples/sim_setup_tonav.json"))
        sim_setup = pytonavsimulation.SimSetup("../../examples/sim_setup_tonav.json")
        calibration = sim_setup.getOdometry().get_tonav_calibration()
        for k, v in sim_params.items():
            setattr(calibration, k, v)

        with open('results/tonav_params_{}.json'.format(i), 'w') as o:
            json.dump({k:(v[0] if len(v) == 1 else v.tolist()) for k, v in sim_params.items()}, o, indent=4, sort_keys=True)

        sim = pytonavsimulation.VioSimulation()
        sim.set_headless()
        sim.set_simulation_length(100)
        try:
            sim.run(sim_setup)
        except:
            print("Worker {} crashed.".format(i))
            return
        
        pytonavsimulation.DebugLogger.getInstance().write_and_clear()
        del sim
        del sim_setup
        del calibration
        with open("gt_eval.json", "r") as o:
            gt_eval = json.load(o)
        last_key = sorted(gt_eval.keys())[-1]
        print("last_key: {}".format(last_key))
        position_error = gt_eval[last_key]["p_B_G"]["error_norm"]
        os.rename("gt_eval.json", "results/gt_eval_{}.json".format(i))
        if position_error >= 1:
            continue
    pytonavsimulation.DebugLogger.getInstance().set_output_file("tonav_output_empty.json")
    return 0


if __name__ == '__main__':
    count = multiprocessing.cpu_count()
    pool = multiprocessing.Pool(processes=count)
    print(pool.map(work, list(range(count))))
