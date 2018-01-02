#!env python3

import argparse
import importlib.util
import json
import numpy as np
import os
import pickle as pkl
import scoop
import shutil
import sys

from deap import algorithms, base, creator, tools
from subprocess import call
from tempfile import TemporaryDirectory
from scoop import futures, shared

DEBUG_OUTPUT=False
MAX_SIM_TIME=1000

def import_tonav():
    if "pytonavsimulation" in globals():
        return
    pytonav_path_candidates = [
        "../build/simulation/pytonavsimulation.cpython-35m-x86_64-linux-gnu.so",
        "../build/simulation/Debug/pytonavsimulation.cpython-36m-darwin.so",
        "../build/simulation/pytonavsimulation.cpython-36m-darwin.so"
    ]
    for candidate in pytonav_path_candidates:
        if os.path.exists(candidate):
            pytonav_path = candidate
            break
    spec = importlib.util.spec_from_file_location("pytonavsimulation", pytonav_path)
    global pytonavsimulation
    pytonavsimulation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pytonavsimulation)

creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0))
creator.create("Individual", np.ndarray, fitness=creator.FitnessMin)

TONAV_CONFIG = {
  "helpers": [
    {
      "id": "kitti_loader",
      "type": "kitti_loader",
      "params": {
        "skip_frames": 0,
        "path": os.path.expanduser("~/kitti_data/2011_09_30/2011_09_30_drive_0034_sync")
      }
    }
  ],
  "odometry": {
    "type": "tonav",
    "params": {
      "load_path": "tonav_params.json"
    }
  }
}

sim_params = {
    "image_noise_variance": (1, -2, 3),
    "accelerometer_variance": (1, -8, 2),
    "gyroscope_variance": (1, -8, 2),
    "accelerometer_random_walk_variance": (1, -7, 1),
    "gyroscope_random_walk_variance": (1, -7, 1),
    "orientation_noise": (3, -7, 2),
    "position_noise": (3, -5, 2),
    "velocity_noise": (3, -6, 2),
    "gyroscope_bias_noise": (3, -8, -2),
    "accelerometer_bias_noise": (3, -6, -2),
    "gyroscope_acceleration_sensitivity_noise": ((3, 3), -8, -5),
    "gyroscope_shape_matrix_noise": ((3, 3), -7, -5),
    "accelerometer_shape_matrix_noise": ((3, 3), -6, -5),
    "position_of_body_in_camera_frame_noise": (3, -6, 1),
    "focal_length_noise": (2, -3, 2),
    "optical_center_noise": (2, -4, 1),
    "radial_distortion_noise": (3, -8, -5),
    "tangential_distortion_noise": (2, -8, -4),
    "camera_delay_time_noise": (1, -6, -2),
    "camera_readout_time_noise": (1, -6, -3)
}
sim_params_keys = sorted(sim_params.keys())

def tonav_params_from_individual(ind):
    params, ind_pos = {}, 0
    for key in sim_params_keys:
        item_shape, _, _ = sim_params[key]
        if isinstance(item_shape, tuple):
            item_len = item_shape[0]*item_shape[1]
        else:
            item_len = item_shape
        if item_shape == 1:
            params[key] = ind[ind_pos]
        else:
            params[key] = ind[ind_pos:(ind_pos+item_len)].reshape(item_shape)
        ind_pos += item_len
    return params


def individual_from_tonav_params(params):
    ind_list = []
    for key in sim_params_keys:
        ind_list.append(params[key].reshape(1, -1))
    ind = np.hstack(ind_list).reshape(-1)
    return creator.Individual(ind)


def gen_param(size, order_min, order_max):
    magnitude = np.random.randint(order_min, order_max)
    values = np.power(10.0, float(magnitude)) * np.random.random(size)
    return values


def create_individual(**kwargs):
    params = {k:gen_param(*sim_params[k]) for k in sim_params_keys}
    ind = individual_from_tonav_params(params)
    return ind


toolbox = base.Toolbox()
toolbox.register("map", scoop.futures.map)
toolbox.register("individual", create_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)


def mutateTonavInd(individual, indpb):
    for i in range(len(individual)):
        if np.random.rand() < indpb:
            change = np.random.exponential(1) + 1
            if np.random.rand() < 0.5:
                individual[i] *= change
            else:
                individual[i] /= change
    return individual, 


def evalTonavKittiSimulation(individual, headless=True):
    import_tonav()
    pytonavsimulation.DebugLogger.getInstance().set_output_file("tonav_output.json")
    params = tonav_params_from_individual(individual)
    params = {k:v.tolist() for k, v in params.items()}

    with TemporaryDirectory() as temp_dir:
        wdir = os.getcwd()
        os.chdir(temp_dir)
        with open("tonav_params.json", "w") as o:
            json.dump(params, o)
        with open("sim_setup_tonav_ga.json", "w") as o:
            json.dump(TONAV_CONFIG, o)
        sim_setup = pytonavsimulation.SimSetup("sim_setup_tonav_ga.json")
        sim = pytonavsimulation.VioSimulation()
        if headless:
            sim.set_headless()
        #max_sim_time = float(shared.getConst("max_sim_time"))
        max_sim_time = 20
        sim.set_simulation_length(max_sim_time)
        sim.run(sim_setup)
        pytonavsimulation.DebugLogger.getInstance().write_and_clear()
        del sim
        del sim_setup
        with open("gt_eval.json", "r") as o:
            gt_eval = json.load(o)
        os.chdir(wdir)
    ttd = 0
    keys = sorted(gt_eval.keys(), key=float)
    last_key = keys[-1]
    errors_pttd = []
    for i in range(1, len(keys)):
        prev_pos = np.asarray(gt_eval[keys[i-1]]["p_B_G"]["true"])
        curr_pos = np.asarray(gt_eval[keys[i]]["p_B_G"]["true"])
        ttd += np.linalg.norm(curr_pos - prev_pos)
        error_norm = gt_eval[keys[i]]["p_B_G"]["error_norm"]
        if error_norm is None:
            return 1e9, 1e9
        errors_pttd.append(error_norm/ttd)
    position_error = gt_eval[last_key]["p_B_G"]["error_norm"]
    mean_error_pttd = sum(errors_pttd)/len(errors_pttd)
    ret = mean_error_pttd, position_error
    if DEBUG_OUTPUT:
        scoop.logger.info("Function evaluated: {:.6f}, {:.2f}".format(*ret))
    return ret

def cxTwoPointCopy(ind1, ind2):
    size = len(ind1)
    cxpoint1 = np.random.randint(1, size)
    cxpoint2 = np.random.randint(1, size - 1)
    if cxpoint2 >= cxpoint1:
        cxpoint2 += 1
    else: # Swap the two cx points
        cxpoint1, cxpoint2 = cxpoint2, cxpoint1

    ind1[cxpoint1:cxpoint2], ind2[cxpoint1:cxpoint2] \
        = ind2[cxpoint1:cxpoint2].copy(), ind1[cxpoint1:cxpoint2].copy()
        
    return ind1, ind2
    

NGEN = 300
FREQ = 1
CXPB = 0.3
MUTPB = 0.6
INDPB = 0.04
    
toolbox.register("evaluate", evalTonavKittiSimulation)
toolbox.register("mate", cxTwoPointCopy)
toolbox.register("mutate", mutateTonavInd, indpb=INDPB)
toolbox.register("select", tools.selTournament, tournsize=3)


def main(args):
    globals()["DEBUG_OUTPUT"] = args.debug
    #shared.setConst(max_sim_time=args.max_sim_time)
    checkpoint = args.checkpoint
    if checkpoint and os.path.exists(checkpoint):
        # A file name has been given, then load the data from the file
        with open(checkpoint, "rb") as cp_file:
            cp = pkl.load(cp_file)
        population = cp["population"]
        for i in range(len(population)):
            del population[i].fitness.values
        start_gen = cp["generation"]
        halloffame = cp["halloffame"]
        logbook = cp["logbook"]
        np.random.set_state(cp["rndstate"])
    else:
        # Start a new evolution
        population = toolbox.population(n=args.population_size)
        start_gen = 0
        halloffame = tools.HallOfFame(maxsize=15, similar=np.array_equal)
        logbook = tools.Logbook()

    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("min", lambda l: min(map(creator.FitnessMin, l)).values)
    stats.register("max", lambda l: max(map(creator.FitnessMin, l)).values)

    for gen in range(start_gen, NGEN):
        offspring = [toolbox.clone(ind) for ind in population]

        # Apply crossover and mutation on the offspring
        for i in range(1, len(offspring), 2):
            if np.random.rand() < CXPB:
                offspring[i - 1], offspring[i] = toolbox.mate(offspring[i - 1], offspring[i])
                del offspring[i - 1].fitness.values, offspring[i].fitness.values

        for i in range(len(offspring)):
            if np.random.rand() < MUTPB:
                offspring[i], = toolbox.mutate(offspring[i])
                del offspring[i].fitness.values

        population = offspring

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in population if not ind.fitness.valid]
                
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        halloffame.update(population)
        population = tools.selBest(population + halloffame.items[:5], len(population))
        record = stats.compile(population)
        logbook.record(gen=gen, evals=len(invalid_ind), **record)
        print(logbook.stream)

        if gen % FREQ == 0:
            # Fill the dictionary using the dict(key=value[, ...]) constructor
            cp = dict(population=population, generation=gen, halloffame=halloffame,
                      logbook=logbook, rndstate=np.random.get_state())

            with open(checkpoint, "wb") as cp_file:
                pkl.dump(cp, cp_file)


def clear_precomputed_features():
    call(["find", os.path.expanduser("~/kitti_data"), "-name", "'*.opencv'", "-delete"])

def recompile_tonav():
    if os.path.exists("../build"):
        shutil.rmtree("../build")
    os.mkdir("../build")
    os.chdir("../build")
    call(["cmake", "-DCMAKE_BUILD_TYPE=Release", ".."])
    call(["make", "-j", "pytonavsimulation"])
    os.chdir("../simulation")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Genetic algortihm for Tonav calibration optimization.')
    parser.add_argument('--should-optimize', action='store_true', default=False, dest="should_optimize", help="If present, optimization will run.")
    parser.add_argument('--checkpoint', default="checkpoint.pkl", help="Checkpoint location (default: checkpoint.pkl).")
    parser.add_argument('--population-size', metavar="N", dest="population_size", default=150, type=int, help="Population size (default: 150).")
    parser.add_argument("--recompile", dest="recompile", action="store_true", default=False, help="Should compile Tonav (default: False).")
    parser.add_argument("--debug", dest="debug", action="store_true", default=False, help="Output debugging information (default: False).")
    parser.add_argument("--max-sim-time", dest="max_sim_time", default=1000, help="Maximum simulation time (default: 1000).")

    args = parser.parse_args()
    if args.should_optimize:
        if args.recompile:
            recompile_tonav()
            #clear_precomputed_features()
        import_tonav()
        main(args)
    else:
        import_tonav()
