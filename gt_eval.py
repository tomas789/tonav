import sys
import json
import numpy as np


def main():
    with open(sys.argv[1], "r") as o:
        gt_eval = json.load(o)

    times = sorted(gt_eval.keys())
    final_error = gt_eval[times[-1]]["p_B_G"]["error_norm"]
    ttd = 0.0
    for i in range(1, len(times)):
        p_Bprev_G = np.asarray(gt_eval[times[i-1]]["p_B_G"]["true"])
        p_Bcurr_G = np.asarray(gt_eval[times[i]]["p_B_G"]["true"])
        dist = np.linalg.norm(p_Bprev_G - p_Bcurr_G)
        ttd += dist
    error_pttd = final_error/ttd
    print("Final error: {:.4f} m.".format(final_error))
    print("Traveled distance: {:.4f} m.".format(ttd))
    print("Error: {:.3f} %TTD.".format(100*error_pttd))


if __name__ == "__main__":
    main()

