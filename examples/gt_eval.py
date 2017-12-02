#!env python3

import json
import matplotlib.pyplot as plt
from math import ceil

def main():
    with open("gt_eval.json", "r") as o:
        data = { float(k):v for k,v in json.load(o).items() }
    keys = sorted(data.keys())

    items = data[list(data.keys())[0]].keys()
    fig, axs = plt.subplots(int(ceil(len(items)/2)), 2, sharex=True, figsize=(12, 7.5))
    for i, item in enumerate(items):
        error_norm = [data[k][item]["error_norm"] for k in keys]
        col = 0 if i % 2 == 0 else 1
        row = int(i/2) if i % 2 == 0 else int((i-1)/2)
        axs[row, col].plot(keys, error_norm)
        axs[row, col].set_yscale('log')
        axs[row, col].set_ylabel(item)

    fig.tight_layout()
    plt.show()

    print(data)

if __name__ == '__main__':
    main()
