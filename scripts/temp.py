from __future__ import division
import os
import argparse
import subprocess as sp
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np
import lvdb


def _single_run(mission_file):
    sp.call(['scrimmage', mission_file])
    runtime_file = \
        os.path.expanduser('~/.scrimmage/logs/latest/runtime_seconds.txt')

    with open(runtime_file, 'r') as f:
        data = f.read().splitlines()
        actual = float(data[0].split(':')[1])
        sim = float(data[1].split(':')[1])

    return sim / actual


def _do_runs(repeats, mission_file):
    avg_ratio = \
        sum([_single_run(mission_file) for _ in range(repeats)]) / repeats

    return avg_ratio


def _prep_mission(count, mission_file):
    tree = ET.parse(mission_file)
    root = tree.getroot()

    run_node = root.find('run')
    run_node.attrib['enable_gui'] = "False"
    run_node.attrib['time_warp'] = "0"

    entity_node = next((n for n in root.findall('entity')
                        if n.find('autonomy').text == 'Boids'))
    count_node = entity_node.find('count')
    count_node.text = str(count)
    tree.write(mission_file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mission_file')
    parser.add_argument('repeats', type=int)
    args = parser.parse_args()

    lvdb.set_trace()
    counts = range(100, 2001, 100)
    data = np.empty((len(counts), 2))
    for i, count in enumerate(counts):
        _prep_mission(count, args.mission_file)

        data[i][0] = count
        data[i][1] = _do_runs(args.repeats, args.mission_file)

    plt.plot(data[:, 0], data[:, 1])
    plt.show()


if __name__ == '__main__':
    main()
