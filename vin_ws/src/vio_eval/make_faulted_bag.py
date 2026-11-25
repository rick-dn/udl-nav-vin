#!/usr/bin/env python3
"""
make_faulted_bag.py - Writes a copy of a bag with faults applied

Reads every message of a recorded bag in order, runs the sensor topics through
FaultEngine (the same fault code as fault_injector.py) and writes the result
to a new bag. Nothing depends on timing, so the same bag and scenario always
give exactly the same output, and replaying it is as repeatable as replaying
the original.

Ground truth, camera_info and MAVROS topics are copied unchanged. GNSS delay
is applied by writing the message later in the bag; its stamp is kept.

The faulted bag also holds faults.csv (when each fault was active) and
fault_spec.json (the scenario with includes resolved). If the output already
exists and was made from the same scenario, it's reused.

Usage: python3 make_faulted_bag.py <bag_dir> <scenario.yaml> <out_dir> [--force]
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

from fault_injector import FaultEngine, HANDLERS, load_scenario


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader


def main():
    parser = argparse.ArgumentParser(description='Write a faulted copy of a bag')
    parser.add_argument('bag', help='input bag folder')
    parser.add_argument('scenario', help='fault scenario YAML')
    parser.add_argument('out', help='output bag folder')
    parser.add_argument('--force', action='store_true', help='rebuild even if the output is up to date')
    args = parser.parse_args()

    bag, out = Path(args.bag), Path(args.out)
    try:
        seed, specs = load_scenario(args.scenario)
    except (ValueError, OSError) as e:
        sys.exit(f'bad scenario: {e}')
    spec = json.dumps({'source_bag': bag.resolve().name, 'seed': seed, 'faults': specs},
                      indent=2, sort_keys=True, default=str)

    if out.exists():
        old = out / 'fault_spec.json'
        if not args.force and old.exists() and old.read_text() == spec:
            print(f'up to date: {out}')
            return
        shutil.rmtree(out)

    reader = open_reader(bag)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for meta in reader.get_all_topics_and_types():
        writer.create_topic(meta)

    now_ns = [0]   # receive time of the message being processed

    def emit(topic, msg, release_t):
        ts = now_ns[0] if release_t is None else max(now_ns[0], int(round(release_t * 1e9)))
        writer.write(topic, serialize_message(msg), ts)

    log_path = out.parent / f'.{out.name}.faults.csv'
    engine = FaultEngine(seed, specs, str(log_path), emit)
    count = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        now_ns[0] = ts
        if topic in HANDLERS:
            method, typ = HANDLERS[topic]
            msg = deserialize_message(data, typ)
            if topic == '/mavros/state':
                writer.write(topic, data, ts)        # copied as is, and seen by the engine
            if topic == '/mavros/state':
                engine.on_state(msg)
            else:
                getattr(engine, method)(msg, topic)
        else:
            writer.write(topic, data, ts)
        count += 1
    engine.close()
    del writer                                        # closes and writes metadata

    shutil.move(str(log_path), out / 'faults.csv')
    (out / 'fault_spec.json').write_text(spec)
    print(f'wrote {out} ({count} messages in)')


if __name__ == '__main__':
    main()
