"""Provide OpenAI interface for SCRIMMAGE."""
from __future__ import print_function
import threading
import warnings
import subprocess
import collections
import time
import os
import signal
import sys
import xml.etree.ElementTree as ET
import importlib
import argparse
import platform

from concurrent import futures
import numpy as np
import grpc

import gym
import gym.spaces
from gym.utils import seeding
import lvdb
if __name__ == "__main__":
    from scrimmage.proto import ExternalControl_pb2, ExternalControl_pb2_grpc
else:
    from .proto import ExternalControl_pb2, ExternalControl_pb2_grpc

if sys.version[0] == '2':
    import Queue as queue
else:
    import queue as queue


class ServerThread(threading.Thread):
    """Start SCRIMMAGE ExternalControl GRPC Service."""

    def __init__(self, queues, address, max_workers=10):
        """Initialize variables."""
        super(ServerThread, self).__init__()
        self.queues = queues
        self.address = address
        self.max_workers = max_workers
        self.stop = False

    def run(self):
        """Start SCRIMMAGE ExternalControl GRPC Service."""
        server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=self.max_workers))

        ExternalControl_pb2_grpc.add_ExternalControlServicer_to_server(
            ExternalControl(self.queues), server)
        server.add_insecure_port(self.address)
        server.start()

        try:
            while not self.stop:
                time.sleep(1)
            server.stop(0)
        except KeyboardInterrupt:
            server.stop(0)


class ScrimmageEnv(gym.Env):
    """OpenAI implementation for SCRIMMAGE."""

    metadata = {'render.modes': ['human']}

    def __init__(self, enable_gui, mission_file,
                 combine_actors=True,
                 num_actors=1, port_offset=1,
                 address="localhost:50051", gdb_args=""):
        """Create queues for multi-threading."""
        self.enable_gui = enable_gui
        self.mission_file = mission_file
        self.combine_actors = combine_actors
        self.address = address
        self.gdb_args = gdb_args
        self.num_actors = num_actors
        self.port_offset = port_offset

        self.rng = None
        self.seed = self._seed(None)

        queue_names = ['env', 'action', 'action_response']

        ip, port = address.split(":")
        self.queues = []
        port = int(port)
        address = ip + ":" + str(port)
        self.queues = {s: queue.Queue() for s in queue_names}
        self.server_thread = ServerThread(self.queues, address)
        self.server_thread.start()

        # startup headless version of scrimmage to get the environment
        self.scrimmage_process = self._start_scrimmage(False, True)

        envs = self.queues['env'].get()
        self.num_actors = len(envs.envs)

        self._clear_queues()
        self._terminate_scrimmage()
        self._clear_queues()

        if self.num_actors == 1:
            self.action_space, self.observation_space, self.reward_range = \
                self._create_spaces(envs.envs[0])
        else:
            if self.combine_actors:
                self._env = ExternalControl_pb2.Environment()
                for e in envs.envs:
                    for p in e.action_spaces.params:
                        self._env.action_spaces.params.extend([p])
                    for p in e.observation_spaces.params:
                        self._env.observation_spaces.params.extend([p])
                self.reward_range = \
                    (sum([e.min_reward for e in envs.envs]),
                     sum([e.max_reward for e in envs.envs]))
                self.action_space, self.observation_space = \
                    self._create_spaces(self._env)[:2]
            else:
                spaces = [self._create_spaces(e) for e in envs.envs]
                action_space, observation_space, reward_range = zip(*spaces)
                self.action_space = gym.spaces.Tuple(action_space)
                self.observation_space = gym.spaces.Tuple(observation_space)
                min_rewards, max_rewards = zip(*reward_range)
                self.reward_range = (min(min_rewards), max(max_rewards))

        signal.signal(signal.SIGINT, self._signal_handler)

    def _create_spaces(self, environment):
        try:
            action_space = \
                _create_tuple_space(environment.action_spaces)
            observation_space = \
                _create_tuple_space(environment.observation_spaces)
            reward_range = (environment.min_reward, environment.max_reward)
            return action_space, observation_space, reward_range
        except AssertionError:
            print(('scrimmage external_control.py: calling terminate '
                   'from __init__ due to env problem'))
            self.close()
            raise

    def _signal_handler(self, signum, frame):
        """Exit cleanly <ctrl-c> (i.e., kill the subprocesses)."""
        print("scrimmage external_control.py: exiting scrimmage from sigint")
        self.close()
        sys.exit(0)

    def _reset(self):
        """Restart scrimmage and return result."""
        self._clear_queues()
        self._terminate_scrimmage()
        self._clear_queues()

        self.scrimmage_process = \
            self._start_scrimmage(self.enable_gui, False)

        return self._return_action_result()[0]

    def _step(self, action):
        """Send action to SCRIMMAGE and return result."""
        actions_pb = ExternalControl_pb2.Actions()

        def _add_action(space, a):
            if not isinstance(a, collections.Iterable):
                a = [a]

            if isinstance(space, gym.spaces.Box):
                action_pb = \
                    ExternalControl_pb2.Action(continuous=a, done=False)
            else:
                action_pb = \
                    ExternalControl_pb2.Action(discrete=a, done=False)

            actions_pb.actions.extend([action_pb])

        if self.num_actors == 1:
            _add_action(self.action_space, action)
        elif self.combine_actors:
            for a in action:
                _add_action(self.action_space, a)
        else:
            for i, a in enumerate(action):
                _add_action(self.action_space.spaces[i], a)

        self.queues['action'].put(actions_pb)
        return self._return_action_result()

    def _render(self, mode='human', close=False):
        """Ignores a render call but avoids an exception.

        If a user wants the environment rendered then the user should
        pass enable_gui=True as a kwarg
        """
        pass

    def _seed(self, seed=None):
        self.rng, seed = seeding.np_random(seed)
        return [seed]

    def _clear_queues(self):
        for key in self.queues:
            _clear_queue(self.queues[key])

    def _start_scrimmage(self, enable_gui, disable_output):

        ip, port = self.address.split(":")
        tree = ET.parse(self.mission_file)
        root = tree.getroot()

        # set the seed using this class' random number generator
        seed_node = root.find('seed')
        if seed_node is None:
            root.append(ET.Element("seed"))
            seed_node = root.find('seed')
        seed_node.text = str(self.rng.randint(0, 2**32 - 1))

        for nd in root.findall('entity_interaction'):
            if nd.text == 'ExternalControlInteraction':
                nd.attrib['server_address'] = self.address
                break

        # enable gui
        run_node = root.find('run')
        run_node.attrib['enable_gui'] = str(enable_gui)
        if not bool(enable_gui):
            run_node.attrib['time_warp'] = "0"

        # disable output
        if disable_output:
            output_node = root.find("output_type")
            output_node.text = ""

        self.temp_mission_file = \
            "." + port + platform.node() + os.path.basename(self.mission_file)

        # print('temp mission file is ' + self.temp_mission_file)
        tree.write(self.temp_mission_file)
        if self.gdb_args:
            cmd = self.gdb_args.split(" ") + \
                ["scrimmage", self.temp_mission_file]
        else:
            cmd = ["scrimmage", self.temp_mission_file]
        cmd = ["scrimmage", self.temp_mission_file]
        # print(cmd)
        return subprocess.Popen(cmd)

    def _close(self):
        """Cleanup spawned threads and subprocesses.

        The thread manages a GRPC server to communicate with scrimmage.  The
        subprocess is the actual scrimmage instance.  This method needs to be
        called in order to make sure a python instance exits cleanly.
        """
        self._terminate_scrimmage()
        self.server_thread.stop = True

    def _return_action_result(self):
        info = {}
        try:
            res = self.queues['action_response'].get(timeout=6000)
        except queue.Empty:
            print('Scrimmage Environment: error getting action result')
            res = ExternalControl_pb2.ActionResults(done=True)
            info['scrimmage_err'] = ""

        try:
            res.action_results
        except AttributeError:
            print('returning error')
            return None, None, True, {}

        if self.num_actors == 1:
            obs = np.array(res.action_results[0].observations.value)
            rew = res.action_results[0].reward
            done = res.action_results[0].done
        else:
            rew = [r.reward for r in res.action_results]
            done = [r.done for r in res.action_results]

            if self.combine_actors:
                obs = np.array([v for r in res.action_results
                                for v in r.observations.value])
                rew = sum(rew)
                done = any(done)
            else:
                obs = [np.array(r.observations.value)
                       for r in res.action_results]


        return obs, rew, done, {}

    def _terminate_scrimmage(self):
        """Terminates scrimmage instance held by the class.

        given how sigints are handled by scrimmage, we need to
        shutdown the network to the autonomy in addition to sending a
        sigint.
        """
        self.scrimmage_process.poll()
        if self.scrimmage_process.returncode is None:
            try:
                os.remove(self.temp_mission_file)
            except OSError:
                pass

            self.queues['action'].put(ExternalControl_pb2.Actions(done=True))

            try:
                self.scrimmage_process.terminate()
                self.scrimmage_process.poll()
                while self.scrimmage_process.returncode is None:
                    self.scrimmage_process.poll()
                    self.queues['action'].put(ExternalControl_pb2.Actions(done=True))
                    time.sleep(0.1)
            except OSError:
                print('could not terminate existing scrimmage process. '
                      'It may have already shutdown.')


class ExternalControl(ExternalControl_pb2_grpc.ExternalControlServicer):
    """GRPC Service to communicate with SCRIMMAGE Autonomy."""

    def __init__(self, queues):
        """Receive queues for multi-threading."""
        self.queues = queues

    def SendEnvironments(self, env, context):
        """Receive Environment proto and send back an action."""
        self.queues['env'].put(env)
        return ExternalControl_pb2.Empty()

    def SendActionResults(self, action_results, context):
        """Receive ActionResult proto and send back an action."""
        self.queues['action_response'].put(action_results)
        if not any([a.done for a in action_results.action_results]):
            try:
                action = self.queues['action'].get(timeout=10000.0)
            except queue.Empty:
                action = ExternalControl_pb2.Action(done=True)
                res = ExternalControl_pb2.ActionResult(done=True)
        else:
            action = ExternalControl_pb2.Action(done=True)
            res = ExternalControl_pb2.ActionResult(done=True)
            self.queues['action_response'].put(res)
        return action


def _create_tuple_space(space_params):
    discrete_extrema = []
    continuous_extrema = []

    def _append(param, dst_lst):
        if param.num_dims != 1 and len(param.maximum) == 1:
            # use same min/max for all dims
            dst_lst += param.num_dims * [[param.minimum[0], param.maximum[0]]]
        else:
            # each min/max is specified individually
            assert len(param.minimum) == len(param.maximum)
            dst_lst += zip(list(param.minimum), list(param.maximum))

    for param in space_params.params:
        if param.discrete:
            _append(param, discrete_extrema)
        else:
            _append(param, continuous_extrema)

    # make sure that discrete entries are ints
    discrete_extrema = [(int(mn), int(mx)) for mn, mx in discrete_extrema]

    if len(discrete_extrema) == 1 and discrete_extrema[0][0] == 0:
        discrete_space = gym.spaces.Discrete(discrete_extrema[0][1] + 1)
    else:
        discrete_extrema = [mx - mn + 1 for mn, mx in discrete_extrema]
        discrete_space = gym.spaces.MultiDiscrete(discrete_extrema)

    if continuous_extrema:
        low, high = zip(*continuous_extrema)
        continuous_space = gym.spaces.Box(np.array(low), np.array(high))

    if discrete_extrema and continuous_extrema:
        return gym.spaces.Tuple((discrete_space, continuous_space))
    elif discrete_extrema:
        return discrete_space
    elif continuous_extrema:
        return continuous_space
    else:
        raise NotImplementedError(
            "received a space with no discrete or continuous components")


def _clear_queue(q):
    try:
        while True:
            q.get(False)
    except queue.Empty:
        pass


def main():
    # Get params file, model:class, and port num from args list
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('--xml', default="/tmp/p", help='Parameters file path')
    parser.add_argument('--actor', type=str,
                        help='what actor to use for acting. '
                        'Input in the form module:Class (e.g., '
                        'load_model:NNModel) and should have the same basic '
                        'methods/properties as load_model:NNModel')
    parser.add_argument("--port-num", type=int, default=50052,
                        help="port to start scrimmage interface on")
    args = parser.parse_args()
    # Initizialized variables
    num_actors = 1
    port_offset = 1
    ip = "localhost"
    queues = []
    server_threads = []

    port = args.port_num
    # Starting up GRPC link
    for i in range(num_actors):
        port = int(port) + i * port_offset
        address = ip + ":" + str(port)
        queues.append(
            {s: queue.Queue() for s in ['env', 'action', 'action_response']})
        server_threads.append(
            ServerThread(queues[-1], address))
        server_threads[-1].start()
    environments = \
        [queues[i]['env'].get() for i in range(num_actors)]
    if len(environments) == 1:
        action_space = _create_tuple_space(environments[0].action_spaces)
        observation_space = \
            _create_tuple_space(environments[0].observation_spaces)
    else:
        action_space = [_create_tuple_space(e.action_spaces)for e in
                        environments]
        observation_space = [_create_tuple_space(e.observation_spaces) for e in
                             environments]
        action_space = gym.spaces.Tuple(action_space)
        observation_space = gym.spaces.Tuple(observation_space)
    # Set up actor model
    actor_module_str, actor_class_str = args.actor.split(":")
    actor_module = importlib.import_module(actor_module_str)
    actor_class = getattr(actor_module, actor_class_str)

    # Actor initialization
    actor = actor_class(action_space, observation_space, args.xml)
    while True:
        # get state from GRPC link
        try:
            res = queues[0]['action_response'].get(timeout=6000)
        except queue.Empty:
            print('Not receiving sensor info from scrimmage')
            res = ExternalControl_pb2.ActionResult(done=True)
        if res.done:
            print("Received end of simulation")
            for s in server_threads:
                s.stop = True
            break
        obs = np.array(res.observations.value)
        # get action from actor
        action = actor.act(obs)
        # return action to GRPC link

        def _step_single(i, space, a):
            if not isinstance(a, collections.Iterable):
                a = [a]

            if isinstance(action_space, gym.spaces.Box):
                action_pb =\
                    ExternalControl_pb2.Action(continuous=a, done=False)
            else:
                action_pb =\
                    ExternalControl_pb2.Action(discrete=a, done=False)

            queues[i]['action'].put(action_pb)

        if num_actors == 1:
            _step_single(0, action_space, action)
        else:
            for i, a in enumerate(action):
                _step_single(i, action_space.spaces[i], a)


if __name__ == "__main__":
    main()
