#!/usr/bin/env python
#
# AMZ-Driverless
# Copyright (c) 2018 Authors:
#   - Juraj Kabzan <kabzanj@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# System Includes

# ROS Include
import rospkg

# Arguments
import argparse

# YAML
import yaml

# Python include
import time
import os

# Process
from fssim.shell import *


class Launcher:

    def __init__(self, args):
        with open(args.config, 'r') as f:
            self.config = yaml.load(f)
        self.args = args

    def setup_report_file(self):
        report_yaml = {"title": self.config["simulation_name"],
                       "number_repetitions": len(self.config["repetitions"]),
                       "repetitions": {}}

        if not os.path.isdir(self.args.output):
            os.makedirs(self.args.output)
            
        self.report_file = self.args.output + '/output.yaml'
        self.fst_report_file = self.args.output + '/fst_output.yaml'

        with open(self.report_file, 'w+') as yamlfile, \
            open(self.fst_report_file, 'w+') as fst_yamlfile:
            yaml.safe_dump(report_yaml, yamlfile, default_flow_style = False)  # Also note the safe_dump
            yaml.safe_dump(report_yaml, fst_yamlfile)  # Also note the safe_dump

    def start(self):
        self.setup_report_file()

        for i, settings in enumerate(self.config['repetitions']):
            print "STARITNG REPETITION: ", i
            path = rospkg.RosPack().get_path("fssim") + "/scripts/automated_res.py"
            launching_script = "python {} --config {} --id {} --output {} ".format(path, self.args.config, i, self.args.output)
            print launching_script
            self.fssim_cmd = Command(launching_script)
            self.fssim_cmd.run()
            
            time.sleep(5.0)
            path = rospkg.RosPack().get_path("fst_interface") + "/scripts/auto_test_handle.py"
            launching_script = "python {} --config {} --id {} --output {} ".format(path, self.args.config, i, self.args.output)
            self.auto_test_handle = Command(launching_script)
            self.auto_test_handle.run()
            self.fssim_cmd.join()
            self.auto_test_handle.ensure_terminated()
            time.sleep(5.0)
        

        print "EXITING LAUNCHER SCRIPT"
        sys.exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Process some integers.')
    parser.add_argument("--config", dest = "config", metavar = "FILE", help = "Config YAML file")
    parser.add_argument("--output", dest = "output", help = "Output YAML file")
    args = parser.parse_args()

    print '\033[94m' + " Welcome to:                         " + '\033[0m'
    print '\033[94m' + "             _________________       " + '\033[0m'
    print '\033[94m' + "            / ____/ ___/_  __/       " + '\033[0m'
    print '\033[94m' + "           / /_   \__ \ / /          " + '\033[0m'   
    print '\033[94m' + "          / __/  ___/ // /           " + '\033[0m'
    print '\033[94m' + "         /_/    /____//_/            " + '\033[0m'
    print '\033[94m' + "      ________________ ______  ___   " + '\033[0m'
    print '\033[94m' + "     / ____/ ___/ ___//  _/  |/  /   " + '\033[0m'
    print '\033[94m' + "    / /_   \__\ \__ \ / // /|_/ /    " + '\033[0m'
    print '\033[94m' + "   / __/  ___/ ___/ _/ // /  / /     " + '\033[0m'
    print '\033[94m' + "  /_/    /____/____/___/_/  /_/      " + '\033[0m'   
    print '\033[94m' + "                                     " + '\033[0m'

    fssim = Launcher(args)
    fssim.start()