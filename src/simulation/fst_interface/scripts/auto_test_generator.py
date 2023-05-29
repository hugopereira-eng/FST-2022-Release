# YAML
import yaml
import os
from signal import signal, SIGINT
import subprocess
import re 
# Arguments
import argparse
import numpy as np


class AutoTestGenerator:

    def __init__(self, args):

        with open(args.config, 'r') as config_f:
            self.config = yaml.load(config_f)
        self.config['repetitions'] = []

        self.parameters = args.parameters
        for i in range(len(self.parameters)):
            self.parameters[i] = self.parameters[i].split()
        
        self.output = args.output
        self.all_combinations = args.all_combinations
         
        if not os.path.isdir(args.output):
            os.makedirs(self.output)
        
        self.fill_reps()
        with open(args.config, 'w') as config_f:
            noalias_dumper = yaml.dumper.SafeDumper
            noalias_dumper.ignore_aliases = lambda self, data: True
            yaml.dump(self.config, config_f, Dumper=noalias_dumper)

        config_f.close()
        

    def fill_reps (self):
        for mission in args.mission:
            if mission == "acceleration" or mission == "skidpad":
                track_list = [mission]
            else:
                track_list = args.track
            for track in track_list:
                if self.all_combinations:
                    self.generate_launch_files({}, "", len(self.parameters)-1, mission, track)
                else:
                    for parameter in self.parameters:
                        for param_value in self.my_arange(float(parameter[1]), float(parameter[2]) + float(parameter[3]), float(parameter[3])):
                            param_dict = {parameter[0]: str(param_value)}
                            launch_name = re.sub("/", "_", parameter[0]) + "_" + str(param_value)
                            self.generate_launch_file(param_dict, launch_name, mission, track)


    def generate_launch_files(self, param_dict, launch_name, depth, mission, track):
        if depth == -1:
            self.generate_launch_file(param_dict, launch_name, mission, track)
            return

        for param_value in self.my_arange(float(self.parameters[depth][1]), float(self.parameters[depth][2]) + float(self.parameters[depth][3]), float(self.parameters[depth][3])):
            param_dict[self.parameters[depth][0]] = str(param_value)
            iter_launch_name = launch_name + re.sub("/", "_", self.parameters[depth][0]) + "_" + str(param_value)
            self.generate_launch_files(param_dict, iter_launch_name, depth - 1, mission, track)

    def generate_launch_file(self, param_dict, launch_name, mission, track):
        filename = self.get_filename(track, mission)
        filename = filename + launch_name + ".launch"

        fo = open(filename, "w+")
        fo.write("<launch>")
        fo.write("<include file=\"$(find simulation_meta_pkg)/missions/simul_" + str(mission) + ".launch\"/>")
        for param_key, param_value in param_dict.iteritems():
            fo.write("<param name=\"" + str(param_key) + "\"" + " value=\"" + str(param_value) + "\"/>")
        self.config['repetitions'].append(self.get_rep(track, filename, param_dict.copy()))
        fo.write("</launch>")
        fo.close()

             
    def my_arange(self, start, end, step):
        return np.linspace(start, end, num=round((end-start)/step), endpoint=False)

    def get_filename(self, track, mission):

        if mission == 'trackdrive' or mission == 'autocross':
            filename = track + "_" + mission
        else:
            filename = mission 

        return self.output + "/" + filename
    
    def get_rep(self, track, filename, param_dict):

        repetition = {'sensors_config_file': "fssim_config/sensors/fst_sensors.yaml",
                      'track_name': track + ".sdf",
                      'autonomous_stack': "roslaunch " + filename,
                      'parameters': param_dict
                    }

        return repetition

    def delete_launch_files(self):
        files = os.listdir(args.output)
        for file in files:
            if file.endswith(".launch"):
                os.remove(os.path.join(args.output, file))
        print("******** DELETED LAUNCH FILES ********")

def signal_handler(signal, frame):
    generator.delete_launch_files()  

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Process some integers.')
    parser.add_argument("--config", dest = "config", metavar = "FILE", help = "Config YAML file", required = True)
    parser.add_argument("--output", dest = "output", help = "Output YAML file", required = True)
    parser.add_argument("-p", "--parameters", action = "append", dest = "parameters", help = "Parameters to test. PARAMETER MIN MAX STEP", required = True)
    parser.add_argument("-t", "--track", action = "append", dest = "track", help = "track to run simulation on", required = False)
    parser.add_argument("-m", "--mission", action = "append", dest = "mission", help = "mission to simulate", required = True, choices=["skidpad", "autocross", "trackdrive", "acceleration"])
    parser.add_argument("--all_combinations", action = "store_true", help = "if provided it will generate all the combinitions with the parameters", required = False)
    args = parser.parse_args()

    generator = AutoTestGenerator(args)
    auto_test_launcher_path = os.path.dirname(os.path.realpath(__file__)) + "/auto_test_launcher.py"
    command =  "python " + auto_test_launcher_path + " --config " + args.config + " --output " + args.output
    print(command)
    signal(SIGINT, signal_handler)
    subprocess.call(command, shell=True)
    generator.delete_launch_files()