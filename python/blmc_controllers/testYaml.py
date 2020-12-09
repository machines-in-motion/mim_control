import numpy
import yaml

config_file = 'yaml_example.yaml'
with open(config_file) as config:
	data_in = yaml.safe_load(config)
for ctrls in data_in["impedance_controllers"]:
	print data_in["impedance_controllers"][ctrls]["is_eeff"]