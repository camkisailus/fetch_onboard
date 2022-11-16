import yaml
# import SFM
import sys

if __name__ == '__main__':
    with open('/home/cuhsailus/Desktop/Research/Spring_22/fetch_sfm_ws/src/semantic_frame_mapping/config/experiment_1.yaml', 'r') as file:
        experiment_config = yaml.safe_load(file)
    for step in experiment_config['frames']:
        print(step)
    # print(experiment_config['regions'])