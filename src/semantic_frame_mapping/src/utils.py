import glob
import yaml

class Frame():
    def __init__(self, info_dict):
        self.name = info_dict['name']
        self.description = info_dict['description']
        self.core_frame_elements = []
        self.optional_frame_elements = []
        for frame_element in info_dict['frame_elements']:
            if frame_element['is_core']:
                self.core_frame_elements.append(frame_element['name'])
            else:
                self.optional_frame_elements.append(frame_element['name'])
        self.preconditions = []
        try:
            preconditions = info_dict['preconditions']
            for precondition in preconditions:
                self.preconditions.append(precondition['name'])
        except KeyError:
            pass

def init_knowledge_base(path, frames):
    knowledge_base = []
    for frame in frames:
        a = path+frame+"_sf.yaml"
        print("Path is {}".format(a))
        with open(path+frame+"_sf.yaml", 'r') as fstream:
            try:
                info_dict = yaml.safe_load(fstream)
                knowledge_base.append(Frame(info_dict))
            except yaml.YAMLError as e:
                print(e)
    return knowledge_base

    # frames_path = glob.glob(path+"*")
    # 
    # for frame in frames_path:
    #     # if frame.endswith("grasp_bottle_sf.yaml"):
    #     #     continue
    #     if frame.endswith("stir_mug_sf.yaml"):
    #         continue
    #     if frame.endswith("drinkserve_sf.yaml"):
    #         continue
    #     print("Loading: ", frame.split('/')[-1])
    #     with open(frame, 'r') as fstream:
            
    

