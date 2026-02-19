import yaml
import os
from .dotdict import DotDict
from ament_index_python.packages import get_package_share_directory

def get_dict(model_name):
    model, tire = model_name.split("_")
    package_path = get_package_share_directory('on_track_sys_id')
    with open(f'{package_path}/models/{model}/{model_name}.txt', 'rb') as f:
        params = yaml.load(f, Loader=yaml.Loader)
    
    return params

def get_dotdict(model_name):
    dict = get_dict(model_name)
    params = DotDict(dict)
    return params