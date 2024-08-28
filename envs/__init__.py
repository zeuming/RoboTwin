from .move_brush import *
from .pick_cup_with_liquids import *
from .empty_cup_place import *
from .pick_cup import *
from .pick_hammer import *
from .dual_bottles_pick import *
from .block_hammer_beat import *
from .hammer_beat_cross import *
from .apple_cabinet_storage import *
from .put_ball_into_dustpan import *
from .block_sweep import *
from .move_box import *
from .block_handover import *
from .mug_hanging import *
from .diverse_bottles_pick import *
from .shoe_place import *
from .multi_object_storage import *
from .block_hammer_beat_hard import *
from .blocks_stack import *
from .shoes_place_hard import *
from .container_place import *
from .open_pot_cover import *
from .bottle_pick import *
from .pick_cuboid_cylinder import *
from .pick_apple import *

# from importlib import import_module
# import os

# # 获取当前目录下的所有 .py 文件（除了 __init__.py）
# package_dir = os.path.dirname(os.path.abspath(__file__))
# all_modules = [f[:-3] for f in os.listdir(package_dir) if f.endswith('.py') and f != '__init__.py']

# # 动态导入所有模块
# for module_name in all_modules:
#     import_module('.' + module_name, package=__name__)
#     try:
#         from . import module_name
#     except:
#         pass

# # 定义 __all__ 列表，以控制哪些名称会被 * 导入
# __all__ = all_modules