"""
配置Python环境，确保可以导入impedance_control模块
在任何使用impedance_control的脚本开头导入此模块
"""
import sys
import os

# 添加impedance_control模块路径
impedance_control_path = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), 
    'src'
)

if impedance_control_path not in sys.path:
    sys.path.insert(0, impedance_control_path)

# 验证导入
try:
    import impedance_control
    print(f"✓ impedance_control模块路径已配置: {impedance_control_path}")
except ImportError as e:
    print(f"✗ 无法导入impedance_control: {e}")
    print(f"  请检查路径: {impedance_control_path}")
