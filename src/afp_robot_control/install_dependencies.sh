#!/bin/bash
# 阻抗控制器依赖安装脚本

echo "======================================"
echo "安装阻抗控制器Python依赖"
echo "======================================"

# 检查Python版本
python3 --version

# 安装基础依赖
echo ""
echo "安装基础依赖: numpy, scipy..."
pip3 install numpy scipy

# 安装Pinocchio
echo ""
echo "安装Pinocchio..."
echo "如果遇到问题，请参考: https://stack-of-tasks.github.io/pinocchio/download.html"

# 方法1: 通过pip安装（推荐）
pip3 install pin

# 如果pip安装失败，尝试conda或从源码编译
# conda install pinocchio -c conda-forge
# 或参考官方文档从源码编译

echo ""
echo "======================================"
echo "验证安装..."
echo "======================================"

python3 -c "import numpy; print('✓ numpy:', numpy.__version__)"
python3 -c "import scipy; print('✓ scipy:', scipy.__version__)"
python3 -c "import pinocchio; print('✓ pinocchio:', pinocchio.__version__)" || echo "✗ pinocchio 安装失败"

echo ""
echo "======================================"
echo "如果pinocchio安装失败，可以使用以下方法："
echo "1. conda install pinocchio -c conda-forge"
echo "2. 参考官方文档: https://stack-of-tasks.github.io/pinocchio/"
echo "======================================"
