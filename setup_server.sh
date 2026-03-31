#!/bin/bash
# setup_server.sh
# 在 Linux 服务器上一键完成：安装依赖、编译 C++、安装 .NET、发布 C#、下载 benchmark
# 用法：bash setup_server.sh

set -e

WORKDIR="$HOME/eecbs_compare"
mkdir -p "$WORKDIR"
cd "$WORKDIR"

echo "===== [1/5] 安装系统依赖 ====="
sudo apt-get update -qq
sudo apt-get install -y cmake libboost-all-dev g++ git wget python3 unzip

echo "===== [2/5] 编译 C++ EECBS ====="
if [ ! -d "EECBS" ]; then
    git clone https://github.com/Jiaoyang-Li/EECBS.git
fi
cd EECBS
cmake -DCMAKE_BUILD_TYPE=RELEASE . -B build
cmake --build build -j$(nproc)
CPP_BIN="$WORKDIR/EECBS/build/eecbs"
echo "C++ 可执行文件: $CPP_BIN"
cd "$WORKDIR"

echo "===== [3/5] 安装 .NET 9 ====="
if ! command -v dotnet &>/dev/null; then
    wget -q https://dot.net/v1/dotnet-install.sh -O dotnet-install.sh
    chmod +x dotnet-install.sh
    ./dotnet-install.sh --channel 9.0
    export PATH="$HOME/.dotnet:$PATH"
    echo 'export PATH="$HOME/.dotnet:$PATH"' >> ~/.bashrc
fi
dotnet --version

echo "===== [4/5] 发布 C# EECBS ====="
# 假设已通过 scp 上传到 ~/eecbs_compare/DotNetEECBS
CS_DIR="$WORKDIR/DotNetEECBS"
if [ -d "$CS_DIR" ]; then
    cd "$CS_DIR/DotNetEECBS"
    dotnet publish -c Release -r linux-x64 --self-contained true -o "$WORKDIR/csharp_bin"
    CS_BIN="$WORKDIR/csharp_bin/DotNetEECBS"
    echo "C# 可执行文件: $CS_BIN"
    cd "$WORKDIR"
else
    echo "[WARN] 未找到 C# 项目目录 $CS_DIR，请手动上传后重新运行"
fi

echo "===== [5/5] 下载 MAPF Benchmark ====="
BENCH_DIR="$WORKDIR/benchmark"
mkdir -p "$BENCH_DIR/maps" "$BENCH_DIR/scens"

# 下载 random 地图集（约 2MB）
if [ ! -f "$BENCH_DIR/random.zip" ]; then
    wget -q "https://movingai.com/benchmarks/mapf/random.zip" -O "$BENCH_DIR/random.zip"
    unzip -q "$BENCH_DIR/random.zip" -d "$BENCH_DIR/random_raw"
    find "$BENCH_DIR/random_raw" -name "*.map" -exec cp {} "$BENCH_DIR/maps/" \;
    find "$BENCH_DIR/random_raw" -name "*.scen" -exec cp {} "$BENCH_DIR/scens/" \;
    echo "地图数量: $(ls $BENCH_DIR/maps/*.map | wc -l)"
    echo "场景数量: $(ls $BENCH_DIR/scens/*.scen | wc -l)"
fi

echo ""
echo "===== 环境准备完成 ====="
echo ""
echo "运行对比测试："
echo "  python3 compare.py \\"
echo "    --cpp $CPP_BIN \\"
echo "    --csharp $CS_DIR/DotNetEECBS \\"
echo "    --maps $BENCH_DIR/maps \\"
echo "    --scens $BENCH_DIR/scens \\"
echo "    --agents 10,20,30,50 \\"
echo "    --time 60 --w 1.2 \\"
echo "    --maps-filter random-32-32 \\"
echo "    --max-maps 5 \\"
echo "    --output results.csv"
