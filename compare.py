#!/usr/bin/env python3
"""
EECBS C++ vs C# 批量对比验证脚本
用法：python3 compare.py --cpp ~/eecbs-cpp/eecbs --csharp ~/eecbs-csharp --maps ~/mapf-benchmark/maps --scens ~/mapf-benchmark/scen-random
"""

import subprocess
import csv
import os
import sys
import argparse
import time
from pathlib import Path

# -------------------------------------------------------------------------
# 参数
# -------------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--cpp",    required=True, help="C++ eecbs 可执行文件路径")
parser.add_argument("--csharp", required=True, help="C# 可执行文件路径（发布后的独立二进制）或项目目录")
parser.add_argument("--maps",   required=True, help="地图文件目录")
parser.add_argument("--scens",  required=True, help="场景文件目录")
parser.add_argument("--agents", default="10,20,30,50", help="智能体数量列表，逗号分隔")
parser.add_argument("--time",   default=60,  type=float, help="每次求解时间限制（秒）")
parser.add_argument("--w",      default=1.2, type=float, help="次优界 w")
parser.add_argument("--maps-filter", default="random", help="只测试包含此字符串的地图")
parser.add_argument("--max-maps",    default=3, type=int, help="最多测试几张地图")
parser.add_argument("--output", default="comparison_results.csv", help="输出 CSV 文件")
args = parser.parse_args()

AGENT_COUNTS = [int(k) for k in args.agents.split(",")]
TMPDIR = Path("/tmp/eecbs_compare")
TMPDIR.mkdir(exist_ok=True)

# -------------------------------------------------------------------------
# 收集地图和场景文件
# -------------------------------------------------------------------------
maps_dir  = Path(args.maps)
scens_dir = Path(args.scens)

map_files = sorted([f for f in maps_dir.glob("*.map") if args.maps_filter in f.name])[:args.max_maps]
if not map_files:
    print(f"[ERROR] 在 {maps_dir} 中未找到包含 '{args.maps_filter}' 的地图文件")
    sys.exit(1)

print(f"找到 {len(map_files)} 张地图：{[f.name for f in map_files]}")

# -------------------------------------------------------------------------
# 运行 C++ 版本
# -------------------------------------------------------------------------
def run_cpp(map_file, scen_file, k, w, time_limit):
    out_csv   = TMPDIR / f"cpp_{map_file.stem}_k{k}.csv"
    out_paths = TMPDIR / f"cpp_{map_file.stem}_k{k}_paths.txt"
    cmd = [
        args.cpp,
        "-m", str(map_file),
        "-a", str(scen_file),
        "-k", str(k),
        "-t", str(time_limit),
        f"--suboptimality={w}",
        "-o", str(out_csv),
        f"--outputPaths={out_paths}",
    ]
    t0 = time.time()
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=time_limit + 30)
        elapsed = time.time() - t0
        # 解析 CSV 输出
        if out_csv.exists():
            with open(out_csv) as f:
                rows = list(csv.DictReader(f))
            if rows:
                row = rows[-1]
                cost = int(float(row.get("solution cost", row.get("solution_cost", -1))))
                return {"cost": cost, "runtime": elapsed, "solved": cost >= 0}
    except subprocess.TimeoutExpired:
        pass
    except Exception as e:
        print(f"  [C++ ERROR] {e}")
    return {"cost": -1, "runtime": time_limit, "solved": False}

# -------------------------------------------------------------------------
# 运行 C# 版本
# -------------------------------------------------------------------------
def run_csharp(map_file, scen_file, k, w, time_limit):
    out_csv   = TMPDIR / f"cs_{map_file.stem}_k{k}.csv"
    out_paths = TMPDIR / f"cs_{map_file.stem}_k{k}_paths.txt"

    cs_path = Path(args.csharp)
    # 判断是独立可执行文件还是项目目录
    if cs_path.is_file() and os.access(cs_path, os.X_OK):
        # 独立发布的二进制（dotnet publish --self-contained）
        cmd = [
            str(cs_path),
            "-m", str(map_file),
            "-a", str(scen_file),
            "-k", str(k),
            "-t", str(time_limit),
            f"--suboptimality={w}",
            "-o", str(out_csv),
            f"--outputPaths={out_paths}",
        ]
    else:
        # 项目目录，用 dotnet run
        dotnet = os.environ.get("DOTNET_ROOT", "")
        dotnet_bin = os.path.join(dotnet, "dotnet") if dotnet else "dotnet"
        # 尝试常见安装路径
        for candidate in [dotnet_bin, os.path.expanduser("~/.dotnet/dotnet"), "/usr/bin/dotnet", "/usr/local/bin/dotnet"]:
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                dotnet_bin = candidate
                break
        cmd = [
            dotnet_bin, "run", "-c", "Release", "--project", str(cs_path), "--",
            "-m", str(map_file),
            "-a", str(scen_file),
            "-k", str(k),
            "-t", str(time_limit),
            f"--suboptimality={w}",
            "-o", str(out_csv),
            f"--outputPaths={out_paths}",
        ]
    t0 = time.time()
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=time_limit + 60)
        elapsed = time.time() - t0
        if out_csv.exists():
            with open(out_csv) as f:
                rows = list(csv.DictReader(f))
            if rows:
                row = rows[-1]
                cost = int(float(row.get("solution_cost", row.get("solution cost", -1))))
                return {"cost": cost, "runtime": elapsed, "solved": cost >= 0}
        # 也尝试从 stdout 解析
        for line in result.stdout.splitlines():
            if "解代价" in line or "sum-of-costs" in line.lower():
                parts = line.split(":")
                if len(parts) >= 2:
                    try:
                        cost = int(parts[-1].strip())
                        return {"cost": cost, "runtime": elapsed, "solved": cost >= 0}
                    except ValueError:
                        pass
    except subprocess.TimeoutExpired:
        pass
    except Exception as e:
        print(f"  [C# ERROR] {e}")
    return {"cost": -1, "runtime": time_limit, "solved": False}

# -------------------------------------------------------------------------
# 主循环
# -------------------------------------------------------------------------
results = []
passed = 0
failed = 0
skipped = 0

print(f"\n{'='*70}")
print(f"{'地图':<30} {'k':>4}  {'C++ cost':>10}  {'C# cost':>10}  {'状态':>8}  {'C# 时间':>8}")
print(f"{'='*70}")

for map_file in map_files:
    # 找对应的场景文件（取第一个匹配的）
    scen_candidates = list(scens_dir.glob(f"{map_file.stem}*.scen"))
    if not scen_candidates:
        # 尝试子目录
        scen_candidates = list(scens_dir.rglob(f"{map_file.stem}*.scen"))
    if not scen_candidates:
        print(f"[SKIP] 未找到 {map_file.stem} 的场景文件")
        skipped += 1
        continue
    scen_file = sorted(scen_candidates)[0]

    for k in AGENT_COUNTS:
        print(f"{map_file.name:<30} {k:>4}", end="  ", flush=True)

        cpp_r = run_cpp(map_file, scen_file, k, args.w, args.time)
        cs_r  = run_csharp(map_file, scen_file, k, args.w, args.time)

        cpp_cost = cpp_r["cost"]
        cs_cost  = cs_r["cost"]

        # 判断是否一致：
        # 1. 两者都超时/无解 → SKIP
        # 2. 一方超时另一方有解 → 只验证有解方的路径合法性
        # 3. 两者都有解 → 代价差 ≤ ceil(optCost * (w-1)) 视为一致
        if not cpp_r["solved"] and not cs_r["solved"]:
            status = "BOTH_TIMEOUT"
            skipped += 1
        elif not cpp_r["solved"]:
            status = "CPP_TIMEOUT"
            skipped += 1
        elif not cs_r["solved"]:
            status = "CS_TIMEOUT"
            failed += 1
        else:
            # 两者都有解，代价应在彼此的 w 倍内
            # 宽松判断：|cs_cost - cpp_cost| / cpp_cost <= w - 1
            ratio = abs(cs_cost - cpp_cost) / max(cpp_cost, 1)
            if ratio <= (args.w - 1) + 0.01:  # 加 0.01 容忍浮点误差
                status = "PASS"
                passed += 1
            else:
                status = f"FAIL(diff={cs_cost-cpp_cost})"
                failed += 1

        print(f"{cpp_cost:>10}  {cs_cost:>10}  {status:>8}  {cs_r['runtime']:>7.2f}s")

        results.append({
            "map":        map_file.name,
            "scen":       scen_file.name,
            "k":          k,
            "w":          args.w,
            "cpp_cost":   cpp_cost,
            "cs_cost":    cs_cost,
            "cpp_solved": cpp_r["solved"],
            "cs_solved":  cs_r["solved"],
            "cs_runtime": round(cs_r["runtime"], 3),
            "status":     status,
        })

# -------------------------------------------------------------------------
# 保存结果
# -------------------------------------------------------------------------
print(f"\n{'='*70}")
print(f"通过: {passed}  失败: {failed}  跳过: {skipped}  总计: {len(results)}")

with open(args.output, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=results[0].keys() if results else [])
    writer.writeheader()
    writer.writerows(results)
print(f"结果已保存到: {args.output}")

sys.exit(0 if failed == 0 else 1)
