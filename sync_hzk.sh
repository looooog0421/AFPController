#!/bin/bash

set -e   # 出错自动退出

BRANCH="hzk"

echo "===== 🟦 一键同步脚本开始 ====="

cd ~/AFPController

echo "📌 当前目录: $(pwd)"
echo "📌 目标分支: $BRANCH"

echo ""
echo "===== 1️⃣ 更新远程仓库信息 ====="
git fetch origin

echo ""
echo "===== 2️⃣ 切换到 main 并拉取最新代码 ====="
git checkout main
git pull origin main

echo ""
echo "===== 3️⃣ 切回你的工作分支：$BRANCH ====="
git checkout $BRANCH

echo ""
echo "===== 4️⃣ 将你的分支 rebase 到最新 main ====="
git rebase main

echo ""
echo "===== 5️⃣ 推送你的分支到远程（安全模式） ====="
git push origin $BRANCH --force-with-lease

echo ""
echo "===== 🟩 同步完成！你的 hzk 分支已与师哥同步 ====="
