#!/bin/bash

# 说明：这个脚本将由 crontab @reboot 调用，环境变量很“干净”，
# 必须：1) 使用绝对路径/显式 cd；2) 把输出重定向到日志便于排查。

set -u

# --- 配置区域 ---
SLEEP_BEFORE_START=10
WORK_DIR="/home/inexbot/robot"

# 需要启动的程序名（按你的实际文件名）
NRC2_BIN="nrc2.out"
ROBOT_BIN="C1102_X86_Linux-RT_Robot-controller"

PASSWORD="123"   # 不推荐明文密码；更推荐配置 sudo 免密（见下方说明）
LOG_FILE="$WORK_DIR/autostart_robot.log"
# ----------------

# cron 环境下 PATH 可能不完整，补齐常用路径
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# 将 stdout/stderr 全部写入日志
mkdir -p "$WORK_DIR"
exec >>"$LOG_FILE" 2>&1

echo "=================================================="
echo "[$(date '+%F %T')] autostart_robot.sh start"
echo "USER=$(whoami)  PWD=$(pwd)  SHELL=$SHELL"

sleep "$SLEEP_BEFORE_START"

cd "$WORK_DIR" || { echo "[$(date '+%F %T')] ERROR: cd $WORK_DIR failed"; exit 1; }
echo "[$(date '+%F %T')] cd to $WORK_DIR ok"

# rc.local 场景下脚本常以 root 运行；此时 sudo 可能打印
# "standard input is not a terminal device"。做一个封装：root 直接执行，非 root 才 sudo。
run_root() {
	if [ "$(id -u)" -eq 0 ]; then
		"$@"
	else
		echo "$PASSWORD" | sudo -S "$@"
	fi
}

kill_by_name() {
    # 改用 pkill -f (full command line match)，确保能匹配到带路径/带参数的进程
    local name="$1"
    echo "[$(date '+%F %T')] kill_by_name (pkill -f): $name"

    # 记录当前匹配到的进程详情（用于排查）
    local match_info
    match_info=$(pgrep -f -a "$name" 2>/dev/null || true)
    if [ -n "$match_info" ]; then
        echo "[$(date '+%F %T')] Processes matching '$name':"
        echo "$match_info"
    else
        echo "[$(date '+%F %T')] No process matching '$name' found."
        return 0
    fi

    # 循环尝试 kill
    for _ in $(seq 1 10); do
        # -f 匹配完整命令行
        if ! pgrep -f "$name" >/dev/null; then
            break
        fi
        run_root pkill -9 -f "$name" || true
        sleep 0.2
    done

    if pgrep -f "$name" >/dev/null; then
        echo "[$(date '+%F %T')] WARN: '$name' still alive after pkill -f."
        pgrep -f -a "$name"
        return 1
    fi
    echo "[$(date '+%F %T')] '$name' killed."
    return 0
}
# 1) 杀死旧的 C1102_X86_Linux（如果你确实需要）
# 注意：若 sudo 需要交互输入密码，cron 下会失败。
# 方案A：用 -S 从标准输入读密码（当前脚本做法）
sleep 3
kill_by_name "C1102_X86_Linux" || true

# 2) 启动 nrc2.out
echo "[$(date '+%F %T')] starting nrc2.out..."
run_root "$WORK_DIR/$NRC2_BIN" &
sleep 3

# 3) 启动机器人主程序
echo "[$(date '+%F %T')] starting $ROBOT_BIN..."
run_root "$WORK_DIR/$ROBOT_BIN"

echo "[$(date '+%F %T')] autostart_robot.sh end"
