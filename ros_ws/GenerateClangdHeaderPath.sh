#!/bin/bash

# 指定要提取子文件夹的目录
TARGET_DIR="/opt/ros/humble/include"  # 替换为你的目标目录

# 输出文件
OUTPUT_FILE="output.txt"

# 清空输出文件
> "$OUTPUT_FILE"

# 遍历子文件夹
for dir in "$TARGET_DIR"/*/; do
    if [ -d "$dir" ]; then
        # 获取绝对路径并格式化
        ABS_PATH="$(realpath "$dir")"
        echo "\"-I$ABS_PATH\"," >> "$OUTPUT_FILE"
    fi
done

echo "子文件夹绝对路径已输出到 $OUTPUT_FILE"