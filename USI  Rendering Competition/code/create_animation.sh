#!/bin/bash

# 炮管动画生成脚本
# 使用方法：
#   ./create_animation.sh mp4    # 生成 MP4 视频
#   ./create_animation.sh gif    # 生成 GIF 动画

if [ ! -f "frame_000.ppm" ]; then
    echo "错误：找不到 frame_000.ppm，请先运行渲染程序生成帧序列"
    exit 1
fi

if ! command -v ffmpeg &> /dev/null; then
    echo "错误：未找到 ffmpeg"
    echo "请先安装 ffmpeg："
    echo "  brew install ffmpeg"
    exit 1
fi

OUTPUT_TYPE=${1:-mp4}  # 默认生成 MP4

if [ "$OUTPUT_TYPE" = "mp4" ]; then
    echo "正在生成 MP4 视频..."
    ffmpeg -y -framerate 10 -i frame_%03d.ppm -c:v libx264 -pix_fmt yuv420p -crf 18 barrel_animation.mp4
    echo "完成！视频已保存为：barrel_animation.mp4"
    
elif [ "$OUTPUT_TYPE" = "gif" ]; then
    echo "正在生成 GIF 动画（这可能需要一些时间）..."
    # 先生成调色板以获得更好的 GIF 质量
    ffmpeg -y -i frame_%03d.ppm -vf "palettegen" palette.png
    # 使用调色板生成 GIF
    ffmpeg -y -framerate 10 -i frame_%03d.ppm -i palette.png -lavfi "paletteuse" barrel_animation.gif
    rm -f palette.png
    echo "完成！GIF 已保存为：barrel_animation.gif"
    
else
    echo "用法：$0 [mp4|gif]"
    echo "  mp4 - 生成 MP4 视频（默认）"
    echo "  gif - 生成 GIF 动画"
    exit 1
fi


