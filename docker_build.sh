#!/bin/bash

# Dockerイメージ名とタグ
IMAGE_NAME="minecraft_ros2"
TAG="latest"

# Dockerビルド
docker build -t ${IMAGE_NAME}:${TAG} .

# ビルド結果の確認
if [ $? -eq 0 ]; then
    echo "Dockerイメージのビルドに成功しました: ${IMAGE_NAME}:${TAG}"
else
    echo "Dockerイメージのビルドに失敗しました"
    exit 1
fi
