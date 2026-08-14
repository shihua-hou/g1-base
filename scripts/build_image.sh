#!/usr/bin/env bash
# 在 arm64 机器（Jetson 或那块开发板）上构建应用镜像。
#
#   bash scripts/build_image.sh              # tag 用 git 短 hash，没有 git 就用时间戳
#   bash scripts/build_image.sh 0814a        # 指定 tag
#
# 构建产物是一个新 tag，不碰正在运行的容器；失败了现场照常跑旧版本。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${G1_IMAGE_NAME:-g1-base}"

if [[ $# -ge 1 ]]; then
    TAG="$1"
elif git -C "$REPO_ROOT" rev-parse --short HEAD >/dev/null 2>&1; then
    TAG="$(git -C "$REPO_ROOT" rev-parse --short HEAD)"
    if [[ -n "$(git -C "$REPO_ROOT" status --porcelain)" ]]; then
        TAG="${TAG}-dirty"
        echo "[build] 警告：工作区有未提交改动，tag 标记为 ${TAG}"
    fi
else
    TAG="$(date +%m%d_%H%M)"
fi

ARCH="$(uname -m)"
if [[ "$ARCH" != "aarch64" && "$ARCH" != "arm64" ]]; then
    echo "[build] 错误：当前是 ${ARCH}，镜像必须在 arm64 上构建，否则 Jetson 跑不了" >&2
    exit 1
fi

BASE_IMAGE="${G1_BASE_IMAGE:-hub-nj.iwhalecloud.com/nexrobot/ros2-unitree-g1-base:C_202607311711}"
if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
    echo "[build] 错误：本地没有基础镜像 ${BASE_IMAGE}" >&2
    echo "[build] 这台机器无法访问 registry 时，需要先从有镜像的机器 docker save/load 过来" >&2
    exit 1
fi

echo "[build] 基础镜像 ${BASE_IMAGE}"
echo "[build] 目标 ${IMAGE_NAME}:${TAG}"
cd "$REPO_ROOT"
docker build --build-arg "BASE_IMAGE=${BASE_IMAGE}" -t "${IMAGE_NAME}:${TAG}" .

echo
echo "[build] 完成：${IMAGE_NAME}:${TAG}"
echo "[build] 启动/切换到这个版本："
echo "         G1_IMAGE_TAG=${TAG} docker compose up -d"
echo "[build] 回滚：把 G1_IMAGE_TAG 换回上一个 tag 再 up -d（docker images 可查）"
