#!/bin/bash
# ROS2 토픽 모니터링

set -e

if [ "$1" = "obb" ]; then
    echo "=== ObbBoxArray 토픽 모니터링 (cv_detect_server 발행) ==="
    echo "Topics:"
    ros2 topic list | grep -i obb || echo "  (없음)"
    echo ""
    echo "메인 PC에서 cv_detect_server가 실행 중인지 확인:"
    echo "  python3 src/devices/jetcobot/vision/cv_detect_server.py"
    echo ""
    echo "토픽 구독 (Ctrl+C로 중단):"
    ros2 topic echo /detect_bridge_node/obb_boxes

elif [ "$1" = "pick" ]; then
    echo "=== PickPoint 토픽 모니터링 (coord_transform_node 발행) ==="
    echo ""
    echo "토픽 구독 (Ctrl+C로 중단):"
    ros2 topic echo /coord_transform_node/pick_point

elif [ "$1" = "all" ]; then
    echo "=== 모든 토픽 목록 ==="
    ros2 topic list
    echo ""
    echo "각 토픽 타입:"
    ros2 topic list -t

else
    echo "Usage: $0 {obb|pick|all}"
    echo ""
    echo "  obb   - ObbBoxArray 토픽 (메인 PC → 제어 PC)"
    echo "  pick  - PickPoint 토픽 (coord_transform_node 발행)"
    echo "  all   - 모든 토픽 목록"
    exit 1
fi
