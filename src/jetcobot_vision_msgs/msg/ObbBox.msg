# 단일 OBB(Oriented Bounding Box) 검출 결과
# frame_id 는 부모 ObbBoxArray.header.frame_id 를 따릅니다.

float32 cx          # 픽셀 중심 X
float32 cy          # 픽셀 중심 Y
float32 w           # OBB 너비 (픽셀)
float32 h           # OBB 높이 (픽셀)
float32 theta       # OBB 장축 각도 (rad, 카메라 좌표계)
int32   id          # 클래스 ID
float32 confidence  # 검출 신뢰도 [0, 1]
