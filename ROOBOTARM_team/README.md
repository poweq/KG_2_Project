# ROBOTARM_team


main.py<br>
->2차 프로젝트 최종 코드<br>

control_ABCpose.py<br>
->a,b,c.d 입력을 받으면 입력신호에 맞춰  A동작, B동작, C동작, D동작을 수행하는 코드<br>
block_color_detect_pose.py<br>
->빨강, 파랑, 노랑, 초록 색상의 블록을 인식해서 각 블록의 맞는 위치에 로봇암이 블록을 전달하는 코드<br>

yolov8_model<br>
ROOBOTARM_team\yolov8_model\runs\detect\train\weights\best.pt(박스 색깔별로 인식하는 모델)<br>
ROOBOTARM_team\yolov8_model\runs\detect\train2\weights\best.pt(박스 전체 인식하는 모델)<br>

boundingbox_central.py<br>
->yolo 모델을 사용해 인식된 객체의 중점을 찍어주는 코드<br>

assessment_webcam.py<br>
-> 웹캠으로 실시간으로 박스 인식 및 박스 칸 색깔인식해서 바운딩 박스 생성

reset.py<br>
-> 로봇암 위치 초기화 코드<br>
gipper_reset.py<br>
-> 그리퍼 초기화 코드<br>


mc.send_angles([69, 70, 14, -2, -90, 0], 20)  # pose0 위치로 이동    
