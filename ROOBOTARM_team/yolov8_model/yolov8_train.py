from ultralytics import YOLO

model = YOLO('yolov8s.pt')

model.train(data='./box_data.yaml' , epochs=100)