from ultralytics import YOLO
import cv2
import glob

# Initialize a YOLO-World model
model = YOLO("yolov8s-world.pt")  # or choose yolov8m/l-world.pt

# Define custom classes
model.set_classes(["window", "glass", "chair", "table", "desk", "building", "tree", "monitor", "TV", "person", "light", "hook", "shelf", "alarm", "refrigerator", "microwave", "cabinet", "faucet", "basin", "water dispenser", "fan", "trash bin"]) # refrigerator, microwave
# model.set_classes(["building", "tree"])

# Set the confidence threshold to a lower value (e.g., 0.25)
confidence_threshold = 0.01

# Execute prediction on an image with the lower confidence threshold
# results = model.predict("./images/IMG_4204.jpg", conf=confidence_threshold)
# results = model.predict("./images/IMG_4208.jpg", conf=confidence_threshold)
# results = model.predict("./images/IMG_4209.jpg", conf=confidence_threshold)
# results = model.predict("./images/IMG_4206.jpg", conf=confidence_threshold)
# img_name = "IMG_4204" 
# img_name = "IMG_4208"
# img_name = "IMG_4209"
# img_name = "IMG_4206"
# results = model.predict(f"./images/{img_name}.jpg", conf=confidence_threshold)
# print('good')

# iterate through all images
img_name = glob.glob('./images/*.jpg')
for img in img_name:
    # ignore those with "output" in the name
    if "output" in img:
        continue    
    results = model.predict(img, conf=confidence_threshold)
    print(f"Output image saved to {img}")
    results[0].save(f"{img}_output.jpg")

# # add image image to the path
# output_path = f"./{img_name}_output.jpg"
# results[0].save(output_path)
# print(f"Output image saved to {output_path}")
