import requests
import os
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import cv2
import numpy as np


class Grounding_Dino:
    def __init__(self):
        model_id = "IDEA-Research/grounding-dino-tiny"

        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id)

        self.image_save_counter = 1

    def detect_object(self, image_path, text):
        '''
            This function detects the object in the image based on the text provided.
            @param: image_path: absolute path to image
        '''
        self.obj_detected = False
        self.detected_object_name = ''
        text = text + '.' ### the format for acccepting text is "text."

        image = Image.open(image_path)
        image_cv = np.array(image)
        image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2BGR)

        inputs = self.processor(images=image, text=text, return_tensors="pt")
        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold=0.5,
            text_threshold=0.3,
            target_sizes=[image.size[::-1]]
        )
        print(results)

        if len(results[0]["boxes"]) != 0:
            self.obj_detected = True
            self.detected_object_name = text

        if self.obj_detected:
            for box, score, label in zip(results[0]["boxes"], results[0]["scores"], results[0]["labels"]):
                print(f"box: {box}, score: {score}, label: {label}")

                x1, y1, x2, y2 = map(int, box.tolist())
                
                label_text = f"{label}: {score:.3f}"
                self.detected_image = cv2.rectangle(image_cv, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.detected_image = cv2.putText(image_cv, label_text, (x1 - 10, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        else:
            print(f"{text} not found in the image.")

    def show_image(self):
        if self.obj_detected:
            cv2.imshow("Grounding_Dino_detection", self.detected_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def save_image(self):
        current_dir = os.path.dirname(__file__)
        images_dir = current_dir + '/images/Grounding_Dino/'

        if self.obj_detected:
            self.image_save_counter += 1
            image_name = self.detected_object_name + f'_{self.image_save_counter}.jpg'

            cv2.imwrite(images_dir + image_name, self.detected_image)
            print("image saved")



if __name__ == "__main__":
    grounding_dino = Grounding_Dino()

    current_dir = os.path.dirname(__file__)
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    pic_dir = src_dir + '/simulation/images/'
    image_path = pic_dir + '_image1.jpg'

    text = "right cube to the green one"

    grounding_dino.detect_object(image_path, text)
    grounding_dino.show_image()
    grounding_dino.save_image()
