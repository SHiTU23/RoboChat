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

    def detect_object(self, image_path, text, all_objects=False):
        '''
            This function detects the object in the image based on the text provided.
            @param: image_path: absolute path to image

            return: object_features, BoundingBox, image with bounding box around the object
            object_features is a dict : {'name' : str, 'probability' : float, 'bounding_box' : list}
            bounding_box is a list : [x, y, w, h]
        '''
        self.obj_detected = False
        text = text + '.' ### the format for acccepting text is "text."
        highest_score = 0

        ### return values
        object_features = {'name' : "",
                           'probability' : [],
                           'bounding_box' : []} ## x, y, w, h
        image_with_bounding_boxed = None

        image = Image.open(image_path)
        image_cv = np.array(image)
        image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2BGR)
        self.detected_image = image_cv.copy()

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
        # print(results)

        if len(results[0]["boxes"]) != 0:
            self.obj_detected = True
            self.detected_object_name = text[:-1] ## remove the '.' from the text

        if self.obj_detected:
            if all_objects:
                for box, score, label in zip(results[0]["boxes"], results[0]["scores"], results[0]["labels"]):
                    # print(f"box: {box}, score: {score}, label: {label}")
                    x1, y1, x2, y2 = map(int, box.tolist())
                    w = abs(x2 - x1)
                    h = abs(y2 - y1)
                    object_features["bounding_box"].append([x1, y1, w, h])
                    object_features["probability"].append(float(score))
                    object_features["name"] = label
                    
                    label_text = f"{label}: {score:.3f}"
                    self.detected_image = cv2.rectangle(image_cv, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    self.detected_image = cv2.putText(image_cv, label_text, (x1 - 10, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                for i, score in enumerate(results[0]["scores"]):
                    score = float(score)
                    if score > highest_score:
                        highest_score = score
                        object_indx = i

                box = results[0]["boxes"][object_indx]
                label = results[0]["labels"][object_indx]
                object_features["name"] = label
                object_features["probability"] = float(highest_score)
                # print(highest_score)

                x1, y1, x2, y2 = map(int, box.tolist()) ## box is a tensor
                w = abs(x2 - x1)
                h = abs(y2 - y1)
                object_features["bounding_box"] = [x1, y1, w, h]

                
                label_text = f"{label}: {highest_score:.3f}"
                self.detected_image = cv2.rectangle(image_cv, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.detected_image = cv2.putText(image_cv, label_text, (x1 - 10, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            print(f"{text} not found in the image.")

        image_with_bounding_boxed = self.detected_image
        return object_features, image_with_bounding_boxed

    def show_image(self):
        if self.obj_detected:
            cv2.imshow("Grounding_Dino_detection", self.detected_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def save_image(self, save_path=''):
        if save_path != '':
            images_dir = save_path
        else:
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
    image_path = pic_dir + '_image66.jpg'
    # image_path = pic_dir + '_image_changedscene1.jpg'


    # text = "objects on the table"
    queries = ["red cube", "blue cube", "green cube", "yellow cube", "pink cube", "cubes", "robot arm"]

    for query in queries:

        object_features, image = grounding_dino.detect_object(image_path, query, all_objects=True)
        print(object_features)
        # cv2.imshow("image", image)
        # cv2.waitKey(0)
        grounding_dino.show_image()
        grounding_dino.save_image()
