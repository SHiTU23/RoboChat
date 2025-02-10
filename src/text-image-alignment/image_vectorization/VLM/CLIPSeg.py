from transformers import AutoProcessor, CLIPSegForImageSegmentation
from PIL import Image
import requests
import torch
import numpy as np
import cv2
import os


class CLIPSeg:
    def __init__(self):
        CLIP_MODEL = "CIDAS/clipseg-rd64-refined"

        self.processor = AutoProcessor.from_pretrained(CLIP_MODEL)
        self.model = CLIPSegForImageSegmentation.from_pretrained(CLIP_MODEL)

        self.image_save_counter = 1

    def segment_object(self, image_path, texts):
        '''
            This function segments an object in the image based on the text provided.
            A list of text can be provised
            @param: image_path: absolute path to image
            @param: text: list of str
        '''
        SEGMENTATION_THRESHOLD = 0.2 ### threshold for segmentation
        SEGMENT_COLORMAP = [(0, 255, 0), (255, 0, 0), (0, 0, 255)]  # Colors for different objects
        largest_segmented_area = 0
        self.detected_object = ''
        best_box = None

        image = Image.open(image_path)
        image_cv = np.array(image)

        ### if the image has 4 channels, remove the alpha channel
        if image_cv.shape[-1] == 4:
            image_cv = image_cv[:, :, :3]

        image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2BGR) # Convert to OpenCV format
        original_h, original_w = image_cv.shape[:2]
        self.segmented_overlay = image_cv.copy()
        self.bounding_boxed_image = image_cv.copy()

        ###########################################################
        ####  FINDING OBJECT IN IMAGE BASED ON TEXT - CLIPSeg  ####
        ###########################################################

        inputs = self.processor(text=texts, images=[image] * len(texts), padding=True, return_tensors="pt")
        outputs = self.model(**inputs)
        logits = outputs.logits  # Shape: (num_texts, H, W)

        probs = torch.sigmoid(logits).detach().numpy()  # Convert logits to probabilities

        for i, text in enumerate(texts):
            object_score = probs[i].max()
            print(f"Object '{text}', {object_score:.2f}")

            mask = (probs[i] >= SEGMENTATION_THRESHOLD).astype(np.uint8) * 255  # Convert to binary mask
            mask_resized = cv2.resize(mask, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

            #### the object is detected in the image
            if object_score >= SEGMENTATION_THRESHOLD:
                self.detected_object = text
                print(f"Object '{self.detected_object}' detected with max probability {object_score:.2f}")
                
                ### find contours of the segmented object
                contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    # Get bounding box around the object
                    x, y, w, h = cv2.boundingRect(contour)
                    area = cv2.contourArea(contour)

                    if area > largest_segmented_area:
                        largest_segmented_area = area
                        best_box = (x, y, w, h)

                if best_box is not None:
                    x, y, w, h = best_box
                    self.bounding_boxed_image = cv2.rectangle(image_cv, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    self.bounding_boxed_image = cv2.putText(image_cv, f"'{self.detected_object}' probability: {object_score:.2f}", (x-20, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                mask_colored = np.zeros_like(image_cv)
                mask_colored[:, :, 0] = mask_resized * (SEGMENT_COLORMAP[i][0] / 255)
                mask_colored[:, :, 1] = mask_resized * (SEGMENT_COLORMAP[i][1] / 255)
                mask_colored[:, :, 2] = mask_resized * (SEGMENT_COLORMAP[i][2] / 255)

                # Blend mask with image
                self.segmented_overlay = cv2.addWeighted(self.segmented_overlay, 0.7, mask_colored, 0.3, 0)
                self.segmented_overlay = cv2.putText(self.segmented_overlay, f"'{self.detected_object}' probability: {object_score:.2f}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                print(f"Object '{text}' not detected")

    def show_segmented_image(self, segmentations=True, bounding_boxes=True):
        if self.detected_object != '':
            if segmentations and bounding_boxes:
                cv2.imshow("CLIPSeg-Segmented Object", self.segmented_overlay)
                cv2.imshow("CLIPSeg-Bounding Boxes", self.bounding_boxed_image)
            elif segmentations:
                cv2.imshow("CLIPSeg-Segmented Object", self.segmented_overlay)
            elif bounding_boxes:
                cv2.imshow("CLIPSeg-Bounding Boxes", self.bounding_boxed_image)
                pass

            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def save_image(self, segmented_image=True, bounding_boxed_image=True):
        current_dir = os.path.dirname(__file__)
        images_dir = current_dir + '/images/CLIPSeg/'

        
        if self.detected_object != '':
            self.image_save_counter += 1
            image_name = self.detected_object + f'_{self.image_save_counter}_T0.2.jpg'

            if segmented_image:
                save_path = images_dir + 'segmented_images/' + image_name
                cv2.imwrite(save_path, self.segmented_overlay)
            if bounding_boxed_image:
                save_path = images_dir + 'bounding_boxed_images/' + image_name
                cv2.imwrite(save_path, self.bounding_boxed_image)



if __name__ == "__main__":

    image_segmentor = CLIPSeg()

    current_dir = os.path.dirname(__file__)
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    pic_dir = src_dir + '/simulation/images/'
    image_path = pic_dir + '_image1.jpg'

    texts = ["the cube on the right side of green cube"]
    image_segmentor.segment_object(image_path, texts)
    image_segmentor.show_segmented_image(segmentations=True, bounding_boxes=True)
    image_segmentor.save_image(segmented_image=True, bounding_boxed_image=True)







