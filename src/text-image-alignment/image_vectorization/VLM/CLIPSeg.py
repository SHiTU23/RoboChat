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






    def segment_object(self, image_path, texts, most_probable_obj=True):
        '''
            This function segments an object in the image based on the text provided.
            A list of text can be provised
            @param: image_path: absolute path to image
            @param: text: list of str

            return: object_features, image with segmented object, image with bounding box around the object
            object_features is a dict : {'name' : str, 'probability' : float, 'bounding_box' : list}
            bounding_box is a list : [x, y, w, h]
        '''
        SEGMENTATION_THRESHOLD = 0.2 ### threshold for segmentation
        SEGMENT_COLORMAP = [(0, 255, 0), (255, 0, 0), (0, 0, 255)]  # Colors for different objects
        _highest_segmentation_ratio = 0
        _largest_segmented_area = 0
        object_score = 0
        best_box = None
        self.detected_object_name = ''

        #### return values:
        object_features = {'name' : "",
                           'probability' : 0,
                           'bounding_box' : []} ## x, y, w, h
        segmented_image = None
        image_with_bounding_boxed = None

        texts = [texts]

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

        _inputs = self.processor(text=texts, images=[image] * len(texts), padding=True, return_tensors="pt")
        _outputs = self.model(**_inputs)
        
        _logits = _outputs.logits  # Shape: (num_texts, H, W)

        probs = torch.sigmoid(_logits).detach().numpy()  # Convert logits to probabilities
        print(len(probs))

        for i, text in enumerate(texts):
            object_score = probs[i].max()
            print(f"Object '{text}', {object_score:.2f}")

            mask = (probs[i] >= SEGMENTATION_THRESHOLD).astype(np.uint8) * 255  # Convert to binary mask
            mask_resized = cv2.resize(mask, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

            #### the object is detected in the image
            if object_score >= SEGMENTATION_THRESHOLD:
                self.detected_object_name = text
                object_features['name'] = self.detected_object_name
                object_features['probability'] = float(object_score)

                self.detected_object_name = self.detected_object_name + f'_T_{SEGMENTATION_THRESHOLD}'

                print(f"Object '{self.detected_object_name}' detected with max probability {object_score:.2f}")
                
                ### find contours of the segmented object
                contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                ##########################
                #### Edge Detection ######
                ##########################
                if most_probable_obj: 
                    gray_image = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
                    edges = cv2.Canny(gray_image, 100, 200)

                    # Find all object contours (potential objects in the image)
                    obj_contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    best_segmentation_ratio = 0 

                    for obj in obj_contours:
                        x, y, w, h = cv2.boundingRect(obj) 
                        object_area = w * h  
                        
                        if object_area > 0:  
                            # Count the number of segmented pixels inside this object
                            mask_inside_obj = mask_resized[y:y+h, x:x+w]
                            segmented_area = np.sum(mask_inside_obj > 0)

                            segmentation_ratio = segmented_area / object_area

                            if segmentation_ratio > best_segmentation_ratio:  
                                best_segmentation_ratio = segmentation_ratio
                                best_object = (x, y, w, h)

                    if best_segmentation_ratio > _highest_segmentation_ratio:
                        _highest_segmentation_ratio = best_segmentation_ratio
                        object_features['name'] = self.detected_object_name
                        object_features['probability'] = float(object_score)
                        object_features['bounding_box'] = best_object if best_object else []

                        if best_object:
                            x, y, w, h = best_object
                            self.bounding_boxed_image = cv2.rectangle(image_cv, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            self.bounding_boxed_image = cv2.putText(
                                image_cv, f"'{self.detected_object_name}' ratio: {best_segmentation_ratio:.2f}, prob: {float(object_score):.2f}",
                                (x - 20, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                            )

                mask_colored = np.zeros_like(image_cv)
                mask_colored[:, :, 0] = mask_resized * (SEGMENT_COLORMAP[i][0] / 255)
                mask_colored[:, :, 1] = mask_resized * (SEGMENT_COLORMAP[i][1] / 255)
                mask_colored[:, :, 2] = mask_resized * (SEGMENT_COLORMAP[i][2] / 255)

                # Blend mask with image
                self.segmented_overlay = cv2.addWeighted(self.segmented_overlay, 0.7, mask_colored, 0.3, 0)
                self.segmented_overlay = cv2.putText(self.segmented_overlay, f"'{self.detected_object_name}' probability: {object_score:.2f}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
            else:
                print(f"Object '{text}' not detected")

        segmented_image = self.segmented_overlay
        image_with_bounding_boxed = self.bounding_boxed_image
        return object_features, segmented_image, image_with_bounding_boxed

    def show_segmented_image(self, segmentations=True, bounding_boxes=True):
        if self.detected_object_name != '':
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

    def save_image(self, segmented_image=True, bounding_boxed_image=True, save_path=''):
        if save_path != '':
            images_dir = save_path
        else:
            current_dir = os.path.dirname(__file__)
            images_dir = current_dir + '/images/CLIPSeg/'

        
        if self.detected_object_name != '':
            self.image_save_counter += 1
            image_name = self.detected_object_name + f'_{self.image_save_counter}.jpg'

            if segmented_image:
                save_path = images_dir + 'segmented_images/' + image_name
                cv2.imwrite(save_path, self.segmented_overlay)
            if bounding_boxed_image:
                save_path = images_dir + 'bounding_boxed_images/' + image_name
                cv2.imwrite(save_path, self.bounding_boxed_image)



if __name__ == "__main__":

    clipseg = CLIPSeg()

    current_dir = os.path.dirname(__file__)
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    pic_dir = src_dir + '/simulation/images/'
    image_path = pic_dir + '_image1.jpg'
    # image_path = pic_dir + '_image_changedscene1.jpg'

    

    texts = "green cube"
    object_features, segmented_image, boundingBoxed_image = clipseg.segment_object(image_path, texts, most_probable_obj=True)
    print(object_features)
    clipseg.show_segmented_image(segmentations=True, bounding_boxes=True)
    # clipseg.save_image(segmented_image=True, bounding_boxed_image=True)


