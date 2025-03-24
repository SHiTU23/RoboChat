"""
Workflow:
    1. Use Grounding DINO to detect all cubes in the scene.
    2. Use adjusted CLIPSeg to find the requested object.
    3. If CLIPSeg does not detect the object, use Grounding DINO to select the object with the highest probability.
    
Return BB:
    - If CLIPSeg is used: return the bounding box (BB) that overlaps with one of the DINO BBs.
    - Else: return the BB from Grounding DINO.
    
    If no object is found, return None.
"""

import cv2
from enum import Enum
import os, sys

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from CLIPSeg import CLIPSeg
from Grounding_DINO import Grounding_Dino


class ModelName(Enum):
    Neither = 0
    CLIP_Seg = 1
    Grounding_Dino = 2

class ClipDino:
    def __init__(self):
        self.clipseg = CLIPSeg()
        self.grounding_dino = Grounding_Dino()
        self.image_path = None  # Will be set in retrieve_object

    def find_all_cubes(self, image_path):
        """
        This functions finds all cubes and returns a list of their bounding boxes
        BBs are [x, y, width, height]
        """
        self.image_path = image_path

        # Step 1: Use DINO to detect all cubes in the scene
        dino_all_features, _ = self.grounding_dino.detect_object(image_path, "cubes", all_objects=True)
        self.all_objects_bbs = dino_all_features.get("bounding_box", [])
        return self.all_objects_bbs


    def retrieve_object(self, object_name):
        """
        1. Detect all cubes using Grounding DINO.
        2. Detect the requested object using CLIPSeg.
        3. Compare the bounding box (BB) from CLIPSeg with all DINO-detected BBs and select the one with overlap.
        4. If CLIPSeg does not detect the object, re-run DINO for the requested object and return its BB.
        
        Returns:
            tuple: (final_detected_object_BB, detected_by) where:
                   - final_detected_object_BB is the bounding box (or []/None if not found).
                   - detected_by is a ModelName enum indicating which model was used.
            If no objects are found at all, returns None.
        """
        # Step 2: Use CLIPSeg to find the requested object
        clip_features, _, _ = self.clipseg.segment_object(self.image_path, object_name, return_most_probable=True)
        clip_bb = clip_features.get("bounding_box", [])
        
        self.final_detected_object_bb = []
        detected_by = ModelName.Neither

        if self.all_objects_bbs:
            # If CLIPSeg returned a bounding box, try to find an overlapping DINO object.
            if clip_bb:
                for box in self.all_objects_bbs:
                    if self.is_overlap(box, clip_bb):
                        self.final_detected_object_bb = box
                        detected_by = ModelName.CLIP_Seg
                        print("CLIPSeg detected the object with overlapping DINO box.")
                        break
            # If no overlapping box is found or CLIPSeg did not return a box, re-run DINO for the specific object.
            if not clip_bb or not self.final_detected_object_bb:
                dino_features, _ = self.grounding_dino.detect_object(self.image_path, object_name, all_objects=False)
                self.final_detected_object_bb = dino_features.get("bounding_box", [])
                detected_by = ModelName.Grounding_Dino
                print("Grounding DINO detected the object.")
            return self.final_detected_object_bb, detected_by
        else:
            return None

    def is_overlap(self, box1, box2):
        """
        Check if two bounding boxes overlap.
        
        Each box is defined as (x, y, width, height).
        """
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2
        box1_x2, box1_y2 = x1 + w1, y1 + h1
        box2_x2, box2_y2 = x2 + w2, y2 + h2

        if x1 > box2_x2 or x2 > box1_x2:
            return False
        if y1 > box2_y2 or y2 > box1_y2:
            return False
        return True

    def extract_centeroid(self):
        """
        Finds the center of the brightest surface within a given bounding box.
            
        Returns:
            tuple: The center coordinates (x, y) in the original image coordinate system,
                   or None if no bright surface is found.
        """
        x, y, w, h = self.final_detected_object_bb
        x_max, y_max = x + w, y + h
        center_point = None

        image = cv2.imread(self.image_path)
        if image is None:
            print("Failed to load image from", self.image_path)
            return None

        # Crop the region of interest (ROI)
        roi = image[y:y_max, x:x_max]
        if roi.size == 0:
            print("ROI is empty. Check the bounding box coordinates.")
            return None

        # Convert ROI to grayscale and threshold to isolate bright areas
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, blockSize=11, C=2)
        

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("No contours found for the brightest surface.")
            return None
        
        # Select the contour with the largest area (brightest surface)
        bright_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(bright_contour)
        
        if M["m00"] == 0:
            return None  # Avoid division by zero
        
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # Translate the centroid coordinates back to the original image coordinate system
        self.center_point = (x + cX, y + cY)
        return self.center_point
    
    def image_retrivedObject(self):
        image = cv2.imread(self.image_path)
        # Draw bounding box from the detected object
        x, y, w, h = self.final_detected_object_bb
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 2)

        if self.center_point is not None:
            cv2.circle(image, self.center_point, 3, (255, 255, 255), thickness=-1)
        else:
            print("surface center not found.")

        return image

if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    pic_dir = os.path.join(src_dir, 'simulation', 'images')
    image_path = os.path.join(pic_dir, '_image28.jpg')

    save_path = os.path.join(current_dir, 'images', 'clipDino', 'pose')

    clip_dino_obj = ClipDino()
    query = "blue cube"
    result = clip_dino_obj.retrieve_object(image_path, query)
    
    if result is not None:
        final_bb, utilized_model = result
        print("Utilized model:", utilized_model)
       
        center_point = clip_dino_obj.extract_centeroid()
        print(center_point)
        image = clip_dino_obj.image_retrivedObject()
    else:
        print("Object not found")

    cv2.imshow("Final Object", image)
    cv2.imwrite(f"{save_path}/{query}_pose.jpg", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
