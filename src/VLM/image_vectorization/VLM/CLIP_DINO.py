### comparing CLIPSeg with Grounding DINO

from CLIPSeg import CLIPSeg
from Grounding_DINO import Grounding_Dino
import os

current_dir = os.path.dirname(__file__)
src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
pic_dir = src_dir + '/simulation/images/'
image_path = pic_dir + '_image1.jpg'

save_path = current_dir + '/images/combine/'

clipseg = CLIPSeg()
grounding_dino = Grounding_Dino()

text = "blue box"

clipseg.segment_object(image_path, text)
clipseg.show_segmented_image(segmentations=True, bounding_boxes=True)
clipseg.save_image(segmented_image=True, bounding_boxed_image=True, save_path=save_path)

grounding_dino.detect_object(image_path, text)
grounding_dino.show_image()
grounding_dino.save_image(save_path=save_path)
