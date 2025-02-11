### comparing CLIPSeg with Grounding DINO

from CLIPSeg import CLIPSeg
from Grounding_DINO import Grounding_Dino
import os
import matplotlib.pyplot as plt
import cv2
import time

current_dir = os.path.dirname(__file__)
src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
pic_dir = src_dir + '/simulation/images/'
# image_path = pic_dir + '_image1.jpg'

save_path = current_dir + '/images/compare/'

fig = plt.figure(figsize=(10, 10))
number_of_images  = 6
columns = 3
rows = number_of_images


clipseg = CLIPSeg()
grounding_dino = Grounding_Dino()

text = "blue box"

print(os.listdir(pic_dir))


for i, image_name in zip(range(1, rows*columns +1, columns),  os.listdir(pic_dir)[:number_of_images]):
    image_path = pic_dir + image_name
    print(image_path)

    clip_features, clip_seg_img, clip_bb_img = clipseg.segment_object(image_path, text)
    dino_features, dino_bb_img = grounding_dino.detect_object(image_path, text)

    print(f"CLIPSeg features: {clip_features}")
    print(f"Grounding DINO features: {dino_features}")

    clip_seg_img = cv2.cvtColor(clip_seg_img, cv2.COLOR_BGR2RGB)
    clip_bb_img = cv2.cvtColor(clip_bb_img, cv2.COLOR_BGR2RGB)
    dino_bb_img = cv2.cvtColor(dino_bb_img, cv2.COLOR_BGR2RGB)

    plt.subplot(rows, columns, i)
    plt.imshow(clip_seg_img)
    plt.axis('off')
    plt.title("CLIPSeg-segmenet objects")

    plt.subplot(rows, columns, i+1)
    plt.imshow(clip_bb_img)
    plt.axis('off')
    plt.title("CLIPSeg-box objects")

    plt.subplot(rows, columns, i+2)
    plt.imshow(dino_bb_img)
    plt.axis('off')
    plt.title("Grounding DINO-box objects")

plt.savefig(save_path + 'compare.png')
plt.show()

