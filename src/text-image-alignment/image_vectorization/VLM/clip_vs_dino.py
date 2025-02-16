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



clipseg = CLIPSeg()
grounding_dino = Grounding_Dino()

text = "red cube"

print(os.listdir(pic_dir))


for i, image_name in enumerate(os.listdir(pic_dir)):
    image_path = pic_dir + image_name
    print(image_path)

    clip_process_time_start = time.time()
    clip_features, clip_seg_img, clip_bb_img = clipseg.segment_object(image_path, text)
    clip_process_time_end = time.time()

    dino_process_time_start = time.time()
    dino_features, dino_bb_img = grounding_dino.detect_object(image_path, text)
    dino_process_time_end = time.time()

    print(f"CLIPSeg features: {clip_features}")
    print(f"Grounding DINO features: {dino_features}")


    clip_seg_img = cv2.cvtColor(clip_seg_img, cv2.COLOR_BGR2RGB)
    clip_bb_img = cv2.cvtColor(clip_bb_img, cv2.COLOR_BGR2RGB)
    dino_bb_img = cv2.cvtColor(dino_bb_img, cv2.COLOR_BGR2RGB)


    fig, axes = plt.subplots(1, 3, figsize=(12, 4), gridspec_kw={'width_ratios': [1, 1, 1]})  # Ensure equal widths
    fig.suptitle(f"query: '{text}'")

    axes[0].imshow(clip_seg_img)
    axes[0].set_title("CLIPSeg-segmenet objects")

    axes[1].imshow(clip_bb_img)
    axes[1].set_title("CLIPSeg-box objects, pt: {:.2f}s".format(clip_process_time_end - clip_process_time_start))

    axes[2].imshow(dino_bb_img)
    axes[2].set_title("Grounding DINO-box objects, pt: {:.2f}s".format(dino_process_time_end - dino_process_time_start))

    # Remove axes for clean display
    for ax in axes:
        ax.axis("off")


    plt.tight_layout()
    plt.savefig(save_path + f'{text}_compare{i}.jpg')
    
    print("image saved")
    # plt.show()


