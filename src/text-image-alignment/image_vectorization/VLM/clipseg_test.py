### comparing CLIPSeg with Grounding DINO

from CLIPSeg import CLIPSeg
import os
import matplotlib.pyplot as plt
import cv2
import time
import numpy as np

current_dir = os.path.dirname(__file__)
src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
pic_dir = src_dir + '/simulation/images/'
# image_path = pic_dir + '_image1.jpg'

save_path = current_dir + '/images/CLIPSeg/test/'

# plt.figure(figsize=(8, 5))
number_of_images  = 10
columns = int(number_of_images/2)
rows = int(number_of_images / columns)
print(columns, rows)


clipseg = CLIPSeg()

queries = ["red cube", "blue cube", "green cube", "yellow cube", "pink cube", "cubes", "robot arm"]
process_time = []

print(os.listdir(pic_dir))

fig, axes = plt.subplots(rows, columns, figsize=(20, 8), gridspec_kw={'width_ratios': [1] * columns})  # Ensure equal widths
axes = np.array(axes).ravel()
for query in queries: 
    average_process_time = 0
    # fig.title = f"query: '{query}'"
    for i, image_name in enumerate(os.listdir(pic_dir)[:number_of_images]):
        image_path = pic_dir + image_name

        clip_process_time_start = time.time()
        clip_features, clip_seg_img, clip_bb_img = clipseg.segment_object(image_path, query)
        clip_process_time_end = time.time()

        process_time.append(clip_process_time_end - clip_process_time_start)

        print(f"CLIPSeg features: {clip_features}")

        clip_seg_img = cv2.cvtColor(clip_seg_img, cv2.COLOR_BGR2RGB)

        axes[i].imshow(clip_seg_img)
        axes[i].set_title(f"image{i}: {image_name}")

        # Remove axes for clean display
        for ax in axes:
            ax.axis("off")

    average_process_time = np.mean(process_time)
    fig.suptitle(f"query: '{query}', average process time: {average_process_time:.2f}s")

    plt.tight_layout()
    plt.savefig(save_path + f'{query}_T0.2.jpg')
    print("image saved")
    # plt.show()


