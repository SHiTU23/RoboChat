### comparing CLIPSeg with Grounding DINO

from clipDino import ClipDino
import os
import matplotlib.pyplot as plt
import cv2
import time
import numpy as np

current_dir = os.path.dirname(__file__)
src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
pic_dir = src_dir + '/simulation/images/'

save_path = current_dir + '/images/clipDino/pose/'

number_of_images  = 10
columns = int(number_of_images/2)
rows = int(number_of_images / columns)
print(columns, rows)


clipDino = ClipDino()

queries = ["red cube", "blue cube", "green cube", "yellow cube", "pink cube"]
process_time = []

print(os.listdir(pic_dir))

fig, axes = plt.subplots(rows, columns, figsize=(20, 8), gridspec_kw={'width_ratios': [1] * columns})  # Ensure equal widths
axes = np.array(axes).ravel()
for query in queries: 
    average_process_time = 0
    # fig.title = f"query: '{query}'"
    for i, image_name in enumerate(os.listdir(pic_dir)[:number_of_images]):
        image_path = pic_dir + image_name

        clipDino_process_time_start = time.time()
        result = clipDino.find_object(image_path, query)
        clipDino_process_time_end = time.time()
        process_time.append(clipDino_process_time_end - clipDino_process_time_start)

        image = cv2.imread(image_path)

        if result is not None:
            clipDino_boundingBox, chosen_model_name = result
            x, y, w, h = clipDino_boundingBox
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            ### center point
            center_point = clipDino.find_bright_surface_center(clipDino_boundingBox)
            if center_point is not None:
                cv2.circle(image, center_point, 2, (255, 0, 0), thickness=-1)
                cv2.putText(image, f"({center_point})", (x - 30, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)


        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        axes[i].imshow(image)
        axes[i].set_title(f"image{i}: {image_name}")

        # Remove axes for clean display
        for ax in axes:
            ax.axis("off")

    average_process_time = np.mean(process_time)
    fig.suptitle(f"query: '{query}', average process time: {average_process_time:.2f}s")

    plt.tight_layout()
    plt.savefig(save_path + f'adaptive_{query}.jpg')
    print("image saved")
    # plt.show()


