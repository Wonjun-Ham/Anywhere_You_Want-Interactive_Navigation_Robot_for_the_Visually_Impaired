# create_topomap.py 개선한 코드.
# 저장 디렉토리 변경됨
# readme 보면,  임의로 정한 RATE에 따라 받아오는 current image를 
# 해당 디렉토리(./deployment/topomaps/images/topomap)에 저장
# 이를 해결하기 위해 생성한 TOPOPMAP image들 간에 유사도를 
# 계산하여, 결과값이 일정 임계값 이하의 값을 갖는 이미지들만 
# 재저장하는 후처리 코드를 작성하였다.
# (저장 디렉토리: ./deployment/topomaps/images/process_topomap) 
# 유사도 측정은 이미지의 구조, 밝기, 대비 등을 고려하는 
# SSIM(구조적 유사도) 알고리즘을 사용하였다.

import os
import cv2
from PIL import Image
from skimage.metrics import structural_similarity as ssim

TOPOMAP_IMAGES_DIR = "../topomaps/images/topomap"
OUTPUT_TOPOMAP_DIR = "../topomaps/images/process_topomap"
topomap_filenames = sorted(os.listdir(TOPOMAP_IMAGES_DIR), key=lambda x: int(x.split(".")[0]))
num_nodes = len(os.listdir(TOPOMAP_IMAGES_DIR))

SSIM_THRESHOLD = 0.6
img_num = 0

def remove_files_in_dir(dir_path: str):
    for f in os.listdir(dir_path):
        file_path = os.path.join(dir_path, f)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print("Failed to delete %s. Reason: %s" % (file_path, e))

if not os.path.isdir(OUTPUT_TOPOMAP_DIR):
    os.makedirs(OUTPUT_TOPOMAP_DIR)
else:
    print(f"{OUTPUT_TOPOMAP_DIR} already exists. Removing previous images...")
    remove_files_in_dir(OUTPUT_TOPOMAP_DIR)

for i in range(num_nodes-1):
    image_path_1 = os.path.join(TOPOMAP_IMAGES_DIR, topomap_filenames[i])
    image1 = cv2.imread(image_path_1)
    image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)

    image_path_2 = os.path.join(TOPOMAP_IMAGES_DIR, topomap_filenames[i+1])
    image2 = cv2.imread(image_path_2)
    image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)

    ssim_score, diff = ssim(image1, image2, channel_axis=2, win_size=11, full=True)
    diff = (diff * 255).astype("uint8")

    print(f"img{i}, img{i+1} SSIM_score: {ssim_score}")

    if i == 0:
        image1 = Image.fromarray(image1)
        image1.save(os.path.join(OUTPUT_TOPOMAP_DIR, f"{img_num}.png"))
        img_num += 1
    
    if ssim_score < SSIM_THRESHOLD:
        image2 = Image.fromarray(image2)
        image2.save(os.path.join(OUTPUT_TOPOMAP_DIR, f"{img_num}.png"))
        img_num += 1
    
    