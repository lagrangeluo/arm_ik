import cv2
import numpy as np

# 读取图像
img = cv2.imread("../data/image_00001.png")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# HSV银色高亮区域
lower_silver = np.array([97, 5, 78])
upper_silver = np.array([122, 53, 124])
mask = cv2.inRange(hsv, lower_silver, upper_silver)

# Step 1: 形态学去噪（可调kernel）
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

# Step 2: 面积筛选（去掉小连通域）
contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
filtered_mask = np.zeros_like(mask)

for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 1000:  # ⬅️ 根据图像大小调节阈值
        cv2.drawContours(filtered_mask, [cnt], -1, 255, thickness=cv2.FILLED)

# Step 3: 可视化高亮
highlighted = img.copy()
highlighted[filtered_mask > 0] = [0, 255, 0]  # 绿色
# output = cv2.addWeighted(img, 0.7, highlighted, 0.3, 0)
output = cv2.addWeighted(img, 0, highlighted, 1, 0)

# heatmap = cv2.applyColorMap(filtered_mask, cv2.COLORMAP_JET)  # or COLORMAP_HOT
# output = cv2.addWeighted(img, 0.6, heatmap, 0.6, 0)

# Step 4: 保存图像
cv2.imwrite("../data/silver_mask_filtered.jpg", filtered_mask)
cv2.imwrite("../data/highlighted_silver_area_filtered.jpg", output)
