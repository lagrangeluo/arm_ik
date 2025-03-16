import numpy as np

def calculate_distance_with_intrinsics(
    fx,              # 焦距（水平方向，像素单位）
    fy,              # 焦距（垂直方向，像素单位）
    object_width_mm, # 物体实际宽度，单位毫米
    object_height_mm,# 物体实际高度，单位毫米
    pixel_width,     # 物体在图像中的像素宽度
    pixel_height     # 物体在图像中的像素高度
):
    """
    使用相机内参和物体尺寸计算相机到物体的距离
    
    参数:
    - fx, fy: 相机内参中的焦距（像素单位）
    - object_width_mm, object_height_mm: 物体的实际尺寸，单位毫米
    - pixel_width, pixel_height: 物体在图像中占据的像素大小
    
    返回:
    - 相机到物体的估计距离（毫米）
    """
    # 基于宽度计算距离
    distance_by_width = (fx * object_width_mm) / pixel_width
    
    # 基于高度计算距离
    distance_by_height = (fy * object_height_mm) / pixel_height
    
    # 计算最终距离（取宽度和高度计算结果的平均值）
    distance = (distance_by_width + distance_by_height) / 2
    
    return {
        "distance_by_width_mm": distance_by_width,
        "distance_by_height_mm": distance_by_height,
        "average_distance_mm": distance
    }

# 使用您提供的参数进行计算
fx_color, fy_color = 453.354, 453.354
cx_color, cy_color = 319.444, 214.468  # 主点偏移（此例中不需要使用）

# 物体实际尺寸（毫米）
object_width_mm = 20  # 2厘米 = 20毫米
object_height_mm = 40  # 4厘米 = 40毫米

# 示例：假设物体在图像中占据100x200像素
def calculate_for_pixel_size(pixel_width, pixel_height):
    result = calculate_distance_with_intrinsics(
        fx_color, fy_color, 
        object_width_mm, object_height_mm, 
        pixel_width, pixel_height
    )
    
    print(f"物体像素大小: {pixel_width}x{pixel_height}")
    print(f"基于宽度的距离估计: {result['distance_by_width_mm']:.2f} 毫米 ({result['distance_by_width_mm']/1000:.2f} 米)")
    print(f"基于高度的距离估计: {result['distance_by_height_mm']:.2f} 毫米 ({result['distance_by_height_mm']/1000:.2f} 米)")
    print(f"平均距离: {result['average_distance_mm']:.2f} 毫米 ({result['average_distance_mm']/1000:.2f} 米)")
    print("----------------------------------")

# 计算不同像素大小下的距离
calculate_for_pixel_size(100, 200)  # 较大物体（近距离）
calculate_for_pixel_size(50, 100)   # 中等距离
calculate_for_pixel_size(25, 50)    # 较远距离

# 函数用于根据给定距离计算物体的像素大小
def calculate_pixel_size_at_distance(
    fx, fy, 
    object_width_mm, object_height_mm, 
    distance_mm
):
    # 根据距离计算像素宽度和高度
    pixel_width = (fx * object_width_mm) / distance_mm
    pixel_height = (fy * object_height_mm) / distance_mm
    
    return pixel_width, pixel_height

print("\n反向计算：给定距离下的像素大小")
distances = [500, 1000, 2000]  # 毫米
for dist in distances:
    pixel_w, pixel_h = calculate_pixel_size_at_distance(
        fx_color, fy_color, 
        object_width_mm, object_height_mm, 
        dist
    )
    print(f"距离 {dist} 毫米 ({dist/1000:.1f} 米) 时，物体像素大小约为: {pixel_w:.1f}x{pixel_h:.1f} 像素")

# 使用像素面积计算距离
def calculate_distance_from_pixel_area(
    fx, fy,
    object_area_mm2,  # 物体的实际面积（平方毫米）
    pixel_area        # 物体在图像中的像素面积
):
    # 使用几何平均焦距
    f_avg = np.sqrt(fx * fy)
    
    # 估计距离（基于面积比例）
    # 注意：这是一个简化的近似
    distance = f_avg * np.sqrt(object_area_mm2 / pixel_area)
    
    return distance

print("\n基于像素面积的距离估计")
# 物体实际面积
object_area_mm2 = object_width_mm * object_height_mm  # 20mm x 40mm = 800 mm²

# 不同的像素面积示例
pixel_areas = [20000, 5000, 1250]  # 像素面积
for area in pixel_areas:
    dist = calculate_distance_from_pixel_area(
        fx_color, fy_color,
        object_area_mm2,
        area
    )
    print(f"物体像素面积为 {area} 时，估计距离为: {dist:.2f} 毫米 ({dist/1000:.2f} 米)")