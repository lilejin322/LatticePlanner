import math
from shapely.geometry import box, Polygon
from shapely.affinity import rotate

def calculate_rotation_angle_rad(polygon: Polygon) -> float:
    # 获取多边形的外部顶点
    vertices = list(polygon.exterior.coords)[:-1]  # 忽略重复的最后一个顶点
    
    # 找到最接近水平的边
    min_angle = None
    for i in range(len(vertices)):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % len(vertices)]
        
        delta_x = p2[0] - p1[0]
        delta_y = p2[1] - p1[1]
        
        # 计算旋转角度（弧度）
        angle_rad = math.atan2(delta_y, delta_x)
        
        # 规范化角度到[0, π)范围
        angle_rad = angle_rad % math.pi
        
        if min_angle is None or abs(angle_rad) < abs(min_angle):
            min_angle = angle_rad
    
    return min_angle

# 创建一个矩形并旋转
rect = box(0.0, 0.0, 2.0, 1.0)
rotated_rect = rotate(rect, math.pi/4, origin='center',use_radians=True)
ro = rotate(rotated_rect, -math.pi/4, origin='center',use_radians=True)

# 计算旋转角度（弧度）
print(list(ro.exterior.coords))