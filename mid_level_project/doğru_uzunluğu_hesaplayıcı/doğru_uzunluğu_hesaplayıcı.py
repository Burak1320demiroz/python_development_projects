import numpy as np

def calculate_line_length(m, a, theta):
    # Theta'yı radyan cinsine çevir
    theta_rad = np.radians(theta)
    
    # E noktasının koordinatları (0,0)
    x_e, y_e = 0, 0
    
    # Düzleme sınırlarını belirle
    x_min, x_max = 0, m
    y_min, y_max = 0, 2 * a
    
    # Eğimin belirlenmesi
    if np.cos(theta_rad) == 0:
        return y_max  # dikey durumda sadece y mesafesi
    
    slope = np.tan(theta_rad)
    
    # X = m çizgisi için çözüm
    x_cross = x_max
    y_cross = slope * x_cross
    if 0 <= y_cross <= y_max:
        return np.sqrt((x_cross - x_e) ** 2 + (y_cross - y_e) ** 2)
    
    # Y = 2a çizgisi için çözüm
    y_cross = y_max
    x_cross = y_cross / slope
    if 0 <= x_cross <= x_max:
        return np.sqrt((x_cross - x_e) ** 2 + (y_cross - y_e) ** 2)
    
    # X = 0 çizgisi için çözüm
    x_cross = x_min
    y_cross = slope * x_cross
    if 0 <= y_cross <= y_max:
        return np.sqrt((x_cross - x_e) ** 2 + (y_cross - y_e) ** 2)
    
    # Y = 0 çizgisi için çözüm
    y_cross = y_min
    x_cross = y_cross / slope
    if 0 <= x_cross <= x_max:
        return np.sqrt((x_cross - x_e) ** 2 + (y_cross - y_e) ** 2)
    
    return None  # Teorik olarak buraya ulaşılmaması lazım

# Kullanım:
m = 120  # AB uzunluğu
a = 100  # BC yarısı (2a = 6)
theta = 45  # E noktasından açılan açı
length = calculate_line_length(m, a, theta)
print(f"Açı {theta} derece olduğunda, uzunluk: {length}")
