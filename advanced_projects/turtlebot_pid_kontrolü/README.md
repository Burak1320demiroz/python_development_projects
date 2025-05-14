# Turtlebot PID Kontrolü

Bu proje, ROS altyapısı ile çalışan bir otonom araçta PID kontrolü kullanarak hedef noktalara yönelmeyi sağlar.

## Özellikler

- `/XEst` ve IMU verisi ile mevcut konum ve yön bilgisini okuma  
- `/planned_path` üzerinden gelen yol noktalarına göre hedefe gitme  
- PID denetleyici ile hız ve yön kontrolü  
- Fren ve motor komutlarını yayınlama  
- Hedefe ulaşıldığında durma ve sonraki nokta için devam etme  

## Gereksinimler

- Python 3  
- ROS (örneğin: Noetic)  
- `rospy`, `numpy`, `networkx`  
- Mesaj tipleri: `geometry_msgs/Point`, `nav_msgs/Path`, `std_msgs/Float64`, `sensor_msgs/Imu`, `rospy_tutorials/Floats`, `cart_sim` paketindeki mesajlar  

## Kullanım

1. ROS ortamını başlatın.  
2. Abone olunan konular (`/XEst`, `/imu/data`, `/planned_path`) yayınlanıyor olmalı.  
3. Aşağıdaki komutla script çalıştırılır:  
   ```bash
   rosrun <paket_adı> turtlebot_pid.py
