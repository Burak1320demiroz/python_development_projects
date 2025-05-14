#!/usr/bin/python3

import math
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from matplotlib.animation import FuncAnimation

KONUM = "/current_position"
CAR_WIDTH = 1  # Aracın genişliği metre cinsinden (örneğin 1.5 metre)

class Konum:
    def __init__(self):
        rospy.init_node("dtar_lite_planner", anonymous=True)
        self.star = None
        self.yaw = 0.0
        rospy.Subscriber(KONUM, Odometry, self.position_callback)

    def position_callback(self, msg: Odometry):
        self.star = msg.pose.pose.position
        
        # Orientation'dan quaternion al
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # Quaternion'ı Euler açılarına çevir (roll, pitch, yaw)
        _, _, self.yaw = euler_from_quaternion(orientation_list)
        
        rospy.loginfo(f"Konum ==> x={self.star.x}, y={self.star.y}, yaw={math.degrees(self.yaw)} derece")

class AutonomousSteeringControl:
    def __init__(self):
        # Direksiyon turu ve tekerlek açısı ilişkisi
        self.max_steering_wheel_turns = 1.4  # Direksiyonun maksimum tur sayısı
        self.max_wheel_angle = 30  # Maksimum tekerlek açısı derece cinsinden
        self.degrees_per_turn = self.max_wheel_angle / self.max_steering_wheel_turns  # 1 direksiyon turu başına düşen tekerlek açısı

    def calculate_steering_angle(self, point1, point2, current_yaw):
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        target_angle = math.atan2(delta_y, delta_x)  # Hedef yönü açısını hesapla
        # steering_angle = target_angle - current_yaw  # Direksiyon açısını hesapla
        steering_angle = target_angle - current_yaw  # Direksiyon açısını hesapla
        # Direksiyon açısını -180 ile 180 derece arasında normalize et
        steering_angle = (steering_angle + math.pi) % (2 * math.pi) - math.pi
        steering_angle_degrees = math.degrees(steering_angle)
        return steering_angle_degrees

    def calculate_pwm_and_wheel_angle(self, steering_angle):
        # Direksiyon açısının büyüklüğüne göre PWM değeri ve tekerlek açısı hesapla
        pwm_base = 128  # Orta nokta
        pwm_range = 127  # PWM aralığı (127 sola, 128-255 sağa)
        max_steering_angle = self.max_wheel_angle  # Maksimum tekerlek açısı derece cinsinden
        wheel_angle = min(max(steering_angle, -max_steering_angle), max_steering_angle)  # Tekerlek açısını direksiyon açısına göre hesapla
        steering_wheel_turns = wheel_angle / self.degrees_per_turn  # Direksiyonun kaç tur döndüğünü hesapla

        if steering_angle < 0:  # Sola dönüş
            pwm_value = pwm_base - int((abs(steering_wheel_turns) / self.max_steering_wheel_turns) * pwm_range)
            pwm_value = max(0, pwm_value)  # PWM sınırları içerisinde kalma

        elif steering_angle > 0:  # Sağa dönüş
            pwm_value = pwm_base + int((abs(steering_wheel_turns) / self.max_steering_wheel_turns) * pwm_range)
            pwm_value = min(255, pwm_value)  # PWM sınırları içerisinde kalma

        else:  # Düz gidiyorsa
            pwm_value = pwm_base  # Orta konumda, direksiyon düz

        return pwm_value, wheel_angle

def main():
    # Konum sınıfı ile ROS düğümünü başlat ve konum güncellemelerini al
    konum = Konum()

    # Direksiyon kontrol sınıfını başlat
    control = AutonomousSteeringControl()

    # Hedef pozisyonu belirle
    target_position = (-4, 4)

    # Matplotlib figür ve eksenleri oluştur
    fig, ax = plt.subplots()
    ax.set_xlim(-30, 30)
    ax.set_ylim(-30, 30)
    ax.set_xlabel('X Koordinatı')
    ax.set_ylabel('Y Koordinatı')
    ax.grid(True)

    # Görselleştirme için vektörler
    left_target_vector = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='r', label="Sol Hedef Yönü")
    right_target_vector = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='b', label="Sağ Hedef Yönü")
    yaw_vector = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='y', label="Mevcut Yön (Yaw)")
    wheel_vector = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='g', label="Tekerlek Açısı")
    current_pos_marker, = ax.plot([], [], 'bo', label="Mevcut Pozisyon")
    target_pos_marker, = ax.plot(target_position[0], target_position[1], 'go', label="Hedef Pozisyon")
    ax.legend()

    def update(frame):
        if konum.star is not None:
            current_position = (konum.star.x, konum.star.y)
            current_yaw = konum.yaw  # Gelen odometry mesajından elde edilen yaw değeri

            # Sağ ve sol noktaların hesaplanması
            left_position = (
                current_position[0] - (CAR_WIDTH / 2) * math.sin(current_yaw),
                current_position[1] + (CAR_WIDTH / 2) * math.cos(current_yaw)
            )
            right_position = (
                current_position[0] + (CAR_WIDTH / 2) * math.sin(current_yaw),
                current_position[1] - (CAR_WIDTH / 2) * math.cos(current_yaw)
            )

            # Sağ ve sol noktalardan hedefe göre direksiyon açılarını hesapla
            left_steering_angle = control.calculate_steering_angle(left_position, target_position, current_yaw)
            right_steering_angle = control.calculate_steering_angle(right_position, target_position, current_yaw)

            # Ortalama direksiyon açısını hesapla
            average_steering_angle = (left_steering_angle + right_steering_angle) / 2

            # Ortalama direksiyon açısına göre PWM ve tekerlek açısını hesapla
            pwm, wheel_angle = control.calculate_pwm_and_wheel_angle(average_steering_angle)

            # Vektörleri güncelle
            left_delta_x = target_position[0] - left_position[0]
            left_delta_y = target_position[1] - left_position[1]
            right_delta_x = target_position[0] - right_position[0]
            right_delta_y = target_position[1] - right_position[1]

            # Aracın mevcut yönünü gösteren vektör
            yaw_x = math.cos(current_yaw)
            yaw_y = math.sin(current_yaw)

            # Tekerlek açısını gösteren vektör
            wheel_angle_rad = math.radians(wheel_angle)
            wheel_x = math.cos(current_yaw + wheel_angle_rad)
            wheel_y = math.sin(current_yaw + wheel_angle_rad)

            left_target_vector.set_offsets([left_position])
            left_target_vector.set_UVC(left_delta_x, left_delta_y)

            right_target_vector.set_offsets([right_position])
            right_target_vector.set_UVC(right_delta_x, right_delta_y)

            yaw_vector.set_offsets([current_position])
            yaw_vector.set_UVC(yaw_x, yaw_y)

            wheel_vector.set_offsets([current_position])
            wheel_vector.set_UVC(wheel_x, wheel_y)

            current_pos_marker.set_data(current_position)

            ax.set_title(f"Ortalama Direksiyon Açisi: {average_steering_angle:.2f} derece, PWM: {pwm}, Tekerlek Açısı: {wheel_angle:.2f} derece")

    ani = FuncAnimation(fig, update, interval=100)  # 100 ms aralıklarla güncelle

    plt.show()

if __name__ == "__main__":
    main()
