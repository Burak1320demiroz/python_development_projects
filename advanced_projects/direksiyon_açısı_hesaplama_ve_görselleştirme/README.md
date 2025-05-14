# Direksiyon Açısı Hesaplama ve Görselleştirme

Bu proje, bir aracın konumuna ve yönüne göre hedefe ulaşmak için gereken direksiyon açısını hesaplar ve bu durumu görselleştirir. ROS ortamında çalışır ve canlı olarak matplotlib ile vektörler çizer.

## Özellikler

- `/current_position` konusundan `Odometry` mesajı alınır
- Quaternion yön verisi Euler (yaw) açısına çevrilir
- Aracın sol ve sağ teker noktaları kullanılarak hedefe olan yönler hesaplanır
- Ortalama direksiyon açısı ve PWM değeri belirlenir
- Mevcut yön, hedef yönü ve tekerlek yönü matplotlib ile canlı olarak çizilir
