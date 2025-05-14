# Position Publisher (Konum Yayınlayıcı)

Bu ROS düğümü, `/base_pose_ground_truth` başlığından alınan `Odometry` mesajlarını işleyerek:

- Tüm konum bilgisini `/current_position` başlığına
- Yalnızca yönelim (orientation/yaw) bilgisini `/current_orientation` başlığına yayınlar.

## Özellikler

- `Odometry` mesajı tam olarak yeniden yayınlanır
- Quaternion yön verisi Euler dönüşümü ile yaw değeri hesaplanır
- Yön bilgisi `Quaternion` tipinde ayrıca yayınlanır
- Konsola pozisyon ve yaw değeri yazdırılır
