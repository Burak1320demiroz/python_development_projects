# Harita Görselleştirici

Bu proje, ROS üzerinden gelen konum, yol, hedef ve engel verilerini matplotlib ile canlı olarak çizer.

## Özellikler

- `/current_position` (`Odometry`) → robot pozisyonu  
- `/planned_path` (`Path`) → planlanan yol  
- `/goal_position` (`Point`) → hedef konum  
- `/obstacles` (`PointCloud`) → sabit engeller  
- `/gecici_engeller` (`Point`) → geçici engeller  
- Matplotlib ile güncellenen grafik (FuncAnimation)
