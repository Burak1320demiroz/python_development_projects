# Doğru Uzunluğu Hesaplayıcı

Bu proje, (0,0) noktasından belirli bir açıyla çizilen doğrunun, m × 2a boyutlarında bir dikdörtgenin kenarlarıyla kesiştiği noktaya kadar olan uzunluğunu hesaplar.

## Özellikler

- Açı değeri derece cinsinden verilir.
- Dikdörtgenin kenarları:
  - x ∈ [0, m]
  - y ∈ [0, 2a]
- E noktasından (0,0) açıyla çıkan doğru çizilir.
- Doğrunun dikdörtgenin sınırlarıyla kesiştiği ilk noktaya kadar olan mesafe hesaplanır.

## Kullanım

```python
from your_module import calculate_line_length

m = 120    # Genişlik
a = 100    # Yüksekliğin yarısı (toplam yükseklik 2a olur)
theta = 45 # Açı (derece)

uzunluk = calculate_line_length(m, a, theta)
print(f"Açı {theta}° için uzunluk: {uzunluk}")
