
# Matplotlib Notları

Matplotlib, Python’da en çok kullanılan **görselleştirme kütüphanelerinden** biridir. Veri analizi, bilimsel hesaplama ve makine öğrenmesi projelerinde grafikler çizmek için sıkça tercih edilir. En çok kullanılan alt modülü **`pyplot`**’tur.

---

## 1. Kurulum ve İçe Aktarma

```bash
pip install matplotlib
```

```python
import matplotlib.pyplot as plt
import numpy as np
```

---

## 2. Temel Grafik Çizimi

```python
x = np.array([1, 2, 3, 4, 5])
y = np.array([2, 4, 6, 8, 10])

plt.plot(x, y)
plt.show()
```

* `plt.plot()` → çizgi grafiği çizer.
* `plt.show()` → grafiği ekrana getirir.

---

## 3. Grafik Özelleştirme

### Başlık, Eksen Adları

```python
plt.plot(x, y)
plt.title("Basit Çizgi Grafiği")
plt.xlabel("X Ekseni")
plt.ylabel("Y Ekseni")
plt.show()
```

### Renk ve Stil

```python
plt.plot(x, y, color="red", linestyle="--", marker="o")
```

* `color` → renk (`"red"`, `"blue"`, `"#00ff00"`)
* `linestyle` → çizgi stili (`"-"`, `"--"`, `":"`)
* `marker` → nokta işareti (`"o"`, `"s"`, `"*"`, `"+"`)

---

## 4. Çoklu Grafik Çizimi

```python
y2 = np.array([1, 3, 5, 7, 9])

plt.plot(x, y, label="2x")
plt.plot(x, y2, label="x+?")

plt.legend()   # açıklama ekler
plt.show()
```

---

## 5. Farklı Grafik Türleri

### Çizgi Grafiği

```python
plt.plot(x, y)
```

### Nokta (Scatter) Grafiği

```python
plt.scatter(x, y)
```

### Çubuk (Bar) Grafiği

```python
plt.bar(x, y)
plt.barh(x, y)   # yatay bar grafiği
```

### Histogram

```python
data = np.random.randn(1000)
plt.hist(data, bins=30, color="orange")
```

### Pasta Grafiği

```python
labels = ["A","B","C","D"]
values = [20, 30, 25, 25]

plt.pie(values, labels=labels, autopct="%1.1f%%")
```

---

## 6. Birden Fazla Grafik (Subplots)

```python
x = np.linspace(0,10,100)
y1 = np.sin(x)
y2 = np.cos(x)

plt.subplot(2,1,1)  # 2 satır, 1 sütun, 1. grafik
plt.plot(x, y1, label="sin(x)")
plt.legend()

plt.subplot(2,1,2)  # 2 satır, 1 sütun, 2. grafik
plt.plot(x, y2, label="cos(x)", color="red")
plt.legend()

plt.show()
```

---

## 7. Figure ve Axes Mantığı (Object-Oriented API)

Daha gelişmiş kontrol için `figure` ve `axes` kullanılır.

```python
fig, ax = plt.subplots()

ax.plot(x, y1, label="sin(x)")
ax.plot(x, y2, label="cos(x)")
ax.set_title("Trigonometri")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.legend()

plt.show()
```

---

## 8. Grafik Özelleştirme

### Grid (Izgara)

```python
plt.plot(x, y)
plt.grid(True)
```

### Eksen Aralıkları

```python
plt.plot(x, y)
plt.xlim(0, 10)
plt.ylim(0, 20)
```

### Yazı Boyutu ve Stil

```python
plt.title("Grafik", fontsize=14, fontweight="bold")
```

### Renk Seçenekleri

* `"r"`: kırmızı
* `"g"`: yeşil
* `"b"`: mavi
* `"c"`: camgöbeği
* `"m"`: mor
* `"y"`: sarı
* `"k"`: siyah

---

## 9. Scatter Grafik Özelleştirme

```python
x = np.random.rand(50)
y = np.random.rand(50)
colors = np.random.rand(50)
sizes = 500 * np.random.rand(50)

plt.scatter(x, y, c=colors, s=sizes, alpha=0.5, cmap="viridis")
plt.colorbar()  # renk skalası
plt.show()
```

---

## 10. Çoklu Figure Kullanımı

```python
plt.figure(1)
plt.plot(x, y1)

plt.figure(2)
plt.plot(x, y2)
```

---

## 11. Kaydetme

```python
plt.plot(x, y)
plt.savefig("grafik.png", dpi=300)   # yüksek çözünürlükle kaydet
```

---

### Çift Eksenli Grafik

```python
x = np.arange(0,10,0.1)
y1 = np.sin(x)
y2 = np.exp(x/3)

fig, ax1 = plt.subplots()

ax2 = ax1.twinx()
ax1.plot(x, y1, "g-")
ax2.plot(x, y2, "b-")

ax1.set_xlabel("X")
ax1.set_ylabel("sin(x)", color="g")
ax2.set_ylabel("exp(x/3)", color="b")
```

### 3D Grafikler

```python
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

x = np.linspace(-5, 5, 100)
y = np.linspace(-5, 5, 100)
X, Y = np.meshgrid(x, y)
Z = np.sin(np.sqrt(X**2 + Y**2))

ax.plot_surface(X, Y, Z, cmap="viridis")
plt.show()
```

---

## 13. Stil Ayarları

```python
plt.style.available  # mevcut stiller
plt.style.use("ggplot")
```

Örnek stiller: `"seaborn"`, `"classic"`, `"ggplot"`, `"dark_background"`

---

## 14. Pandas ile Entegrasyon

Matplotlib, Pandas DataFrame ile çok kolay kullanılabilir.

```python
import pandas as pd

df = pd.DataFrame({
    "yas": [25,30,22,40],
    "maas": [4000, 5000, 3500, 7000]
})

df.plot(kind="bar")
plt.show()
```

---

## 15. Özet

* `plot()` → çizgi grafiği
* `scatter()` → nokta grafiği
* `bar()`, `barh()` → çubuk grafik
* `hist()` → histogram
* `pie()` → pasta grafiği
* `subplot()` → çoklu grafik
* `figure` ve `axes` → daha fazla kontrol
* `savefig()` → kaydetme
* `style` → grafik temasını değiştirme
* 3D grafikler ve çift eksenli grafikler desteklenir

---