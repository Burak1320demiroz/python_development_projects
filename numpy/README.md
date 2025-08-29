
# 📘 NumPy Notları (Basit + Orta Seviye)

NumPy (**Numerical Python**) → Python’da **bilimsel hesaplama** için kullanılan en güçlü kütüphanelerden biridir.
Özellikle:
 **Matris ve vektör işlemleri**
 **İstatistik**
 **Lineer cebir**
 **Veri işleme** için kullanılır.

---

## NumPy Kurulum ve Kullanım

```bash
pip install numpy   # Kurulum
```

```python
import numpy as np  # NumPy’yi genelde np kısaltmasıyla kullanırız
```

---

## Array (Dizi) Oluşturma

### 🔹 Temel Dizi

```python
arr = np.array([1, 2, 3, 4])
print(arr)   # [1 2 3 4]
```

### 🔹 Çok Boyutlu Dizi

```python
arr2 = np.array([[1, 2, 3], [4, 5, 6]])
# 2 satır, 3 sütunluk matris
```

### 🔹 Sıralı Diziler

```python
np.arange(0, 10, 2)   # [0 2 4 6 8]
np.linspace(0, 1, 5)  # [0.   0.25 0.5 0.75 1. ]
```

### 🔹 Hazır Diziler

```python
np.zeros((2,3))   # 2x3 sıfırlardan oluşan matris
np.ones((3,3))    # 3x3 1’lerden oluşan matris
np.eye(4)         # 4x4 birim matris
np.random.rand(2,2)   # 0-1 arası rastgele sayılar
np.random.randint(1, 10, (3,3))  # 1-10 arası tam sayılar
```

---

## Array Özellikleri

```python
a = np.array([[1,2,3],[4,5,6]])

a.shape   # (2,3) → boyut
a.ndim    # 2 → kaç boyutlu
a.size    # 6 → toplam eleman sayısı
a.dtype   # int32, float64 gibi veri tipi
```

---

## Elemanlara Erişim (Indexing & Slicing)

```python
arr = np.array([10,20,30,40,50])

arr[0]     # 10 (ilk eleman)
arr[-1]    # 50 (son eleman)
arr[1:4]   # [20 30 40] (1’den 3. indise kadar)

matris = np.array([[1,2,3],[4,5,6]])
matris[0,1]   # 2 (0. satır 1. sütun)
matris[:,1]   # [2 5] (tüm satırların 1. sütunu)
```

---

## Matematiksel İşlemler

### 🔹 Eleman Bazlı İşlemler

```python
arr = np.array([1,2,3,4])

arr + 10    # [11 12 13 14]
arr * 2     # [2 4 6 8]
arr ** 2    # [ 1  4  9 16]
```

### 🔹 Matematiksel Fonksiyonlar

```python
np.sqrt(arr)   # karekök
np.log(arr)    # logaritma
np.sin(arr)    # sinüs
```

### 🔹 İstatistiksel Fonksiyonlar

```python
arr = np.array([10,20,30,40,50])

np.mean(arr)   # 30.0 (ortalama)
np.median(arr) # 30.0 (medyan)
np.std(arr)    # standart sapma
np.var(arr)    # varyans
np.min(arr)    # minimum
np.max(arr)    # maksimum
np.sum(arr)    # toplam
```

---

## Şekil Değiştirme (Reshape & Transpose)

```python
arr = np.arange(12)
arr.reshape(3,4)   # 3x4 matris

matris = np.array([[1,2,3],[4,5,6]])
matris.T           # transpozunu alır
```

---

## Birleştirme ve Ayırma

```python
a = np.array([1,2,3])
b = np.array([4,5,6])

np.concatenate([a,b])  # [1 2 3 4 5 6]

m1 = np.array([[1,2],[3,4]])
m2 = np.array([[5,6]])
np.vstack([m1,m2])     # dikey birleştirme
np.hstack([m1,m2.T])   # yatay birleştirme
```

---

## Koşullu Seçim (Boolean Indexing)

```python
arr = np.array([10,20,30,40,50])

arr[arr > 25]        # [30 40 50]
arr[arr % 20 == 0]   # [20 40]
```

---

## Kopyalama ve Görünüm

```python
a = np.array([1,2,3])
b = a        # aynı belleği paylaşır
b[0] = 99
print(a)     # [99  2  3]

c = a.copy() # bağımsız kopya
c[0] = 0
print(a)     # [99  2  3] değişmez
```

---

## Lineer Cebir (Linear Algebra)

```python
A = np.array([[1,2],[3,4]])
B = np.array([[2,0],[1,3]])

np.dot(A,B)          # matris çarpımı
np.linalg.det(A)     # determinant
np.linalg.inv(A)     # ters matris
np.linalg.eig(A)     # özdeğer ve özvektörler
```

---

## Rastgele Sayılar

```python
np.random.seed(42)   # sonuçların aynı çıkması için sabitleme
np.random.rand(2,2)  # 0-1 arası
np.random.randn(2,2) # normal dağılım
np.random.randint(1,10,5) # 1-10 arası rastgele 5 sayı
```

---

## Eksik Veriler

```python
arr = np.array([1,2,np.nan,4])

np.isnan(arr)        # [False False  True False]
np.nanmean(arr)      # nan’leri görmezden gelerek ortalama
```

---

## Dosya İşlemleri

```python
np.savetxt("data.txt", arr)   # txt’ye kaydet
np.loadtxt("data.txt")        # txt’den yükle

np.save("data.npy", arr)      # numpy formatında kaydet
np.load("data.npy")           # numpy formatından yükle
```

---

## Broadcasting

Farklı boyuttaki diziler otomatik genişletilerek işlenir.

```python
a = np.array([1,2,3])
b = np.array([[10],[20],[30]])

a + b
# [[11 12 13]
#  [21 22 23]
#  [31 32 33]]
```

 **Axis parametresi** → işlemin satır mı sütun mu yapılacağını belirler

```python
arr = np.array([[1,2,3],[4,5,6]])
np.sum(arr, axis=0)   # sütun toplamı [5 7 9]
np.sum(arr, axis=1)   # satır toplamı [6 15]
```

 **Fancy Indexing** → birden fazla indeksi seçme

```python
arr = np.array([10,20,30,40,50])
arr[[0,2,4]]   # [10 30 50]
```

 **Sort ve Argsort**

```python
arr = np.array([40,10,50,20])
np.sort(arr)      # [10 20 40 50]
np.argsort(arr)   # [1 3 0 2] (indeks sırası)
```

 **Unique** → tekrar edenleri silme

```python
arr = np.array([1,2,2,3,3,3])
np.unique(arr)   # [1 2 3]
```
