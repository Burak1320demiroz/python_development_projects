
# Pandas Notları

Pandas, Python’da **veri analizi ve veri işleme** için kullanılan en önemli kütüphanelerden biridir. Özellikle **tablo (DataFrame) ve seri (Series) veri yapıları**, **eksik veri işlemleri**, **veri okuma/yazma** ve **veri gruplama** gibi işlemler için kullanılır.

---

## 1. Pandas Kurulum ve İçeri Aktarma

```bash
pip install pandas
```

```python
import pandas as pd
```

---

## 2. Temel Veri Yapıları

### Series

Tek boyutlu veri yapısıdır. NumPy array’e benzer, ancak etiketli indekslere sahiptir.

```python
s = pd.Series([10, 20, 30, 40], index=['a','b','c','d'])
print(s)
# a    10
# b    20
# c    30
# d    40
```

### DataFrame

Tablo şeklinde veri yapısıdır. Satırlar ve sütunlardan oluşur.

```python
data = {
    "isim": ["Ali", "Ayşe", "Mehmet"],
    "yas": [25, 30, 22],
    "sehir": ["İstanbul", "Ankara", "Bursa"]
}

df = pd.DataFrame(data)
print(df)
```

---

## 3. DataFrame Temel İşlemler

```python
df.head()      # ilk 5 satır
df.tail(3)     # son 3 satır
df.shape       # satır ve sütun sayısı
df.info()      # genel bilgi
df.describe()  # sayısal sütunların istatistiksel özeti
df.columns     # sütun adları
df.index       # satır indeksleri
df.dtypes      # sütunların veri tipleri
```

---

## 4. Veri Okuma ve Yazma

```python
pd.read_csv("data.csv")           # csv dosyası okuma
df.to_csv("output.csv", index=False)  # csv dosyası yazma

pd.read_excel("data.xlsx")        # excel dosyası okuma
df.to_excel("output.xlsx")        # excel dosyası yazma

pd.read_json("data.json")         # json okuma
df.to_json("output.json")         # json yazma
```

---

## 5. Sütun ve Satırlara Erişim

```python
df["isim"]         # tek sütun
df[["isim","yas"]] # birden fazla sütun

df.loc[0]          # etiket ile satır seçme
df.iloc[1]         # indeks ile satır seçme

df.loc[0, "isim"]  # belirli hücre
df.iloc[1, 2]      # satır/sütun indeks ile
```

---

## 6. Filtreleme ve Koşullu Seçim

```python
df[df["yas"] > 25]           # yaşı 25’ten büyük olanlar
df[df["sehir"] == "Ankara"]  # şehri Ankara olanlar

df[(df["yas"] > 20) & (df["sehir"] == "Bursa")]  
# Koşulları birleştirme
```

---

## 7. Yeni Sütun Ekleme ve Güncelleme

```python
df["yas2"] = df["yas"] * 2         # yeni sütun ekleme
df["sehir"] = df["sehir"].str.upper()  # sütunu güncelleme
```

---

## 8. Silme İşlemleri

```python
df.drop("yas2", axis=1, inplace=True)   # sütun silme
df.drop(0, axis=0, inplace=True)        # satır silme
```

---

## 9. Eksik Veriler

```python
df.isnull()        # eksik veriler True/False
df.isnull().sum()  # eksik veri sayısı
df.dropna()        # eksik satırları silme
df.fillna(0)       # eksikleri 0 ile doldurma
df["yas"].fillna(df["yas"].mean())  # sütunun ortalamasıyla doldurma
```

---

## 10. Sıralama

```python
df.sort_values("yas")             # küçükten büyüğe
df.sort_values("yas", ascending=False)  # büyükten küçüğe
```

---

## 11. Gruplama ve Aggregation

```python
df.groupby("sehir")["yas"].mean()   # şehirlere göre yaş ortalaması
df.groupby("sehir").agg({"yas": ["mean","max","min"]})
```

---

## 12. Pivot Table

```python
df.pivot_table(values="yas", index="sehir", aggfunc="mean")
```

---

## 13. Birleştirme ve Join İşlemleri

```python
df1 = pd.DataFrame({"id": [1,2,3], "isim": ["Ali","Ayşe","Mehmet"]})
df2 = pd.DataFrame({"id": [1,2,3], "yas": [25,30,22]})

pd.merge(df1, df2, on="id")       # join işlemi
pd.concat([df1, df2], axis=0)     # dikey birleştirme
pd.concat([df1, df2], axis=1)     # yatay birleştirme
```

---

## 14. Apply ve Lambda Fonksiyonları

```python
df["yas_kare"] = df["yas"].apply(lambda x: x**2)
```

---

## 15. Tarih ve Zaman İşlemleri

```python
tarih_df = pd.DataFrame({"tarih": pd.date_range("2025-01-01", periods=5, freq="D")})
tarih_df["gun"] = tarih_df["tarih"].dt.day
tarih_df["ay"] = tarih_df["tarih"].dt.month
```

---

## 16. İleri Düzey Seçimler

```python
df.query("yas > 25 and sehir == 'ANKARA'")  # SQL benzeri sorgu
df["isim"].unique()     # eşsiz değerler
df["isim"].nunique()    # farklı değer sayısı
df["isim"].value_counts() # değerlerin frekansı
```

---

## 17. İndeksleme ve Resetleme

```python
df.set_index("isim", inplace=True)   # sütunu indeks yap
df.reset_index(inplace=True)         # indeksi sıfırla
```

---

## 18. String Fonksiyonları

```python
df["isim"].str.lower()
df["isim"].str.contains("a")
df["isim"].str.replace("Ali","Veli")
```

---

## 19. Kaydetme ve Yükleme

```python
df.to_csv("veri.csv", index=False)
df2 = pd.read_csv("veri.csv")
```

---
