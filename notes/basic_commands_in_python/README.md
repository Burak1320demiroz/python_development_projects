

---

### 1. **`print(x)`**

**İşlev**: `x` değişkeninin değerini ekrana yazdırır.

```python
x = 5
print(x)  # 5
```

---

### 2. **`input(prompt)`**

**İşlev**: Kullanıcıdan veri alır. `prompt`, kullanıcıya gösterilecek mesajdır.

```python
name = input("Adınızı girin: ")
print("Merhaba, " + name)  # Kullanıcı "Ali" girdiyse: "Merhaba, Ali"
```

---

### 3. **`len(x)`**

**İşlev**: `x`'in uzunluğunu (eleman sayısını) döndürür. Genellikle listeler, dizeler ve diğer iterable veri türleri için kullanılır.

```python
text = "Merhaba"
print(len(text))  # 7
```

---

### 4. **`type(x)`**

**İşlev**: `x`'in veri tipini döndürür.

```python
x = 5
print(type(x))  # <class 'int'>
```

---

### 5. **`int(x)`**

**İşlev**: `x`'i tam sayıya dönüştürür.

```python
x = "10"
print(int(x))  # 10
```

---

### 6. **`float(x)`**

**İşlev**: `x`'i ondalıklı sayıya dönüştürür.

```python
x = "10.5"
print(float(x))  # 10.5
```

---

### 7. **`str(x)`**

**İşlev**: `x`'i string (metin) türüne dönüştürür.

```python
x = 10
print(str(x))  # '10'
```

---

### 8. **`range(start, stop, step)`**

**İşlev**: `start` ile `stop` arasındaki sayıları, belirtilen `step` kadar artırarak döndürür. `stop` dahil edilmez.

```python
for i in range(1, 10, 2):
    print(i)  # 1, 3, 5, 7, 9
```

---

### 9. **`sum(x)`**

**İşlev**: `x`'teki tüm elemanları toplar. `x` bir liste, tuple, vb. olabilir.

```python
numbers = [1, 2, 3, 4]
print(sum(numbers))  # 10
```

---

### 10. **`min(x)` ve `max(x)`**

**İşlev**: `x`'teki en küçük ve en büyük değeri döndürür.

```python
numbers = [1, 2, 3, 4]
print(min(numbers))  # 1
print(max(numbers))  # 4
```

---

### 11. **`abs(x)`**

**İşlev**: `x`'in mutlak değerini döndürür.

```python
x = -5
print(abs(x))  # 5
```

---

### 12. **`round(x, n)`**

**İşlev**: `x`'i `n` ondalıklı basamağa yuvarlar.

```python
x = 3.14159
print(round(x, 2))  # 3.14
```

---

### 13. **`list(x)`**

**İşlev**: `x`'i listeye dönüştürür. Genellikle iterable (yinelemeli) veri türlerini listeye dönüştürmek için kullanılır.

```python
x = (1, 2, 3)
print(list(x))  # [1, 2, 3]
```

---

### 14. **`dict()`**

**İşlev**: Boş bir sözlük (dictionary) oluşturur.

```python
my_dict = dict()
print(my_dict)  # {}
```

---

### 15. **`set(x)`**

**İşlev**: `x`'i kümeye dönüştürür (tekrarlanan öğeleri çıkarır).

```python
x = [1, 2, 3, 3, 4]
print(set(x))  # {1, 2, 3, 4}
```

---

### 16. **`del x`**

**İşlev**: `x`'i siler. Genellikle değişkenler veya dizilerin elemanları için kullanılır.

```python
x = [1, 2, 3]
del x[1]  # x artık [1, 3]
print(x)
```

---

### 17. **`append(x)`**

**İşlev**: Bir listeye `x` öğesini ekler.

```python
my_list = [1, 2]
my_list.append(3)
print(my_list)  # [1, 2, 3]
```

---

### 18. **`pop()`**

**İşlev**: Listenin son elemanını çıkarır ve döndürür.

```python
my_list = [1, 2, 3]
print(my_list.pop())  # 3
print(my_list)  # [1, 2]
```

---

### 19. **`extend(x)`**

**İşlev**: Bir listeye başka bir listeyi ekler.

```python
my_list = [1, 2]
my_list.extend([3, 4])
print(my_list)  # [1, 2, 3, 4]
```

---

### 20. **`join(x)`**

**İşlev**: Bir dizi elemanı, belirli bir ayraçla birleştirir.

```python
words = ['Python', 'programming', 'is', 'fun']
print(" ".join(words))  # "Python programming is fun"
```

---

### 21. **`enumerate(x)`**

**İşlev**: Bir listeyi iterasyona sokarken, her öğenin indeksini de verir.

```python
my_list = ['a', 'b', 'c']
for index, value in enumerate(my_list):
    print(index, value)
# 0 a
# 1 b
# 2 c
```

---

### 22. **`zip(*iterables)`**

**İşlev**: Birden fazla iterabl'i (liste, tuple vb.) yan yana getirir ve bunları birleştirir.

```python
names = ['Alice', 'Bob', 'Charlie']
ages = [24, 30, 22]
zipped = list(zip(names, ages))
print(zipped)  # [('Alice', 24), ('Bob', 30), ('Charlie', 22)]
```

---

### 23. **`map(function, iterable)`**

**İşlev**: Bir fonksiyonu, bir iterabl'deki her elemana uygular.

```python
numbers = [1, 2, 3, 4]
squared = list(map(lambda x: x**2, numbers))
print(squared)  # [1, 4, 9, 16]
```

---

### 24. **`filter(function, iterable)`**

**İşlev**: Bir iterabl'deki öğeleri, verilen fonksiyona göre filtreler (True döndürenleri alır).

```python
numbers = [1, 2, 3, 4, 5]
even_numbers = list(filter(lambda x: x % 2 == 0, numbers))
print(even_numbers)  # [2, 4]
```

---

### 25. **`lambda`**

**İşlev**: Kısa, anonim fonksiyonlar tanımlar.

```python
add = lambda x, y: x + y
print(add(5, 3))  # 8
```

---

### 26. **`try`...`except`**

**İşlev**: Hata (exception) yakalama mekanizmasıdır. Kod bloğunda oluşan hataları işler.

```python
try:
    x = 1 / 0
except ZeroDivisionError:
    print("Sıfıra bölme hatası!")
```

---

### 27. **`assert`**

**İşlev**: Şartları kontrol eder ve eğer şart sağlanmazsa hata fırlatır.

```python
x = 10
assert x == 10  # Şart sağlanır, hiçbir şey olmaz
assert x == 5  # AssertionError hatası verir
```

---

### 28. **`with`** (Context Manager)

**İşlev**: Kaynakların düzgün bir şekilde açılıp kapatılmasını sağlar. Dosya işlemleri için sıkça kullanılır.

```python
with open("file.txt", "w") as f:
    f.write("Merhaba Python!")
```

---

### 29. **`if`, `else`, `elif`**

**İşlev**: Koşullu ifadeleri kontrol etmek için kullanılır. `if` belirli bir şartı kontrol eder, `elif` (else if) ek bir şartı kontrol eder, ve `else` herhangi bir şart sağlanmazsa yapılacak işlemi belirtir.

#### Örnek:

```python
x = 10
if x > 5:
    print("x 5'ten büyük")
elif x == 5:
    print("x 5'e eşit")
else:
    print("x 5'ten küçük")
# Çıktı: x 5'ten büyük
```

* **`if`**: İlk şart kontrol edilir.
* **`elif`**: Eğer önceki şart yanlışsa, başka bir koşul kontrol edilir.
* **`else`**: Hiçbir koşul sağlanmazsa, `else` bloğundaki kod çalıştırılır.

---

### 30. **`def`** (Fonksiyon Tanımlama)

**İşlev**: `def` anahtar kelimesi, fonksiyonları tanımlamak için kullanılır. Fonksiyonlar, belirli bir işlemi birden çok yerde kullanmak için tekrarlanabilir kod bloklarıdır.

#### Örnek:

```python
def greet(name):
    return f"Merhaba, {name}!"

print(greet("Ali"))  # Merhaba, Ali!
```

* **`def`**: Fonksiyonun başlangıcını belirtir.
* **Fonksiyon adı**: `greet`, fonksiyonun adıdır.
* **Parametre**: `name`, fonksiyona verilen değeri tutar.

---

### 31. **`class`** (Sınıf Tanımlama)

**İşlev**: `class`, bir nesne yönelimli programlama (OOP) konsepti olan sınıf tanımlamak için kullanılır. Sınıflar, nesnelerin (objelerin) özelliklerini ve davranışlarını tanımlar.

#### Örnek:

```python
class Dog:
    def __init__(self, name, age):
        self.name = name
        self.age = age

    def bark(self):
        return f"{self.name} havlıyor!"

# Sınıf kullanımı
my_dog = Dog("Karabas", 3)
print(my_dog.bark())  # Karabas havlıyor!
```

* **`class`**: Sınıfı tanımlar.
* **`__init__`**: Sınıfın başlangıç (constructor) metodudur ve nesne oluşturulduğunda çağrılır.
* **`self`**: Nesneye (örneğe) referans verir, sınıf içindeki her metoda ilk parametre olarak gelir.

---

### 32. **`pass`**

**İşlev**: Hiçbir şey yapmayan bir komuttur. Genellikle bir bloğun tamamlanmadığı ya da geliştirileceği durumlarda kullanılır.

#### Örnek:

```python
def placeholder_function():
    pass  # Bu fonksiyon henüz tamamlanmadı
```

* **`pass`**: Bu komut, bir şey yapmamak için kullanılır. Kodun çalışmasını engellemeden bloğun tamamlanmadığını belirtebiliriz.

---

### 33. **`break`**

**İşlev**: Bir döngüyü (for veya while) anında sonlandırmak için kullanılır.

#### Örnek:

```python
for i in range(10):
    if i == 5:
        break
    print(i)
# Çıktı: 0 1 2 3 4
```

* **`break`**: Döngüyü anında sonlandırır. Bu örnekte, i değeri 5 olduğunda döngü sona erer.

---

### 34. **`continue`**

**İşlev**: Döngüde bir adımı atlamak için kullanılır. `continue` komutunun ardından gelen kodlar çalıştırılmaz, döngü bir sonraki iterasyona geçer.

#### Örnek:

```python
for i in range(5):
    if i == 3:
        continue  # 3'ü atla
    print(i)
# Çıktı: 0 1 2 4
```

* **`continue`**: Bu komut, döngüde belirli bir adımı atlamamızı sağlar.

---

### 35. **`else` (Döngü ile Kullanım)**

**İşlev**: `else`, döngülerin sonlanması durumunda çalışan bir bloktur. `for` veya `while` döngülerinde `break` komutuyla erken çıkış yapılmazsa çalışır.

#### Örnek:

```python
for i in range(5):
    print(i)
else:
    print("Döngü bitti")
# Çıktı:
# 0
# 1
# 2
# 3
# 4
# Döngü bitti
```

* **`else`**: Döngü sonlandırıldığında (veya bitirildiğinde), `else` bloğu çalışır. `break` komutu döngüyü erken bitirirse, `else` bloğu çalışmaz.

---

### 36. **`try`...`except` (Hata Yakalama)**

**İşlev**: Kodda oluşabilecek hataları yakalamak ve işlem yapabilmek için kullanılır. Bu, hataların programın çökmesine neden olmasını engeller.

#### Örnek:

```python
try:
    x = 1 / 0  # Sıfıra bölme hatası
except ZeroDivisionError:
    print("Sıfıra bölme hatası!")
```

* **`try`**: Hata alınabilecek kod bloğu.
* **`except`**: Hata durumunda yapılacak işlemi belirtir.

---

### 37. **`finally`**

**İşlev**: `try` ve `except` bloğundan sonra çalışacak, hata olsa bile mutlaka çalışacak kısımdır.

#### Örnek:

```python
try:
    x = 1 / 0
except ZeroDivisionError:
    print("Sıfıra bölme hatası!")
finally:
    print("Her durumda bu mesajı görürsünüz.")
```

* **`finally`**: Hata olsa da olmasa da bu blok çalışır.

---

### 38. **`lambda`** (Anonim Fonksiyonlar)

**İşlev**: Tek satırlık, anonim fonksiyonlar tanımlar. `lambda`, fonksiyonlar için kısa ve hızlı bir yazım sağlar.

#### Örnek:

```python
add = lambda x, y: x + y
print(add(5, 3))  # 8
```

* **`lambda`**: Kısa fonksiyonlar yazmak için kullanılır.

---

### 39. **`assert`**

**İşlev**: Belirli bir şartın doğru olup olmadığını kontrol eder. Eğer yanlışsa, program bir hata verir.

#### Örnek:

```python
x = 10
assert x == 10  # Şart doğru, bir şey olmaz
assert x == 5   # AssertionError hatası verir
```

* **`assert`**: Belirtilen koşulun doğru olduğunu doğrular, yanlışsa hata fırlatır.

---

### 40. **`global` ve `nonlocal`**

* **`global`**: Bir değişkenin global (program genelinde erişilebilen) olduğunu belirtir.

  ```python
  x = 5
  def func():
      global x
      x = 10
  func()
  print(x)  # 10
  ```

* **`nonlocal`**: Bir değişkenin, dış fonksiyonlarda tanımlanan ancak iç fonksiyonlarda değiştirilebilen bir değişken olduğunu belirtir.

  ```python
  def outer():
      x = 5
      def inner():
          nonlocal x
          x = 10
      inner()
      print(x)  # 10
  outer()
  ```

---