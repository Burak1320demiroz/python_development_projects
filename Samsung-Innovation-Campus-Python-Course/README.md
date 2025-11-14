# Samsung-Innovation-Campus-Python-Course

`print` fonksiyonu, parantez içindeki metni veya değeri ekrana yazdırmak için kullanılır.
`sep` parametresi, birden fazla argüman yazdırırken aralarına hangi karakterin geleceğini belirler.
`end` parametresi, satır sonunda varsayılan `\n` yerine kullanılacak karakteri seçmeye yarar.
`try-except` bloğu, hatalı girişlerde programın çökmesini engelleyip kullanıcıya uyarı vermemizi sağlar.
Çarpma operatörü (`*`), metinleri tekrar etmek veya sayıları çarpmak için kullanılır.
Toplama operatörü (`+`), sayıları toplar; metinlerde ise birleştirme (concatenation) yapar.
`format` metodu, metin içine değişken yerleştirmek için kullanılır.
`float()` fonksiyonu, kullanıcıdan alınan string girişini ondalıklı sayıya dönüştürmek için kullanılır.
`input()` fonksiyonu, konsoldan metin olarak kullanıcı girdisi almak için kullanılır.
Aritmetik operatörler: `+` (toplama), `-` (çıkarma), `*` (çarpma), `/` (bölme), `//` (tam bölme), `%` (kalan), `**` (üs alma).
`type()` fonksiyonu, bir değişkenin veri tipini (int, float, str, bool, list, tuple, dict, complex, None) döndürür.
Operatör önceliği: parantez içi işlemler önce, sonra üs alma, çarpma/bölme, en son toplama/çıkarma yapılır.
`int()` ve `str()` fonksiyonları, veri tiplerini birbirine dönüştürmek için kullanılır; string ile sayısal işlem yapmadan önce dönüşüm gerekir.
String'lerde `*` operatörü kullanılırken sağ tarafta mutlaka tam sayı olmalıdır, aksi halde TypeError oluşur.
Simultaneous assignment (`x, y = 100, 200`) aynı anda birden fazla değişkene değer atamak için kullanılır; değişken ve değer sayısı eşit olmalıdır.
Multiple assignment (`num1 = num2 = num3 = 200`) aynı değeri birden fazla değişkene atamak için kullanılır.
Birleşik atama operatörleri (`+=`, `-=`, `*=`, `/=`) değişkenin değerini kısa yoldan güncellemek için kullanılır.
`isdigit()` metodu, string'in sadece rakamlardan oluşup oluşmadığını kontrol eder ve boolean döndürür.
`ZeroDivisionError`, bir sayının sıfıra bölünmeye çalışıldığında oluşan hata türüdür.
Karşılaştırma operatörleri (`>`, `<`, `==`, `!=`, `>=`, `<=`) iki değeri karşılaştırır ve boolean (True/False) döndürür.
`bool()` fonksiyonu, herhangi bir değeri boolean'a dönüştürür; 0, None, boş string ve boş liste False, diğerleri True döndürür.
`==` operatörü değerleri karşılaştırır, `is` operatörü ise aynı nesneye referans edip etmediğini kontrol eder.
`in` operatörü, bir değerin başka bir değer (string, liste vb.) içinde olup olmadığını kontrol eder.
Zincirleme karşılaştırma (`0 < n < 200`) birden fazla koşulu tek satırda kontrol etmek için kullanılır.
Mantıksal operatörler (`and`, `or`, `not`) boolean değerlerle çalışır; `and` her ikisi True ise True, `or` en az biri True ise True, `not` değeri tersine çevirir.
`range()` fonksiyonu, belirli bir aralıktaki sayıları üretmek için kullanılır; `range(2, 7)` 2'den 6'ya kadar sayılar üretir.
`if` ifadesi bir koşul kontrol eder; koşul True ise altındaki blok çalıştırılır.
`elif` birden fazla koşulu sırayla kontrol etmek için kullanılır; önceki koşul False ise bir sonraki kontrol edilir.
`else` hiçbir koşul sağlanmadığında çalışacak kodu belirtmek için kullanılır.
Girintileme (indentation) Python'da blokları belirlemek için kullanılır; if, elif, else blokları girintileme ile ayrılır.
`IndentationError`, girintileme hatası olduğunda oluşan hata türüdür; if bloğu içindeki kodlar mutlaka girintili olmalıdır.
`split()` metodu string'i belirtilen ayırıcıya göre böler ve liste döndürür; varsayılan ayırıcı boşluktur, `split(',')` virgül ile ayırır.
`math.sqrt()` fonksiyonu, bir sayının karekökünü hesaplar; `import math` ile math modülü import edilmelidir.
İç içe koşullu ifadeler (nested if-else) bir `if-else` bloğunun başka bir `if-else` bloğunun içine yerleştirilmesiyle oluşur; karmaşık koşulları kontrol etmek için kullanılır.
`random` modülü rastgele sayılar üretmek için kullanılır; `import random` ile import edilmelidir.
`random.randint(a, b)` a ile b arasında (dahil) rastgele tam sayı üretir.
`random.randrange(n)` 0 ile n-1 arasında rastgele sayı üretir; yazı tura gibi iki seçenekli oyunlar için kullanılır.
`eval()` fonksiyonu string'i Python kodu olarak çalıştırır; güvenlik açısından dikkatli kullanılmalıdır, kullanıcı girdisiyle kullanılmamalıdır.
`.lower()` metodu string'i küçük harfe çevirir; büyük/küçük harf duyarsız karşılaştırmalar için kullanılır.
`.upper()` metodu string'i büyük harfe çevirir; çıktıyı büyük harfle göstermek için kullanılır.
`for` döngüsü tekrarlayan işlemler için kullanılır; liste, string gibi sıralı nesneler üzerinde iterasyon yapar.
`in` operatörü döngülerde sıralı nesnelerin elemanlarını dolaşmak için kullanılır; `for i in range(5)` gibi.
Iterasyon (iteration) bir döngünün her tekrarıdır; döngü kontrol değişkeni her iterasyonda farklı değer alır.
Döngü kontrol değişkeni (`i`, `j` vb.) döngü içinde kullanılan değişkendir; kullanılmayan değişkenler için `_` (underscore) kullanılabilir.
`range(start, end, step)` fonksiyonu: `start` varsayılan 0, `step` varsayılan 1, `end` dahil değildir; sadece tam sayılarla çalışır.
`list()` fonksiyonu bir sıralı nesneyi (string, range vb.) listeye dönüştürür; `list("Hello")` → `['H', 'e', 'l', 'l', 'o']`.
`sum()` fonksiyonu bir sıralı nesnenin (liste, range vb.) elemanlarının toplamını döndürür; `sum(range(1, 11))` → `55`.
`while` döngüsü koşul True olduğu sürece çalışır; tekrar sayısı belirsiz olduğunda veya koşul tabanlı döngülerde kullanılır.
`break` ifadesi döngüyü tamamen sonlandırır; koşul sağlandığında döngüden çıkmak için kullanılır.
`continue` ifadesi mevcut iterasyonu atlayıp bir sonraki iterasyona geçer; belirli durumları atlamak için kullanılır.
`random.random()` 0 ile 1 arasında rastgele ondalıklı sayı üretir.
`random.shuffle(liste)` listenin elemanlarını rastgele karıştırır; liste yerinde değiştirilir.
`random.choice(liste)` listeden rastgele bir eleman seçer ve döndürür.
`random.sample(liste, n)` listeden rastgele n eleman seçer ve liste olarak döndürür; elemanlar tekrar etmez.
`import random as rd` modülü farklı bir isimle (alias) import etmek için kullanılır; `rd.randint()` gibi kullanılır.
`format()` detayları: `{:5d}` tam sayı için 5 karakter genişlik, `{:6.3f}` ondalıklı sayı için 6 karakter genişlik ve 3 ondalık basamak.
İç içe döngüler (nested loops) bir döngünün başka bir döngünün içinde olmasıdır; çarpım tablosu, matris işlemleri için kullanılır.
`for` döngüsü tekrar sayısı belli olduğunda daha uygundur; `while` döngüsü tekrar sayısı belirsiz veya koşul tabanlı olduğunda daha uygundur.
`while True` sonsuz döngü oluşturur; içeride `break` ile çıkılmalıdır; sayı tahmin oyunu gibi uygulamalarda kullanılır.
`append()` metodu listeye yeni bir eleman ekler; `liste.append(değer)` şeklinde kullanılır.
`len()` fonksiyonu bir sıralı nesnenin (liste, string vb.) uzunluğunu döndürür; `len([1,2,3])` → `3`.
`extend()` metodu bir listenin elemanlarını başka bir listeye ekler; `liste1.extend(liste2)` şeklinde kullanılır.
`[::-1]` slice notasyonu bir sıralı nesneyi ters çevirir; `[1,2,3][::-1]` → `[3,2,1]`.
Snake matrix (yılan matrisi) çift satırlarda soldan sağa, tek satırlarda sağdan sola artan sayıların düzenlendiği matris yapısıdır.
Palindrom sayı sağdan sola ve soldan sağa okunduğunda aynı olan sayıdır; `121`, `3443` gibi.
`%` (mod) operatörü bir sayının başka bir sayıya bölümünden kalanı döndürür; `10 % 3` → `1`, `123 % 10` → `3` (son basamak).
`//` (tam bölme) operatörü bir sayının başka bir sayıya bölümünden tam kısmı döndürür; `10 // 3` → `3`, `123 // 10` → `12` (son basamak hariç).
Liste (list) birden fazla değeri tek bir değişkende saklamak için kullanılır; köşeli parantez `[]` ile tanımlanır, farklı veri tiplerini içerebilir.
Liste indeksleme 0'dan başlar; `liste[0]` ilk eleman, `liste[-1]` son eleman, `liste[-2]` sondan ikinci elemandır.
`IndexError` liste indeksleme sırasında geçersiz indeks kullanıldığında oluşan hata türüdür; liste 6 elemanlıysa maksimum indeks 5'tir.
`insert(index, değer)` metodu listeye belirli bir indeks konumuna eleman ekler; `liste.insert(2, 'x')` 2. indekse 'x' ekler.
`pop()` metodu listenin son elemanını siler ve döndürür; `liste.pop()` son elemanı döndürür, `liste.pop(2)` 2. indeksteki elemanı döndürür.
`remove(değer)` metodu listeden belirtilen değeri siler; değer listede yoksa `ValueError` oluşur, bu yüzden önce `in` ile kontrol edilmelidir.
`del` anahtar kelimesi listeden indeks ile eleman silmek için kullanılır; `del liste[3]` 3. indeksteki elemanı siler.
Liste birleştirme `+` operatörü ile yapılır; `liste1 + liste2` iki listenin elemanlarını birleştirir.
`max(liste)` fonksiyonu listedeki en büyük değeri döndürür; `min(liste)` en küçük değeri döndürür.
`any(liste)` fonksiyonu listede herhangi bir True değer varsa True döndürür; tüm elemanlar False ise False döndürür.
Liste dilimleme (slicing) `liste[başlangıç:bitiş:adım]` formatında kullanılır; bitiş indeksi dahil değildir, `liste[1:5]` indeks 1'den 4'e kadar alır.
`liste[1:]` indeks 1'den sona kadar, `liste[:5]` baştan indeks 4'e kadar, `liste[:]` tüm listeyi kopyalar.
`liste[::2]` 2'şer adımla, `liste[::-1]` ters sırada, `liste[-7:-2]` negatif indeks ile dilimleme yapar.
`in` ve `not in` operatörleri listede eleman aramak için kullanılır; `10 in liste` True/False döndürür, güvenli silme işlemlerinde kullanılır.
Tuple (demet) listeye benzer ancak değiştirilemez (immutable) veri tipidir; parantez `()` ile tanımlanır, `tuple[0] = 100` hataya neden olur.
`index(değer)` metodu listede bir elemanın indeksini bulur; `liste.index(33)` → `2` (33'ün indeksi).
`count(değer)` metodu listede bir elemanın kaç kez geçtiğini döndürür; `liste.count(22)` → `3` (22'nin sayısı).
`sort()` metodu listeyi artan sırada sıralar; `liste.sort(reverse=True)` azalan sırada sıralar, liste yerinde değiştirilir.
`reverse()` metodu listenin elemanlarını ters sıraya çevirir; `liste.reverse()` liste yerinde değiştirilir.
`list(range(1, 10))` range fonksiyonu ile liste oluşturur; `[1, 2, 3, 4, 5, 6, 7, 8, 9]` döndürür.
Dictionary (sözlük) anahtar-değer (key-value) çiftlerini saklamak için kullanılır; süslü parantez `{}` ile tanımlanır, `{'Name': 'David', 'Age': 26}` gibi.
Sözlük anahtarları benzersiz olmalıdır ve değiştirilemez (immutable) veri tiplerinden olmalıdır; string, sayı, tuple kullanılabilir.
Sözlük elemanlarına erişim `sözlük[anahtar]` ile yapılır; `person['Name']` → `'David'`.
Sözlüğe eleman ekleme/değiştirme `sözlük[anahtar] = değer` ile yapılır; anahtar varsa değer değişir, yoksa yeni eleman eklenir.
Sözlükten eleman silme `del sözlük[anahtar]` veya `sözlük.pop(anahtar)` ile yapılır; anahtar yoksa `KeyError` oluşur.
`len(sözlük)` fonksiyonu sözlükteki anahtar-değer çifti sayısını döndürür; `len({'a': 1, 'b': 2})` → `2`.
Sözlüklerde `in` operatörü sadece anahtarlarda arama yapar; `'Name' in person` True, `'David' in person` False (değerde olsa bile).
Sözlük karşılaştırması `==` ve `!=` ile yapılabilir; `>` `<` `>=` `<=` kullanılamaz, `TypeError` oluşur.
`keys()` metodu sözlüğün tüm anahtarlarını döndürür; `dict_keys(['apple', 'banana'])` formatında döner.
`values()` metodu sözlüğün tüm değerlerini döndürür; `dict_values([5000, 4000])` formatında döner.
`items()` metodu sözlüğün tüm anahtar-değer çiftlerini tuple olarak döndürür; `dict_items([('a', 1), ('b', 2)])` formatında döner.
Sözlük ve liste farkı: liste elemanları silindiğinde indeksler değişir, sözlükte anahtarlar sabit kalır; `dic.pop(0)` sonrası `dic[1]` değişmez.
`format()` metodu indeks kullanarak argümanları yeniden sıralayabilir; `'{1} and {0}'.format('Python', 'Java')` → `'Java and Python'`.
`format()` ile alan genişliği ve ondalık basamak: `{0:4d}` 4 karakter genişlik, `{0:.2f}` 2 ondalık, `{0:10.3f}` 10 karakter genişlik 3 ondalık.
`format()` ile aynı argümanı tekrar kullanma: `'{0}, {0}, {0}'.format('Python')` → `'Python, Python, Python'`.
`format()` ile binlik ayırıcı: `{:,}` sayıları binlik gruplara ayırır; `'{:,}'.format(3000)` → `'3,000'`.
`format()` ile string formatı: `{:16s}` 16 karakter genişlik, `{:16s}` string için hizalama.
JSON (JavaScript Object Notation) verilerini Python sözlüğüne dönüştürmek için `json.loads()` kullanılır; `import json` gerekir.
`json.dump(sözlük, dosya)` Python sözlüğünü JSON dosyasına kaydeder; `json.dumps(sözlük)` JSON string'e dönüştürür.
`KeyError` sözlükte olmayan bir anahtara erişmeye veya silmeye çalışıldığında oluşan hata türüdür.

## Unit 12 - Sequence Data Types
Sequence (sıralı) veri tipleri: liste, tuple, string, range gibi sıralı veri yapılarıdır.
`in` operatörü bir değerin sequence içinde olup olmadığını kontrol eder; `10 in [10, 20, 30]` → `True`.
`not in` operatörü bir değerin sequence içinde olmadığını kontrol eder; `10 not in [20, 30]` → `True`.
Sequence objeleri `+` operatörü ile birleştirilebilir; `[1, 2] + [3, 4]` → `[1, 2, 3, 4]`, `'hello' + 'world'` → `'helloworld'`.
Sequence objeleri `*` operatörü ile tekrarlanabilir; `[1, 2] * 3` → `[1, 2, 1, 2, 1, 2]`, `'hi' * 2` → `'hihi'`.
`range` objesi doğrudan `*` ile çarpılamaz, önce `list()` veya `tuple()` ile dönüştürülmelidir; `list(range(5)) * 2`.
Sequence objeleri birbiriyle çarpılamaz; `[1, 2] * [3, 4]` → `TypeError: can't multiply sequence by non-int of type 'list'`.
`count()` metodu sequence içinde belirli bir elemanın kaç kez geçtiğini sayar; `[1, 1, 2].count(1)` → `2`, `'hello'.count('l')` → `2`.
Sequence objelerinde pozitif indeksleme 0'dan başlar; `'hello'[0]` → `'h'`, `[1, 2, 3][2]` → `3`.
Sequence objelerinde negatif indeksleme sondan başa doğru sayar; `'hello'[-1]` → `'o'`, `[1, 2, 3][-2]` → `2`.
`len()` fonksiyonu sequence uzunluğunu döndürür; `len([1, 2, 3])` → `3`, `len('hello')` → `5`.
`len()` ile son elemana erişim: `nations[len(nations) - 1]` son elemanı döndürür; alternatif olarak `nations[-1]` daha kısa ve okunabilir.
Tuple (demet) değiştirilemez (immutable) bir sequence veri tipidir; parantez `()` ile tanımlanır.
Tuple'lar parantez olmadan da oluşturulabilir; `t1 = 'a', 'b', 'c'` ile `t2 = ('a', 'b', 'c')` aynıdır.
Tek elemanlı tuple için sonuna virgül konulmalıdır; `(100)` integer, `(100,)` tuple.
Tuple elemanları değiştirilemez; `t[0] = 100` → `TypeError: 'tuple' object does not support item assignment`.
Packing: Birden fazla değeri tek bir tuple'a paketleme; `a = (1, 2, 3)`.
Unpacking: Tuple'daki değerleri ayrı değişkenlere açma; `x, y, z = (1, 2, 3)`.
Swap (değiş tokuş) işlemi Python'da tuple unpacking ile kolayca yapılır; `a, b = b, a`.
Tuple sıralama için önce listeye dönüştürülür; `temp = list(tup); temp.sort()`.
Tuple karşılaştırması yapılabilir; string karşılaştırmasında ASCII kodları kullanılır; `('A', 'B') < ('A', 'C')` → `True`.
`ord()` fonksiyonu karakterin ASCII kodunu döndürür; `ord('A')` → `65`, `ord('a')` → `97`.

## Unit 13 - Two-Dimensional Lists
İki boyutlu liste (two-dimensional list) liste içinde liste yapısıdır; matris veya tablo gibi verileri saklamak için kullanılır; `[[1, 2, 3], [4, 5, 6], [7, 8, 9]]` gibi.
İki boyutlu listede elemanlara erişim `liste[satır][sütun]` formatında yapılır; `list_array[0][2]` ilk satırın üçüncü elemanını döndürür.
İki boyutlu listede satıra erişim `liste[satır]` ile yapılır; `list_array[0]` → `[1, 2, 3]`.
İki boyutlu listede eleman değiştirme `liste[satır][sütun] = değer` ile yapılır; `list_array[1][1] = 300`.
İki boyutlu listeyi yazdırma: `for item in list_array: print(item)` satırları yazdırır, `for i in list_array: for j in i: print(j)` tüm elemanları yazdırır.
Unpacking ile iki boyutlu listeyi yazdırma sadece satırlar eşit uzunluktaysa çalışır; `for i, j, k in list_array` satırlar 3 elemanlıysa çalışır.
Satırlar farklı uzunluktaysa unpacking kullanılamaz; `ValueError: not enough values to unpack` oluşur.
İç içe for döngüsü (double for loop) iki boyutlu listeleri yazdırmanın en güvenli yöntemidir; farklı uzunluktaki satırlarla da çalışır.
İndeks kullanarak iki boyutlu listeyi yazdırma: `for i in range(len(list_array)): for j in range(len(list_array[i])): print(list_array[i][j])`.
İki boyutlu liste ataması referans (reference/shallow copy) kullanır; `list1 = list_array` yapıldığında `list1` ve `list_array` aynı nesneye referans eder, birinde yapılan değişiklik diğerini etkiler.
Shallow copy (sığ kopya) sadece üst seviye referansı kopyalar; iç içe listeler hala aynı nesneye referans eder.
Gerçek kopya (deep copy) oluşturmak için `copy.deepcopy()` kullanılır; `import copy` gerekir; `list1 = copy.deepcopy(list_array)` ile bağımsız kopya oluşturulur.
`is` operatörü iki değişkenin aynı nesneye referans edip etmediğini kontrol eder; `list_array is list1` shallow copy'de `True`, deep copy'de `False` döner.
For döngüsü ile iki boyutlu liste oluşturma: iç döngüde satır oluşturulur, dış döngüde satırlar listeye eklenir; `line = []; line.append(0); list1.append(line)`.
İki boyutlu listede rastgele değer atama: `random.randrange(0, 2)` ile 0 veya 1 değerleri atanabilir; sinema koltuk rezervasyonu gibi uygulamalarda kullanılır.
Matris formülü: `(satır-1) * sütun_sayısı + sütun + 1` formülü ile iki boyutlu matris elemanlarının değerleri hesaplanabilir; `value = (i - 1) * 4 + j * 1 + 1`.
Jagged list (düzensiz liste) satırları farklı uzunlukta olan iki boyutlu listedir; `[[0], [0, 1], [0, 1, 2]]` gibi.
Jagged list oluşturma: her satırın uzunluğu farklı olabilir; `for i in list1: line = []; for j in range(i): line.append(j); list2.append(line)`.
Jagged list doğrudan da tanımlanabilir; `list2 = [[0], [0, 1], [0, 1, 2]]` gibi.
Jagged list yazdırma: iç içe for döngüsü kullanılır; `for i in list2: for j in i: print(j, end=' '); print()`.
Jagged list'te rastgele değer atama: `random.randint(1, 100)` ile her satır için farklı uzunlukta rastgele değerler atanabilir.
İki boyutlu listede `len(list_array)` satır sayısını, `len(list_array[i])` i. satırın eleman sayısını döndürür.
İki boyutlu listede boş koltuk sayısını hesaplama: `if seat[i][j] == 0: available += 1` ile boş elemanlar sayılabilir.
List comprehension ile iki boyutlu liste oluşturma: `[[0] * 2 for i in range(3)]` veya `[[0 for j in range(2)] for i in range(3)]` formatında kullanılır.
List comprehension ile jagged list oluşturma: `[[0] * i for i in [1, 2, 3, 4, 5]]` formatında kullanılır.
While döngüsü ile iki boyutlu listeyi yazdırma: `while i < len(list_array): a, b, c = list_array[i]; print(a, b, c); i += 1` unpacking ile yapılır.
Satranç tahtası deseni: `(i + j) % 2 == 0` koşulu ile 1 ve 0 değerleri atanarak oluşturulur.
Dosya işlemleri: `open()` fonksiyonu ile dosya açılır; `'w'` (yazma), `'r'` (okuma), `'a'` (ekleme), `'w+t'` (yazma/okuma), `'r+t'` (okuma/yazma), `'a+t'` (ekleme/okuma) modları kullanılır.
`write()` metodu dosyaya yazma yapar; yazılan karakter sayısını döndürür; `f.write('text')` formatında kullanılır.
`read()` metodu dosyadan okuma yapar; `f.read()` tüm dosyayı, `f.read(5)` ilk 5 karakteri okur.
`readline()` metodu dosyadan satır satır okuma yapar; `f.readline()` bir satır okur ve string döndürür.
`close()` metodu dosyayı kapatır; `f.close()` formatında kullanılır.
`with` statement dosyayı otomatik olarak açar ve kapatır; `with open('file.txt', 'w') as f: f.write('text')` formatında kullanılır; hata oluşsa bile dosya kapatılır.
`try-finally` bloğu `with` statement alternatifi olarak kullanılabilir; `try: f.write('text'); finally: f.close()` formatında kullanılır.

## Unit 14 - Dictionary Method-1
`setdefault()` metodu dictionary'de anahtar yoksa varsayılan değerle ekler, varsa değiştirmez; `dic.setdefault(key, default_value)` formatında kullanılır.
`setdefault()` ile eleman sayma: `dic.setdefault(ch, 0); dic[ch] += 1` formatında kullanılır; `if ch not in dic.keys(): dic[ch] = 0` yerine daha kısa kod yazılır.
`update()` metodu dictionary'deki mevcut anahtarın değerini değiştirir veya yeni anahtar ekler; `dic.update(a=50)` veya `dic.update({1: 'ONE'})` formatında kullanılır.
`update()` metodu birden fazla anahtarı tek seferde güncelleyebilir; `dic.update(a=900, f=60)` formatında kullanılır.
`update()` metodu string olmayan anahtarlar için dictionary kullanılmalıdır; `dic.update({1: 'ONE', 3: 'THREE'})` formatında kullanılır.
`update()` metodu liste veya tuple ile kullanılabilir; `dic.update([[2, 'TWO'], [4, 'FOUR']])` veya `dic.update(((2, 'TWO'), (4, 'FOUR')))` formatında kullanılır.
Dictionary'de anahtar-değer ekleme/düzenleme: `dic[key] = value` formatında kullanılır; anahtar yoksa ekler, varsa günceller.
Dictionary'de anahtar kontrolü: `if key in dic:` veya `if key in dic.keys():` formatında kullanılır.
Dictionary'de anahtar-değer çifti silme: `del dic[key]` veya `dic.pop(key)` formatında kullanılır.