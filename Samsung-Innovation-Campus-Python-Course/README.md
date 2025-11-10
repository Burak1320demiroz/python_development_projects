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
Unit 2 akış şeması ve Python referans notları eklenecek; materyal hazır olduğunda güncellenecektir.
