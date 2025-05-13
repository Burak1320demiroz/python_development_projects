import random
import string
from itertools import product

# Şifre Kırma Uygulaması
# Bu uygulama, rastgele bir 5 karakterli şifre oluşturur ve ardından bu şifreyi brute-force yöntemiyle bulmaya çalışır.
# Kullanıcıdan şifreyi tahmin etmesini istemek yerine, uygulama kendi oluşturduğu şifreyi bulmaya çalışacak.
# Bu uygulama, şifre kırma işlemini gerçekleştirmek için brute-force yöntemini kullanır.
# Karakter kümesi: harfler

def rastgele_sifre_uret():
    return ''.join(random.choices(string.ascii_lowercase, k=5))

sifre = rastgele_sifre_uret()
print("Şifre (gizli):", sifre)

def sifre_bul(target):
    harfler = string.ascii_lowercase
    sayac = 0
    for kombinasyon in product(harfler, repeat=5):
        sayac += 1
        tahmin = ''.join(kombinasyon)
        if tahmin == target:
            print("Şifre bulundu:", tahmin)
            print("Toplam deneme sayısı:", sayac)
            break

sifre_bul(sifre)
