import random
import string
from itertools import product
import time

# Şifre Kırma Uygulaması
# Bu uygulama, rastgele bir 5 karakterli şifre oluşturur ve ardından bu şifreyi brute-force yöntemiyle bulmaya çalışır.
# Kullanıcıdan şifreyi tahmin etmesini istemek yerine, uygulama kendi oluşturduğu şifreyi bulmaya çalışacak.
# Bu uygulama, şifre kırma işlemini gerçekleştirmek için brute-force yöntemini kullanır.
# Karakter kümesi: harfler + rakamlar + özel karakterler

ozel_karakterler = "!@#$%^&*()-_+="
karakterler = string.ascii_lowercase + string.digits + ozel_karakterler

def rastgele_sifre_uret():
    return ''.join(random.choices(karakterler, k=5))

sifre = rastgele_sifre_uret()
print("Şifre (gizli):", sifre)

def sifre_bul(target):
    sayac = 0
    for kombinasyon in product(karakterler, repeat=5):
        sayac += 1
        tahmin = ''.join(kombinasyon)
        if tahmin == target:
            print("Şifre bulundu:", tahmin)
            print("Toplam deneme sayısı:", sayac)
            break

sifre_bul(sifre)

start_time = time.time()
end_time = time.time()
print("Şifre bulma süresi:", end_time - start_time, "saniye")

