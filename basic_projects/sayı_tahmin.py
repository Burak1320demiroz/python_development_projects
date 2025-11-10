import random

print(10*".", "SSS", 10*".")
print("Sayi")
print("Sayi")
print("Sayi")

alt_sinir = int(input("Bilgisayarin Sececeği Sayi Araligin Alt Siniri Ne Olsun ==> "))
üst_sinir = int(input("Bilgisayarin Sececeği Sayi Araligin Üst Siniri Ne Olsun ==> "))
bilgisaray = random.randint(alt_sinir, üst_sinir) 
tahmin_sayisi = 0
deneme_hakki = int(input("Kaç deneme hakkin olsun ==> "))

while tahmin_sayisi < deneme_hakki:
    tahmin = int(input(f"Bilgisaray aklindan {alt_sinir} ile {üst_sinir} arasi sayi tuttu sence kac ==> "))
    tahmin_sayisi += 1
    if bilgisaray == tahmin:
        print("Doğru Tahmin Ettin")
        print("KRAL")
    elif bilgisaray < tahmin:
        print("Cok büyük bir sayi secitn daha kücük sec")
    elif bilgisaray > tahmin:
        print("Cok kücük bir sayi sectin daha büyük sec")
    else:
        print("gecersiz sayi")

print(50*"-")

print("Bilgisayarin Sayisi ==> ", bilgisaray)
print("Senin Sayin ==> ", tahmin)
print(f"Tebrikler {tahmin_sayisi} denemede doğru cevabi buldun.")

print(50*"-")