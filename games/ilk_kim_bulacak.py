import random

hedef_sayi = 5
bilgisayar_deneme = 0
kullanici_deneme = 0

print(f"Oyun basladi! Hedef sayiyi ({hedef_sayi}) ilk bulan kazanir!")

while True:
    # Computer's guess
    bilgisayar = random.randint(1, 10)
    bilgisayar_deneme += 1
    print(f"\nBilgisayar {bilgisayar_deneme}. denemede {bilgisayar} sayisini secti.")
    
    if bilgisayar == hedef_sayi:
        print(f"Maalesef! Bilgisayar {hedef_sayi}'i {bilgisayar_deneme}. denemede buldu ve kazandi!")
        break
    
    # User's guess
    while True:
        try:
            kullanici = int(input(f"{kullanici_deneme + 1}. deneme - 1-10 arasi bir sayi secin: "))
            if 1 <= kullanici <= 10:
                break
            else:
                print("Lutfen 1 ile 10 arasinda bir sayi girin!")
        except:
            print("Gecersiz giris! Lutfen bir sayi girin.")
    
    kullanici_deneme += 1
    
    if kullanici == hedef_sayi:
        print(f"Tebrikler! {hedef_sayi}'i {kullanici_deneme}. denemede buldunuz ve kazandiniz!")
        break
    else:
        print(f"{kullanici} sayisi hedef degil. Devam ediyoruz...")