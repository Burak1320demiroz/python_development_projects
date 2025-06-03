print("En Buyuk Sayiyi Bulma Programi")
print("-----------------------------")

while True:
    try:
        sayi1 = float(input("Birinci sayiyi girin: "))
        sayi2 = float(input("Ikinci sayiyi girin: "))
        sayi3 = float(input("Ucuncu sayiyi girin: "))
        
        # En buyuk sayiyi bulma
        if sayi1 >= sayi2 and sayi1 >= sayi3:
            en_buyuk = sayi1
        elif sayi2 >= sayi1 and sayi2 >= sayi3:
            en_buyuk = sayi2
        else:
            en_buyuk = sayi3
        
        print(f"\nGirilen sayilar: {sayi1}, {sayi2}, {sayi3}")
        print(f"En buyuk sayi: {en_buyuk}")
        
        devam = input("\nBaska bir hesaplama yapmak ister misiniz? (E/H): ").upper()
        if devam != 'E':
            print("Program sonlandirildi.")
            break
            
    except ValueError:
        print("Hata: Lutfen gecerli bir sayi girin!")