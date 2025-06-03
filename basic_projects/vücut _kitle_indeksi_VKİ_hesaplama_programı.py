print("Vücut Kitle İndeksi (VKİ) Hesaplama Programi")
print("--------------------------------------------")

while True:
    try:
        boy = float(input("Boyunuzu metre cinsinden girin (Örnek: 1.75): "))
        kilo = float(input("Kilonuzu kilogram cinsinden girin (Örnek: 68): "))
        
        if boy <= 0 or kilo <= 0:
            print("Hata: Boy ve kilo değerleri pozitif olmalidir!")
            continue
        
        # VKİ hesaplama
        vki = kilo / (boy ** 2)
        
        print(f"\nVücut Kitle İndeksiniz: {vki:.2f}")
        
        # Değerlendirme
        if vki < 18.5:
            print("Durum: Zayif")
        elif 18.5 <= vki < 25:
            print("Durum: Normal")
        elif 25 <= vki < 30:
            print("Durum: Fazla Kilolu")
        else:
            print("Durum: Obez")
        
        devam = input("\nBaşka bir hesaplama yapmak ister misiniz? (E/H): ").upper()
        if devam != 'E':
            print("Program sonlandirildi.")
            break
            
    except ValueError:
        print("Hata: Lütfen geçerli bir sayi girin!")
    except ZeroDivisionError:
        print("Hata: Boy değeri sifir olamaz!")