import math

print("Daire Alan ve Çevre Hesaplama Programi")
print("--------------------------------------")

while True:
    try:
        yaricap = float(input("Dairenin yariçapini girin (cm cinsinden): "))
        
        if yaricap <= 0:
            print("Hata: Yariçap pozitif bir değer olmalidir!")
            continue
        
        alan = math.pi * (yaricap ** 2)
        cevre = 2 * math.pi * yaricap
        
        print(f"\nDaire Bilgileri (Yariçap: {yaricap} cm)")
        print(f"Alan: {alan:.2f} cm²")
        print(f"Çevre: {cevre:.2f} cm")
        
        devam = input("\nBaşka bir hesaplama yapmak ister misiniz? (E/H): ").upper()
        if devam != 'E':
            print("Program sonlandirildi.")
            break
            
    except ValueError:
        print("Hata: Lütfen geçerli bir sayi girin!")