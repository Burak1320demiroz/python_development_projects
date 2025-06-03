import math

def hesap_makinesi():
    print("=== Sekiz İşlem Hesap Makinesi===")
    print("İşlemler:")
    print("Toplama       : +")
    print("Çikarma       : -")
    print("Çarpma        : *")
    print("Bölme         : /")
    print("Üs alma       : ^")
    print("Mod alma      : %")
    print("Karekök       : sqrt")
    print("Faktoriyel    : !")
    
    islem = input("İşlem girin: ").strip()
    
    if islem in ["+", "-", "*", "/", "^", "%"]:
        a = float(input("1. sayi: "))
        b = float(input("2. saii: "))

        if islem == "+":
            print("Sonuç:", a + b)
        elif islem == "-":
            print("Sonuç:", a - b)
        elif islem == "*":
            print("Sonuç:", a * b)
        elif islem == "/":
            if b != 0:
                print("Sonuç:", a / b)
            else:
                print("Hata: Sifira bölünemez.")
        elif islem == "^":
            print("Sonuç:", a ** b)
        elif islem == "%":
            print("Sonuç:", a % b)
    
    elif islem == "sqrt":
        a = float(input("Sayi: "))
        if a >= 0:
            print("Sonuç:", math.sqrt(a))
        else:
            print("Hata: Negatif sayinin karekökü alinamaz.")

    elif islem == "!":
        a = int(input("Sayi (pozitif tam sayi): "))
        if a >= 0:
            print("Sonuç:", math.factorial(a))
        else:
            print("Hata: Negatif sayinin faktoriyeli alinamaz.")
    
    else:
        print("Geçersiz işlem seçtiniz.")

hesap_makinesi()
