def hesap_makinesi():
    print("Basit Hesap Makinesi")
    print("Toplama: +")
    print("Çikarma: -")
    print("Çarpma: *")
    print("Bölme: /")

    sayi1 = float(input("1. sayiyi girin: "))
    sayi2 = float(input("2. sayiyi girin: "))
    islem = input("İşlem seçin (+, -, *, /): ")

    if islem == "+":
        print("Sonuç:", sayi1 + sayi2)
    elif islem == "-":
        print("Sonuç:", sayi1 - sayi2)
    elif islem == "*":
        print("Sonuç:", sayi1 * sayi2)
    elif islem == "/":
        if sayi2 != 0:
            print("Sonuç:", sayi1 / sayi2)
        else:
            print("Hata: Sifira bölme yapilamaz.")
    else:
        print("Geçersiz işlem seçtiniz.")

hesap_makinesi()
