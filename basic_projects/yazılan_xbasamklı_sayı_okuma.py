def sayi_okunusu(sayi):
    sayi_str = str(sayi)
    basamak_sayisi = len(sayi_str)
    
    birler = {
        '0': 'sifir', '1': 'bir', '2': 'iki', '3': 'üç', '4': 'dört', '5': 'beş',
        '6': 'alti', '7': 'yedi', '8': 'sekiz', '9': 'dokuz'
    }
    
    onlar = {
        '1': 'on', '2': 'yirmi', '3': 'otuz', '4': 'kirk', '5': 'elli',
        '6': 'altmiş', '7': 'yetmiş', '8': 'seksen', '9': 'doksan'
    }
    
    binler = {
        1: 'bin', 2: 'milyon', 3: 'milyar', 4: 'trilyon', 5: 'katrilyon'
    }

    def uc_basamak_okunus(uc_basamak):
        okunus = []
        if len(uc_basamak) == 3:
            if uc_basamak[0] != '0':
                if uc_basamak[0] == '1':
                    okunus.append('yüz')
                else:
                    okunus.append(birler[uc_basamak[0]] + ' yüz')
            if uc_basamak[1] != '0':
                okunus.append(onlar[uc_basamak[1]])
            if uc_basamak[2] != '0':
                okunus.append(birler[uc_basamak[2]])
        elif len(uc_basamak) == 2:
            if uc_basamak[0] != '0':
                okunus.append(onlar[uc_basamak[0]])
            if uc_basamak[1] != '0':
                okunus.append(birler[uc_basamak[1]])
        elif len(uc_basamak) == 1:
            okunus.append(birler[uc_basamak[0]])
        return ' '.join(okunus)
    
    if basamak_sayisi == 1:
        print(f"Sayinin Okunuşu: {birler[sayi_str]}")
        print(f"Basamak Sayisi: 1 (Birler)")
        return
    
    # Sayıyı üçerli gruplara ayırma
    gruplar = []
    while sayi_str:
        gruplar.append(sayi_str[-3:])
        sayi_str = sayi_str[:-3]
    gruplar.reverse()
    
    okunuslar = []
    for i in range(len(gruplar)):
        grup = gruplar[i]
        if grup == '000':
            continue
        grup_okunus = uc_basamak_okunus(grup)
        kalan_grup_sayisi = len(gruplar) - i - 1
        if kalan_grup_sayisi > 0 and grup != '000':
            if grup == '1' and kalan_grup_sayisi == 1:
                grup_okunus = ''
            grup_okunus += ' ' + binler[kalan_grup_sayisi]
        okunuslar.append(grup_okunus.strip())
    
    sonuc = ' '.join(okunuslar).strip()
    if not sonuc:
        sonuc = 'sifir'
    
    print(f"Sayinin Okunuşu: {sonuc}")
    print(f"Basamak Sayisi: {basamak_sayisi}")

print("Sayi Okuma Programi (Çikmak için 'çik' yazin)")
while True:
    girdi = input("\nBir sayi giriniz: ").strip().lower()
    
    if girdi == 'çik':
        print("Program sonlandirildi.")
        break
    
    elif girdi.isdigit() or (girdi[0] == '-' and girdi[1:].isdigit()):  # Negatif sayıları da kontrol ediyoruz
        sayi = int(girdi)
        sayi_okunusu(sayi)
    
    # isdigit() metodu pozitif tam sayıları kontrol ediyor

    else:
        print("Geçersiz giriş! Lütfen bir sayi girin veya çikmak için 'çik' yazin.")
