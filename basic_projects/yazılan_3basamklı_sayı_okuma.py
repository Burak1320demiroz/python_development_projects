def sayi_okunusu(sayi):
    sayi_str = str(sayi)
    sayi_okunusu_dict = {
        '0': 'sifir', '1': 'bir', '2': 'iki', '3': 'üç', '4': 'dört', '5': 'beş',
        '6': 'alti', '7': 'yedi', '8': 'sekiz', '9': 'dokuz'
    }
    onlar_dict = {
        '1': 'on', '2': 'yirmi', '3': 'otuz', '4': 'kirk', '5': 'elli',
        '6': 'altmiş', '7': 'yetmiş', '8': 'seksen', '9': 'doksan'
    }
    basamak_sayisi = len(sayi_str)
    okunus = []
    if basamak_sayisi == 3:  # Yüzler basamağı varsa
        okunus.append(sayi_okunusu_dict[sayi_str[0]] + " yüz")
        if sayi_str[1] != '0':  # Onlar basamağı varsa
            okunus.append(onlar_dict[sayi_str[1]])
        if sayi_str[2] != '0':  # Birler basamağı varsa
            okunus.append(sayi_okunusu_dict[sayi_str[2]])
    print("Sayinin Okunuşu:", " ".join(okunus))

sayi = int(input("Bir sayi giriniz ((3 Basamakli Olsun) ==> "))
sayi_okunusu(sayi)