def asal_sayi_mi(sayi):
    if sayi <= 1:
        return False  # 1 ve daha küçük sayılar asal değildir.
    for i in range(2, int(sayi ** 0.5) + 1):  # Sayının kareköküne kadar kontrol et
        if sayi % i == 0:
            return False  # Bölünebilen bir sayı varsa asal değildir.
    return True

sayi = int(input("Bir sayi girin: "))

if asal_sayi_mi(sayi):
    print(f"{sayi} bir asal sayidir.")
else:
    print(f"{sayi} bir asal sayi degildir.")
