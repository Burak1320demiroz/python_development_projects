import time
import random
import sys

# Oyun Durumları
su = 75
patates = 30
gida = 15
enerji = 65
anten_parcasi = False
iletisim_kuruldu = False
current_room = "Kaza Alanı"
inventory = []

def clear_screen():
    print("\033[H\033[J")  # ANSI escape code ile ekran temizleme

def print_header(title):
    print("\033[95m" + "═" * 60)
    print(f"■ {title.upper()} ■".center(60))
    print("═" * 60 + "\033[0m\n")

def progress_bar(value, max_value, color, label):
    bar_length = 30
    filled = int(round(bar_length * value / max_value))
    bar = f"{color}▓" * filled + "░" * (bar_length - filled)
    return f"\033[97m{label}: \033[0m{bar} {value}/{max_value}"

def show_status():
    clear_screen()
    print_header("astronot durum paneli")
    
    print(progress_bar(su, 100, "\033[94m", "Su"))
    print(progress_bar(enerji, 100, "\033[93m", "Enerji"))
    print(progress_bar(gida, 30, "\033[92m", "Gıda"))
    print(progress_bar(patates, 50, "\033[33m", "Patates"))
    
    print("\n\033[97mENVANTER:\033[0m")
    print(" ".join([f"[\033[96m{item}\033[0m]" for item in inventory]) if inventory else "Boş")
    input("\nDevam etmek için Enter'a basın...")

def location_art(location):
    arts = {
        "Kaza Alanı": [
            r"   ___   ",
            r"  /   \  ",
            r" |  X  | ",
            r"  \___/  ",
            r"  /| |\  ",
            r" /_| |_\ "
        ],
        "İletişim Merkezi": [
            r"  ╔════╗ ",
            r"  ║ δ= ║ ",
            r"  ╚════╝ ",
            r"   /▲\   ",
            r"  /_■_\  "
        ],
        "Ana Yaşam Modülü": [
            r"  ░▒▓███ ",
            r" █▓▒░░▒▓█",
            r" █░░◯░░█ ",
            r" █▓▒░▒▓█ ",
            r"  ▀▀▀▀▀  "
        ]
    }
    print("\033[90m" + "\n".join(arts.get(location, [])) + "\033[0m")

def hikaye_giris():
    clear_screen()
    print("\033[94m" + "="*60)
    print("■ MARS GÖREVİ: DÖNÜŞÜ OLMAYAN YOLCULUK ■".center(60))
    print("="*60 + "\033[0m")
    time.sleep(1)
    
    print("\n\033[93mYıl 2045. Mars'a yapılan ilk insanlı görevdeki dönüş aracı\n"
          "kalkış sırasında hasar gördü. Siz -Astronot Alex- Mars yüzeyinde\n"
          "mahsur kaldınız. Son raporlar:\033[0m")
    time.sleep(2)
    show_status()
    
    print("\n\033[91mKRİTİK UYARI: İletişim sistemi çöktü! Dünya ile bağlantı kurmalısınız!\033[0m")
    time.sleep(3)

def harita_goster():
    clear_screen()
    print_header("mars üssü haritası")
    location_art(current_room)
    print("\n\033[97m[1] Kaza Alanı\033[0m" + (" ← Şu anki konum" if current_room == "Kaza Alanı" else ""))
    print("\033[97m[2] Ana Yaşam Modülü\033[0m" + (" ← Şu anki konum" if current_room == "Ana Yaşam Modülü" else ""))
    print("\033[97m[3] İletişim Merkezi\033[0m" + (" ← Şu anki konum" if current_room == "İletişim Merkezi" else ""))
    print("\033[97m[4] Sera Alanı\033[0m" + (" ← Şu anki konum" if current_room == "Sera Alanı" else ""))
    print("\033[97m[5] Malzeme Deposu\033[0m" + (" ← Şu anki konum" if current_room == "Malzeme Deposu" else ""))
    print("\n\033[97m[0] Geri Dön\033[0m")

def astronot():
    global su, patates, gida, enerji, current_room, anten_parcasi, iletisim_kuruldu
    
    hikaye_giris()
    
    while True:
        clear_screen()
        print_header(f"{current_room} - ana menü")
        location_art(current_room)
        
        print("\033[97m[1] Durum Raporu")
        print("[2] Haritayı Aç")
        print("[3] Kaynak Kullan")
        print("[4] Bölgeyi İncele")
        print("[5] İletişim Sistemini Dene")
        print("[0] Çıkış\033[0m")
        
        secim = input("\n\033[97m» Seçiminiz: \033[0m")
        
        if secim == "1":
            show_status()
        
        elif secim == "2":
            while True:
                harita_goster()
                try:
                    hedef = int(input("\n\033[97m» Gitmek istediğiniz bölge: \033[0m"))
                    odalar = ["", "Kaza Alanı", "Ana Yaşam Modülü", "İletişim Merkezi", "Sera Alanı", "Malzeme Deposu"]
                    
                    if hedef == 0:
                        break
                    elif 1 <= hedef <=5:
                        current_room = odalar[hedef]
                        enerji -= 15
                        print(f"\n\033[97m{odalar[hedef]}'e gidiliyor...\033[0m")
                        time.sleep(1)
                        if enerji <=0:
                            print("\033[91mKRİTİK: Enerjiniz bitti! Hayatta kalamadınız!\033[0m")
                            return
                        break
                    else:
                        print("\033[91mGeçersiz bölge!\033[0m")
                        time.sleep(1)
                except:
                    print("\033[91mHatalı giriş!\033[0m")
                    time.sleep(1)
        
        elif secim == "3":
            print("\n\033[92m--- KAYNAKLAR ---")
            print("1. Su İç (-10L, +5% Enerji)")
            print("2. Patates Ye (-5kg, +10% Enerji)")
            print("3. Gıda Paketi Tüket (-2 paket, +15% Enerji)")
            print("0. Geri Dön\033[0m")
            
            kaynak = input("Seçiminiz: ")
            if kaynak == "1":
                if su >=10:
                    su -=10
                    enerji = min(100, enerji+5)
                    print("\nSoğuk suyu yudumluyorsun...")
                else:
                    print("\033[91mYeterli su yok!\033[0m")
            elif kaynak == "2":
                if patates >=5:
                    patates -=5
                    enerji = min(100, enerji+10)
                    print("\nÇiğ patates çıtır çıtır...")
                else:
                    print("\033[91mYeterli patates yok!\033[0m")
            elif kaynak == "3":
                if gida >=2:
                    gida -=2
                    enerji = min(100, enerji+15)
                    print("\nKonserve gıda mideni dolduruyor...")
                else:
                    print("\033[91mYeterli gıda paketi yok!\033[0m")
        
        elif secim == "4":
            if current_room == "Kaza Alanı" and not anten_parcasi:
                print("\nEnkazın içinde parlayan bir metal görüyorsun...")
                time.sleep(1)
                print("1. Yakından incele")
                print("2. Göz ardı et")
                if input("Seçim: ") == "1":
                    print("\nKırık iletişim anteni buldun! Envantere eklendi.")
                    inventory.append("Anten Parçası")
                    anten_parcasi = True
            
            elif current_room == "İletişim Merkezi":
                print("\n\033[96mBOZUK İLETİŞİM KONSOLU: 'Anten bağlantısı yok' hatası veriyor.\033[0m")
                if "Anten Parçası" in inventory:
                    print("1. Anteni takmayı dene")
                    if input("Seçim: ") == "1":
                        print("\nAnteni yerleştiriyorsun...")
                        time.sleep(2)
                        print("Kıvılcımlar çakmaya başlıyor!")
                        iletisim_kuruldu = True
        
        elif secim == "5":
            if current_room == "İletişim Merkezi" and iletisim_kuruldu:
                print("\n\033[92mSinyal gücü: %65... Frekans ayarlanıyor...\033[0m")
                time.sleep(1)
                print("1. SOS sinyali gönder")
                print("2. Manuel frekans dene")
                
                if input("Seçim: ") == "2":
                    print("\nFrekans taraması başlıyor...")
                    for i in range(3):
                        print(f"{i+1}. Deneme...")
                        time.sleep(1)
                    if random.randint(1,10) >3:
                        print("\033[92mDünya ile bağlantı kuruldu! Kurtarma ekibi yolda!\033[0m")
                        print("=== OYUNU KAZANDINIZ ===")
                        break
                    else:
                        print("\033[91mBağlantı başarısız! Tekrar dene!\033[0m")
            else:
                print("\033[91mİletişim sistemi çalışmıyor!\033[0m")
        
        elif secim == "0":
            print("\n\033[97mGörev sonlandırılıyor...\033[0m")
            sys.exit()
        
        else:
            print("\033[91mGeçersiz seçim!\033[0m")
            time.sleep(1)
        
        # Kaynak azalması ve kontrol
        su = max(0, su-2)
        enerji = max(0, enerji-5)
        if su <=0 or enerji <=0:
            print("\033[91mHAYATİ KAYIP: Yaşam destek sistemleri durdu!\033[0m")
            break

astronot()