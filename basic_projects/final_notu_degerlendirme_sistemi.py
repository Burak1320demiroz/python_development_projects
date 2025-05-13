print("="*40)
print("Final Notu Degerlendirme Sistemi")
print("="*40)

final = int(input("Final notunu gir (0-100): "))

if final < 0 or final > 100:
    print("Hata: Not 0 ile 100 arasinda olmali!")
elif final >= 90:
    print("Harf Notun: A")
elif final >= 80:
    print("Harf Notun: B")
elif final >= 70:
    print("Harf Notun: C")
elif final >= 60:
    print("Harf Notun: D")
elif final >= 50:
    print("Harf Notun: E")
else:
    print("Harf Notun: F - Kaldin")

print("="*40)
print("Program sonlandi.")
