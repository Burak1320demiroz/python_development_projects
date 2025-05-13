import random

print(50*"-")
print(10*".", "TKM", 10*".")
print("Tas")
print("Kağit")
print("Makas")
print(50*"-")

secenekler = {1: "Tas", 2: "Makas", 3: "Kagit"}
kullanici = int(input("Oyun Başladi! Taş = 1, Makas = 2, Kağit = 3 ==> "))
bilgisayar = random.choice(["Tas", "Makas", "Kagit"])
kullanici_secimi = secenekler.get(kullanici)

print(50*"-")
print(f"Bilgisayar: {bilgisayar}")
print(f"Kullanici: {kullanici_secimi}")
print(50*"-")

if bilgisayar == kullanici_secimi:
    print("Berabere")
elif (bilgisayar == "Tas" and kullanici_secimi == "Makas") or \
     (bilgisayar == "Makas" and kullanici_secimi == "Kagit") or \
     (bilgisayar == "Kagit" and kullanici_secimi == "Tas") or \
     (bilgisayar == "Tas" and kullanici_secimi == "Kağit") or \
     (bilgisayar == "Makas" and kullanici_secimi == "Tas") or \
     (bilgisayar == "Kagit" and kullanici_secimi == "Makas"):
    print("Kaybettin")
else:
    print("Kazandin")

print(50*"-")