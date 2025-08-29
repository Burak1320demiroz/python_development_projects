#  Kivy

## 1. Genel Tanım

* Kivy, **açık kaynaklı** ve **platformlar arası (cross-platform)** bir GUI geliştirme kütüphanesidir.
* Python ile yazılmıştır ve Windows, Linux, macOS, Android, iOS üzerinde çalışır.
* Modern, dokunmatik uyumlu arayüzler geliştirmeye odaklanmıştır.

---

## 2. Temel Özellikler

* **Cross-platform desteği:** Aynı kod ile farklı işletim sistemlerinde çalışır.
* **GPU hızlandırma:** Grafiksel işlemler OpenGL ES 2.0 üzerinden yapılır.
* **Dokunmatik desteği:** Çoklu dokunma (multi-touch) olaylarını destekler.
* **Widget tabanlı yapı:** Button, Label, TextInput, Slider gibi hazır widget’lar içerir.
* **KV Dilini kullanabilme:** Arayüzü Python kodundan ayrı bir dosyada tanımlamaya olanak sağlar.

---

## 3. Kullanım Alanları

* **Mobil uygulama geliştirme (Android, iOS).**
* **Masaüstü uygulamaları (Linux, Windows, Mac).**
* **Dokunmatik ekran arayüzleri.**
* **Oyunlar ve etkileşimli uygulamalar.**
* **Veri görselleştirme panelleri.**

---

## 4. Kivy’nin Mimarisi

* **Core Provider:** Kivy’nin çekirdek servisleri (dosya, pencere, giriş-çıkış).
* **UIX (User Interface eXtension):** Kullanıcı arayüzü bileşenleri (Button, Label, Layout’lar).
* **Input Providers:** Fare, klavye, dokunmatik ekran, sensörlerden girişleri yönetir.
* **Graphics:** OpenGL üzerinden hızlandırılmış grafik motoru sağlar.

---

## 5. Önemli Widget’lar

* `Label` → Metin göstermek için.
* `Button` → Tıklanabilir düğme.
* `TextInput` → Kullanıcıdan yazı almak için.
* `Image` → Resim göstermek için.
* `Slider` → Kaydırma çubuğu.
* `BoxLayout`, `GridLayout`, `StackLayout` → Arayüz düzeni için.

---

## 6. Basit Örnek Kod

```python
from kivy.app import App
from kivy.uix.button import Button

class MyApp(App):
    def build(self):
        return Button(text="Merhaba Kivy", font_size=32)

if __name__ == "__main__":
    MyApp().run()
```

Bu kodda:

* `App` sınıfı temel uygulamayı temsil eder.
* `build()` metodu arayüzü oluşturur.
* `Button` widget’i ekrana bir buton yerleştirir.

---

## 7. KV Dili ile Örnek

`main.py`

```python
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout

class MyLayout(BoxLayout):
    pass

class MyApp(App):
    def build(self):
        return MyLayout()

if __name__ == "__main__":
    MyApp().run()
```

`my.kv`

```kv
<MyLayout>:
    orientation: "vertical"
    Button:
        text: "Buton 1"
    Button:
        text: "Buton 2"
```

Burada:

* `KV dili` ile arayüz tasarımı ayrı tutulur.
* Daha okunabilir ve düzenlenebilir bir yapı sağlar.

---
