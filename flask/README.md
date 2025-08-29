# Flask 

## 1. Flask Nedir?

* Flask, Python ile yazılmış **mikro web framework**’üdür.
* Django gibi büyük framework’lere kıyasla daha hafif ve özelleştirilebilir yapıdadır.
* Temelinde **WSGI** standardına dayanır.

## 2. Flask’ın Avantajları

* Basit, öğrenmesi kolay.
* Esnek, modüler ve genişletilebilir.
* Hazır paketlerle (Flask-Login, Flask-SQLAlchemy vb.) kolayca büyütülebilir.
* Küçük projelerden büyük sistemlere kadar ölçeklenebilir.

## 3. Flask Projesi Nasıl Kurulur?

```bash
pip install flask
```

* Basit bir Flask uygulaması:

```python
from flask import Flask

app = Flask(__name__)

@app.route("/")
def hello():
    return "Merhaba Flask!"

if __name__ == "__main__":
    app.run(debug=True)
```

* `debug=True` geliştirme sırasında otomatik yeniden başlatma sağlar.

## 4. Flask Yapısı

Bir proje genelde şu yapıya sahiptir:

```
/project
    app.py
    /templates
        index.html
    /static
        style.css
```

* `templates` → HTML dosyaları
* `static` → CSS, JS, resimler gibi statik dosyalar

## 5. Routing (Yönlendirme)

* Kullanıcıların girdiği URL’lere karşılık gelen fonksiyonları belirler.

```python
@app.route("/about")
def about():
    return "Hakkında sayfası"
```

* Parametreli route:

```python
@app.route("/user/<string:name>")
def user(name):
    return f"Kullanıcı: {name}"
```

## 6. HTTP Metotları

* GET (varsayılan), POST, PUT, DELETE gibi metotlar desteklenir.

```python
@app.route("/login", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        return "Giriş yapılıyor..."
    return "Login sayfası"
```

## 7. HTML Şablonları (Jinja2)

* Flask, **Jinja2** şablon motorunu kullanır.
* Örnek:

```html
<!DOCTYPE html>
<html>
<head>
    <title>{{ title }}</title>
</head>
<body>
    <h1>Merhaba {{ user }}</h1>
</body>
</html>
```

* Python’dan gönderim:

```python
from flask import render_template

@app.route("/profile/<name>")
def profile(name):
    return render_template("profile.html", user=name, title="Profil")
```

## 8. Form İşlemleri

* `request.form` ile kullanıcıdan alınan form verisi işlenir.

```python
from flask import request

@app.route("/form", methods=["POST"])
def form():
    data = request.form["username"]
    return f"Girilen kullanıcı adı: {data}"
```

## 9. Redirect ve URL Yönetimi

* Başka sayfaya yönlendirme:

```python
from flask import redirect, url_for

@app.route("/home")
def home():
    return "Ana Sayfa"

@app.route("/go-home")
def go_home():
    return redirect(url_for("home"))
```

## 10. Flask ile Veritabanı

* ORM için genellikle **Flask-SQLAlchemy** kullanılır.

```bash
pip install flask-sqlalchemy
```

Örnek:

```python
from flask_sqlalchemy import SQLAlchemy

app.config["SQLALCHEMY_DATABASE_URI"] = "sqlite:///users.db"
db = SQLAlchemy(app)

class User(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(100))
```

## 11. Flask Blueprint

* Büyük projelerde modüler yapı sağlar.

```python
from flask import Blueprint

admin_bp = Blueprint("admin", __name__, url_prefix="/admin")

@admin_bp.route("/dashboard")
def dashboard():
    return "Admin Paneli"
```

## 12. Flask ile API Geliştirme

* JSON response döndürmek için:

```python
from flask import jsonify

@app.route("/api/data")
def api_data():
    return jsonify({"mesaj": "API çalışıyor!"})
```
