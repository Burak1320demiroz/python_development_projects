import os
import json
from datetime import datetime


PRIORITIES = ["acil", "yapilmali", "bos vaktinde bak"]

def generate_filename():
    """Tarih ve saat bilgisi iceren dosya adi olusturur"""
    now = datetime.now()
    return f"tasks_{now.strftime('%Y%m%d_%H%M')}.json"

def load_tasks():
    """Gorevleri dosyadan yukler"""
    tasks = []
    try:
        task_files = [f for f in os.listdir() if f.startswith('tasks_') and f.endswith('.json')]
        if task_files:
            task_files.sort(reverse=True)
            filename = task_files[0]
            with open(filename, 'r', encoding='utf-8') as f:
                tasks = json.load(f)
        return tasks
    except Exception as e:
        print(f"Dosya okunurken hata olustu: {e}")
        return []

def save_tasks(tasks):
    """Gorevleri dosyaya kaydeder"""
    try:
        filename = generate_filename()
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(tasks, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"Dosya yazilirken hata olustu: {e}")


def print_tasks(tasks):
    """Gorev listesini formatli sekilde yazdirir"""
    if not tasks:
        print("\nHenuz gorev bulunmamaktadir.")
        return
    
    print("\n--- GOREV LISTESI ---")
    for index, task in enumerate(tasks, start=1):
        status = "[✓]" if task['tamamlandi'] else "[ ]"
        print(f"{index}. {status} [{task['oncelik'].upper()}] {task['gorev']}")
    print("----------------------")

def get_priority():
    """Gecerli bir oncelik degeri alir"""
    while True:
        print("Oncelik seciniz (acil/yapilmali/bos vaktinde bak):")
        priority = input().strip().lower()
        if priority in PRIORITIES:
            return priority
        print("Hata: Gecersiz oncelik degeri!")

def add_task(tasks):
    """Yeni gorev ekler"""
    while True:
        new_task = input("\nYeni gorev: ").strip()
        if new_task:
            priority = get_priority()
            tasks.append({
                'gorev': new_task,
                'tamamlandi': False,
                'oncelik': priority,
                'olusturulma': datetime.now().isoformat()
            })
            save_tasks(tasks)
            print(f"'{new_task}' gorevi eklendi.")
            break
        else:
            print("Hata: Bos gorev eklenemez!")

def toggle_task_status(tasks):
    """Gorev tamamlama durumunu degistirir"""
    print_tasks(tasks)
    if not tasks:
        return
    
    while True:
        try:
            task_num = int(input("\nDurumunu degistirmek istediginiz gorev numarasi: "))
            if 1 <= task_num <= len(tasks):
                tasks[task_num-1]['tamamlandi'] = not tasks[task_num-1]['tamamlandi']
                save_tasks(tasks)
                print("Gorev durumu guncellendi!")
                break
            else:
                print(f"Hata: 1-{len(tasks)} arasinda bir numara girin!")
        except ValueError:
            print("Hata: Lutfen gecerli bir sayi girin!")

def edit_task(tasks):
    """Gorev duzenler"""
    print_tasks(tasks)
    if not tasks:
        return
    
    while True:
        try:
            task_num = int(input("\nDuzenlemek istediginiz gorev numarasi: "))
            if 1 <= task_num <= len(tasks):
                new_text = input(f"Yeni gorev metni ({tasks[task_num-1]['gorev']}): ").strip()
                if new_text:
                    tasks[task_num-1]['gorev'] = new_text
                    tasks[task_num-1]['oncelik'] = get_priority()
                    save_tasks(tasks)
                    print("Gorev basariyla guncellendi!")
                    break
                else:
                    print("Hata: Gorev metni bos olamaz!")
            else:
                print(f"Hata: 1-{len(tasks)} arasinda bir numara girin!")
        except ValueError:
            print("Hata: Lutfen gecerli bir sayi girin!")

def delete_task(tasks):
    """Gorev siler"""
    print_tasks(tasks)
    if not tasks:
        return
    
    while True:
        try:
            task_num = int(input("\nSilmek istediginiz gorev numarasi: "))
            if 1 <= task_num <= len(tasks):
                removed_task = tasks.pop(task_num-1)
                save_tasks(tasks)
                print(f"'{removed_task['gorev']}' gorevi silindi!")
                break
            else:
                print(f"Hata: 1-{len(tasks)} arasinda bir numara girin!")
        except ValueError:
            print("Hata: Lutfen gecerli bir sayi girin!")

def main_menu():
    """Ana menuyu gosterir"""
    print("\n--- TO-DO LIST UYGULAMASI ---")
    print("1. Gorevleri Listele")
    print("2. Yeni Gorev Ekle")
    print("3. Gorev Duzenle")
    print("4. Gorev Sil")
    print("5. Durum Degistir")
    print("6. Cikis")

def main():
    tasks = load_tasks()
    
    print("\n--- TO-DO LIST UYGULAMASI ---")
    if tasks:
        print(f"{len(tasks)} gorev basariyla yuklendi!")
    else:
        print("Yeni liste olusturuldu.")
    
    while True:
        main_menu()
        choice = input("\nSeciminiz (1-6): ")
        
        if choice == '1':
            print_tasks(tasks)
        elif choice == '2':
            add_task(tasks)
        elif choice == '3':
            edit_task(tasks)
        elif choice == '4':
            delete_task(tasks)
        elif choice == '5':
            toggle_task_status(tasks)
        elif choice == '6':
            print("\nProgramdan cikiliyor...")
            break
        else:
            print("Hata: Gecersiz secim! Lutfen 1-6 arasi bir sayi girin.")

if __name__ == "__main__":
    main()