from collections import deque

# Kuyruk oluşturma
queue = deque()
queue.append(1)  # Kuyruğa eleman ekleme
queue.append(2)
queue.append(3)
print(queue.popleft())  # 1'i çıkarır (ilk eklenen ilk çıkar)
print(queue)
