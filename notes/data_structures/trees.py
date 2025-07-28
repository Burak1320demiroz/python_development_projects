class Node:
    def __init__(self, value):
        self.value = value
        self.left = None
        self.right = None

# Ağacın kökünü oluşturma
root = Node(10)
root.left = Node(5)
root.right = Node(15)

print(root.value)  # 10
print(root.left.value)  # 5
print(root.right.value)  # 15
