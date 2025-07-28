# Adjacency List kullanarak grafik oluşturma
graph = {
    "A": ["B", "C"],
    "B": ["A", "D", "E"],
    "C": ["A", "F"],
    "D": ["B"],
    "E": ["B", "F"],
    "F": ["C", "E"]
}

print(graph)

# Grafikteki komşuları listeleme
for node in graph:
    print(f"{node}: {', '.join(graph[node])}")  