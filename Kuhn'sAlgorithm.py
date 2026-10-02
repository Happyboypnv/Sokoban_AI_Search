def readInput():
    n, m, e = map(int, input().split())
    adj = [[] for _ in range(n + 1)]
    for _ in range(e):
        u, v = map(int, input().split())
        adj[u].append(v)
    return n, m, e, adj
def solve(n, m, e, adj):
    visited = [False] * (n + 1)
    matching = [-1] * (m + 1)
    ans = 0
    def tryToMatch(u):
        if visited[u]: return False
        visited[u] = True
        for v in adj[u]:
            if matching[v] == -1 or tryToMatch(matching[v]):
                matching[v] = u
                return True
        return False
    for i in range(1, n + 1):
        visited = [False] * (n + 1)
        if tryToMatch(i): ans += 1
    print(ans)
    for i in range(1, m + 1):
        print(i, matching[i])

if __name__ == "__main__":
    n, m, e, adj = readInput()
    solve(n, m, e, adj)
# O(n*m)
# O(m + n)