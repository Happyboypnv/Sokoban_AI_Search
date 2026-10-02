def readInput():
    n, m = map(int, input().split())
    a = [[0]*(m + 1) for _ in range(n + 1)]
    for i in range(1, n + 1):
        row = list(map(int, input().split()))
        for j in range(1, m + 1):
            a[i][j] = row[j - 1]
    return n, m, a
def solveSub1(n, m, a):
    visitedX = [False] * (n + 1)
    visitedY = [False] * (m + 1)
    matching = [-1] * (m + 1)
    lx = [0] * (n + 1)
    ly = [0] * (m + 1)
    for i in range(1, n + 1):
        lx[i] = min(a[i][j] for j in range(1, m + 1))
    def tryToMatch(u):
        if visitedX[u]: return False
        visitedX[u] = True
        for v in range(1, m + 1):
            if not visitedY[v] and a[u][v] == lx[u] + ly[v]:
                visitedY[v] = True
                if matching[v] == -1 or tryToMatch(matching[v]):
                    matching[v] = u
                    return True
        return False
    for i in range(1, n + 1): # O(N)
        while True: # O(N) co the +delta toi da n lan
            visitedX = [False] * (n + 1)
            visitedY = [False] * (m + 1)
            if tryToMatch(i): # O(N^2) bipartite graph co the co max n*n edges
                break
            delta = float('inf')
            for u in range(1, n + 1): # O(N)
                if visitedX[u]:
                    for v in range(1, m + 1): # O(N)
                        if not visitedY[v]:
                            delta = min(delta, a[u][v] - lx[u] - ly[v])
            for u in range(1, n + 1):
                if visitedX[u]: lx[u] += delta
            for v in range(1, m + 1):
                if visitedY[v]: ly[v] -= delta
    ans = 0
    for v in range(1, m + 1):
        if matching[v] != -1:
            ans += a[matching[v]][v]
    print(ans)
def solveSub2(n, m, a):
    transposed_a = [[0] * (n + 1) for _ in range(m + 1)]
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            transposed_a[j][i] = a[i][j]

    solveSub1(m, n, transposed_a)

if __name__ == "__main__":
    n, m, a = readInput()
    if n <= m: solveSub1(n, m, a)
    else: solveSub2(n, m, a)
# -> O(N^4)