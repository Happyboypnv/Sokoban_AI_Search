def readInput():
    n, m = map(int, input().split())
    a = [[0]*(m + 1) for _ in range(n + 1)]
    for i in range(1, n + 1):
        row = list(map(int, input().split()))
        for j in range(1, m + 1):
            a[i][j] = row[j - 1]
    return n, m, a
def solve(n, m, a):
    if n > m:
        transposed_a = [[0]* (n +  1) for _ in range(m + 1)]
        for i in range(1, n + 1):
            for j in range(1, m + 1):
                transposed_a[j][i] = a[i][j]
        solve(m, n, transposed_a)
    else:
        matching = [0]*(m + 1)
        lx = [0]*(n + 1)
        ly = [0]*(m + 1)
        trace = [0]*(m + 1)
        for i in range(1, n + 1): 
            lx[i] = min(a[i][j] for j in range(1, m + 1))
        for i in range(1, n + 1): 
            matching[0] = i
            j0 = 0

            minv = [float('inf')]*(m + 1)
            visited = [False]*(m + 1)
            while True: 
                visited[j0] = True
                i0 = matching[j0]
                delta = float('inf')
                j1 = 0
                for j in range(1, m + 1): 
                    if not visited[j]: 
                        cur_slack = a[i0][j] - lx[i0] - ly[j]
                        if cur_slack < minv[j]: 
                            trace[j] = j0
                            minv[j] = cur_slack
                        if minv[j] < delta: 
                            delta = minv[j]
                            j1 = j
                for j in range(0, m + 1): 
                    if visited[j]: 
                        lx[matching[j]] += delta
                        ly[j] -= delta
                    else:
                        minv[j] -= delta
                j0 = j1
                if matching[j0] == 0: 
                    break
            while True: 
                pre_j = trace[j0]
                matching[j0] = matching[pre_j]
                j0 = pre_j
                if j0 == 0: break

        ans = 0
        for j in range(1, m + 1): 
            if matching[j] != 0: 
                ans += a[matching[j]][j]
        print(ans)


        
if __name__ == "__main__":
    n, m, a = readInput()
    solve(n, m, a)          