def calculate_storage_index(N, o, NL, NC, NA):
    # index arrays
    x = []
    y = []
    z = []
    for i in range(0, N * 3, 3):
        x.append(o[i] // 1000)
        y.append(o[i + 1] // 1000)
        z.append(o[i + 2] // 10 - 3)

    if not (len(x) == len(y) == len(z) == N):
        raise ValueError("The lengths of x, y, z, and N should be the same")

    storage_index = 0
    for i in range(N):
        storage_index = (x[i] + y[i] * NL + z[i] * (NL * NC)) * (NL * NC * NA) ** i + storage_index

    return storage_index


lista = []
i = 0
ii = 0
for i1 in range(500, 9501, 1000):
    for j1 in range(500, 9501, 1000):
        for k1 in range(30, 31, 10):
            for i2 in range(500, 9501, 1000):
                for j2 in range(500, 9501, 1000):
                    for k2 in range(30, 31, 10):
                        for i3 in range(500, 9501, 1000):
                            for j3 in range(500, 9501, 1000):
                                for k3 in range(30, 31, 10):
                                    if ([i1, j1, k1] == [i2, j2, k2]) or ([i1, j1, k1] == [i3, j3, k3]) or (
                                        [i2, j2, k2] == [i3, j3, k3]):
                                        i += 1
                                        print([i1, j1, k1], [i2, j2, k2], [i3, j3, k3])
                                        continue
                                    lista.append(
                                        calculate_storage_index(3, [i1, j1, k1, i2, j2, k2, i3, j3, k3], 10, 10, 1))
                                    ii += 1

print(sorted(lista))
