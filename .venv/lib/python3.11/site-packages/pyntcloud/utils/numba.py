from numba import jit


@jit
def groupby_count(xyz, indices, out):
    for i in range(xyz.shape[0]):
        out[indices[i]] += 1
    return out


@jit
def groupby_sum(xyz, indices, N, out):
    for i in range(xyz.shape[0]):
        out[indices[i]] += xyz[i][N]
    return out


@jit
def groupby_max(xyz, indices, N, out):
    for i in range(xyz.shape[0]):
        if xyz[i][N] > out[indices[i]]:
            out[indices[i]] = xyz[i][N]
    return out
