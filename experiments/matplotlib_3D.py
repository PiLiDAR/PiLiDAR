import numpy as np
import matplotlib.pyplot as plt


def plot_3D(pointcloud):
    xs = pointcloud[:, 0]
    ys = pointcloud[:, 1]
    zs = pointcloud[:, 2]
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(projection='3d')
    ax.set_box_aspect((np.ptp(xs), np.ptp(ys), np.ptp(zs)))
    img = ax.scatter(xs, ys, zs, s=1)  # , c=t_low, cmap=plt.hot())
    fig.colorbar(img)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


if __name__ == "__main__":
    # https://www.tutorialspoint.com/plotting-points-on-the-surface-of-a-sphere-in-python-s-matplotlib

    # Generate points on the surface of a sphere
    u, v = np.mgrid[0:2 * np.pi:5j, 0:np.pi:5j]
    x = np.cos(u) * np.sin(v)
    y = np.sin(u) * np.sin(v)
    z = np.cos(v)

    # Reshape the points to match the expected input format for plot_3D
    sphere_pointcloud = np.vstack((x.flatten(), y.flatten(), z.flatten())).T  # np.random.rand(1000, 3)
    plot_3D(sphere_pointcloud)
