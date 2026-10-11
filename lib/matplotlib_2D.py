import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class plot_2D:
    def __init__(self, pause=0.005, plotrange=1000, s=1):
        self.x_list = np.array([])
        self.y_list = np.array([])
        self.color_list = np.array([])

        self.fig = plt.figure(figsize=(8, 8))
        self.fig.patch.set_facecolor('black')  # set the figure's background color to black
        self.ax = self.fig.add_subplot(111)
        self.plotrange = plotrange
        self.ax.set_ylim([-self.plotrange, self.plotrange])
        self.ax.set_xlim([-self.plotrange, self.plotrange])
        self.ax.set_title('LiDAR', fontsize=18)
        self.ax.set_facecolor('black')
        # self.ax.xaxis.grid(True, color='yellow', linestyle='dashed')
        # self.ax.yaxis.grid(True, color='yellow', linestyle='dashed')
        # self.ax.set_xticks(np.arange(-self.plotrange, self.plotrange+1, 100))
        # self.ax.set_yticks(np.arange(-self.plotrange, self.plotrange+1, 100))
        self.ani = self.__initialize_animation__()
        self.pause = pause
        self.s = s

    def __initialize_animation__(self):
        return animation.FuncAnimation(self.fig, self.animate, init_func=self.init, frames=1, interval=1, blit=True)

    def __gray2rgb__(self, luminances):
        return np.column_stack((luminances, luminances, luminances))

    def init(self):
        return self.ax,

    def animate(self, i):
        line = getattr(self, 'line', None)
        if line is not None:
            line.remove()

        self.line = self.ax.scatter(self.x_list, self.y_list, c=self.color_list/255, s=self.s)
        return self.line,

    # def update_lists(self, x_list, y_list, luminance_list):
    #     self.x_list = np.asarray(x_list)
    #     self.y_list = np.asarray(y_list)
    #     self.color_list = self.__gray2rgb__(np.asarray(luminance_list))
    #     plt.pause(self.pause)

    def update(self, points_2d):  # points_2d: np.array([[x, y, luminance], ...])
        line = getattr(self, 'line', None)
        if line is not None:
            line.remove()

        self.x_list = points_2d[:, 0]
        self.y_list = points_2d[:, 1]
        self.color_list = self.__gray2rgb__(points_2d[:, 2])

        self.line = self.ax.scatter(self.x_list, self.y_list, c=self.color_list/255, s=self.s)

        # # Set the plot range based on the maximum absolute value in x_list and y_list
        # self.plotrange = max(max(abs(self.x_list)), max(abs(self.y_list)))
        # self.ax.set_xlim([-self.plotrange, self.plotrange])
        # self.ax.set_ylim([-self.plotrange, self.plotrange])

        plt.pause(self.pause)

    def close(self):
        plt.close()


if __name__ == "__main__":
    plot = plot_2D(plotrange=1000, s=100)

    # Test the update method
    x_list = [0, 100, 200, 300]
    y_list = [0, 200, 400, 600]
    luminance_list = [128, 160, 180, 210]

    # Create a points_2d array with your test points
    points_2d = np.column_stack((x_list, y_list, luminance_list))

    # Convert points_2d to a float array
    points_2d = points_2d.astype(float)

    # Test the update method
    plot.update(points_2d)

    # Test the animate method by iteratively scaling all x coordinates once per second
    for i in range(100):
        points_2d[:, 0] *= 1.1  # scale the x coordinates
        plot.update(points_2d)
        plot.animate(i)
        plt.pause(0.05)  # pause for 1 second

    # Keep the plot open until the user closes it
    plt.show()
    plot.close()
