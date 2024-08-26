import sys
import os
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class RealTimePlotApp(QMainWindow):
    def __init__(self):
        super(RealTimePlotApp, self).__init__()
        self.setWindowTitle("Real-Time Plots")
        self.setGeometry(100, 100, 800, 600)
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout(self.central_widget)

        # Create the Figure and Canvas for the plots
        self.plot_figure, self.plot_axes = plt.subplots(2, 1)
        self.plot_canvas = FigureCanvas(self.plot_figure)
        layout.addWidget(self.plot_canvas)

        pipe_name = '/tmp/my_named_pipe'

        if not os.path.exists(pipe_name):
            print("Named pipe does not exist.")
            exit(1)

        self.pipe = open(pipe_name, 'r')

        self.plot_data1 = []
        self.plot_data2 = []
        self.max_data_points = 50
        self.animation_interval = 1  # in milliseconds

    def update_plot(self, frame):

        line = self.pipe.readline()
        if line:        
          dataString = line.strip()
          datas = dataString.split(',')
          data_point1 = float(datas[0])
          data_point2 = float(datas[1])

        # Generate random data for both plots for demonstration purposes
        #data_point1 = random.randint(1, 100)
        #data_point2 = random.randint(1, 100)
        self.plot_data1.append(data_point1)
        self.plot_data2.append(data_point2)

        # Limit the data points to show on both plots
        if len(self.plot_data1) > self.max_data_points:
            self.plot_data1 = self.plot_data1[-self.max_data_points:]
        if len(self.plot_data2) > self.max_data_points:
            self.plot_data2 = self.plot_data2[-self.max_data_points:]

        # Update the first subplot (plot 1)
        self.plot_axes[0].clear()
        self.plot_axes[0].plot(range(len(self.plot_data1)), self.plot_data1, 'b-')
        self.plot_axes[0].set_xlabel('Time')
        self.plot_axes[0].set_ylabel('Value')
        self.plot_axes[0].set_title('Real-Time Plot 1')
        self.plot_axes[0].grid(True)

        # Update the second subplot (plot 2)
        self.plot_axes[1].clear()
        self.plot_axes[1].plot(range(len(self.plot_data2)), self.plot_data2, 'r-')
        self.plot_axes[1].set_xlabel('Time')
        self.plot_axes[1].set_ylabel('Value')
        self.plot_axes[1].set_title('Real-Time Plot 2')
        self.plot_axes[1].grid(True)

        # Update the second subplot (plot 2)
        #self.plot_axes[2].clear()
        #self.plot_axes[2].text(0, 0, 'Data: '+str(data_point1), fontsize = 18)
        #self.plot_axes[2].axis('off')

        # Redraw the canvas
        self.plot_canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RealTimePlotApp()

    # Use FuncAnimation to update the plot at high frequency
    animation = FuncAnimation(window.plot_figure, window.update_plot, interval=window.animation_interval)

    window.show()
    sys.exit(app.exec_())

