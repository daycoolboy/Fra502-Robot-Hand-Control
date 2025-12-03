import matplotlib
matplotlib.use('Agg')  
import matplotlib.pyplot as plt
import numpy as np

class AngleRecorder:

    def __init__(self):
        self.thumb_cam  = []
        self.index_cam  = []
        self.middle_cam = []
        self.ring_cam   = []
        self.pinky_cam  = []

        self.thumb_fb  = []
        self.index_fb  = []
        self.middle_fb = []
        self.ring_fb   = []
        self.pinky_fb  = []

    def add(self, arr):
        self.thumb_cam.append(arr[0])
        self.index_cam.append(arr[1])
        self.middle_cam.append(arr[2])
        self.ring_cam.append(arr[3])
        self.pinky_cam.append(arr[4])

    def add_feedback(self, arr):
        self.thumb_fb.append(arr[0])
        self.index_fb.append(arr[1])
        self.middle_fb.append(arr[2])
        self.ring_fb.append(arr[3])
        self.pinky_fb.append(arr[4])

    def smooth(self, data, w=10):
        data = np.array(data)
        if len(data) < w:
            return data
        return np.convolve(data, np.ones(w)/w, mode='same')

    def plot(self):

        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        axes[0].set_title("Vision → Desired Servo Angles")
        axes[0].plot(self.smooth(self.thumb_cam),  label="Thumb")
        axes[0].plot(self.smooth(self.index_cam),  label="Index")
        axes[0].plot(self.smooth(self.middle_cam), label="Middle")
        axes[0].plot(self.smooth(self.ring_cam),   label="Ring")
        axes[0].plot(self.smooth(self.pinky_cam),  label="Pinky")
        axes[0].grid(True)
        axes[0].legend()

        axes[1].set_title("STM32 → Actual Servo Angles")
        if len(self.thumb_fb) > 0:
            axes[1].plot(self.smooth(self.thumb_fb),  label="Thumb FB")
            axes[1].plot(self.smooth(self.index_fb),  label="Index FB")
            axes[1].plot(self.smooth(self.middle_fb), label="Middle FB")
            axes[1].plot(self.smooth(self.ring_fb),   label="Ring FB")
            axes[1].plot(self.smooth(self.pinky_fb),  label="Pinky FB")
            axes[1].legend()
        else:
            axes[1].text(0.5, 0.5, "No Servo Feedback Received", ha='center')

        axes[1].grid(True)

        plt.tight_layout(rect=[0, 0, 1, 0.96])

        plt.savefig("finger_plot.png")  
        print("Saved: finger_plot.png")
