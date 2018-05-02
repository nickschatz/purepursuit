import math
import tkinter as tk
from PIL import ImageTk, Image

from py_pursuit_pathing.splines import ComboSpline, CubicSpline, LinearSpline, QuinticSpline, ArcSpline
from py_pursuit_pathing.pose import Pose


class Application(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)

        self.knots = [Pose(0, -10, 0), Pose(20, -10, math.pi / 4), Pose(20.1, 7, 7*math.pi/16)]
        self.spline = ComboSpline(self.knots)
        self.pt_frames = []
        self.pack()
        self.create_widgets()

    def draw_field(self):
        self.field_img = ImageTk.PhotoImage(Image.open("2018-field.gif"))
        self.cv.create_image(397 // 2, 404 // 2, image=self.field_img)

    def create_widgets(self):
        self.cv = tk.Canvas(self, width=397, height=404)
        self.draw_field()
        self.cv.pack(side='top', fill='both', expand='yes')

        self.cmd_frame = tk.Frame(self)
        self.cmd_frame.pack(side="top")

        self.recalc_button = tk.Button(self.cmd_frame, text="Reticulate", command=self.reticulate)
        self.recalc_button.pack(side="left")

        self.add_button = tk.Button(self.cmd_frame, text="Add point", command=self.add_point)
        self.add_button.pack(side="left")

        self.print_button = tk.Button(self.cmd_frame, text="Print", command=lambda: print(self.knots))
        self.print_button.pack(side="left")

        self.fit_select = tk.StringVar(self, "Cubic")
        self.fit_control = tk.OptionMenu(self.cmd_frame, self.fit_select, "Linear", "Cubic", "Quintic", "Biarc")
        self.fit_control.pack(side="left")

        self.point_frame = tk.Frame(self)
        self.point_frame.pack(side="top")

        self.add_point()
        self.add_point()

    def reticulate(self):
        self.draw_field()
        self.knots = []
        for frame in self.pt_frames:
            children = frame.winfo_children()
            x = float(children[0].get())
            y = float(children[1].get())
            h = float(children[2].get()) * math.pi / 180
            self.knots.append(Pose(x,y,h))
        if self.fit_select.get() == "Combo":
            self.spline = ComboSpline(self.knots)
        elif self.fit_select.get() == "Quintic":
            self.spline = QuinticSpline(self.knots)
        elif self.fit_select.get() == "Cubic":
            self.spline = CubicSpline(self.knots)
        elif self.fit_select.get() == "Biarc":
            self.spline = ArcSpline(self.knots)
        else:
            self.spline = LinearSpline(self.knots)
        self.draw_spline()

    def remove_point(self, frame):
        self.pt_frames.remove(frame)
        frame.destroy()

    def add_point(self):
        new_pt_frame = tk.Frame(self.point_frame)
        new_pt_frame.pack(side="top")
        x_field = tk.Entry(new_pt_frame)
        x_field.pack(side="left")
        y_field = tk.Entry(new_pt_frame)
        y_field.pack(side="left")
        heading_field = tk.Entry(new_pt_frame)
        heading_field.pack(side="left")
        remove_button = tk.Button(new_pt_frame, text="Remove", command=lambda: self.remove_point(new_pt_frame))
        remove_button.pack(side="left")

        self.pt_frames.append(new_pt_frame)

    def real_to_canvas(self, x, y):
        return int(397*(x / 27)), 404//2 - int(404*(y / 27))

    def draw_spline(self):
        for t in range(100):
            t0 = t / 100
            t1 = (t+1) / 100
            pt1 = self.spline.get_point(t0)
            pt2 = self.spline.get_point(t1)
            x0, y0 = self.real_to_canvas(pt1.x, pt1.y)
            x1, y1 = self.real_to_canvas(pt2.x, pt2.y)
            self.cv.create_line(x0, y0, x1, y1, fill="#0000ff", width=3)


if __name__ == "__main__":
    root = tk.Tk()
    app = Application(master=root)
    app.mainloop()