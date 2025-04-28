import tkinter as tk
import numpy as np
import math


class RotatingTorus:

    def changed(self, event=None):
        self.alpha = self.a_slide.get()
        self.beta = self.b_slide.get()
        self.u_steps = self.det_slide.get()
        self.v_steps = self.det_slide.get()

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("3D График")
        self.root.resizable(False, False)

        self.color = "#FF99FF"

        self.canvas = tk.Canvas(root, width=800, height=600, bg="white")
        self.canvas.grid(row=0, column=1)

        self.a_slide = tk.Scale(self.root, from_=1, to=10, length=200, orient='vertical', command=self.changed)
        self.a_slide.grid(row=0, column=0)
        self.b_slide = tk.Scale(self.root, from_=1, to=10, length=200, orient='vertical', command=self.changed)
        self.b_slide.grid(row=0, column=2)
        self.det_slide = tk.Scale(self.root, from_=10, to=100, length=500, orient='horizontal', command=self.changed)
        self.det_slide.grid(row=1, column=1)

        self.alpha = 1
        self.beta = 1.0
        self.u_steps = 10
        self.v_steps = 10

        self.theta = 0.0
        self.phi = 0.0
        self.rotation_speed = 0.003

        self.light_direction = np.array([0, 0, -1])  # свет направлен из камеры

        self.generate_torus_points()
        self.animate()

    def generate_torus_points(self):
        u = np.linspace(0, 2 * math.pi, self.u_steps)
        v = np.linspace(0, 2 * math.pi, self.v_steps)
        U, V = np.meshgrid(u, v, indexing='ij')

        self.X = (self.alpha + self.beta * np.cos(V)) * np.cos(U)
        self.Y = (self.alpha + self.beta * np.cos(V)) * np.sin(U)
        self.Z = self.beta * np.sin(V)

    def rotate_points(self):
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(self.phi), -math.sin(self.phi)],
            [0, math.sin(self.phi), math.cos(self.phi)]
        ])
        Ry = np.array([
            [math.cos(self.theta), 0, math.sin(self.theta)],
            [0, 1, 0],
            [-math.sin(self.theta), 0, math.cos(self.theta)]
        ])
        return np.dot(Rx, Ry)

    def project(self, x, y, z):
        scale = 100
        x_proj = x * scale + 400
        y_proj = -y * scale + 300
        return x_proj, y_proj

    def calculate_depth(self, points):
        return np.mean([p[2] for p in points])

    def calculate_normal(self, p0, p1, p2):
        v1 = np.array(p1) - np.array(p0)
        v2 = np.array(p2) - np.array(p0)
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        return normal / norm if norm != 0 else normal

    def compute_lighting(self, normal):
        dot = np.dot(normal, self.light_direction)
        return max(0.1, min(1.0, dot))  # от 0.1 до 1.0 — минимальная яркость 10%

    def shade_color(self, hex_color, brightness):
        hex_color = hex_color.lstrip("#")
        rgb = [int(hex_color[i:i+2], 16) for i in (0, 2, 4)]
        shaded = [int(min(255, max(0, c * brightness))) for c in rgb]
        return "#%02x%02x%02x" % tuple(shaded)

    def draw(self):
        self.generate_torus_points()
        self.canvas.delete("all")
        rotation_matrix = self.rotate_points()
        polygons = []

        for i in range(self.u_steps - 1):
            for j in range(self.v_steps - 1):
                points_3d = [
                    (self.X[i, j], self.Y[i, j], self.Z[i, j]),
                    (self.X[i + 1, j], self.Y[i + 1, j], self.Z[i + 1, j]),
                    (self.X[i + 1, j + 1], self.Y[i + 1, j + 1], self.Z[i + 1, j + 1]),
                    (self.X[i, j + 1], self.Y[i, j + 1], self.Z[i, j + 1])
                ]

                rotated_points = np.dot(points_3d, rotation_matrix.T)
                depth = self.calculate_depth(rotated_points)
                normal = self.calculate_normal(*rotated_points[:3])
                brightness = self.compute_lighting(normal)
                color = self.shade_color(self.color, brightness)

                pixels = []
                for point in rotated_points:
                    px, py = self.project(*point)
                    pixels.extend([px, py])

                if len(pixels) == 8:
                    polygons.append({
                        'points': [pixels[0], pixels[1], pixels[2], pixels[3], pixels[4], pixels[5]],
                        'depth': depth,
                        'color': color
                    })
                    polygons.append({
                        'points': [pixels[0], pixels[1], pixels[6], pixels[7], pixels[4], pixels[5]],
                        'depth': depth,
                        'color': color
                    })

        polygons.sort(key=lambda x: -x['depth'])

        for poly in polygons:
            self.canvas.create_polygon(
                poly['points'],
                fill=poly['color'],
                outline="black",
                width=1
            )

    def animate(self):
        self.theta += self.rotation_speed
        self.phi += self.rotation_speed * 0.7
        self.draw()
        self.root.after(1, self.animate)


if __name__ == "__main__":
    root = tk.Tk()
    app = RotatingTorus(root)
    root.mainloop()