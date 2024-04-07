import cv2
import numpy as np
import tkinter as tk

class EclipseSimulation:
    def __init__(self, sun_radius=60, moon_radius=25, earth_radius=35):  # Adjusted sizes for all bodies
        self.sun_radius = sun_radius
        self.moon_radius = moon_radius
        self.earth_radius = earth_radius
        self.sun_center = (0, 0)  # Placeholder
        self.earth_center = (0, 0)  # Placeholder
        self.moon_distance = 200  # Increased distance for better visualization
        self.moon_angle = 0
        self.earth_angle = 0
        self.sun_angle = 0
        self.frame_width = 0
        self.frame_height = 0
        self.get_screen_resolution()

    def get_screen_resolution(self):
        root = tk.Tk()
        self.frame_width = root.winfo_screenwidth()
        self.frame_height = root.winfo_screenheight()
        root.destroy()

    def draw_circle(self, image, center, radius, color, thickness=-1):
        cv2.circle(image, center, radius, color, thickness)

    def generate_frame(self):
        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)

        # Draw Sun
        self.sun_center = (self.frame_width // 2, self.frame_height // 2)
        cv2.circle(frame, self.sun_center, self.sun_radius, (0, 255, 255), -1)

        # Update Earth angle
        self.earth_angle += 0.01

        # Update Moon angle
        self.moon_angle += 0.02

        # Draw Earth
        self.earth_center = (int(self.sun_center[0] + 2 * self.moon_distance * np.cos(self.earth_angle)),
                             int(self.sun_center[1] + 2 * self.moon_distance * np.sin(self.earth_angle)))
        self.draw_circle(frame, self.earth_center, self.earth_radius, (0, 255, 0), -1)

        # Draw Moon
        self.moon_center = (int(self.earth_center[0] + self.moon_distance * np.cos(self.moon_angle)),
                            int(self.earth_center[1] + self.moon_distance * np.sin(self.moon_angle)))
        self.draw_circle(frame, self.moon_center, self.moon_radius, (128, 128, 128), -1)

        # Draw rays from Sun
        for angle in np.arange(0, 2 * np.pi, 0.1):
            ray_start = (self.sun_center[0] + self.sun_radius * np.cos(angle),
                         self.sun_center[1] + self.sun_radius * np.sin(angle))
            ray_end = (self.sun_center[0] + 10 * self.sun_radius * np.cos(angle),
                       self.sun_center[1] + 10 * self.sun_radius * np.sin(angle))

            # Check if the ray intersects with the Earth
            intersection_point_earth = self.ray_circle_intersection(ray_start, ray_end, self.earth_center, self.earth_radius)
            if intersection_point_earth is not None:
                # Cut off the ray when it hits the Earth
                ray_end = intersection_point_earth

            # Check if the ray intersects with the Moon
            intersection_point_moon = self.ray_circle_intersection(ray_start, ray_end, self.moon_center, self.moon_radius)
            if intersection_point_moon is not None:
                # Cut off the ray when it hits the Moon
                ray_end = intersection_point_moon

            cv2.line(frame, (int(ray_start[0]), int(ray_start[1])),
                     (int(ray_end[0]), int(ray_end[1])), (0, 255, 255), 1)

        return frame

    def ray_circle_intersection(self, start, end, center, radius):
        # Vector from start to end of the ray
        d = np.array(end) - np.array(start)

        # Vector from center of the circle to start of the ray
        f = np.array(start) - np.array(center)

        # Solving quadratic equation parameters
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - radius * radius

        discriminant = b * b - 4 * a * c

        if discriminant < 0:
            # No intersection
            return None

        t1 = (-b + np.sqrt(discriminant)) / (2 * a)
        t2 = (-b - np.sqrt(discriminant)) / (2 * a)

        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            # Intersection points not on the ray
            return None

        t = min(t1, t2)

        intersection_point = np.array(start) + t * d

        return intersection_point

    def run_simulation(self):
        while True:
            frame = self.generate_frame()
            cv2.imshow('Eclipse Simulation', frame)
            if cv2.waitKey(50) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    eclipse_sim = EclipseSimulation()
    eclipse_sim.run_simulation()
