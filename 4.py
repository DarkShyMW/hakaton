import cv2
import numpy as np

class VisionBasedCar:
    def __init__(self, arduino):
        self.arduino = arduino
        self.camera = cv2.VideoCapture(0)  # Подключение камеры
        self.default_speed = 1350
        self.angle_coefficient = 0.5  # Коэффициент для вычисления угла

    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            return None, None

        # Преобразование изображения
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)  # Поиск границ

        # Обнаружение линий с помощью преобразования Хафа
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=200)
        return frame, lines

    def calculate_angle(self, lines):
        if lines is None:
            return 90  # Нет линии — двигаться прямо

        left_lines = []
        right_lines = []

        # Разделение линий на левую и правую группы
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Уклон линии
                if slope < 0:  # Левая линия
                    left_lines.append(line)
                else:  # Правая линия
                    right_lines.append(line)

        # Вычисление средней линии
        if left_lines:
            left_mean = np.mean([line[0] for line in left_lines], axis=0)
        else:
            left_mean = None

        if right_lines:
            right_mean = np.mean([line[0] for line in right_lines], axis=0)
        else:
            right_mean = None

        # Вычисление центральной линии
        if left_mean is not None and right_mean is not None:
            center_x = (left_mean[0] + right_mean[0]) / 2
        elif left_mean is not None:
            center_x = left_mean[0] + 100  # Ориентироваться на левую линию
        elif right_mean is not None:
            center_x = right_mean[0] - 100  # Ориентироваться на правую линию
        else:
            return 90  # Если нет линий, двигаться прямо

        # Вычисление угла поворота
        frame_center = 320  # Средняя точка камеры (предполагается разрешение 640x480)
        offset = center_x - frame_center
        angle = 90 - offset * self.angle_coefficient
        return max(65, min(115, angle))  # Ограничение угла поворота

    def follow_line(self):
        while True:
            frame, lines = self.process_frame()
            if frame is None:
                continue

            angle = self.calculate_angle(lines)
            self.arduino.set_angle(angle)
            self.arduino.set_speed(self.default_speed)

            # Показ изображения для отладки
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def stop(self):
        self.arduino.stop()
        self.camera.release()
        cv2.destroyAllWindows()
