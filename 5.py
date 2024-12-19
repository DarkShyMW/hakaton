import cv2
import numpy as np

class StopLineCar:
    def __init__(self, arduino):
        self.arduino = arduino
        self.camera = cv2.VideoCapture(0)  # Подключение камеры
        self.default_speed = 1350

    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            return None, None

        # Преобразование изображения
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)  # Пороговое преобразование

        # Поиск контуров
        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        stop_line_contour = None
        for contour in contours:
            # Определяем прямоугольник вокруг контура
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h
            if aspect_ratio > 5 and 10 < w < 300:  # Условие на горизонтальную линию
                stop_line_contour = contour
                break

        return frame, stop_line_contour

    def calculate_distance(self, contour, frame_height):
        if contour is None:
            return float('inf')  # Если линия не найдена, возвращаем "бесконечное" расстояние

        # Найти нижнюю точку контура
        bottom_point = max(contour, key=lambda point: point[0][1])[0]

        # Вычисление расстояния до стоп-линии
        distance_pixels = frame_height - bottom_point[1]
        distance_cm = distance_pixels * 0.05  # Коэффициент преобразования пикселей в см
        return distance_cm

    def stop_before_line(self):
        while True:
            frame, stop_line_contour = self.process_frame()
            if frame is None:
                continue

            frame_height = frame.shape[0]
            distance = self.calculate_distance(stop_line_contour, frame_height)

            if distance < 15:
                self.arduino.stop()
                print("Остановка перед стоп-линией")
                break
            else:
                self.arduino.set_speed(self.default_speed)

            # Для отладки показываем изображение
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def stop(self):
        self.arduino.stop()
        self.camera.release()
        cv2.destroyAllWindows()
