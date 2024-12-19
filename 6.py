import cv2
import numpy as np

class DistanceTracker:
    def __init__(self, arduino):
        self.arduino = arduino
        self.camera = cv2.VideoCapture(0)  # Камера для захвата изображения
        self.quarter_count = 0  # Счетчик пройденных четвертей
        self.previous_position = None  # Для отслеживания движения
        self.marker_positions = [100, 200, 300, 400]  # Пример маркеров для четвертей (настраиваем по трассе)

    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            return None
        return frame

    def detect_marker(self, frame):
        # Преобразуем изображение в серый цвет и ищем маркеры (например, линии)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Для простоты будем искать контуры, которые могут быть маркерами
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 50 and h > 50:  # Фильтруем по минимальным размерам маркеров
                return (x, y)  # Возвращаем координаты маркера
        return None

    def track_distance(self):
        while True:
            frame = self.process_frame()
            if frame is None:
                continue

            # Ищем маркер на пути
            marker_position = self.detect_marker(frame)

            if marker_position:
                x, y = marker_position

                # Проверяем, если мы прошли определенную четверть
                for marker in self.marker_positions:
                    if x > marker and (self.previous_position is None or self.previous_position < marker):
                        self.previous_position = x
                        self.quarter_count += 1
                        print(f"Пройдено {self.quarter_count} четвертей стадиона.")
                        break

            # Отображаем изображение для отладки
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def stop(self):
        self.arduino.stop()
        self.camera.release()
        cv2.destroyAllWindows()
