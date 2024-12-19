import cv2
import numpy as np

class DistanceMeasurementCar:
    def __init__(self):
        self.camera = cv2.VideoCapture(0)  # Подключение камеры
        self.quarters_count = 0
        self.previous_marker = None  # Предыдущий обнаруженный маркер

    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            return None, None

        # Преобразование изображения
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)  # Поиск границ

        # Обнаружение маркеров или разметки
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return frame, contours

    def detect_marker(self, contours):
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Отбрасываем слишком маленькие контуры
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h
                if 2 < aspect_ratio < 5:  # Пример условия для распознавания маркера
                    return (x, y, w, h)
        return None

    def measure_distance(self):
        while True:
            frame, contours = self.process_frame()
            if frame is None:
                continue

            marker = self.detect_marker(contours)

            if marker:
                x, y, w, h = marker
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Отображение маркера
                center_x = x + w // 2

                # Логика определения четвертей
                if self.previous_marker is not None:
                    prev_x, prev_y = self.previous_marker
                    if abs(center_x - prev_x) > 50:  # Пороговое значение
                        self.quarters_count += 1
                        print(f"Пройдено четвертей: {self.quarters_count}")

                self.previous_marker = (center_x, y)

            # Отображение результата
            cv2.putText(frame, f"Quarters: {self.quarters_count}", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def stop(self):
        self.camera.release()
        cv2.destroyAllWindows()
