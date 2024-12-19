import cv2
import numpy as np

class CrosswalkNavigationCar:
    def __init__(self, arduino):
        self.arduino = arduino
        self.camera = cv2.VideoCapture(0)  # Подключение камеры
        self.default_speed = 1350
        self.slow_speed = 1400

    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            return None

        # Преобразование в чёрно-белый формат
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)  # Белые области

        return frame, binary

    def detect_crosswalk(self, binary):
        # Нахождение контуров
        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Условие для больших белых областей
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h
                if 2 < aspect_ratio < 6:  # Соотношение сторон полосы зебры
                    return True  # Пешеходный переход найден
        return False

    def detect_stop_line(self, binary):
        # Нахождение горизонтальных линий
        lines = cv2.HoughLinesP(binary, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=10)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(y1 - y2) < 10:  # Почти горизонтальная линия
                    return True  # Стоп-линия найдена
        return False

    def navigate_crosswalk(self):
        while True:
            frame, binary = self.process_frame()
            if frame is None:
                continue

            # Проверка наличия пешеходного перехода
            if self.detect_crosswalk(binary):
                self.arduino.set_speed(self.slow_speed)
                print("Пешеходный переход: снижение скорости")

            # Проверка наличия стоп-линии
            elif self.detect_stop_line(binary):
                self.arduino.stop()
                print("Стоп-линия обнаружена: остановка")
                break
            else:
                self.arduino.set_speed(self.default_speed)

            # Отображение для отладки
            cv2.imshow("Frame", frame)
            cv2.imshow("Binary", binary)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def stop(self):
        self.arduino.stop()
        self.camera.release()
        cv2.destroyAllWindows()
