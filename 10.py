import cv2
import numpy as np

class TrafficLightStopCar:
    def __init__(self, arduino):
        self.arduino = arduino
        self.camera = cv2.VideoCapture(0)  # Камера беспилотника
        self.default_speed = 1350
        self.stop_speed = 1250
        self.slow_speed = 1400
        self.stop_distance_range = (15, 40)  # Расстояние в см, на котором должна остановиться машина

    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            return None
        return frame

    def detect_traffic_light(self, frame):
        # Преобразуем изображение в HSV для лучшего выделения цветов
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Диапазоны для каждого цвета светофора
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([90, 255, 255])

        # Маски для каждого цвета
        mask_red = cv2.inRange(hsv, red_lower, red_upper)
        mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
        mask_green = cv2.inRange(hsv, green_lower, green_upper)

        # Проверяем наличие цветов
        red_detected = np.any(mask_red)
        yellow_detected = np.any(mask_yellow)
        green_detected = np.any(mask_green)

        return red_detected, yellow_detected, green_detected

    def recognize_signal(self, red_detected, yellow_detected, green_detected):
        if red_detected and yellow_detected:
            return "Красный+Желтый"
        elif red_detected:
            return "Красный"
        elif yellow_detected:
            return "Желтый"
        elif green_detected:
            return "Зеленый"
        return "Неизвестный"

    def stop_at_traffic_light(self):
        while True:
            frame = self.process_frame()
            if frame is None:
                continue

            # Распознаем сигнал светофора
            red_detected, yellow_detected, green_detected = self.detect_traffic_light(frame)
            signal = self.recognize_signal(red_detected, yellow_detected, green_detected)

            # Если сигнал не зеленый, останавливаемся в пределах 15-40 см от светофора
            if signal != "Зеленый":
                print(f"Сигнал светофора: {signal} - Остановка.")
                self.arduino.set_speed(self.stop_speed)  # Устанавливаем минимальную скорость
                self.arduino.dist(40)  # Останавливаемся через 40 условных единиц (настраиваем по необходимости)
                break
            else:
                # Если сигнал зеленый, продолжаем движение
                print("Сигнал светофора: Зеленый - Движение.")
                self.arduino.set_speed(self.slow_speed)

            # Отображение для отладки
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def stop(self):
        self.arduino.stop()
        self.camera.release()
        cv2.destroyAllWindows()
