import cv2
import numpy as np

class TrafficLightStartCar:
    def __init__(self, arduino):
        self.arduino = arduino
        self.camera = cv2.VideoCapture(0)  # Камера для захвата изображения
        self.default_speed = 1350  # Стандартная скорость движения
        self.stop_speed = 1250  # Скорость для остановки

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
        if green_detected:
            return "Зеленый"
        elif red_detected:
            return "Красный"
        elif yellow_detected:
            return "Желтый"
        return "Неизвестный"

    def start_on_green_light(self):
        print("Программа запущена. Ожидаем зеленый сигнал светофора...")
        while True:
            frame = self.process_frame()
            if frame is None:
                continue

            # Распознаем сигнал светофора
            red_detected, yellow_detected, green_detected = self.detect_traffic_light(frame)
            signal = self.recognize_signal(red_detected, yellow_detected, green_detected)

            # Если зеленый сигнал, начинаем движение
            if signal == "Зеленый":
                print("Сигнал светофора: Зеленый - Начало движения.")
                self.arduino.set_speed(self.default_speed)  # Устанавливаем стандартную скорость
                break  # Выход из цикла, начало движения

            else:
                print(f"Сигнал светофора: {signal} - Ожидание зеленого сигнала.")
                self.arduino.set_speed(self.stop_speed)  # Останавливаемся
                # Отображаем изображение для отладки
                cv2.imshow("Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    def stop(self):
        self.arduino.stop()
        self.camera.release()
        cv2.destroyAllWindows()
