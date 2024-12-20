import cv2
import numpy as np

class Arduino:
    def set_speed(self, speed):
        print(f"Установлена скорость: {speed}")
    
    def set_angle(self, angle):
        print(f"Установлен угол поворота: {angle}")
    
    def stop(self):
        print("Автомобиль остановлен")

# Инициализация Arduino
arduino = Arduino()

# Константы
CAR_SPEED = 1300  # Скорость движения
RED_THRESHOLD = 170000  # Порог для красного цвета
GREEN_THRESHOLD = 170000  # Порог для зеленого цвета
LINE_THRESHOLD = 50  # Порог для обнаружения линии
ANGLE_CORRECTION = 10  # Коррекция угла

# Функция для обнаружения разметки
def detect_line(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, LINE_THRESHOLD)

    if lines is not None:
        for rho, theta in lines[0]:
            angle = np.rad2deg(theta)  # Преобразуем угол в градусы
            offset = angle - 90  # Отклонение от прямого движения
            return offset
    return 0  # Если линия не найдена

# Функция для обнаружения светофора
def detect_traffic_light(image):
    height, width, _ = image.shape

    # Разделяем изображение на три сектора (верхний, средний, нижний)
    top = image[0:height//3, 2*width//3:]
    middle = image[height//3:2*height//3, 2*width//3:]
    bottom = image[2*height//3:, 2*width//3:]

    # Определяем цвет в каждом секторе
    if np.sum(top[:, :, 2]) > RED_THRESHOLD:  # Красный цвет
        return "red"
    elif np.sum(bottom[:, :, 1]) > GREEN_THRESHOLD:  # Зеленый цвет
        return "green"
    else:
        return "none"

# Основной цикл
def main():
    cap = cv2.VideoCapture(0)  # Открываем камеру

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Обнаруживаем линию разметки
        line_offset = detect_line(frame)
        if line_offset < -10:
            arduino.set_angle(90 - ANGLE_CORRECTION)  # Поворот вправо
        elif line_offset > 10:
            arduino.set_angle(90 + ANGLE_CORRECTION)  # Поворот влево
        else:
            arduino.set_angle(90)  # Прямое движение

        # Проверяем светофор
        light = detect_traffic_light(frame)
        if light == "red":
            arduino.stop()
        elif light == "green":
            arduino.set_speed(CAR_SPEED)

        # Отображаем изображение для отладки
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
