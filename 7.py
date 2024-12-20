Original file line number	Diff line number	Diff line change
@@ -0,0 +1,75 @@
import cv2
import numpy as np
class StopSignDetectionCar:
    def __init__(self, arduino):
        self.arduino = arduino
        self.camera = cv2.VideoCapture(0)  # Подключение камеры
        self.default_speed = 1350
        self.stop_sign_template = cv2.imread("stop_sign_template.jpg", 0)  # Шаблон знака
    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            return None, None
        # Преобразование в HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Определение красного цвета
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        # Нахождение контуров
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return frame, contours
    def detect_stop_sign(self, frame, contours):
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Отбрасываем маленькие области
                # Аппроксимация контура
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                if len(approx) > 8:  # Проверка на круглую форму
                    x, y, w, h = cv2.boundingRect(approx)
                    roi = frame[y:y+h, x:x+w]
                    # Сравнение с шаблоном знака
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    resized_roi = cv2.resize(gray_roi, (self.stop_sign_template.shape[1], self.stop_sign_template.shape[0]))
                    result = cv2.matchTemplate(resized_roi, self.stop_sign_template, cv2.TM_CCOEFF_NORMED)
                    if np.max(result) > 0.7:  # Если совпадение достаточно высокое
                        return True
        return False
    def stop_before_sign(self):
        while True:
            frame, contours = self.process_frame()
            if frame is None:
                continue
            if self.detect_stop_sign(frame, contours):
                self.arduino.stop()
                print("Остановка перед знаком 'Движение запрещено'")
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
