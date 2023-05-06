import math
import numpy as np
import matplotlib.pyplot as plt


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class BezierCurve:
    def __init__(self, control_points):
        self.control_points = np.array([(p.x, p.y) for p in control_points])
        self.degree = len(self.control_points) - 1

    def get_position(self, t):
        '''Вычисление координат на кривой Безье'''
        n = self.degree
        b = [math.factorial(n) / (math.factorial(i) * math.factorial(n - i)) * ((1 - t) ** (n - i)) * (t ** i) for i in
             range(n + 1)]
        return np.dot(b, self.control_points)


class Tractor:
    def __init__(self, x=0, y=0, v=0, a=0):
        self.x, self.y, self.v, self.a = x, y, v, a
        self.angle, self.bezier_curve, self.previous_error = 0, None, 0

    def move(self, dt):
        '''Движение трактора вдоль кривой Безье'''
        if self.bezier_curve is None:
            return

        x, y = self.bezier_curve.get_position(self.v)
        dx, dy = x - self.x, y - self.y
        self.angle = math.atan2(dy, dx)

        self.v += self.a * dt
        dx, dy = self.v * math.cos(self.angle) * dt, self.v * math.sin(self.angle) * dt
        self.x, self.y = self.x + dx, self.y + dy

    def set_trajectory(self, trajectory):
        '''Задание траектории движения трактора через кривые Безье'''
        self.bezier_curve = BezierCurve(trajectory)

    def set_acceleration(self, p1, p2, p3):
        '''Вычисоление ускорения. Щас ускорение является постоянным и задается вручную'''
        self.a = 0.5

    def is_inside_safe_zone(self, threshold):
        '''Проверка на нахождение трактора в безопасной зоне'''
        if self.bezier_curve is None:
            return False

        x, y = self.bezier_curve.get_position(self.v)
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2) <= threshold

    def correct_direction(self, dt):
        '''Корректировка направления движения трактора, если он вышел за пределы безопасной зоны'''
        if self.bezier_curve is None:
            return

        x, y = self.bezier_curve.get_position(self.v)
        dx, dy = x - self.x, y - self.y
        target_angle = math.atan2(dy, dx)
        error = target_angle - self.angle

        if error > math.pi:
            error -= 2 * math.pi
        elif error < -math.pi:
            error += 2 * math.pi

        k_p, k_d = 0.5, 0.1 # Пропорциональную составляющую и дифференциальная составляющую регулятора
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        self.a = k_p * error + k_d * derivative


def simulate_tractor(tractor, dt=0.1):
    '''Функция для симуляции движения трактора и вывода параметров на каждом шаге'''

    x = []
    y = []
    v = []
    time = []
    a = []
    angle = []


    # Проверяем, задана ли траектория
    if tractor.bezier_curve is None:
        print("Траектория не задана")
        return

    # Выводим заголовок таблицы
    print("{:^10s} {:^10s} {:^10s} {:^10s} {:^10s} {:^10s}".format("Time", "X", "Y", "Speed", "Angle", "Acceleration"))

    t = 0
    while t <= total_time:
        # Движение трактора
        tractor.move(dt)

        # Корректировка направления движения, если трактор вышел за пределы безопасной зоны
        if not tractor.is_inside_safe_zone(threshold=10):
            tractor.correct_direction(dt)

        # Выводим параметры трактора
        print("{:10.2f} {:10.2f} {:10.2f} {:10.2f} {:10.2f} {:10.2f}".format(t, tractor.x, tractor.y, tractor.v, tractor.angle, tractor.a))

        t += dt

        x.append(tractor.x)
        y.append(tractor.y)
        v.append(tractor.v)
        a.append(tractor.a)
        time.append(t)
        angle.append(tractor.angle)

    fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(12, 8))

    axes[0][0].plot(x, y)
    axes[0][0].set_title('x(y)')
    axes[0][0].set_xlabel('x, m')
    axes[0][0].set_ylabel('y, m')

    axes[1][1].plot(time, v)
    axes[1][1].set_title('v(t)')
    axes[1][1].set_xlabel('time, s')
    axes[1][1].set_ylabel('v, m/s')

    axes[0][1].plot(time, angle)
    axes[0][1].set_title('angle(t)')
    axes[0][1].set_xlabel('time, s')
    axes[0][1].set_ylabel('angle, deg')


    axes[1][0].plot(time, a)
    axes[1][0].set_title('acceleration(t)')
    axes[1][0].set_xlabel('time, s')
    axes[1][0].set_ylabel('acc, m/s^2')


    fig.suptitle("Graphs of motion parameters", fontsize=16)
    fig.subplots_adjust(left=None,
                        bottom=None,
                        right=None,
                        top=None,
                        wspace=0.25,
                        hspace=0.3, )
    plt.show()

if __name__ == '__main__':
    # Создание объекта трактора
    tractor = Tractor()

    # Задание траектории движения трактора
    #trajectory = [Point(0, 0), Point(2, 5), Point(5, 7), Point(8, 5), Point(10, 0)]
    #trajectory = [Point(0, 0), Point(10, 5), Point(15, 10), Point(35, 3), Point(7500, -5)]
    trajectory = [Point(0, 0), Point(2, 4), Point(4, 1), Point(6, 3)]
    tractor.set_trajectory(trajectory)

    # Задание ускорения на основе трех точек пути
    tractor.set_acceleration(trajectory[0], trajectory[1], trajectory[2])

    # Запуск симуляции движения трактора
    dt = 0.1
    total_time = 3
    simulate_tractor(tractor, dt)




