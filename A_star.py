from tkinter import *
import enum
import numpy as np
import heapq
import time
import _thread


class PointState(enum.Enum):
    # 障碍物
    BARRIER = 'black'
    # 未使用
    UNUSED = 'white'
    # 已经加入过openlist的方格
    TRAVERSED = 'blue'
    # 路径
    PATH = 'red'


class AstarMap:
    class Point:
        def __init__(self, x, y, f, g, father, state, rectangle):
            # x坐标
            self.x = x
            # y坐标
            self.y = y
            # f = g + h, h为预估代价，这里使用欧式
            self.f = f
            # 从寻路起点到这个点的代价
            self.g = g
            # 父节点
            self.father = father
            # 当前点状态
            self.state = state
            # 当前点对应画布上的矩形
            self.rectangle = rectangle

        # 重写比较，方便堆排序
        def __lt__(self, other):
            if self.f < other.f:
                return True
            else:
                return False

    def __init__(self, *args):
        # 行数
        self.row = args[0]
        # 列数
        self.col = args[1]
        # 方格尺寸
        self.size = args[2]
        # 起点
        self.start = args[3]
        # 终点
        self.end = args[4]
        # 每次绘制的延迟时间
        self.delay = args[5]

        self.root = Tk()
        self.root.title("navigation")
        self.canva = Canvas(self.root, width=self.col * self.size + 3, height=self.row * self.size + 3)
        # 生成方格集合
        self.points = self.init_points()
        # 生成网格
        self.init_mesh()

        self.canva.bind("<Button-1>", self.init_barrier)
        self.canva.bind("<Button-2>", self.clear_map)
        self.canva.bind("<Button-3>", self.navigation)

        self.canva.pack(side=TOP, expand=YES, fill=BOTH)
        self.root.resizable(0, 0)
        self.root.mainloop()

    # 初始化画布与方格状态
    def init_points(self):
        points = [[self.Point(x, y, 0, 0, None, PointState.UNUSED.value,
                              self.canva.create_rectangle((x * self.size + 3, y * self.size + 3),
                                                          ((x + 1) * self.size + 3, (y + 1) * self.size + 3),
                                                          fill=PointState.UNUSED.value)) \
                   for y in range(self.row)] for x in range(self.col)]
        return points

    def init_mesh(self):
        for i in range(self.row + 1):
            self.canva.create_line((3, i * self.size + 3), (self.col * self.size + 3, i * self.size + 3))
        for i in range(self.col + 1):
            self.canva.create_line((i * self.size + 3, 3), (i * self.size + 3, self.row * self.size + 3))

    # 鼠标左键点击设置障碍
    def init_barrier(self, event):
        x = int((event.x + 3) / self.size)
        y = int((event.y + 3) / self.size)
        if x < self.col and y < self.row:
            if self.points[x][y].state == PointState.BARRIER.value:
                self.points[x][y].state = PointState.UNUSED.value
                self.canva.itemconfig(self.points[x][y].rectangle, fill=self.points[x][y].state)
            else:
                self.points[x][y].state = PointState.BARRIER.value
                self.canva.itemconfig(self.points[x][y].rectangle, fill=self.points[x][y].state)

    # 清空当前画布
    def clear_map(self, event):
        for i in range(self.col):
            for j in range(self.row):
                if (self.points[i][j].state != PointState.BARRIER.value):
                    self.points[i][j].state = PointState.UNUSED.value
                    self.canva.itemconfig(self.points[i][j].rectangle, fill=self.points[i][j].state)

    def navigation(self, event):
        _thread.start_new_thread(self.find_path, (self.start, self.end))

    # 寻路
    def find_path(self, start, end):
        x1 = start[0]
        y1 = start[1]
        x2 = end[0]
        y2 = end[1]

        # 用最小堆存点，使每次取出的点都预估代价最小
        openlist = []
        # 两个set用于查找，存储坐标的二元组
        closeset = set()
        openset = set()
        # 将起点加入openlist,每格距离设置为10，是为了使斜着走时距离设置为14，方便计算
        heapq.heappush(openlist, self.points[x1][y1])
        openset.add((x1, y1))
        # 寻路循环
        while 1:
            # 从openlist中取出代价最小点
            p_min = heapq.heappop(openlist)
            openset.remove((p_min.x, p_min.y))
            # 将这个点放入closelist中
            closeset.add((p_min.x, p_min.y))
            # 遍历八个方向

            for i in range(-1, 2):
                for j in range(-1, 2):
                    if i == 0 and j == 0:
                        continue
                    # 当前要判断的点的坐标
                    x_new = p_min.x + i
                    y_new = p_min.y + j
                    # 如果点越过画布边界则continue
                    if x_new >= self.col or x_new < 0 or y_new >= self.row or y_new < 0:
                        continue
                    # 更新寻找点坐标
                    p_new = self.points[x_new][y_new]
                    # 判断该点是否为四角
                    oblique = i != 0 and j != 0
                    # 该点不在close表中，且不是障碍点
                    if (x_new, y_new) not in closeset and self.points[x_new][y_new].state != PointState.BARRIER.value:
                        # 在open表中
                        if (x_new, y_new) in openset:
                            # 如果通过起点到p_min再到p_new的代价比起点到p_new的代价小，则更新p_new的代价，将p_min设置为p_new的父节点
                            if ((14 if oblique else 10) + p_min.g) < p_new.g:
                                # 如果在角落，则pMin到pNew的代价为14，否则为10
                                p_new.g = p_min.g + (14 if oblique else 10)
                                # p_new.f = p_new.g + 10 * (abs(x2 - x_new) + abs(y2 - y_new)) # 曼哈顿距离
                                p_new.f = p_new.g + 10 * round(np.linalg.norm(np.array([x2, y2]) - np.array([x_new, y_new])))  # 欧式距离
                                p_new.father = p_min
                        # 如果不在open表中，则代表这个点第一次被访问，直接将p_min设置为p_new的父节点
                        else:
                            p_new.g = p_min.g + (14 if oblique else 10)
                            # p_new.f = p_new.g + 10 * (abs(x2 - x_new) + abs(y2 - y_new)) # 曼哈顿距离
                            p_new.f = p_new.g + 10 * round(np.linalg.norm(np.array([x2, y2]) - np.array([x_new, y_new])))  # 欧式距离
                            p_new.father = p_min
                            p_new.state = PointState.TRAVERSED.value
                            self.canva.itemconfig(p_new.rectangle, fill=PointState.TRAVERSED.value)
                            # 将这个点加入openlist
                            heapq.heappush(openlist, p_new)
                            openset.add((x_new, y_new))
            # 检测是否寻路完成
            if (x2, y2) in openset:
                p_next = self.points[x2][y2]
                p_next.state = PointState.PATH.value
                self.canva.itemconfig(p_next.rectangle, fill=PointState.PATH.value)
                while p_next.father:
                    p_next = p_next.father
                    self.points[p_next.x][p_next.y].state = PointState.PATH.value
                    self.canva.itemconfig(self.points[p_next.x][p_next.y].rectangle, fill=PointState.PATH.value)
                break
            # 如果寻路不完成但openlist长度为0，则没有可达路径
            if len(openlist) == 0:
                print('No path!')
                break
            # 等待绘制
            time.sleep(self.delay)


if __name__ == "__main__":
    # 参数为行数，列数，方格尺寸,起点坐标，终点坐标，延迟时间
    star = AstarMap(15, 15, 30, (5, 12), (10, 2), 0.02)
