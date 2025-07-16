
class mocapUnit():
    def __init__(self, threshold, dim, ratio) -> None:
        self.threshold = threshold
        self.pre_val = [0 for _ in range(dim)]
        self.val = [0 for _ in range(dim)]
        self.ratio = ratio

    def move(self, val_list):
        for i in range(len(val_list)):
            val = self.pre_val[i] * (1-self.ratio) + val_list[i] * self.ratio
            val = max(val, self.threshold[i][0])
            val = min(val, self.threshold[i][1])

            self.val[i] = val
            self.pre_val[i] = val
    


class btnCtrlUnit():
    def __init__(self, speed, limit, val) -> None:
        self.speed = speed
        self.limit = limit
        self.val = val

    def move(self, direction):
        self.val += direction*self.speed
        self.val = max(self.val, self.limit[0])
        self.val = min(self.val, self.limit[1])
        return self.val
