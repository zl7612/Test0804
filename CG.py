import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time

start = time.time()
# 初始化参数
W = 16 # 大卷长度
lengths = [3,6,7] #小卷长度
demands = [25,20,18] #小卷需求
M = len(lengths) #小卷种类

# W = 222 # 大卷长度
# lengths = [15,20,26,34,47,58,69,73,86,97,153,162,178] #小卷长度
# demands = [44,3,48,45,52,62,74,95,63,152,48,75,69] #小卷需求
# M = len(lengths) #小卷种类

# 传统解法
N = sum(demands) # 可供使用的卷数量，最大化
model = gp.Model("Traditional Method")
y = model.addVars(N, vtype=GRB.BINARY, name = 'y') # y:0-1变量，y[i]被切割为1，反之为0
x = model.addVars(N, M, vtype=GRB.INTEGER, name ='x') # x:N*M个整数变量，N行M列
for j in range(M): # 对小卷种类遍历
    model.addConstr(gp.quicksum(x[i,j] for i in range(N)) >= demands[j])
for i in range(N): # 对最大卷数遍历
    model.addConstr(gp.quicksum(lengths[j] * x[i,j] for j in range(M)) <= W*y[i])
# 目标
model.setObjective(gp.quicksum(y[i] for i in range(N)))
# 求解
model.optimize() # 默认最小化
end1 = time.time()
Traditional_Time = end1 - start
print(end1-start)
print('------------------------------------------------\n\n')

# 列生成
class Masterproblem:
    def __init__(self, lengths, demands, W):
        self.M = len(lengths)
        self.lengths = lengths
        self.demands = demands
        self.W = W
        self.x_index = 0

    def Create_Model(self):
        self.x = []
        self.model = gp.Model("Masterproblem")
        for i in range(self.M):
            self.x.append(self.model.addVar(obj = 1, lb = 0, ub = GRB.INFINITY, vtype = GRB.CONTINUOUS, name="x"+str(i)))
        self.x_index = self.M
        self.constrs = self.model.addConstrs((self.x[i] * (self.W // self.lengths[i]) >= self.demands[i]) for i in range(self.M))

    def Dual_vars(self): #对偶变量
        return [self.constrs[i].getAttr(GRB.Attr.Pi) for i in range(len(self.constrs))]

    def solve(self, flag = 0):
        # self.model.Params.OutputFlag = flag
        self.model.optimize()

    def update_constraints(self, column_coeff):
        self.column = gp.Column(column_coeff, self.model.getConstrs())
        self.model.addVar(obj = 1, lb = 0, vtype = GRB.CONTINUOUS, name="x" + str(self.x_index), column = self.column)
        self.x_index += 1

    def print_status(self,m):
        print("-----------------------------------\n",
              str(m)+"th","master objective value: ",self.model.ObjVal,
              "\n-----------------------------------")

    def to_int(self):
        for x in self.model.getVars():
            x.setAttr("VType", GRB.INTEGER)

    def write(self,m):
        self.model.write("model"+str(m)+".lp")

class Subproblem:
    def __init__(self, lengths, W):
        self.lengths = lengths
        self.W = W
        self.M = len(lengths)

    def Create_Model(self):
        self.model = gp.Model("Subproblem")
        self.y = self.model.addVars(self.M, lb = 0, ub = GRB.INFINITY, vtype = GRB.INTEGER, name = "y")
        self.model.addConstr((gp.quicksum(self.y[i] * self.lengths[i] for i in range(self.M)) <= self.W))
        """
        81*y1 + 70*y2 + 68*y3 <= 218
        """

    def Objective(self,pi):
        self.model.setObjective(gp.quicksum(pi[i] * self.y[i] for i in range(self.M)), sense = GRB.MAXIMIZE)

    def solve(self, flag=0):
        # self.model.Params.OutputFlag = flag
        self.model.optimize()

    def get_solution(self):
        return [self.model.getVars()[i].x for i in range(self.M)]

    def get_reduced_cost(self):
        return self.model.ObjVal

    def write(self,m):
        self.model.write("sub_model" + str(m) + ".lp")

interation = 1000 #最大迭代次数
cutting = Masterproblem(lengths, demands, W) #初始化主问题
# Masterproblem = gp.Model("Masterproblem")
cutting.Create_Model() #建立主问题的模型
sub_prob = Subproblem(lengths, W) #初始化子问题
sub_prob.Create_Model() #建立子问题模型

for i in range(interation):
    cutting.solve() # 求解主问题
    pi = cutting.Dual_vars() # 得到主问题的对偶变量
    cutting.write(i)
    cutting.print_status(i)

    sub_prob.Objective(pi) # 给子问题目标函数赋值
    sub_prob.solve() # 求解子问题
    y = sub_prob.get_solution() # 子问题最优解
    reduced_cost = sub_prob.get_reduced_cost() # 子问题reduced cost
    sub_prob.write(i)
    cutting.update_constraints(column_coeff=y)
    if reduced_cost <= 1: # 1-X <= 0继续，x小于1，退出
       break
cutting.to_int()
cutting.solve()

end2 = time.time()
CG_Time = end2 - end1
print(end2-end1)

print("传统解法时间：" , Traditional_Time)
print("列生成时间：" , CG_Time)