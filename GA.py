from matplotlib.lines import Line2D
import numpy.random as rnd
import numpy as np 
import matplotlib.pyplot as plt
from math import sqrt
import PathVisualization as PV
from matplotlib.colors import ListedColormap
import random

# function that returns the sign of an integer --> porta in PATHVISUALIZATION
def sign(x):
    if x == 0:
        return 0
    elif x>0:
        return 1
    else:
        return -1

# function used to find the next point in the segment between 2 points in a path
def next_point(current_point,x_dist,y_dist,ratio):
    
    next_point = current_point
    x_inc = sign(x_dist)
    y_inc = sign(y_dist)
    x_dist = abs(x_dist)
    y_dist = abs(y_dist)

    if y_dist*ratio >= x_dist:
        next_point[1] += y_inc
    else:
        next_point[0] += x_inc

    return next_point

# function to check if a segment crosses an obstacle
def check_segment(start, end):
    found_obstacle = False
    current_point = np.copy(start)

    x_distance = end[0]-start[0]
    y_distance = end[1]-start[1]
    if y_distance == 0:
        ratio = 10000
    else:
        ratio = abs(float(x_distance)/float(y_distance))

    while not np.array_equal(current_point,end) and not found_obstacle:
        x_distance = end[0]-current_point[0]
        y_distance = end[1]-current_point[1]
        current_point = next_point(current_point,x_distance,y_distance,ratio)
        if map[current_point[0],current_point[1]] == 255:
            found_obstacle = True
        
    return not found_obstacle

        
#function to check if a path crosses an UAV  
def check_crosses_UAV(path,lines,start,goal,tg,path2,lines2,start2,goal2,tg2):
    tmax=tg2[-1]
    if(tg[-1]>tg2[-1]):
       tmax= tg[-1]
    for t in range(int(tmax)+1):
        if(cacalculate_distance(coordinates_over_time(path,lines,start,goal,tg,t),coordinates_over_time(path2,lines2,start2,goal2,tg2,t))<=d):
          return False
    return True

# function to get a random point between the bounds
def get_random_path(bounds):
    #return (np.round(rnd.uniform(bounds[0],bounds[1],2))).astype(int)
    path=np.zeros((num_points+1,3))
    for i in range(num_points+1):
        path[i][0]=np.random.randint(int(bounds[0]), int(bounds[1])+1)
        path[i][1]=np.random.randint(int(bounds[0]), int(bounds[1])+1)
        path[i][2]= np.random.randint(vmax/2, vmax)
    return path

# function that initializes the variables of the problem
# f = fitting function
# bounds = bounds of the search space
# n = number of particles
# map = obstcales map of the environment
# map_dim = length of the map border
# num_points = number of points forming a path
def initialization(f,bounds,bounds2,n, map, map_dim, num_points):
    paths = np.zeros((n,num_points+1,3))
    paths = paths.tolist()
    particle_val = paths[:]
    
    paths2 = np.zeros((n,num_points+1,3))
    paths2 = paths2.tolist()
    
    global_best2 = []
    global_best = []
    global_best_val = map_dim * num_points*102
    isTrue=True
    for p in range(n):  # generate random points for a path

        paths[p] = get_random_path(bounds)
        paths[p][0][0]=start[0]
        paths[p][0][1]=start[1]
        paths[p] = [i.astype(int) for i in paths[p]]
        i = 1
        while i < num_points :
            num_tries = 0
            while (not check_segment(np.array([paths[p][i-1][0],paths[p][i-1][1]]),np.array([paths[p][i][0],paths[p][i][1]])) or map[paths[p][i][0],paths[p][i][1]] == 255) and num_tries<20:
                num_tries += 1
                paths[p][i][0] = np.random.randint(int(bounds[0]), int(bounds[1])+1)
                paths[p][i][1] = np.random.randint(int(bounds[0]), int(bounds[1])+1)
            i += 1
        num_tries = 0
        while (not check_segment(np.array([paths[p][-1][0],paths[p][-1][1]]),np.array([goal[0],goal[1]])) or \
            not check_segment(np.array([paths[p][-2][0],paths[p][-2][1]]),np.array([paths[p][-1][0],paths[p][-1][1]])) or map[paths[p][-1][0],paths[p][-1][1]] == 255) and num_tries<20:
            num_tries += 1
            paths[p][-1][0] = np.random.randint(int(bounds[0]), int(bounds[1])+1)
            paths[p][-1][1] = np.random.randint(int(bounds[0]), int(bounds[1])+1)
        

        paths2[p] = get_random_path(bounds2)
        paths2[p][0][0]=start2[0]
        paths2[p][0][1]=start2[1]
        paths2[p] = [i.astype(int) for i in paths2[p]]

        i=1
        while i < num_points:
            num_tries = 0
            while (not check_segment(np.array([paths2[p][i-1][0],paths2[p][i-1][1]]),np.array([paths2[p][i][0],paths2[p][i][1]])) or map[paths2[p][i][0],paths2[p][i][1]] == 255) and num_tries<20:
                num_tries += 1
                paths2[p][i][0] = np.random.randint(int(bounds[0]), int(bounds[1])+1)
                paths2[p][i][1] = np.random.randint(int(bounds[0]), int(bounds[1])+1)
            i+=1
        num_tries = 0
        while (not check_segment(np.array([paths2[p][-1][0],paths2[p][-1][1]]),np.array([goal2[0],goal2[1]])) or \
            not check_segment(np.array([paths2[p][-2][0],paths2[p][-2][1]]),np.array([paths2[p][-1][0],paths2[p][-1][1]])) or map[paths2[p][-1][0],paths2[p][-1][1]] == 255) and num_tries<20:
            num_tries += 1
            paths2[p][-1][0] = np.random.randint(int(bounds[0]), int(bounds[1])+1)
            paths2[p][-1][1] = np.random.randint(int(bounds[0]), int(bounds[1])+1)
        particle_val[p] = f([paths[p],paths2[p]])
        if(particle_val[p] < global_best_val):
            global_best = paths[p]
            global_best2 = paths2[p]
            global_best_val = f([global_best,global_best2])
    return np.array(paths), np.array(particle_val), global_best,np.array(paths2), global_best2


#Genetic Algorithm
def genetic_algorithm(f, bounds, bounds2, n, crossover_rate, mutation_rate, tol, map, num_iterations, map_dim, num_points, goal):
    paths, particle_val,global_best, paths2, global_best2 = initialization(f, bounds,bounds2, n, map, map_dim, num_points)
    # Khởi tạo cho GA
    population = np.zeros((n,2,num_points+1,3))
    for i in range(n):
        population[i]=np.array([[i.astype(int) for i in paths[i]],[i.astype(int) for i in paths2[i]]]).astype(int)
    # print(np.array(population).astype(int))
    population=np.array(population).astype(int)
    fitness_values = particle_val
    old_global_best_val = map_dim * num_points*2
    global_best_val = f([global_best,global_best2])
    iteration = 1
    iterations_no_improv = 1
    found_new_global_best = False
    local_best_val=[]

    plt.ion()
    fig, ax = plt.subplots()
    cmap = ListedColormap(["white", "red"])
    ax.matshow(map, cmap=cmap)
    l = list()
    for i in range(n):
        l.append(ax.plot([], [])[0])

    # Khởi tạo cho GA
    # for _ in range(n):
    #     individual = generate_individual(bounds, bounds2, num_points, goal)
    #     population.append(individual)
    #     population2.append(individual[:])

    # Vòng lặp của GA
    while iteration < num_iterations and iterations_no_improv < 50:
        
        print("Vòng lặp thứ " + str(iteration) + "      độ dài đường đi tốt nhất hiện tại = " + str(global_best_val))

        found_new_global_best = False

        # Các phép chọn lọc, lai ghép và đột biến dành riêng cho GA
        offspring = np.zeros((n,2,num_points+1,3))
        for i in range(n):
            parent1 = tournament_selection(population, fitness_values)
            parent2 = tournament_selection(population, fitness_values)
            child = crossover(parent1, parent2, crossover_rate)
            child = mutate(child, bounds, mutation_rate)
            offspring[i]=child
        # Đánh giá độ thích nghi của cá thể con
        offspring=np.array(offspring).astype(int)
        offspring_fitness_values = [f(individual) for individual in offspring]
        fitness_values=[f(individual) for individual in population]
        # Tiến hoa quânf thể của GA
        population, fitness_values = elitism(population, fitness_values, offspring, offspring_fitness_values)
        iteration += 1

        # Cập nhật đường đi tốt nhất toàn cầu
        best_index = np.argmin(fitness_values)
        print(best_index)
        if fitness_values[best_index] < global_best_val:
            found_new_global_best = True
            old_global_best_val = global_best_val
        global_best = population[best_index][0]
        global_best2 = population[best_index][1]
        global_best_val = fitness_values[best_index]

        # print([global_best,global_best2])
        # print(f([global_best,global_best2]))
        # print(population[best_index])
        # print(f(population[best_index]))


        if found_new_global_best:
            iterations_no_improv = 1
        else:
            iterations_no_improv += 1

        for i in range(n):
            paths[i]=population[i][0]
        # plot the paths
        for i in range(n):
            path = np.array(paths[i])
            x = [point[0] for point in path]
            y = [point[1] for point in path]
            x = np.append(np.array(start[0]),np.append(x,goal[0]))
            y = np.append(np.array(start[1]),np.append(y,goal[1]))
            l[i].set_xdata(y)
            l[i].set_ydata(x)
        # plot the paths
        plt.pause(0.1)
        plt.draw()
        plt.savefig('./images/animation/iteration'+str(iteration-1)+'.png')
    # plt.ioff()
    # plt.show()
    plt.figure()
    print("Tìm kiếm hoàn thành!")
    print("Độ dài đường đi tối ưu: " + str(global_best_val))
    print("Đường đi tối ưu UAV1: " + str(global_best))
    print("Đường đi tối ưu UAV2: " + str(global_best2))
    cmap = ListedColormap(["white","red"])
    PV.show_path(global_best,map,map_dimension,cmap,goal)
    PV.show_path2(global_best2,map,map_dimension,cmap,goal2)
    plt.savefig('./images/final_path.png')
    PV.create_gif(iteration)
    plt.show(block=True)
    return global_best,global_best2, global_best_val, iteration

def tournament_selection(population, fitness_values):
    # Lựa chọn 2 cá thể từ quần thể bằng phương pháp giải đấu
    # Bạn có thể sửa đổi phương pháp lựa chọn này tùy thuộc vào yêu cầu của bài toán
    tournament_size = 3
    tournament_indices = random.sample(range(len(population)), tournament_size)
    tournament_fitness_values = [fitness_values[i] for i in tournament_indices]
    winner_index = tournament_indices[np.argmin(tournament_fitness_values)]
    return population[winner_index]

def crossover(parent1, parent2, crossover_rate):
    # Lai ghép 2 cá thể cha mẹ để tạo ra cá thể con
    # Bạn có thể sửa đổi phương pháp lai ghép này tùy thuộc vào yêu cầu của bài toán
    if random.random() < crossover_rate:
        child = np.zeros((2,num_points+1,3))
        crossover_point = random.randint(1, num_points)
        child[0][:crossover_point] = parent1[0][:crossover_point]
        child[0][crossover_point:] = parent2[0][crossover_point:]
        crossover_point = random.randint(1, num_points)
        child[1][:crossover_point] = parent1[1][:crossover_point]
        child[1][crossover_point:] = parent2[1][crossover_point:]
        return child
    else:
        if random.random() < 0.5:
            return parent1
        else:
            return parent2

def mutate(individual, bounds, mutation_rate):
    # Đột biến cá thể
    # Bạn có thể sửa đổi phương pháp đột biến này tùy thuộc vào yêu cầu của bài toán
    #if random.random() < mutation_rate:
    for i in range(len(individual[0])):
        if random.random() < mutation_rate:
            if i!=0:
                individual[0][i] = [np.random.randint(int(bounds[0]),int(bounds[1])+1), np.random.randint(int(bounds[0]), int(bounds[1])+1),np.random.randint(vmax/2, vmax+1)]
            else:
                individual[0][i] = [individual[0][i][0], individual[0][i][1],np.random.randint(vmax/2, vmax+1)]
    #if random.random() < mutation_rate:
    for i in range(len(individual[1])):
        if random.random() < mutation_rate:
            if i!=0:
                individual[1][i] = [np.random.randint(int(bounds[0]),int(bounds[1])+1), np.random.randint(int(bounds[0]), int(bounds[1])+1),np.random.randint(vmax/2, vmax+1)]
            else:
                individual[1][i] = [individual[1][i][0], individual[1][i][1],np.random.randint(vmax/2, vmax+1)]    
    return individual 

def elitism(population, fitness_values, offspring, offspring_fitness_values):
    # Giữ lại các cá thể tốt nhất từ quần thể gốc và con cái
    # Bạn có thể sửa đổi phương pháp này tùy thuộc vào yêu cầu của bài toán
    combined_population = list(population)+list(offspring)
    combined_fitness_values = list(fitness_values) + list(offspring_fitness_values)
    sorted_indices = np.argsort(combined_fitness_values)
    new_population = [combined_population[i] for i in sorted_indices[:len(population)]]
    new_fitness_values = [combined_fitness_values[i] for i in sorted_indices[:len(population)]]
    return np.array(new_population).astype(int), new_fitness_values


def evaluate_fitness(paths):
    line = np.zeros((num_points+1,2))
    tg=np.zeros((num_points+1))
    line2 = np.zeros((num_points+1,2))
    tg2=np.zeros((num_points+1))
    for i in range(num_points+1):
        if(i==0):
            distance = sqrt((paths[0][i][0]-start[0])**2 + ((paths[0][i][1]-start[1])**2))
            line[i]=calculate_slope_intercept(start,paths[0][i])
            tg[i]=distance/paths[0][i][2]
            continue
        if(i==num_points):
            distance = sqrt((goal[0]-paths[0][-1][0])**2 + ((goal[1]-paths[0][-1][1])**2))
            line[i]=calculate_slope_intercept(paths[0][-1],goal)
            tg[i]=distance/paths[0][i][2]+tg[-2]
        if(i<num_points and i>0):
            distance = sqrt((paths[0][i][0]-paths[0][i-1][0])**2 + (paths[0][i][1]-paths[0][i-1][1])**2)
            line[i]=calculate_slope_intercept(paths[0][i-1],paths[0][i])
            tg[i]=distance/paths[0][i][2]+tg[i-1]
    for i in range(num_points+1):
        if(i==0):
            distance = sqrt((paths[1][i][0]-start2[0])**2 + ((paths[1][i][1]-start2[1])**2))
            line2[i]=calculate_slope_intercept(start,paths[1][i])
            tg2[i]=distance/paths[1][i][2]
            continue
        if(i==num_points):
            distance = sqrt((goal2[0]-paths[1][-1][0])**2 + ((goal2[1]-paths[1][-1][1])**2))
            line2[i]=calculate_slope_intercept(paths[1][-1],goal2)
            tg2[i]=distance/paths[1][i][2]+tg2[-2]
        if(i<num_points and i>0):
            distance = sqrt((paths[1][i][0]-paths[1][i-1][0])**2 + (paths[1][i][1]-paths[1][i-1][1])**2)
            line2[i]=calculate_slope_intercept(paths[1][i-1],paths[1][i])
            tg2[i]=distance/paths[1][i][2]+tg2[i-1]
    isCollided1=0
    isCollided2=0
    i=1
    while i <= num_points:
        if (not check_segment(np.array([paths[0][i-1][0],paths[0][i-1][1]]),np.array([paths[0][i][0],paths[0][i][1]])) or map[paths[0][i][0],paths[0][i][1]] == 255):
            isCollided1+=1
        i += 1
    if (not check_segment(np.array([paths[0][-1][0],paths[0][-1][1]]),np.array([goal[0],goal[1]])) or map[paths[0][-1][0],paths[0][-1][1]] == 255):
        isCollided1+=1
    i=1
    while i <= num_points:
        if (not check_segment(np.array([paths[1][i-1][0],paths[1][i-1][1]]),np.array([paths[1][i][0],paths[1][i][1]])) or map[paths[1][i][0],paths[1][i][1]] == 255):
            isCollided2+=1
        i += 1
    if (not check_segment(np.array([paths[1][-1][0],paths[1][-1][1]]),np.array([goal2[0],goal2[1]])) or map[paths[1][-1][0],paths[1][-1][1]] == 255):
        isCollided2+=1  

    if (not check_crosses_UAV(paths[0],line,start,goal,tg,paths[1],line2,start2,goal2,tg2)):
        isCollided1+=1
        isCollided2+=1

    x=paths[0]
    y=paths[1]
    total_time1 = 0
    total_time2 =0
    distance=0
    total_distance = 0
    distance=sqrt((x[1][0]-x[0][0])**2 + (x[1][1]-x[0][1])**2)
    total_distance+=distance
    total_time1 += distance/x[0][2]
    distance=sqrt((goal[0]-x[-1][0])**2 + (goal[1]-x[-1][1])**2)
    total_distance+=distance
    total_time1 += distance/x[-1][2]
    distance=sqrt((y[1][0]-y[0][0])**2 + (y[1][1]-y[0][1])**2)
    total_distance+=distance
    total_time2 += distance/y[0][2]
    distance = sqrt((goal2[0]-x[-1][0])**2 + (goal2[1]-y[-1][1])**2)
    total_distance+=distance
    total_time2 += distance/y[-1][2]
    for i in range(len(x)):
        if(i>1):
            distance=sqrt((x[i][0]-x[i-1][0])**2 + (x[i][1]-x[i-1][1])**2)
            total_distance+=distance
            total_time1 += distance/x[i-1][2]
            distance=sqrt((y[i][0]-y[i-1][0])**2 + (y[i][1]-y[i-1][1])**2)
            total_distance+=distance
            total_time2 += distance/y[i-1][2]
    return total_time1+total_time2+ (total_time1*isCollided1+total_time2*isCollided2)*5

#calculate Calculate coordinates over time
def coordinates_over_time(path,lines,start,goal,tg,t):
    x=start[0]
    y=start[1]
    if t>0 and t<=tg[0]:
        x=(t/tg[0])*path[0][0]+x
        y=x*lines[0][0]+lines[0][1]
        return x, y
    if t>tg[-2] and t<tg[-1]:
        x=((t-tg[-2])/(tg[-1]-tg[-2]))*(goal[0]-path[-1][0])+path[-1][0]
        y= x*lines[-1][0]+lines[-1][1] 
        return x, y
    if t>=tg[-1]:
        return goal[0], goal[1]
    for i in range(len(path)-2):
        if t>tg[i]and t<=tg[i+1]:
            x=((t-tg[i])/(tg[i+1]-tg[i]))*(path[i+1][0]-path[i][0])+path[i][0]
            y= x*lines[i+1][0]+lines[i+1][1] 
            return x, y
    return x, y

#calculate_slope_intercept
def calculate_slope_intercept(point1, point2):
    if point2[0] - point1[0] != 0:
        m = (point2[1] - point1[1]) / (point2[0] - point1[0])
        c = point1[1] - m * point1[0]
    else:
        # Xử lý khi đường thẳng song song với trục y
        m = np.inf  # Đặt giá trị hệ số gấp đôi vô cùng
        c = point1[1]  # Đặt giá trị hệ số là NaN (không hợp lệ)
        return m, c

#distance between 2 point
def cacalculate_distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)


import shutil, os
shutil.rmtree('./images/animation')
os.mkdir('./images/animation')

# Sử dụng GA
map_dimension = 100
num_points = 5
bounds = [0, map_dimension - 1]
bounds2 = [0, map_dimension - 1]
n = 100  # Kích thước quần thể
crossover_rate = 0.8
mutation_rate = 0.1
tol = 1e-12  # Độ chính xác mong muốn
num_iterations = 300



num_obstacles = 18
d=4
vmax=10
map = PV.generate_map(num_obstacles,map_dimension)
start2 = (0,0)
goal2 = (map_dimension-1,map_dimension-1)
start = (5,0)
goal = (map_dimension-1,map_dimension-5)
dimension_bounds=[0,map_dimension-1]

# Thực hiện GA
genetic_algorithm(evaluate_fitness, bounds, bounds2, n, crossover_rate, mutation_rate, tol, map, num_iterations, map_dimension, num_points, goal)
