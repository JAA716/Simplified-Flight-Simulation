import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import direct

# Threat information
threats = [(6, 5, 3), (10, 15, 2), (14, 11, 1), (22, 5, 4), (22, 13, 2),
           (29, 11, 2), (28, 17, 3), (32, 17, 1), (35, 5, 3), (34, 10, 4)]

# Define start and end points for Mission 1
start_point = [3, 12]
end_point = [40, 13]

# Penalty parameters

# Initial penalty parameter for threat violation
rho_i = 0.01
# Exponent for threat violation penalty term
penalty_term_p = 3
penalty_term_v = 0.0001
penalty_term_u = 0.01

#Compare to reduce the cost in part 2 and 3
angleMax = 31
lmin = 1
total_func_evals = 0

def distance_between_points(point1, point2):
    if isinstance(point1, (tuple, list)) and isinstance(point2, (tuple, list)):
        x1, y1 = point1
        x2, y2 = point2
    else:
        x1, y1 = point1, point1
        x2, y2 = point2, point2
    dx, dy = x2 - x1, y2 - y1
    distance = math.sqrt(dx**2 + dy**2)
    return distance

def check_inside_threat(x, y, cx, cy, r):
    point = [x, y]
    center = [cx, cy]
    distance_from_center = distance_between_points(point, center)
    return distance_from_center - r

# Objective function calculation
def calculate_objective(waypoints):
    global total_func_evals
    cost = 0
    reduce_angle = 0
    reduce_length = 0
    for i in range(len(waypoints) - 1):
        point1 = waypoints[i]
        point2 = waypoints[i + 1]
        lj = distance_between_points(point1, point2)
        
        # Part 2 of the Objective Function
        for a in range(len(waypoints) - 1):
            newPoint1 = waypoints[a]
            newPoint2 = waypoints[a+1]
            ljNew = distance_between_points(newPoint1, newPoint2)
            angleJ = math.acos((lj * ljNew) / (abs(lj) * abs(ljNew)))
            angles = (angleMax - angleJ)
            if angles > 0:
                angles = 0
            reduce_angle = reduce_angle + penalty_term_v * (angles ** 2)
        
        # Part 3 of the Objective Function
        penalty_length = (lj - lmin)
        if penalty_length > 0:
            penalty_length = 0
        reduce_length = reduce_length + penalty_term_u * (penalty_length ** 2)
        
        # Part 1 of the Objective Function
        lji_sum = 0
        for threat in threats:
            cx, cy, r = threat
            # Maximum step size for sampling
            sigma_max = 1  
            # Number of sampling points
            kmax = 1 + (-(-lj // sigma_max))  
            # Sampling step size
            delta_lambda = 1 / kmax  
            lambda_b = 0
            lambda_e = 0
            ukx_prev = point1[0] if isinstance(point1, (list, tuple)) else point1
            uky_prev = point1[1] if isinstance(point1, (list, tuple)) else point1
            for k in range(int(kmax)):
                delta_k = k * delta_lambda
                ukx = ukx_prev + delta_k * (point2[0] - ukx_prev) if isinstance(point2, (list, tuple)) else ukx_prev + delta_k * (point2 - ukx_prev)
                uky = uky_prev + delta_k * (point2[1] - uky_prev) if isinstance(point2, (list, tuple)) else uky_prev + delta_k * (point2 - uky_prev)
                result_cur = check_inside_threat(ukx, uky, cx, cy, r)
                result_prev = check_inside_threat(ukx_prev, uky_prev, cx, cy, r)
                if k == 0 and result_cur <= 0:
                    lambda_b = 0
                elif k > 0 and result_cur <= 0 and result_prev > 0:
                    kappa = result_cur / (result_cur - result_prev)
                    lambda_b = (k - kappa) * delta_lambda
                elif k > 0 and result_cur > 0 and result_prev <= 0:
                    kappa = result_cur / (result_cur - result_prev)
                    lambda_e = (k - kappa) * delta_lambda
                    lji = (lambda_e - lambda_b) * lj
                    lji_sum += lji
                elif k == kmax - 1 and result_cur <= 0:
                    lji = (1 - lambda_b) * lj
                    lji_sum += lji
                ukx_prev = ukx
                uky_prev = uky
        cost += lj + rho_i * (lji_sum ** penalty_term_p) + reduce_angle + reduce_length
    total_func_evals += 1
    return cost

def direct_optimization(objective_function, start_point, end_point, max_iter=None):
    lower_bounds = [start_point[0], 0]
    upper_bounds = [end_point[0], 20]
    num_vars = len(lower_bounds)
    bounds = list(zip(lower_bounds, upper_bounds))

    def scipy_objective(x):
        waypoints = [[x[i], x[i+1]] for i in range(0, len(x), 2)]
        waypoints.insert(0, start_point)
        waypoints.append(end_point)
        return objective_function(waypoints)

    if max_iter is None:
        result = direct(scipy_objective, bounds)
    else:
        result = direct(scipy_objective, bounds, maxiter=max_iter)

    optimal_x = result.x
    optimal_waypoints = [[optimal_x[i], optimal_x[i+1]] for i in range(0, len(optimal_x), 2)]
    optimal_cost = result.fun

    print(f"Final optimal cost value found: {optimal_cost}")
    return optimal_waypoints

#Missions
print(f"------------------------------ Mission 1 ------------------------------")
#1
start_point = [3, 12]
end_point = [40, 13]
initial_guess = [(11, 18), (17, 18), (23, 18), (29, 18), (35, 18)]

optimal_route = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
# Visualize the route and threats
plt.figure(figsize=(8, 6))

# Plot the threats
for threat in threats:
    cx, cy, r = threat
    circle = plt.Circle((cx, cy), r, color='r', fill=False)
    plt.gca().add_artist(circle)

# Plot the optimal route
if optimal_route is not None:
    x_coords = [start_point[0]] + [point[0] for point in optimal_route] + [end_point[0]]
    y_coords = [start_point[1]] + [point[1] for point in optimal_route] + [end_point[1]]
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)

# Set plot limits and labels
plt.xlim(0, 45)
plt.ylim(0, 20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title('Optimal Route', fontsize=16)

plt.show()

#Change in rho
rho_i = 0.04
optimal_route = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
# Visualize the route and threats
plt.figure(figsize=(8, 6))

# Plot the threats
for threat in threats:
    cx, cy, r = threat
    circle = plt.Circle((cx, cy), r, color='r', fill=False)
    plt.gca().add_artist(circle)

# Plot the optimal route
if optimal_route is not None:
    x_coords = [start_point[0]] + [point[0] for point in optimal_route] + [end_point[0]]
    y_coords = [start_point[1]] + [point[1] for point in optimal_route] + [end_point[1]]
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)

# Set plot limits and labels
plt.xlim(0, 45)
plt.ylim(0, 20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title('Optimal Route', fontsize=16)

plt.show()

#Change in rho
rho_i = 0.16
optimal_route = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
# Visualize the route and threats
plt.figure(figsize=(8, 6))

# Plot the threats
for threat in threats:
    cx, cy, r = threat
    circle = plt.Circle((cx, cy), r, color='r', fill=False)
    plt.gca().add_artist(circle)

# Plot the optimal route
if optimal_route is not None:
    x_coords = [start_point[0]] + [point[0] for point in optimal_route] + [end_point[0]]
    y_coords = [start_point[1]] + [point[1] for point in optimal_route] + [end_point[1]]
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)

# Set plot limits and labels
plt.xlim(0, 45)
plt.ylim(0, 20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title('Optimal Route', fontsize=16)

plt.show()

#Change in rho
rho_i = 0.64
optimal_route = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
# Visualize the route and threats
plt.figure(figsize=(8, 6))

# Plot the threats
for threat in threats:
    cx, cy, r = threat
    circle = plt.Circle((cx, cy), r, color='r', fill=False)
    plt.gca().add_artist(circle)

# Plot the optimal route
if optimal_route is not None:
    x_coords = [start_point[0]] + [point[0] for point in optimal_route] + [end_point[0]]
    y_coords = [start_point[1]] + [point[1] for point in optimal_route] + [end_point[1]]
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)

# Set plot limits and labels
plt.xlim(0, 45)
plt.ylim(0, 20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title('Optimal Route', fontsize=16)

plt.show()

rho_i = 0.01
penalty_term_v = 0
penalty_term_u = 0

# Problem 1
print("~~ Problem 1 ~~")
start_point = [3, 12]
end_point = [40, 13]
initial_guess = [(11, 18), (17, 18), (23, 18), (29, 18), (35, 18)]
optimal_route_1 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_1 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_1 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)

# Problem 2
rho_i = 0.01
penalty_term_v = 0
penalty_term_u = 0.01
print("~~ Problem 2 ~~")
start_point = [3, 12]
end_point = [40, 13]
initial_guess = [(11, 18), (17, 18), (23, 18), (29, 18), (35, 18)]
optimal_route_2 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_2 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_2 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)

rho_i = 0.01
penalty_term_v = 0
penalty_term_u = 0

# Problem 3
print("~~ Problem 3 ~~")
rho_i = 0.01
penalty_term_v = 0.0001
penalty_term_u = 0.01
start_point = [3, 12]
end_point = [40, 13]
initial_guess = [(6, 12), (14, 12.2), (22, 12.5), (30, 12.7), (38, 12.9)]
optimal_route_3 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_3 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_3 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)

rho_i = 0.01
penalty_term_v = 0
penalty_term_u = 0.01

# Problem 4
print("~~ Problem 4 ~~")
start_point = [3, 12]
end_point = [40, 13]
initial_guess = [(6, 12), (14, 12.2), (22, 12.5), (30, 12.7), (38, 12.9)]
optimal_route_4 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_4 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_4 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)





#2
print(f"------------------------------ Mission 2 ------------------------------")
#Change in rho
rho_i = 0.01
start_point = [3, 12]
#different starting point
end_point = [40, 5]
initial_guess = [(11, 18), (17, 18), (23, 18), (29, 18), (35, 18)]
optimal_route1 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
# Visualize the route and threats
plt.figure(figsize=(8, 6))

# Plot the threats
for threat in threats:
    cx, cy, r = threat
    circle = plt.Circle((cx, cy), r, color='r', fill=False)
    plt.gca().add_artist(circle)

# Plot the optimal route
if optimal_route1 is not None:
    x_coords = [start_point[0]] + [point[0] for point in optimal_route1] + [end_point[0]]
    y_coords = [start_point[1]] + [point[1] for point in optimal_route1] + [end_point[1]]
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)

# Set plot limits and labels
plt.xlim(0, 45)
plt.ylim(0, 20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title('Optimal Route', fontsize=16)

plt.show()

#Change in rho
rho_i = 0.04
optimal_route1 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
# Visualize the route and threats
plt.figure(figsize=(8, 6))

# Plot the threats
for threat in threats:
    cx, cy, r = threat
    circle = plt.Circle((cx, cy), r, color='r', fill=False)
    plt.gca().add_artist(circle)

# Plot the optimal route
if optimal_route1 is not None:
    x_coords = [start_point[0]] + [point[0] for point in optimal_route1] + [end_point[0]]
    y_coords = [start_point[1]] + [point[1] for point in optimal_route1] + [end_point[1]]
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)

# Set plot limits and labels
plt.xlim(0, 45)
plt.ylim(0, 20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title('Optimal Route', fontsize=16)

plt.show()

#Change in rho
rho_i = 0.16
optimal_route1 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
# Visualize the route and threats
plt.figure(figsize=(8, 6))

# Plot the threats
for threat in threats:
    cx, cy, r = threat
    circle = plt.Circle((cx, cy), r, color='r', fill=False)
    plt.gca().add_artist(circle)

# Plot the optimal route
if optimal_route1 is not None:
    x_coords = [start_point[0]] + [point[0] for point in optimal_route1] + [end_point[0]]
    y_coords = [start_point[1]] + [point[1] for point in optimal_route1] + [end_point[1]]
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)

# Set plot limits and labels
plt.xlim(0, 45)
plt.ylim(0, 20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title('Optimal Route', fontsize=16)

plt.show()

#Change in rho
rho_i = 0.64
optimal_route1 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
# Visualize the route and threats
plt.figure(figsize=(8, 6))

# Plot the threats
for threat in threats:
    cx, cy, r = threat
    circle = plt.Circle((cx, cy), r, color='r', fill=False)
    plt.gca().add_artist(circle)

# Plot the optimal route
if optimal_route1 is not None:
    x_coords = [start_point[0]] + [point[0] for point in optimal_route1] + [end_point[0]]
    y_coords = [start_point[1]] + [point[1] for point in optimal_route1] + [end_point[1]]
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)

# Set plot limits and labels
plt.xlim(0, 45)
plt.ylim(0, 20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title('Optimal Route', fontsize=16)

plt.show()

rho_i = 0.01
penalty_term_v = 0
penalty_term_u = 0

# Problem 5
print("~~ Problem 5 ~~")
start_point = [3, 12]
end_point = [40, 5]
initial_guess = [(11, 18), (17, 18), (23, 18), (29, 18), (35, 18)]
optimal_route_5 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_5 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_5 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)

rho_i = 0.01
penalty_term_v = 0.0001
penalty_term_u = 0

# Problem 6
print("~~ Problem 6 ~~")
start_point = [3, 12]
end_point = [40, 5]
initial_guess = [(11, 18), (17, 18), (23, 18), (29, 18), (35, 18)]
optimal_route_6 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_6 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_6 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)

rho_i = 0.01
penalty_term_v = 0
penalty_term_u = 0

# Problem 7
print("~~ Problem 7 ~~")
start_point = [3, 12]
end_point = [40, 5]
initial_guess = [(6, 12), (14, 12.2), (22, 12.5), (30, 12.7), (38, 12.9)]
optimal_route_7 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_7 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_7 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)

rho_i = 0.01
penalty_term_v = 0.0001
penalty_term_u = 0.01 

# Problem 8 
print("~~ Problem 8 ~~")
start_point = [3, 12]
end_point = [40, 5]
initial_guess = [(6, 12), (14, 12.2), (22, 12.5), (30, 12.7), (38, 12.9)]
optimal_route_8 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_8 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
rho_i = rho_i * 4
penalty_term_u = penalty_term_u*4
penalty_term_v = penalty_term_v*4
optimal_route_8 = direct_optimization(calculate_objective, start_point, end_point,max_iter=1000)
