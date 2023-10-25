import sys
import itertools
import random
import math
import numpy as np

def calculate_distance(city1, city2):
    # Calculate the Euclidean distance between two cities using linear algebra
    coord1 = np.array(city1)
    coord2 = np.array(city2)
    return np.linalg.norm(coord1 - coord2)

def total_distance(path, distances):
    # Calculate the total distance of a path
    return sum(distances[path[i]][path[i+1]] for i in range(len(path) - 1))

def simulated_annealing_tsp(cities, distances, max_iterations, initial_temperature):
    num_cities = len(cities)
    current_solution = random.sample(cities, num_cities)  # Start with a random initial solution
    best_solution = current_solution[:]
    current_distance = total_distance(current_solution, distances)
    best_distance = current_distance

    for iteration in range(max_iterations):
        # Generate a neighbor solution by reversing a random segment of the current solution
        i, j = sorted(random.sample(range(num_cities), 2))
        neighbor_solution = current_solution[:i] + list(reversed(current_solution[i:j])) + current_solution[j:]

        # Calculate the distance of the neighbor solution
        neighbor_distance = total_distance(neighbor_solution, distances)

        # If the neighbor solution is better or accepted due to temperature, update the current solution
        if neighbor_distance < current_distance or random.random() < math.exp((current_distance - neighbor_distance) / initial_temperature):
            current_solution = neighbor_solution
            current_distance = neighbor_distance

            # Update the best solution if necessary
            if current_distance < best_distance:
                best_solution = current_solution[:]
                best_distance = current_distance

        # Decrease temperature (annealing schedule)
        initial_temperature *= 0.995

    return best_solution, best_distance

if __name__ == "__main__":
    cities = [
        (0, 0),
        (1, 2),
        (2, 4),
        (3, 1),
    ]  # Replace with your city coordinates

    num_cities = len(cities)

    # Generate random distances between cities as a 2D matrix
    distances = [[0] * num_cities for _ in range(num_cities)]
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            distances[i][j] = distances[j][i] = calculate_distance(cities[i], cities[j])

    max_iterations = 10000
    initial_temperature = 1000.0
    
    final_solution, total_dist = simulated_annealing_tsp(cities, distances, max_iterations, initial_temperature)
    print("Optimal Path:", final_solution)
    print("Total Distance:", total_dist)
