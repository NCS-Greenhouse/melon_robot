import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def initialize_particles(num_particles, region_limits):
    """
    Initialize particles in 3D space.

    Parameters:
    - num_particles: Number of particles to initialize.
    - region_limits: Tuple (min_limits, max_limits) defining the 3D region.

    Returns:
    - particles: NumPy array of shape (num_particles, 3) representing 3D positions.
    """

    min_limits, max_limits = region_limits

    # Generate random 3D positions for particles within the specified region
    particles = np.random.uniform(min_limits, max_limits, size=(num_particles, 3))

    return particles

def motion_model(particles, previous_positions, noise_scale=0.5):
    """
    Apply a simple motion model to predict the next position of particles.

    Parameters:
    - particles: NumPy array of shape (num_particles, 3) representing current 3D positions.
    - previous_positions: NumPy array of shape (num_particles, 3) representing previous positions.
    - noise_scale: Scaling factor for introducing small adjustments (noise) in the motion model.

    Returns:
    - predicted_positions: NumPy array of shape (num_particles, 3) representing predicted 3D positions.
    """

    # Simple motion model: Add small adjustments to the previous positions
    adjustments = noise_scale * np.random.randn(*particles.shape)
    predicted_positions = previous_positions + adjustments

    return predicted_positions

def update_particles_from_pointcloud(particles, P_petiole):
    """
    Update particle weights based on 3D point cloud of petioles.

    Parameters:
    - particles: NumPy array of shape (num_particles, 3) representing current 3D positions.
    - P_petiole: NumPy array of shape (num_petioles, 3) representing 3D positions of petioles.

    Returns:
    - weights: NumPy array of shape (num_particles,) representing particle weights.
    """

    weights = np.zeros(len(particles))

    for i, particle in enumerate(particles):
        # Compute weight based on the distance to the closest petiole in P_petiole
        distances = np.linalg.norm(particle - P_petiole, axis=1)
        min_distance = np.min(distances)

        # Assign weight inversely proportional to the distance
        weights[i] = 1 / (min_distance + 1e-6)  # Adding a small epsilon to avoid division by zero

    # Normalize weights
    weights /= np.sum(weights)

    return weights

def visualize_particles_with_weights(particles, weights):
    """
    Visualize 3D particles with colors representing their weights.

    Parameters:
    - particles: NumPy array of shape (num_particles, 3) representing 3D positions.
    - weights: NumPy array of shape (num_particles,) representing particle weights.
    """

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Scale weights for better visualization
    scaled_weights = 100 * weights

    # Scatter plot with colors based on weights
    scatter = ax.scatter(particles[:, 0], particles[:, 1], particles[:, 2], c=scaled_weights, cmap='viridis')

    # Add colorbar to show the correspondence between color and weight
    cbar = plt.colorbar(scatter)
    cbar.set_label('Particle Weights')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([0.2,1.0])
    ax.set_ylim([-0.5,0.5])
    ax.set_zlim([0,1.56])
    ax.set_title('Particle Visualization with Weights')

    plt.show()

def resample_particles(particles, weights, num_random_points = 20, noise_scale=0.003):
    """
    Resample particles based on their weights.

    Parameters:
    - particles: NumPy array of shape (num_particles, 3) representing current 3D positions.
    - weights: NumPy array of shape (num_particles,) representing particle weights.

    Returns:
    - resampled_particles: NumPy array of shape (num_particles, 3) representing resampled 3D positions.
    """

    num_particles = len(particles)
    indices = np.arange(num_particles)

    # Resample particles based on their weights
    resampled_indices = np.random.choice(indices, size=num_particles, replace=True, p=weights)

    # Select the particles with the resampled indices
    resampled_particles = particles[resampled_indices]


    # Add additional random noise to the resampled particles
    noise = noise_scale * np.random.randn(*resampled_particles.shape)
    resampled_particles += noise
    # Assign uniform weights after resampling
    resampled_weights = np.ones(num_particles) / num_particles
    print(np.sum(resampled_weights))
    # Introduce additional random points
    random_points = generate_random_points_3d(num_random_points, region_limits)
    resampled_particles = np.vstack([resampled_particles, random_points])
    resampled_weights = np.concatenate([resampled_weights, np.ones(num_random_points) / (num_particles + num_random_points)])

    return resampled_particles, resampled_weights

def generate_line(num_points, region_limits):
    """
    Generate a line within specified region limits.

    Parameters:
    - num_points: Number of points to sample along the line.
    - region_limits: Tuple of two points representing the region limits in the form ((min_x, min_y, min_z), (max_x, max_y, max_z)).

    Returns:
    - line_points: NumPy array of shape (num_points, 3) representing points along the generated line.
    """

    # Extract region limits
    min_limits, max_limits = region_limits

    # Generate random parameters for the line within the specified limits
    t_values = np.linspace(0, 1, num_points)
    x_values = np.linspace(min_limits[0], max_limits[0], num_points)
    y_values = np.linspace(min_limits[1], max_limits[1], num_points)
    z_values = np.linspace(min_limits[2], max_limits[2], num_points)

    # Combine the parameters to get points along the line
    line_points = np.column_stack((x_values, y_values, z_values))

    return line_points

def generate_random_points_3d(num_points, region_limits):
    # Extract individual limits for x, y, and z from region_limits
    x_limit, y_limit, z_limit = zip(*region_limits)

    # Generate random x, y, and z coordinates within the specified limits
    x_coordinates = np.random.uniform(low=x_limit[0], high=x_limit[1], size=num_points)
    y_coordinates = np.random.uniform(low=y_limit[0], high=y_limit[1], size=num_points)
    z_coordinates = np.random.uniform(low=z_limit[0], high=z_limit[1], size=num_points)
    
    # Merge the coordinates into a single NumPy array
    coordinates = np.array([x_coordinates, y_coordinates, z_coordinates]).T
    
    return coordinates


# Add a bounding box, any points outside this b_box, generate a new PP

# Example usage:
num_particles = 3000
region_limits = ((0.2, -0.5, 0), (1.0, 0.5, 1.56))  # Adjust these limits based on your scenario
expanded_region_limits = ((-1., -1., 0.), (1.0, 1.0,  1.8)) # Adjust these limits based on your scenario
# P_petiole = np.random.rand(50, 3)  # Replace with your actual 3D point cloud from YOLOv5

num_points = 200
P_petiole_1 = generate_random_points_3d(num_points,((0.55, -0.05, 0.1), (0.65, 0.05, 0.15)))
P_petiole_2 = generate_random_points_3d(num_points,((0.55, -0.05, 1.3), (0.65, 0.05, 1.4)))
P_petiole = P_petiole_1
particles = initialize_particles(num_particles, expanded_region_limits)

# Visualize the line
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(P_petiole[:, 0], P_petiole[:, 1], P_petiole[:, 2], c='b', marker='o', label='Generated Line')
# # Set axis labels
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# # Set plot title
# ax.set_title('Generated Line Visualization')
# ax.set_xlim([0.2,1.0])
# ax.set_ylim([-0.5,0.5])
# ax.set_zlim([0,1.56])
# # Show the plot
# plt.legend()
# plt.show()

for i in range(20):
    # Example usage:
    previous_positions = particles  # Assuming particles have been initialized
    predicted_positions = motion_model(particles, previous_positions, noise_scale=0.3)

    # Update particle weights
    
    weights = update_particles_from_pointcloud(particles, P_petiole)

    resampled_particles, new_weights = resample_particles(particles, weights)
    # visualize_particles_with_weights(resampled_particles,weights)

    particles = resampled_particles
    weights = new_weights


print("Resampled particles:")
print(len(resampled_particles))

visualize_particles_with_weights(resampled_particles,weights)
# visualize_particles_with_weights(P_petiole,weights[0:30])
