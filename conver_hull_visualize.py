import matplotlib.pyplot as plt

# Function to find the intersection of two lines
def get_intersection(line1, line2):
    # Unpack the lines
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    
    # Calculate the denominator
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    
    # If denominator is zero, lines are parallel and do not intersect
    if denom == 0:
        return None
    
    # Calculate the intersection point
    intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
    
    # Check if the intersection point is within the bounds of both line segments
    if (min(x1, x2) <= intersect_x <= max(x1, x2) and min(y1, y2) <= intersect_y <= max(y1, y2) and
        min(x3, x4) <= intersect_x <= max(x3, x4) and min(y3, y4) <= intersect_y <= max(y3, y4)):
        return (intersect_x, intersect_y)
    else:
        return None

# Function to find all intersection points
def find_intersections(lines):
    intersections = []
    n = len(lines)
    
    for i in range(n):
        for j in range(i + 1, n):
            intersection = get_intersection(lines[i], lines[j])
            if intersection:
                intersections.append(intersection)
    
    return intersections

# Function to plot lines and intersection points
def plot_lines_and_intersections(lines, intersections):
    plt.figure(figsize=(8, 8))
    
    # Plot the lines
    for line in lines:
        x1, y1, x2, y2 = line
        plt.plot([x1, x2], [y1, y2], color='blue', linewidth=2)  # Line in blue
    
    # Plot intersection points
    for point in intersections:
        plt.scatter(point[0], point[1], color='red', zorder=5, label="Intersection" if not plt.gca().get_label() else "")
    
    # Add labels for axes
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    
    # Add grid
    plt.grid(True)
    
    # Show the plot
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')  # Maintain aspect ratio
    plt.show()

def main():
    # Read number of line segments
    N = int(input(""))
    
    # Read the line segments
    lines = []
    for _ in range(N):
        x1, y1, x2, y2 = map(int, input().split())
        lines.append((x1, y1, x2, y2))
    
    # Find the intersections
    intersections = find_intersections(lines)
    
    # Visualize the lines and intersections
    plot_lines_and_intersections(lines, intersections)

# Call the main function to execute the program
if __name__ == "__main__":
    main()
