from collections import deque

# Function to calculate the orientation (cross product)
def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

# Function to find the intersection of two line segments
def line_intersection(A, B, C, D):
    # A, B are the endpoints of the first segment, C, D are the endpoints of the second segment
    denom = (B[0] - A[0]) * (D[1] - C[1]) - (B[1] - A[1]) * (D[0] - C[0])
    
    if denom == 0:
        return None  # Parallel lines, no intersection
    
    # Calculate the intersection point using Cramer's rule
    t1 = ((C[0] - A[0]) * (D[1] - C[1]) - (C[1] - A[1]) * (D[0] - C[0])) / denom
    t2 = ((C[0] - A[0]) * (B[1] - A[1]) - (C[1] - A[1]) * (B[0] - A[0])) / denom

    if 0 <= t1 <= 1 and 0 <= t2 <= 1:  # If the intersection point lies on both line segments
        intersection = (A[0] + t1 * (B[0] - A[0]), A[1] + t1 * (B[1] - A[1]))
        return intersection
    return None

# Function to calculate the convex hull of a set of points using the Graham scan algorithm
def convex_hull(points):
    points = sorted(set(points))  # Sort and remove duplicates
    if len(points) <= 1:
        return points
    
    # Build the lower hull
    lower = []
    for p in points:
        while len(lower) >= 2 and not ccw(lower[-2], lower[-1], p):
            lower.pop()
        lower.append(p)

    # Build the upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and not ccw(upper[-2], upper[-1], p):
            upper.pop()
        upper.append(p)
    # Remove the last point of each half because it's repeated at the beginning of the other half
    print(lower)
    print(upper)
    return lower[:-1] + upper[:-1]

# Function to calculate the area of the polygon formed by the convex hull points
def polygon_area(points):
    n = len(points)
    area = 0
    for i in range(n):
        j = (i + 1) % n  # Next point in the list (loop back to 0 if at the end)
        area += points[i][0] * points[j][1]
        area -= points[j][0] * points[i][1]
    return abs(area) / 2

# Main function to calculate the maximum area of polygon formed by intersections of line segments
def max_area_of_polygon(segments):
    intersection_points = set()

    # Step 1: Find all intersections between line segments
    for i in range(len(segments)):
        for j in range(i + 1, len(segments)):
            segment1 = segments[i]
            segment2 = segments[j]
            A = (segment1[0], segment1[1])
            B = (segment1[2], segment1[3])
            C = (segment2[0], segment2[1])
            D = (segment2[2], segment2[3])

            intersection = line_intersection(A, B, C, D)
            if intersection:
                intersection_points.add(intersection)

    # Step 2: Compute the convex hull from the intersection points
    if len(intersection_points) < 3:
        return 0  # Not enough points to form a polygon

    hull_points = convex_hull(list(intersection_points))

    # Step 3: Calculate the area of the polygon formed by the convex hull
    return polygon_area(hull_points)

# Input
N = int(input())  # Number of line segments
segments = []
for _ in range(N):
    x1, y1, x2, y2 = map(int, input().split())
    segments.append((x1, y1, x2, y2))

# Calculate and output the maximum area polygon
result = max_area_of_polygon(segments)
print((result) if int(result) != result else int(result))

