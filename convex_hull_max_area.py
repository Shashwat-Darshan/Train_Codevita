# from collections import deque

# # Function to calculate the orientation (cross product)
# def ccw(A, B, C):
#     # ccw tends to find the convex hull stack values in a clockwise order
#     return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

# # Function to find the intersection of two line segments
# def line_intersection(A, B, C, D):
#     # A, B are the endpoints of the first segment, C, D are the endpoints of the second segment
#     denom = (B[0] - A[0]) * (D[1] - C[1]) - (B[1] - A[1]) * (D[0] - C[0])
    
#     if denom == 0:
#         return None  # Parallel lines, no intersection
    
#     # Calculate the intersection point using Cramer's rule
#     t1 = ((C[0] - A[0]) * (D[1] - C[1]) - (C[1] - A[1]) * (D[0] - C[0])) / denom
#     t2 = ((C[0] - A[0]) * (B[1] - A[1]) - (C[1] - A[1]) * (B[0] - A[0])) / denom

#     if 0 <= t1 <= 1 and 0 <= t2 <= 1:  # If the intersection point lies on both line segments
#         intersection = (A[0] + t1 * (B[0] - A[0]), A[1] + t1 * (B[1] - A[1]))
#         return intersection
#     return None

# # Function to calculate the convex hull of a set of points using the Graham scan algorithm
# def convex_hull(points):
#     # print("Convex_hull called")
#     # print(f"before sorting {points}")
#     points = sorted(set(points))  # Sort and remove duplicates
#     print(f"after sorting {points}")
#     if len(points) <= 1:
#         return points
    
#     # Build the lower hull
#     lower = []
#     for p in points:
#         while len(lower) >= 2 and not ccw(lower[-2], lower[-1], p):
#             lower.pop()
#         lower.append(p)

#     # Build the upper hull
#     upper = []
#     for p in reversed(points):
#         while len(upper) >= 2 and not ccw(upper[-2], upper[-1], p):
#             upper.pop()
#         upper.append(p)
#     # Remove the last point of each half because it's repeated at the beginning of the other half
#     print(lower)
#     print(upper)
#     return lower[:-1] + upper[:-1]

# # Function to calculate the area of the polygon formed by the convex hull points
# def polygon_area(points):
#     n = len(points)
#     area = 0
#     for i in range(n):
#         j = (i + 1) % n  # Next point in the list (loop back to 0 if at the end)
#         area += points[i][0] * points[j][1]
#         area -= points[j][0] * points[i][1]
#     return abs(area) / 2

# # Main function to calculate the maximum area of polygon formed by intersections of line segments
# def max_area_of_polygon(segments):
#     intersection_points = set()

#     # Step 1: Find all intersections between line segments
#     for i in range(len(segments)):
#         for j in range(i + 1, len(segments)):
#             segment1 = segments[i]
#             segment2 = segments[j]
#             A = (segment1[0], segment1[1])
#             B = (segment1[2], segment1[3])
#             C = (segment2[0], segment2[1])
#             D = (segment2[2], segment2[3])

#             intersection = line_intersection(A, B, C, D)
#             if intersection:
#                 intersection_points.add(intersection)
#     # print(intersection_points)
#     # Step 2: Compute the convex hull from the intersection points
#     if len(intersection_points) < 3:
#         return 0  # Not enough points to form a polygon

#     hull_points = convex_hull(list(intersection_points))

#     # Step 3: Calculate the area of the polygon formed by the convex hull
#     return polygon_area(hull_points)

# # Input
# N = int(input())  # Number of line segments
# segments = []
# for _ in range(N):
#     x1, y1, x2, y2 = map(int, input().split())
#     segments.append((x1, y1, x2, y2))

# # Calculate and output the maximum area polygon
# result = max_area_of_polygon(segments)
# print((result) if int(result) != result else int(result))

from itertools import combinations
from collections import defaultdict
from math import isclose

def intersection(p1, p2, q1, q2):
    # Calculate intersection point of two segments if it exists
    def det(a, b, c, d):
        return a * d - b * c

    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = q1
    x4, y4 = q2

    denom = det(x1 - x2, y1 - y2, x3 - x4, y3 - y4)
    if isclose(denom, 0):  # Parallel or coincident
        return None

    px = det(det(x1, y1, x2, y2), x1 - x2, det(x3, y3, x4, y4), x3 - x4) / denom
    py = det(det(x1, y1, x2, y2), y1 - y2, det(x3, y3, x4, y4), y3 - y4) / denom

    # Check if the intersection point lies on both segments
    if (
        min(x1, x2) <= px <= max(x1, x2) and
        min(y1, y2) <= py <= max(y1, y2) and
        min(x3, x4) <= px <= max(x3, x4) and
        min(y3, y4) <= py <= max(y3, y4)
    ):
        return (px, py)
    return None

def shoelace_area(points):
    # Calculate polygon area using shoelace formula
    n = len(points)
    area = 0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        area += x1 * y2 - y1 * x2
    return abs(area) // 2

def find_polygons(segments):
    # Find intersection points and build a graph
    points = set()
    edges = defaultdict(list)
    for (x1, y1, x2, y2), (x3, y3, x4, y4) in combinations(segments, 2):
        p = intersection((x1, y1), (x2, y2), (x3, y3), (x4, y4))
        if p:
            points.add(p)
            edges[(x1, y1)].append(p)
            edges[(x2, y2)].append(p)
            edges[(x3, y3)].append(p)
            edges[(x4, y4)].append(p)
            edges[p].extend([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])

    # Perform DFS to find cycles (polygons)
    def dfs(node, start, visited, path):
        if node == start and len(path) > 2:
            polygons.append(path[:])
            return
        if node in visited:
            return
        visited.add(node)
        for neighbor in edges[node]:
            dfs(neighbor, start, visited, path + [neighbor])
        visited.remove(node)

    polygons = []
    for point in points:
        dfs(point, point, set(), [point])

    # Filter valid polygons and calculate their areas
    max_area = 0
    for polygon in polygons:
        if len(set(polygon)) > 2:  # Valid polygon
            max_area = max(max_area, shoelace_area(polygon))
    return max_area

# Input
n = int(input())
segments = [tuple(map(int, input().split())) for _ in range(n)]
print(find_polygons(segments))
