# this is the custom convex hull to increase the wideness of points+ add height of edge points
import math

def cross(o, a, b):
    """
    Compute the cross product of vectors OA and OB.
    A positive cross product indicates a counter-clockwise turn.
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def calculate_distance(p1, p2):
    """
    Calculate the Euclidean distance between two points (p1 and p2).
    """
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

def convex_hull(points):
    """
    Compute the convex hull of a set of 2D points using Andrew's monotone chain algorithm.
    Returns the vertices of the convex hull in counter-clockwise order.
    """
    # Sort points lexicographically by x, then by y.
    points = sorted(points)

    # Build the lower hull.
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build the upper hull.
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Remove the last point of each half because it is repeated.
    return lower[:-1] + upper[:-1]

def filter_points(points):
    filtered = {}
    points=sorted(points)
    # print(points)
    min_x=points[0][0]
    val=[0,0]
    max_x=points[-1][0] 
    # print(min_x,max_x)
    for x, y in points:
        if x==min_x:
            temp=y-val[0]
            val[0]=max(temp,-temp)
        elif x==max_x:
            
            val[1]=abs(y-val[1])
        if x not in filtered or y < filtered[x]:
            filtered[x] = y  # Keep the point with the smallest y for each x
    # print("vales",val)
    return sorted((x, y) for x, y in filtered.items()),(0 if val[0]==points[0][1] else val[0]) +(  0 if val[1]==points[-1][1] else val[1])
def calculate_bowl_perimeter(points):
    """
    Calculate the perimeter of the bowl formed by the lower convex hull.
    """
    sum=0

    points,edge_values= filter_points(points)
    # print(f"edge values {edge_values}")
    hull = convex_hull(points)
    lower_hull = [hull[0]]
    r=0
    i=0
    # if hull i's x values is greater than hull i+1's x value then pop i+1
    while(i<len(hull)-1):
        if hull[i][1]<hull[i+1][1]:
            r=1
        if hull[i][0]>hull[i+1][0]:
            hull.pop(i+1)
        else:
            i+=1
    # print(hull)
    # Extract only the points forming the lower part of the hull (bowl shape).
    prev_x = hull[0][0]
    for i in range(1, len(hull)):
        if hull[i][0] >= prev_x:
            lower_hull.append(hull[i])
            prev_x = hull[i][0]

    # Calculate the perimeter of the lower hull.
    perimeter = 0.0
    for i in range(len(lower_hull) - 1):
        perimeter += calculate_distance(lower_hull[i], lower_hull[i + 1])
    return round(perimeter+edge_values)

def main():
    """
    Main function to read input and output the perimeter of the bowl.
    """
    # Input number of points
    n = int(input())

    # Input points
    points = []
    for _ in range(n):
        x, y = map(int, input().split())
        points.append((x, y))

    # Calculate and output the perimeter of the bowl
    print(calculate_bowl_perimeter(points))

if __name__ == "__main__":
    main()
