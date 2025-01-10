# this is the dp solution for the problem B
from collections import defaultdict

# Function to calculate the worth of a string based on the sum of its character place values
def calculate_worth(string):
    return sum(ord(char) - ord('a') + 1 for char in string)

def solve(strings, costs, contradictory_pairs, budget):
    N = len(strings)
    
    # Create a mapping from string to its index
    string_to_index = {strings[i]: i for i in range(N)}
    
    # Create a graph for contradictory pairs
    contradicted = [0] * N  # Will store bitmasks for contradictions
    for a, b in contradictory_pairs:
        idx_a = string_to_index[a]
        idx_b = string_to_index[b]
        contradicted[idx_a] |= (1 << idx_b)
        contradicted[idx_b] |= (1 << idx_a)
    
    # Initialize DP array where dp[mask] stores the maximum worth for the subset represented by mask
    dp = [-1] * (1 << N)
    dp[0] = 0  # Base case: empty set has worth 0 and cost 0
    
    # Precompute worth of each string
    worth = [calculate_worth(s) for s in strings]
    
    # DP with bitmask
    for mask in range(1 << N):  # Loop through all subsets (bitmasks)
        for i in range(N):  # Try adding each string i to the current subset
            # If string i is not already in the subset
            if (mask & (1 << i)) == 0:
                # Check if adding string i causes any contradictions
                if (mask & contradicted[i]) == 0:
                    new_mask = mask | (1 << i)
                    new_cost = sum(costs[j] for j in range(N) if (new_mask & (1 << j)) != 0)
                    # If the cost is within the budget, update the dp state
                    if new_cost <= budget:
                        dp[new_mask] = max(dp[new_mask], dp[mask] + worth[i])
    
    # The answer is the maximum value in dp where the cost is <= budget
    return max([value if value != -1 else 0 for value in dp])

# Read input
N, M = map(int, input().split())
strings = input().split()
costs = list(map(int, input().split()))
contradictory_pairs = [input().split() for _ in range(M)]
budget = int(input())

# Solve the problem
result = solve(strings, costs, contradictory_pairs, budget)
print(result)
