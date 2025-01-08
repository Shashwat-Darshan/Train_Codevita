# Pointer-Based Techniques

## GETTING READY

### Two Pointers

- [ ] Used for problems involving searching, sorting, or subarrays (e.g., finding pairs with a given sum, merging sorted arrays).
  - **Worst Case Time Complexity**: O(n)
  - **When to Use**: When you need to process elements in a linear fashion with two indices.

### Three Pointers

- [ ] Typically used in problems involving multiple sorted arrays or partitioning problems.
  - **Worst Case Time Complexity**: O(n)
  - **When to Use**: When dealing with multiple sorted arrays or partitioning problems.

### Sliding Window

- [ ] Effective for solving problems related to subarrays or substrings (e.g., finding the maximum sum of subarray of size k, or the smallest subarray with a given sum).
  - **Worst Case Time Complexity**: O(n)
  - **When to Use**: When you need to find or optimize a subarray or substring within a given array or string.

## Greedy Algorithms

- [ ] Coin change problem (min number of coins).
  - **Worst Case Time Complexity**: O(n)
  - **When to Use**: When you need to make a series of choices to achieve the optimal solution.
- [ ] Activity selection problem.
  - **Worst Case Time Complexity**: O(n log n)
  - **When to Use**: When you need to select the maximum number of activities that don't overlap.
- [ ] Huffman encoding.
  - **Worst Case Time Complexity**: O(n log n)
  - **When to Use**: When you need to compress data efficiently.
- [ ] Fractional knapsack.
  - **Worst Case Time Complexity**: O(n log n)
  - **When to Use**: When you need to maximize the total value of items that can be carried in a knapsack.
- [ ] Optimal merges (merge intervals).
  - **Worst Case Time Complexity**: O(n log n)
  - **When to Use**: When you need to merge overlapping intervals efficiently.

## Dynamic Programming (DP)

- [ ] **Knapsack Variants**: 0/1, fractional, unbounded.
  - **Worst Case Time Complexity**: O(nW) for 0/1 knapsack
  - **When to Use**: When you need to solve optimization problems with overlapping subproblems.
- [ ] **Subsequence/Subarray Problems**: Longest Increasing Subsequence, Longest Common Subsequence, Maximum Subarray Sum.
  - **Worst Case Time Complexity**: O(n^2) for LCS
  - **When to Use**: When you need to find the optimal subsequence or subarray.
- [ ] **Matrix DP**: Problems like grid traversal, pathfinding, or matrix chain multiplication.
  - **Worst Case Time Complexity**: O(n^3) for matrix chain multiplication
  - **When to Use**: When you need to solve problems involving grids or matrices.
- [ ] **Bitmask DP**: Used for problems with state representation via bits (e.g., Traveling Salesman Problem).
  - **Worst Case Time Complexity**: O(2^n * n)
  - **When to Use**: When you need to solve problems with a large number of states.
- [ ] **DP with Optimizations**: Divide and Conquer Optimization, Slope Optimization.
  - **Worst Case Time Complexity**: Varies
  - **When to Use**: When you need to optimize the time complexity of standard DP solutions.

## Graph Algorithms

### Graph Traversals

- [ ] Depth-First Search (DFS).
  - **Worst Case Time Complexity**: O(V + E)
  - **When to Use**: When you need to explore all nodes and edges of a graph.
- [ ] Breadth-First Search (BFS).
  - **Worst Case Time Complexity**: O(V + E)
  - **When to Use**: When you need to find the shortest path in an unweighted graph.

### Shortest Path Algorithms

- [ ] Dijkstra’s Algorithm
  - **Worst Case Time Complexity**: O(V^2) or O(E + V log V) with a priority queue
  - **When to Use**: When you need to find the shortest path in a graph with non-negative weights.
- [ ] Bellman-Ford Algorithm
  - **Worst Case Time Complexity**: O(VE)
  - **When to Use**: When you need to find the shortest path in a graph with negative weights.
- [ ] Floyd-Warshall Algorithm
  - **Worst Case Time Complexity**: O(V^3)
  - **When to Use**: When you need to find shortest paths between all pairs of vertices.
- [ ] A* Search
  - **Worst Case Time Complexity**: O(E)
  - **When to Use**: When you need to find the shortest path with heuristics.

### Minimum Spanning Tree (MST)

- [ ] Kruskal’s Algorithm
  - **Worst Case Time Complexity**: O(E log E)
  - **When to Use**: When you need to find the minimum spanning tree using a greedy approach.
- [ ] Prim’s Algorithm
  - **Worst Case Time Complexity**: O(V^2) or O(E + V log V) with a priority queue
  - **When to Use**: When you need to find the minimum spanning tree starting from a specific vertex.

### Union-Find/Disjoint Set Union (DSU)

- [ ] Used for connected components or detecting cycles.
  - **Worst Case Time Complexity**: O(α(n)) per operation, where α is the inverse Ackermann function
  - **When to Use**: When you need to manage and merge disjoint sets efficiently.

### Topological Sorting

- [ ] Used in DAGs for task scheduling.
  - **Worst Case Time Complexity**: O(V + E)
  - **When to Use**: When you need to order tasks that have dependencies.

### Max Flow Algorithms

- [ ] Ford-Fulkerson
  - **Worst Case Time Complexity**: O(max_flow * E)
  - **When to Use**: When you need to find the maximum flow in a flow network.
- [ ] Edmonds-Karp
  - **Worst Case Time Complexity**: O(VE^2)
  - **When to Use**: When you need a more efficient implementation of Ford-Fulkerson.
- [ ] Dinic’s Algorithm
  - **Worst Case Time Complexity**: O(V^2E)
  - **When to Use**: When you need to find the maximum flow in a flow network more efficiently than Edmonds-Karp.

### Cycle Detection

- [ ] Tarjan’s Algorithm
  - **Worst Case Time Complexity**: O(V + E)
  - **When to Use**: When you need to find strongly connected components in a graph.
- [ ] DFS-based cycle detection
  - **Worst Case Time Complexity**: O(V + E)
  - **When to Use**: When you need to detect cycles in a graph using depth-first search.

## String Algorithms

### Pattern Matching

- [ ] Knuth-Morris-Pratt (KMP)
  - **Worst Case Time Complexity**: O(n + m)
  - **When to Use**: When you need to find occurrences of a pattern in a text efficiently.
- [ ] Rabin-Karp
  - **Worst Case Time Complexity**: O(nm) (average case O(n + m))
  - **When to Use**: When you need to find occurrences of a pattern in a text using hashing.
- [ ] Z Algorithm
  - **Worst Case Time Complexity**: O(n + m)
  - **When to Use**: When you need to find occurrences of a pattern in a text efficiently.

### Suffix Array/Tree

- [ ] Used for substring problems, LCS, etc.
  - **Worst Case Time Complexity**: O(n log n) for construction
  - **When to Use**: When you need to perform fast substring queries.

### Trie

- [ ] Efficient prefix-based search.
  - **Worst Case Time Complexity**: O(m) per query, where m is the length of the query
  - **When to Use**: When you need to perform fast prefix-based searches.

### Rolling Hashing

- [ ] For substring comparisons (e.g., finding repeated substrings).
  - **Worst Case Time Complexity**: O(n)
  - **When to Use**: When you need to compare substrings efficiently using hashing.

## String Manipulation

- [ ] Used for problems involving operations on strings.
  - **Worst Case Time Complexity**: O(n) for traversal, O(n^2) for concatenation
  - **When to Use**: When you need to perform operations like searching, concatenation, and modification of strings.

## Mathematics and Number Theory

### Prime Numbers

- [ ] Sieve of Eratosthenes
  - **Worst Case Time Complexity**: O(n log log n)
  - **When to Use**: When you need to find all prime numbers up to a given limit.
- [ ] Segmented Sieve
  - **Worst Case Time Complexity**: O(n log log n)
  - **When to Use**: When you need to find all prime numbers in a large range.

### Greatest Common Divisor (GCD)

- [ ] Euclid’s Algorithm
  - **Worst Case Time Complexity**: O(log min(a, b))
  - **When to Use**: When you need to find the GCD of two numbers efficiently.
- [ ] Extended Euclid’s Algorithm
  - **Worst Case Time Complexity**: O(log min(a, b))
  - **When to Use**: When you need to find the GCD and the coefficients of Bézout's identity.

### Modulo Arithmetic

- [ ] Modular Exponentiation
  - **Worst Case Time Complexity**: O(log n)
  - **When to Use**: When you need to compute large powers modulo a number efficiently.
- [ ] Modular Inverse
  - **Worst Case Time Complexity**: O(log n)
  - **When to Use**: When you need to find the modular inverse of a number.
- [ ] Chinese Remainder Theorem
  - **Worst Case Time Complexity**: O(k log n), where k is the number of equations
  - **When to Use**: When you need to solve systems of simultaneous congruences.

### Combinatorics

- [ ] Factorials and binomial coefficients
  - **Worst Case Time Complexity**: O(n) for factorial computation
  - **When to Use**: When you need to compute combinatorial values.
- [ ] Pascal’s Triangle
  - **Worst Case Time Complexity**: O(n^2)
  - **When to Use**: When you need to compute binomial coefficients efficiently.

### Matrix Exponentiation

- [ ] Efficient computation of large powers of matrices.
  - **Worst Case Time Complexity**: O(n^3 log k)
  - **When to Use**: When you need to compute large powers of matrices efficiently.

## Sorting and Searching

### Sorting Algorithms

- [ ] Merge Sort, Quick Sort, Heap Sort, etc.
  - **Worst Case Time Complexity**: O(n log n)
  - **When to Use**: When you need to sort elements efficiently.
- [ ] Custom comparators for complex sorting.
  - **Worst Case Time Complexity**: O(n log n)
  - **When to Use**: When you need to sort elements based on custom criteria.

### Binary Search

- [ ] Used for search problems on sorted data.
  - **Worst Case Time Complexity**: O(log n)
  - **When to Use**: When you need to find an element in a sorted array efficiently.

### Ternary Search

- [ ] Used for unimodal functions or optimization problems.
  - **Worst Case Time Complexity**: O(log n)
  - **When to Use**: When you need to find the maximum or minimum of a unimodal function.

## Hashing and Data Structures

### Hash Maps and Hash Sets

- [ ] Used for fast lookups, duplicate checks.
  - **Worst Case Time Complexity**: O(1) on average
  - **When to Use**: When you need fast insertions, deletions, and lookups.

### Fenwick Tree / Binary Indexed Tree (BIT)

- [ ] Efficient range queries and updates.
  - **Worst Case Time Complexity**: O(log n) per operation
  - **When to Use**: When you need to perform range queries and updates efficiently.

### Segment Tree

- [ ] Supports range queries with lazy propagation.
  - **Worst Case Time Complexity**: O(log n) per operation
  - **When to Use**: When you need to perform range queries and updates efficiently.

### Sparse Table

- [ ] Used for static range queries like RMQ (Range Minimum Query).
  - **Worst Case Time Complexity**: O(1) per query after O(n log n) preprocessing
  - **When to Use**: When you need to perform static range queries efficiently.

### Priority Queues

- [ ] Min-Heap/Max-Heap for efficient selection problems.
  - **Worst Case Time Complexity**: O(log n) per operation
  - **When to Use**: When you need to efficiently retrieve the minimum or maximum element.

### Deque/Monotonic Queues

- [ ] Optimizing sliding window problems.
  - **Worst Case Time Complexity**: O(1) per operation
  - **When to Use**: When you need to optimize sliding window problems efficiently.

## Divide and Conquer

- [ ] Merge Sort
  - **Worst Case Time Complexity**: O(n log n)
  - **When to Use**: When you need to sort elements efficiently.
- [ ] Quick Sort
  - **Worst Case Time Complexity**: O(n^2) (average case O(n log n))
  - **When to Use**: When you need to sort elements efficiently.
- [ ] Binary Search on Answer (optimization problems)
  - **Worst Case Time Complexity**: O(log n)
  - **When to Use**: When you need to find the optimal solution in a search space.

## Backtracking and Recursion

### Permutations and Combinations

- [ ] Generating all permutations and combinations.
  - **Worst Case Time Complexity**: O(n!)
  - **When to Use**: When you need to generate all possible permutations or combinations.

### Constraint Satisfaction Problems

- [ ] Sudoku solver, N-Queens problem.
  - **Worst Case Time Complexity**: Varies (typically exponential)
  - **When to Use**: When you need to solve problems with constraints that must be satisfied.

### Subset Generation

- [ ] Power sets, subset sums.
  - **Worst Case Time Complexity**: O(2^n)
  - **When to Use**: When you need to generate all subsets of a set.

## Specialized Techniques

### Bit Manipulation

- [ ] XOR tricks.
  - **Worst Case Time Complexity**: O(1) per operation
  - **When to Use**: When you need to perform fast bitwise operations.
- [ ] Subset generation using bits.
  - **Worst Case Time Complexity**: O(2^n)
  - **When to Use**: When you need to generate subsets using bitwise operations.

### Meet-in-the-Middle

- [ ] Efficiently solving problems with large constraints.
  - **Worst Case Time Complexity**: O(2^(n/2))
  - **When to Use**: When you need to solve problems with large input sizes by dividing the problem into two smaller subproblems.
