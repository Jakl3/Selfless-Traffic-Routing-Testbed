# Results of Floyd-Warshall vs Dijkstra Algorithms for STR SUMO
## Algorithm Discussion
In this semi-selfless routing algorithm, I utilized the Floyd-Warshall Algorithm for pathfinding and a 
system of dynamic weights. Instead of the weight of an edge being its distance, I used the travel time 
across that edge as the metric. This travel time is defined as the quotient of the length of that edge 
by the average velocity across that edge, if vehicles are present on that edge. Otherwise, the travel 
time will be equal to the maximum speed allowed across that edge.
```python
total_velocity = 0
number_of_vehicles = len(vehicles_on_edge[edge])
if number_of_vehicles > 0:
    for vehicle in vehicles_on_edge[edge]:
        total_velocity += vehicle.velocity
    average_velocity = total_velocity * (1/number_of_vehicles)
else:
    average_velocity = max_speed[edge]
weight[edge] = length_of_edge[edge] / average_velocity
```
By this definition, for a congested road where average velocity is near zero, the weight of an edge is 
significantly increased. This ensures that the vehicle will not pick a congested edge and instead will take 
one with a shorter travel time, even if the total distance traveled is greater.

## Floyd-Warshall Algorithm

In the Floyd-Warshall Algorithm, given a directed graph G with n vertices, it computes the shortest path 
between each pair of vertices i and j. This is stored in a distance matrix d[][], where d[i][j] for any 
vertices i and j stores the length of the shortest path between the vertex i and the vertex j.

The computation is done as follows:
* In the k-th phase, where k < n, d[i][j] stores the shortest length of all pairs in the k-th phase.
* The algorithm then considers the alternate route, traversing from vertex i to vertex k then to vertex j. 
  If this distance of this route is less than that of vertex i to vertex j, the corresponding shortest
  distance is updated.
* At every new phase, d[][] is recalculated, where d_new[i][j] = min(d[i][j], d[i][k] + d[k][j])

To retrieve the path, a predecessor matrix p[][] is used, where p[i][j] stores the predecessor 
of the shortest path. Initially, this will be j if an edge exists between vertex i and vertex j. If 
a new shorter distance is found in the k-th phase, p[i][j] will equal p[i][k].

```python
for k in range(n):
    for i in range(n):
        for j in range(n):
            # Checks if the alternate edge exists
            if (d[i][k] == inf or d[k][j] == inf):
                continue

            # Checks if the alternate route is better than current
            if (d[i][k] + d[k][j] < d[i][j]):
                d[i][j] = d[i][k] + d[k][j]
                p[i][j] = p[i][k]
```
The Floyd-Warshall algorithm was picked because of its computational efficiency and adaptability to new 
situations. As opposed to a traditional Dijkstra's Algorithm where computations could be repeated as a 
shortest path must be found for every vehicle, Floyd-Warshall can precompute all the paths based on the 
travel times at that exact moment. In addition, if a vehicle were to decide to change destination, the path 
can easily be adjusted without recomputing the best possible path.

## Results
The results of 1,000 trials are below. Note that in each trial, there is around ten cars sent out. 
This means that when there are 344 and 118 deadlines missed, that is 334 and 118 deadlines out of 
10,000 cars.
```php
Results from 1000 trials:
>> Dijkstra [average timespan, total vehicle number, deadlines missed]
Average timespan: 237.30666666666667, deadlines missed: 344.0
>> Test [average timespan, total vehicle number, deadlines missed]
Average timespan: 212.11645555555558, deadlines missed: 118.0
>>> Differences (this is a representative sample):
> Better
Dijkstra: [323.2  10.    0. ], Test: [219.5  10.    0. ]
Dijkstra: [501.9  10.    0. ], Test: [340.4  10.    0. ]
Dijkstra: [544.2  10.    3. ], Test: [348.2  10.    0. ]
Dijkstra: [306.9  10.    0. ], Test: [230.3  10.    0. ]
Dijkstra: [459.9  10.    0. ], Test: [274.3  10.    0. ]
Dijkstra: [122.8  10.    0. ], Test: [67.4   10.    0. ]
Dijkstra: [588.6  10.    2. ], Test: [314.1  10.    0. ]
Dijkstra: [45.2   10.    0. ], Test: [6.2    10.    0. ]
Dijkstra: [658.0   9.    2. ], Test: [371.4  10.    0. ]
Dijkstra: [485.7  10.    0. ], Test: [299.0  10.    0. ]
> Worse
Dijkstra: [134.2  10.    0. ], Test: [152.9  10.    0. ]
Dijkstra: [193.5  10.    0. ], Test: [220.   10.    0. ]
Dijkstra: [78.3   10.    0. ], Test: [99.3   10.    0. ]
Dijkstra: [153.6  10.    0. ], Test: [171.7  10.    0. ]
Dijkstra: [532.8  10.    2. ], Test: [547.8  10.    2. ]
Dijkstra: [161.8  10.    0. ], Test: [181.2  10.    0. ]
Dijkstra: [229.4  10.    0. ], Test: [287.8  10.    0. ]
Dijkstra: [85.8   10.    0. ], Test: [126.6  10.    0. ]
Dijkstra: [323.9  10.    0. ], Test: [338.9  10.    0. ]
Dijkstra: [376.2  10.    0. ], Test: [431.7  10.    0. ]
>>> Test performed 11.875651535443232% better than Dijkstra's
>>> The two algorithsm differed for 79.4% of the trials
>>> Mean Difference (Dijkstra - Test): 31.725706689057024
>>> For cases where the two algorithsm differed, the performance of Test is 113.48722799185249% that of Dijkstra

```
The results show that in generally, the semi-selfless Floyd-Warshall Algorithm in combination 
with travel-time-defined weights performs better than the traditional selfish Dijkstra's Algorithm. The 
nearly 12% improvement is significant because of the simple structure of the road network, as well as the 
high number of trials ran.

Based on these results, Floyd-Warshall generally significantly outperforms Dijkstra's in long trips, 
where the average travel time is over 300 units. For cases where the average travel time of Dijkstra's is 
less than 300 units, Floyd-Warshall performs marginally worse.

In addition, the Floyd-Warshall Algorithm achieves this task while having significantly less deadlines 
missed, with nearly 1/3 that of Dijkstra's Algorithm.
