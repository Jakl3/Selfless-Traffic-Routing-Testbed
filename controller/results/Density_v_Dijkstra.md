# Results of DensityDijkstra vs Dijkstra Algorithms for STR SUMO
## Density Dijkstra
This algorithm is based off of Dijkstra's Algorithm, with the main difference
being that it accounts for the density of the road (the amount of cars on the road
divided by the length of the road). This would directly address the problem of
multiple cars picking the fastest route without regard for how many cars are
currently on that route.

The results show that, for trials where Dijkstra's Algorithm results in relatively
quick average travel times, the Density Dijkstra's algorithm results in even quicker
times.

This was just a preliminary test. After seeing the results, I do not think it performs
well enough to safely conclude that it is consistently better than Dijkstra's. After this,
I plan to implement a different algorithm that will hopefully address the goal of
selfless traffic routing.

```python
Results from 1000 trials:
>>> Time taken (hr:min:sec): [1:11:14]

>>> Dijkstra [average timespan, total vehicle number, deadlines missed]
Average timespan: 229.82524444444442, deadlines missed: 271.0

>>> DensityDijkstra [average timespan, total vehicle number, deadlines missed]
Average timespan: 227.55284444444447, deadlines missed: 271.0

>>> Differences:
Dijkstra: [92.3 10.   0. ],     Density: [ 5. 10.  0.]
Dijkstra: [67.3 10.   0. ],     Density: [16.1 10.   0. ]
Dijkstra: [30.1 10.   0. ],     Density: [ 9.5 10.   0. ]
Dijkstra: [46.8 10.   0. ],     Density: [29.1 10.   0. ]
Dijkstra: [32.5 10.   0. ],     Density: [ 6.5 10.   0. ]
Dijkstra: [41.8 10.   0. ],     Density: [32.2 10.   0. ]
Dijkstra: [46.5 10.   0. ],     Density: [14.1 10.   0. ]
Dijkstra: [48.2 10.   0. ],     Density: [42. 10.  0.]
Dijkstra: [45.3 10.   0. ],     Density: [25.7 10.   0. ]
Dijkstra: [37.8 10.   0. ],     Density: [10.6 10.   0. ]
Dijkstra: [34.9 10.   0. ],     Density: [ 6.4 10.   0. ]
Dijkstra: [171.1  10.    0. ],  Density: [42. 10.  0.]
Dijkstra: [34.1 10.   0. ],     Density: [10.6 10.   0. ]
Dijkstra: [51.3 10.   0. ],     Density: [19.1 10.   0. ]
Dijkstra: [32.5 10.   0. ],     Density: [ 6.5 10.   0. ]
Dijkstra: [80.4 10.   0. ],     Density: [ 6.4 10.   0. ]
Dijkstra: [90.6 10.   0. ],     Density: [32.2 10.   0. ]
Dijkstra: [34.9 10.   0. ],     Density: [ 6.4 10.   0. ]
Dijkstra: [19.7 10.   0. ],     Density: [ 5.6 10.   0. ]
Dijkstra: [28.9 10.   0. ],     Density: [10. 10.  0.]
Dijkstra: [34.9 10.   0. ],     Density: [ 6.4 10.   0. ]
Dijkstra: [47. 10.  0.],        Density: [13.2 10.   0. ]
Dijkstra: [46.8 10.   0. ],     Density: [29.1 10.   0. ]
Dijkstra: [63.5 10.   0. ],     Density: [23.3 10.   0. ]
Dijkstra: [90.6 10.   0. ],     Density: [32.2 10.   0. ]
Dijkstra: [19.7 10.   0. ],     Density: [ 5.6 10.   0. ]
Dijkstra: [34.1 10.   0. ],     Density: [10.6 10.   0. ]
Dijkstra: [31.2 10.   0. ],     Density: [ 6.4 10.   0. ]
Dijkstra: [48.9 10.   0. ],     Density: [24.8 10.   0. ]
Dijkstra: [56.4 10.   0. ],     Density: [ 6.4 10.   0. ]
Dijkstra: [48.9 10.   0. ],     Density: [24.8 10.   0. ]
Dijkstra: [92.3 10.   0. ],     Density: [ 5. 10.  0.]
Dijkstra: [46.5 10.   0. ],     Density: [14.1 10.   0. ]
Dijkstra: [47. 10.  0.],        Density: [13.2 10.   0. ]
Dijkstra: [41.4 10.   0. ],     Density: [24.8 10.   0. ]
Dijkstra: [171.4  10.    0. ],  Density: [133.3  10.    0. ]
Dijkstra: [80.4 10.   0. ],     Density: [ 6.4 10.   0. ]
Dijkstra: [41.8 10.   0. ],     Density: [32.2 10.   0. ]
Dijkstra: [44.8 10.   0. ],     Density: [ 7.8 10.   0. ]
Dijkstra: [27.7 10.   0. ],     Density: [ 5. 10.  0.]
Dijkstra: [47. 10.  0.],        Density: [13.2 10.   0. ]
Dijkstra: [40.1 10.   0. ],     Density: [ 4. 10.  0.]
Dijkstra: [27.7 10.   0. ],     Density: [ 5. 10.  0.]
Dijkstra: [19.7 10.   0. ],     Density: [ 5.6 10.   0. ]
Dijkstra: [67.3 10.   0. ],     Density: [10. 10.  0.]
Dijkstra: [31.6 10.   0. ],     Density: [ 7.8 10.   0. ]
Dijkstra: [40.1 10.   0. ],     Density: [ 4. 10.  0.]
Dijkstra: [44.8 10.   0. ],     Density: [ 7.8 10.   0. ]
Dijkstra: [92.3 10.   0. ],     Density: [ 5. 10.  0.]
Dijkstra: [19.7 10.   0. ],     Density: [ 5.6 10.   0. ]
Dijkstra: [45. 10.  0.],        Density: [23.7 10.   0. ]
Dijkstra: [50.4 10.   0. ],     Density: [13.2 10.   0. ]
Dijkstra: [63.5 10.   0. ],     Density: [23.3 10.   0. ]
Dijkstra: [171.4  10.    0. ],  Density: [133.3  10.    0. ]
Dijkstra: [17.4 10.   0. ],     Density: [ 6.5 10.   0. ]
Dijkstra: [44.8 10.   0. ],     Density: [ 7.8 10.   0. ]
Dijkstra: [33.5 10.   0. ],     Density: [ 5.6 10.   0. ]
Dijkstra: [17.4 10.   0. ],     Density: [ 6.5 10.   0. ]
Dijkstra: [48.9 10.   0. ],     Density: [24.8 10.   0. ]
Dijkstra: [122.8  10.    0. ],  Density: [67.4 10.   0. ]
Dijkstra: [31.2 10.   0. ],     Density: [ 6.4 10.   0. ]
Dijkstra: [122.8  10.    0. ],  Density: [67.4 10.   0. ]
Dijkstra: [63.5 10.   0. ],     Density: [23.3 10.   0. ]
Dijkstra: [122.8  10.    0. ],  Density: [67.4 10.   0. ]
Dijkstra: [41.8 10.   0. ],     Density: [32.2 10.   0. ]

>>> The two algorithsm differed for 6.5% of the trials
>>> Mean Timespan Difference (Dijkstra - Density): 34.96000000000002
>>> For cases where the two algorithsm differed,
    the performance of Density is 269.9117691042321% that of Dijkstra
```
