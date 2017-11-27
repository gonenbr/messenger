"""
   Copyright 2011 Shao-Chuan Wang <shaochuan.wang AT gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
"""

import heapq

def dijkstra(adj, costs, s, t,ids=None):
    ''' Return predecessors and min distance if there exists a shortest path 
        from s to t; Otherwise, return None '''
    Q = []     # priority queue of items; note item is mutable.
    d = {s: 0} # vertex -> minimal distance
    Qd = {}    # vertex -> [d[v], parent_v, v]
    p = {}     # predecessor
    visited_set = set([s])
    #the next line is new from 2-11
    dijkstra_ids={}

#31-10 initialzation step
    for v in adj.get(s, []):#means that if the key is not there return empty list
        d[v] = costs[s, v]
        item = [d[v], s, v]
        heapq.heappush(Q, item)
        Qd[v] = item
    from sys import maxint
    while Q:
        print Q
        cost, parent, u = heapq.heappop(Q)
        if u not in visited_set:
            print 'visit:', u
            p[u]= parent
            # ids={}
            if ids is not None:
                # if ids.has_key((parent,u)):
                    # dijkstra_ids[parent,u]=ids[parent,u]
                # dijkstra_ids[u] = [parent].extend(ids[parent, u])
                dijkstra_ids[u] = [parent]+ids[parent, u]
            visited_set.add(u)
            if u == t:
                return p, d[u],dijkstra_ids
            for v in adj.get(u, []):
                cost_uv =costs.get((u, v), maxint)#this is a new line 31-10
                if cost_uv is maxint:
                    pass
                if d.get(v):
                    # if d[v] > costs[u, v] + d[u]:
                    if d[v] > cost_uv + d[u]:
                        # d[v] =  costs[u, v] + d[u]
                        d[v] = cost_uv + d[u]
                        Qd[v][0] = d[v]    # decrease key
                        Qd[v][1] = u       # update predecessor
                        heapq._siftdown(Q, 0, Q.index(Qd[v]))
                else:
                    # d[v] = costs[u, v] + d[u]
                    d[v] = cost_uv + d[u]
                    item = [d[v], u, v]
                    heapq.heappush(Q, item)
                    Qd[v] = item

    return None

# this is a temp comment this is the old version for hte funtion before inserting the ids into this function

# def initializecost(adj,costvalueinit,paths):
#     cost={}
#     ids={}
#     path_temp =[]
#     # adj={(2,4):[(2,5),(4,6)]}
#     for adj_node in adj:
#         for target_node in adj[adj_node]:
#             cost[(adj_node,target_node)]=costvalueinit
#             for path in paths:
#                 if path_temp.index(adj_node):
#                     if path_temp.index(target_node):
#                         ids[((adj_node,target_node))]
#     return cost


#this version from the 29-10 has the addition of adding the ids DS to the function

def initializecost(adj,costvalueinit,paths=None):
    cost={}
    ids={}
    # path_temp =[]
    # adj={(2,4):[(2,5),(4,6)]}
    for adj_node in adj:
        for target_node in adj[adj_node]:
            cost[(adj_node,target_node)]=costvalueinit
            if paths is not None:
                for i,path in enumerate(paths):
                    if adj_node in path and target_node in path:
                        ids[((adj_node,target_node))]=i
    return cost,ids


def make_undirected(cost):
    ucost = {}
    for k, w in cost.iteritems():
        ucost[k] = w
        ucost[(k[1],k[0])] = w
    return ucost

if __name__=='__main__':
    # adjacent list
    adj ={(1, 2): [(2, 2)], (3, 2): [(3, 1), (3, 3), (2, 2)], (4, 4): [(4, 3), (3, 4)], (3, 3): [(3, 2), (4, 3), (2, 3), (3, 4)], (4, 1): [(3, 1)], (3, 1): [(2, 1), (3, 2), (4, 1)], (2, 1): [(3, 1)], (1, 3): [(1, 4), (2, 3)], (2, 3): [(2, 2), (3, 3), (1, 3)], (1, 4): [(1, 3)], (4, 3): [(3, 3), (4, 4)], (2, 2): [(1, 2), (2, 3), (3, 2)], (3, 4): [(3, 3), (4, 4)]}

    # adj = {1: [2, 3, 6],
    #        2: [1, 3, 4],
    #        3: [1, 2, 4, 6],
    #        4: [2, 3, 5, 7],
    #        5: [4, 6, 7],
    #        6: [1, 3, 5, 7],
    #        7: [4, 5, 6]}
    path1 = [(2,1),(3,1),(3,2),(3,3),(4,3),(4,4)]
    path2 = [(1, 4), (1,3), (2, 3), (2, 2), (3, 2), (3, 1),(4, 1)]
    path3 = [(1, 2), (2, 2), (2, 3), (3, 3), (3, 4), (4, 4)]
    paths = [path1,path2,path3]



    cost,ids=initializecost(adj,1,paths)
    s, t = 1, 7  # start and stop
    predecessors, min_cost,dijkstra_ids = dijkstra(adj, cost, s, t)
    c = t
    path = [c]
    print 'min cost:', min_cost
    while predecessors.get(c):
        path.insert(0, predecessors[c])
        c = predecessors[c]

    print 'shortest path:', path


    # adjacent list
    adj = { 1: [2,3,6],
            2: [1,3,4],
            3: [1,2,4,6],
            4: [2,3,5,7],
            5: [4,6,7],
            6: [1,3,5,7],
            7: [4,5,6]}

    # edge costs
    cost = { (1,2):7,
            (1,3):9,
            (1,6):14,
            (2,3):10,
            (2,4):15,
            (3,4):11,
            (3,6):2,
            (4,5):6,
            (5,6):9,
            (4,7):2,
            (5,7):1,
            (6,7):12}

    cost = make_undirected(cost)

    s, t = 1, 7#start and stop
    predecessors, min_cost = dijkstra(adj, cost, s, t)
    c = t
    path = [c]
    print 'min cost:', min_cost
    while predecessors.get(c):
        path.insert(0, predecessors[c])
        c = predecessors[c]

    print 'shortest path:', path
