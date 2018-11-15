import time
from math import sqrt
import random
import numpy as np

class Edge:
    def __init__(self, u, v, weight):
        self.u = u
        self.v = v
        self.weight = weight
        

class DisjointSetUnion:
    def __init__(self, size):
        self.parent_ = list(range(size))
        self.ranks_ = [0] * size

    def find(self, node):
        if self.parent_[node] != node:
            self.parent_[node] = self.find(self.parent_[node])
        return self.parent_[node]

    def union_sets(self, first, second):
        f_root = self.find(first)
        s_root = self.find(second)
        if f_root == s_root:
            return False
        if self.ranks_[f_root] < self.ranks_[s_root]:
            self.parent_[f_root] = s_root
        elif self.ranks_[f_root] > self.ranks_[s_root]:
            self.parent_[s_root] = f_root
        else:
            self.parent_[s_root] = f_root
            self.ranks_[f_root] += 1
        return True

    def result(self):
        return list(map(self.find, self.parent_))
    
    def size(self):
        return len(self.parent_)


def kruskal(n, edges, ds, num_of_clusters=1):
    edges.sort(key=lambda edge: edge.weight)

    mst = []

    for edge in edges:
        set_u = ds.find(edge.u)
        set_v = ds.find(edge.v)
        if set_u != set_v:
            ds.union_sets(set_u, set_v)
            mst.append(edge)
            if len(mst) == n - num_of_clusters:
                return ds

    return ds


def cluster_kruskal(ver, edges_t, num_of_clusters):
    res = DisjointSetUnion(len(ver))
    
    edges = []
    for e_t in edges_t:
        e = Edge(e_t[1], e_t[2], e_t[0])
        edges.append(e)
    
    res = kruskal(len(ver), edges, res, num_of_clusters)

    return res


def cluster_greed(ver, edges, num_of_clusters):
    n = len(ver)
    
    ds = DisjointSetUnion(n)
    
    parents = []
    while len(parents) != num_of_clusters:
        p = random.randint(0, n)
        if p not in parents:
            parents.append(p)
    
    max_w = max(edges)[0]
    dists = [[max_w + 1 for i in range(n)] for j in range(n)]
    
    for edge in edges:
        u = edge[1]
        v = edge[2]
        if v in parents:
            dists[u][v] = edge[0]
        
    for i in range(n):
        if i not in parents:
            parent = dists[i].index(min(dists[i]))
            set_u = ds.find(i)
            set_v = ds.find(parent)
            ds.union_sets(set_u, set_v)

    return ds
