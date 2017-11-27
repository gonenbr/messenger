import unittest
import FindRoute
import MapAlgo

import sys
import pdb
import functools
import traceback
def debug_on(*exceptions):
    if not exceptions:
        exceptions = (AssertionError, )
    def decorator(f):
        @functools.wraps(f)
        def wrapper(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            except exceptions:
                info = sys.exc_info()
                traceback.print_exception(*info)
                pdb.post_mortem(info[2])
        return wrapper
    return decorator



class TestUM(unittest.TestCase):
    def setUp(self):
        pass

    @debug_on()
    def test_maproute(self):
        decodedr1 = [(2, 1), (3, 1), (3, 2), (3, 3), (4, 3), (4, 4)]
        cost1={((2, 1), (3, 1)):12,((3, 1), (3, 2)):10,((3, 2), (3, 3)):4,((3, 3), (4, 3)):150,((4, 3), (4, 4)):210}
        # decodedr2 = {(1, 2): [(2, 2)], (2, 2): [(2, 3)], (2, 3): [(3, 3)], (3, 3): [(3, 4)], (3, 4): [(4, 4)]}
        decodedr2 = [(1, 2), (2, 2), (2, 3), (3, 3), (3, 4), (4, 4)]
        cost2 = {((1, 2), (2, 2)): 12, ((2, 2), (2, 3)): 10, ((2, 3), (3, 3)): 4, ((3, 3), (3, 4)): 150,
                 ((3, 4), (4, 4)): 210}
        # decodedr3 = {(1, 4): [(1, 3)], (1, 3): [(2, 3)], (2, 3): [(2, 2)], (2, 2): [(3, 2)], (3, 2): [(3, 1)],(3,1):[(4,1)]}
        decodedr3 = [(1, 4), (1, 3), (2, 3), (2, 2), (3, 2), (3, 1), (4, 1)]
        cost3 = {((1, 4), (1, 3)): 12, ((1, 3), (2, 3)): 10, ((2, 3), (2, 2)): 4, ((2, 2), (3, 2)): 150,
                 ((3, 2), (3, 1)): 210,((3, 1), (4, 1)):4}
        decoded_paths_decoder = [decodedr1, decodedr2, decodedr3]
        costs_array=[cost1,cost2,cost3]
        # adjacent = FindRoute.createMapGraph(decoded_paths_decoder)
        result_map = {(1, 2): [(2, 2)], (3, 2): [(3, 1), (3, 3), (2, 2)], (4, 4): [(4, 3), (3, 4)],
         (3, 3): [(3, 2), (4, 3), (2, 3), (3, 4)], (4, 1): [(3, 1)], (3, 1): [(2, 1), (3, 2), (4, 1)], (2, 1): [(3, 1)],
         (1, 3): [(1, 4), (2, 3)], (2, 3): [(2, 2), (3, 3), (1, 3)], (1, 4): [(1, 3)], (4, 3): [(3, 3), (4, 4)],
         (2, 2): [(1, 2), (2, 3), (3, 2)], (3, 4): [(3, 3), (4, 4)]}
        result_cost={((2, 1), (3, 1)): 12, ((2, 3), (2, 2)): 4, ((3, 4), (4, 4)): 210, ((2, 3), (3, 3)): 4, ((1, 3), (2, 3)): 10, ((3, 1), (3, 2)): 10, ((3, 1), (4, 1)): 4, ((3, 2), (3, 1)): 210, ((1, 4), (1, 3)): 12, ((2, 2), (2, 3)): 10, ((4, 3), (4, 4)): 210, ((2, 2), (3, 2)): 150, ((3, 3), (4, 3)): 150, ((3, 2), (3, 3)): 4, ((3, 3), (3, 4)): 150, ((1, 2), (2, 2)): 12}
        # result = {(1, 2): [(2, 2)], (3, 2): [(3, 3), (3, 1),(2,2)], (4, 4): [(4,3),(3,4)], (3, 3): [(4, 3), (3, 4),(3,2),(2,3)], (4, 1): [(3,1)], (3, 1): [(3, 2), (4, 1),(2,1)], (2, 1): [(3, 1)], (1, 3): [(2, 3)], (2, 3): [(3, 3), (2, 2)], (1, 4): [(1, 3)], (4, 3): [(4, 4)], (2, 2): [(2, 3), (3, 2)], (3, 4): [(4, 4)]}
        self.assertEqual(FindRoute.create_map_graph(decoded_paths_decoder,costs_array), (result_map,result_cost))
    def test_dijkstra(self):
        rout_maps = {(1,2):[(2,2)],(2,2):[(2,1),(2,3)],(2,1):[(3,1)],(2,3):[(3,3)],(3,1):[(3,2)],(3,3):[(3,2)]
       ,(3,2):[(4,2)],(4,2):[(4,3)],(4,3):[(4,4),(5,3)],(5,3):[(5,4)],(5,4):[(5,5)],(4,4):[(3,4)]
        ,(3,4):[(3,5)],(3,5):[(4,5)],(4,5):[(5,5)]}
        cost = {((1,2),(2,2)):2,((2,2),(2,1)):1,((2,1),(3,1)):4,((3,1),(3,2)):5,((3,2),(4,2)):7,((4,2),(4,3)):1,((4,3),(5,3)):3,((5,3),(5,4)):2,((5,4),(5,5)):4
        ,((2,2),(2,3)):2,((2,3),(3,3)):3,((3,3),(3,2)):2,((4,3),(4,4)):2,((4,4),(3,4)):4,((3,4),(3,5)):5,((3,5),(4,5)):3,((4,5),(5,5)):3}
        ref_predecessors = {(3, 2): (3, 3), (4, 4): (4, 3), (3, 3): (2, 3), (5, 5): (5, 4), (3, 1): (2, 1), (5, 4): (5, 3), (2, 1): (2, 2), (2, 3): (2, 2), (4, 3): (4, 2), (2, 2): (1, 2), (4, 2): (3, 2), (3, 4): (4, 4), (5, 3): (4, 3)}
        ref_min_cost = 26
        s, t = (1, 2), (5, 5)  # start and stop
        self.assertEqual(MapAlgo.dijkstra(rout_maps, cost, s, t), (ref_predecessors, ref_min_cost))
        # predecessors, min_cost = MapAlgo.dijkstra(rout_maps, cost, s, t)
    def test_decoder(self):
        encoded = '_p~iF~ps|U_ulLnnqC_mqNvxq`@'
        decoded = [(38.5, -120.2), (40.7, -120.95), (43.252, -126.453)]
        self.assertEqual(FindRoute.decode(encoded), decoded)
    def test_encoder(self):
        encoded = '_p~iF~ps|U_ulLnnqC_mqNvxq`@'
        decoded = [(38.5, -120.2), (40.7, -120.95), (43.252, -126.453)]
        self.assertEqual(FindRoute.encode(decoded), encoded)





if __name__ == '__main__':
    unittest.main()
