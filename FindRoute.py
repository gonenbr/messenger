
def extractRouteSetFile(route_json_file_name):
    fp=open(route_json_file_name)
    import json
    route_json = json.load(fp)
    return route_json
#todo work on the polyline there, it is endcoded small steps of differentials and use this to find the matches

# def extractRouteSetFile(jsonmapresponse)
#     jsonmapresponse =[]
#     jsonmapresponse[]
#todo need to find the number of chunks in a different way. need to check it after doing the left shift and inverstion meaning call this function later on
def findnumberofchuncks(number):
    number_of_chunks=6#27-10 was 6 changed to 7 because on vey small values of numbers it returns 0
    # if number<0:
    #     number = -number
    for index in range(0,8):
        if (0xf8000000 & (number<<(5*index)))>0:
            if number_of_chunks-index==0:
                return 1
            return number_of_chunks-index
    return 1


def py2_round(value):
    #Google's polyline algorithm uses the same rounding strategy as Python 2, which is different from JS for negative values
    from math import floor,copysign
    return floor(abs(value)+0.5)*copysign(1,value)

    # return Math.floor(Math.abs(value) + 0.5) * Math.sign(value);



def encode(cord_list):
    encodedpath=''
    chuncks=[]
    initial_lat=0
    initial_long=0
    for cord in cord_list:
        lat=cord[0]
        lat*=1E5
        lat=lat-initial_lat
        initial_lat=cord[0]*1E5
        lat = int(round(lat))
        lat_sign = lat
        lat<<=1
        if lat_sign<0:
            lat=~lat
        output=''
        while lat >= 0x20:
            output += chr((0x20 | (lat & 0x1f)) + 63);
            lat >>= 5;
        output += chr(lat + 63);
        encodedpath+=output
        # return output;

        # test_number_of_chuncks = findnumberofchuncks(lat)
        # for index in range(0,test_number_of_chuncks):
        #     chunk = 0x1f& (lat>>(5*index))
        #     if index!=test_number_of_chuncks-1:
        #         chunk|=0x20
        #     chunk+=63
        #     chuncks.append(chunk)
        #     encodedpath+=chr(chunk)

#deals with longtitude
        long = cord[1]
        long *= 1E5
        long = long - initial_long
        initial_long = cord[1]*1E5
        long = int(round(long))
        long_sign = long
        long <<= 1
        if long_sign < 0:
            long = ~long
        output = ''
        while long >= 0x20:
            output += chr((0x20 | (long & 0x1f)) + 63);
            long >>= 5;
        output += chr(long + 63);
        encodedpath += output

        # test_number_of_chuncks = findnumberofchuncks(long)
        # for index in range(0, test_number_of_chuncks):
        #     chunk = 0x1f & (long >> (5 * index))
        #     if index != test_number_of_chuncks-1:
        #         chunk |= 0x20
        #     chunk += 63
        #     chuncks.append(chunk)
        #     encodedpath += chr(chunk)

    return encodedpath

def extract_cost_and_time_and_route_from_route(json,starting_time):
#todo 5-11 need to work on addeing the absolute time to the cost

    from datetime import time,datetime,timedelta
    # starting_time=time(10)
    # starting_time+time(minute=5)
    path_time=starting_time

    out_set = []
    cost={}
    for step in json[0]['legs'][0]['steps']:
        leg_duration = step['duration']['value']
        decode_result=[]
        decode_result = decode(step['polyline']['points'])
        step_duration = leg_duration/(len(decode_result)-1)
        for index,cord in enumerate(decode_result):
            if index == len(decode_result)-1:
                break#mean that finsished the build of the cost array for this leg
#the next section deals with the first cord after the last leg and the need to connect between the last leg and the new leg
#the cost per couple of edges will be registred only once since a already existing edge being registered will overide the old one
            path_time+=timedelta(seconds=step_duration)
            if out_set and index==0:
                cost[(out_set[-1],cord)]=[step_duration,path_time]
            cost[(cord,decode_result[index+1])]=[step_duration,path_time]
        # out_set.extend(decode(step['duration']))
        out_set.extend(decode_result)
    return out_set,cost

"""
The extractcostfromroute function will extract the route from the json and will create a list
with [(x,y),...] cordinates. it will also extract the cost function from the json 
"""
def extract_cost_and_route_from_route(json):
    out_set = []
    cost={}
    for step in json[0]['legs'][0]['steps']:
        leg_duration = step['duration']['value']
        decode_result=[]
        decode_result = decode(step['polyline']['points'])
        step_duration = leg_duration/(len(decode_result)-1)
        for index,cord in enumerate(decode_result):
            if index == len(decode_result)-1:
                break#mean that finsished the build of the cost array for this leg
#the next section deals with the first cord after the last leg and the need to connect between the last leg and the new leg
#the cost per couple of edges will be registred only once since a already existing edge being registered will overide the old one
            if out_set and index==0:
                cost[(out_set[-1],cord)]=step_duration
            cost[(cord,decode_result[index+1])]=step_duration
        # out_set.extend(decode(step['duration']))
        out_set.extend(decode_result)
    return out_set,cost


def decode(point_str):
    '''Decodes a polyline that has been encoded using Google's algorithm
    http://code.google.com/apis/maps/documentation/polylinealgorithm.html

    This is a generic method that returns a list of (latitude, longitude)
    tuples.

    :param point_str: Encoded polyline string.
    :type point_str: string
    :returns: List of 2-tuples where each tuple is (latitude, longitude)
    :rtype: list

    '''

    # sone coordinate offset is represented by 4 to 5 binary chunks
    coord_chunks = [[]]
    for char in point_str:

        # convert each character to decimal from ascii
        value = ord(char) - 63

        # values that have a chunk following have an extra 1 on the left
        split_after = not (value & 0x20)
        value &= 0x1F

        coord_chunks[-1].append(value)

        if split_after:
            coord_chunks.append([])

    del coord_chunks[-1]

    coords = []

    for coord_chunk in coord_chunks:
        coord = 0

        for i, chunk in enumerate(coord_chunk):
            coord |= chunk << (
            i * 5)  # there is a 1 on the right if the coord is negative if coord & 0x1: coord = ~coord #invert coord >>= 1
        # coord /= 100000.0
        if coord & 1:#means inverted a negative number
            coord = ~coord
        coord= coord>>1



        coord = (coord/1E5)

        coords.append(coord)

    # convert the 1 dimensional list to a 2 dimensional list and offsets to
    # actual values
    points = []
    prev_x = 0
    prev_y = 0
    for i in xrange(0, len(coords) - 1, 2):
        if coords[i] == 0 and coords[i + 1] == 0:
            continue

        prev_x += coords[i + 1]
        prev_y += coords[i]
        # a round to 6 digits ensures that the floats are the same as when
        # they were encoded
        points.append((round(prev_y, 6), round(prev_x, 6)))

    return points


def cordexists(cord, path):
    return path.has_key(cord)


def addcordtographmap(cord, adjacent):
    # adjacent={}
    adjacent[cord]=[]#means empty adjacent for now
    # id_graph[cord]

#todo 26-10 need to see that it handles well all sorts of extreme cases
#26-10 11:35 am added the update of the son with the father node
# def updatefatherandson(cord, path, adjacent,ids, path_id):
def updatefatherandson(cord, path, adjacent):
    # path=[]
    # ids={}
    index=path.index(cord)
    if index==0:
        return#incase of the first in the list
    # adjacent={}
    adjacent_value=adjacent.get(path[index-1])
    father = path[index - 1]
    # adjacent_value=[]
    if cord not in adjacent_value:
        # ids[(father,cord)]=path_id
        adjacent_value.append(cord)
        adjacent[path[index-1]]=adjacent_value
    #this is the son update
    adjacent_value = adjacent.get(path[index])

    if father not in adjacent_value:
        adjacent_value.append(father)
        adjacent[path[index]]=adjacent_value

#this is a temp version before adding the ids

# def updatefatherandson(cord, path, adjacent):
#     # path=[]
#     index = path.index(cord)
#     if index == 0:
#         return  # incase of the first in the list
#     # adjacent={}
#     adjacent_value = adjacent.get(path[index - 1])
#
#     # adjacent_value=[]
#     if cord not in adjacent_value:
#         adjacent_value.append(cord)
#         adjacent[path[index - 1]] = adjacent_value
#     # this is the son update
#     adjacent_value = adjacent.get(path[index])
#     father = path[index - 1]
#     if father not in adjacent_value:
#         adjacent_value.append(father)
#         adjacent[path[index]] = adjacent_value






    #this will update the son

"""
the function create_map_graph is responsible to make a joint path array meaning the standard representation of the 
routes map, but also it creates the costs dict which is a little bit different in term of the representaion of it
compared to the map graph
"""
def create_map_graph(paths,costs_array):
    #maybe don't need this line#make sets from the paths and find intersections, put it in a DS for checking later
    #go over each path and make a graph from the points encounterd if a point was already in the graph, adjust the adjacent of if
    #continue that way
    adjacent={}
    costs={}
    # id_graph={}
#assumption here is that the costs array and the paths have the same length.
    for path_index,path in enumerate(paths):
        costs.update(costs_array[path_index])
        for idx, cord in enumerate(path):
            if cordexists(cord,adjacent):
                # updatefatherandson(cord, path, adjacent,id_graph,path_index)
                updatefatherandson(cord, path, adjacent)
                continue
            addcordtographmap(cord,adjacent)
            # updatefatherandson(cord,path,adjacent,id_graph,path_index)
            updatefatherandson(cord, path, adjacent)
    return adjacent,costs





#example of parsed route
#[(69.6831, 64.23482), (69.68322, 64.23504), (69.68334, 64.23532), (69.6834, 64.23564), (69.6834, 64.23604), (69.68343, 64.2363), (69.68346, 64.23646), (69.68349, 64.23666), (69.68362, 64.2372), (69.68314, 64.2372), (69.68316, 64.23722), (69.68318, 64.23722), (69.6832, 64.23722), (69.6832, 64.23724), (69.68322, 64.23724), (69.68322, 64.23726), (69.68324, 64.23726), (69.68326, 64.23728), (69.68326, 64.2373), (69.68328, 64.2373), (69.68328, 64.23732), (69.68328, 64.23734), (69.68328, 64.23736), (69.6833, 64.23736), (69.6833, 64.23738), (69.68331, 64.2374), (69.68331, 64.23742), (69.68331, 64.23744), (69.68332, 64.23746), (69.68332, 64.23748), (69.68333, 64.23748), (69.68333, 64.2375), (69.68334, 64.2375), (69.68334, 64.23752), (69.68335, 64.23752), (69.68336, 64.23754), (69.68337, 64.23754), (69.68338, 64.23754), (69.68339, 64.23754), (69.68339, 64.23756), (69.6834, 64.23756), (69.68341, 64.23756), (69.68342, 64.23756), (69.68343, 64.23756), (69.68344, 64.23757), (69.68345, 64.23757), (69.68346, 64.23757), (69.68347, 64.23758), (69.68348, 64.23758), (69.68348, 64.23759), (69.68349, 64.23759), (69.68349, 64.2376), (69.6835, 64.2376), (69.6835, 64.23761), (69.68351, 64.23761), (69.68351, 64.23762), (69.68351, 64.23763), (69.68351, 64.23764), (69.68352, 64.23764), (69.68352, 64.23765), (69.68383, 64.23783), (69.68408, 64.23797), (69.68421, 64.23803), (69.68432, 64.23807), (69.68453, 64.23813), (69.68474, 64.23815), (69.68493, 64.23817), (69.6851, 64.23817), (69.68517, 64.23818), (69.68544, 64.23819), (69.68597, 64.23824), (69.68624, 64.23829), (69.68647, 64.23832), (69.68664, 64.23835), (69.68695, 64.23842), (69.6878, 64.23877), (69.68813, 64.23892), (69.68836, 64.23907), (69.68859, 64.23928), (69.68872, 64.23943), (69.68883, 64.23958), (69.68902, 64.23981), (69.68905, 64.23984), (69.6891, 64.23989), (69.68917, 64.23992), (69.68924, 64.23995), (69.68965, 64.24008), (69.6898, 64.24011), (69.69033, 64.24028), (69.69048, 64.24035), (69.6756, 64.2355), (69.67609, 64.23608), (69.67656, 64.23652), (69.67669, 64.23664), (69.67704, 64.23696), (69.67755, 64.2373), (69.67768, 64.23738), (69.67805, 64.23754), (69.67848, 64.2377), (69.67889, 64.23782), (69.67906, 64.23784), (69.67925, 64.23788), (69.67952, 64.23792), (69.68105, 64.23797), (69.68138, 64.23798), (69.68209, 64.23801), (69.66896, 64.2378), (69.66897, 64.23797), (69.66897, 64.23818), (69.66898, 64.23819), (69.66898, 64.23826), (69.66899, 64.23837), (69.66902, 64.23846), (69.66905, 64.23861), (69.6699, 64.24096), (69.67015, 64.24159), (69.6702, 64.24174), (69.67047, 64.24241), (69.67054, 64.24264), (69.67095, 64.24377), (69.67134, 64.24458), (69.67141, 64.24473), (69.67164, 64.2453), (69.67173, 64.24561), (69.66604, 64.22982), (69.66609, 64.22982), (69.6661, 64.22982), (69.66613, 64.22982), (69.66614, 64.22982), (69.66615, 64.22983), (69.66616, 64.22984), (69.66617, 64.22985), (69.66618, 64.22986), (69.66618, 64.22987), (69.66618, 64.22988), (69.66625, 64.23055), (69.66627, 64.23056), (69.66629, 64.23057), (69.66633, 64.23058), (69.66647, 64.23059), (69.66649, 64.2306), (69.66653, 64.2306), (69.66655, 64.23062), (69.66657, 64.23064), (69.66661, 64.2307), (69.66729, 64.23083), (69.66678, 64.22888), (69.66704, 64.22946)]

def getRoutesSet(str_json):
    # out_set = set()
    out_set=[]
    # out_set={}
    for step in str_json[0]['legs'][0]['steps']:
        out_set.extend(decode(step['polyline']['points']))
    return out_set

#this calculates the distance between 2 points with lat and long. it finds the best spherical route between them
#https://stackoverflow.com/questions/40357886/google-maps-v3-distance-between-two-points-different-values
#http://www.movable-type.co.uk/scripts/latlong.html
#on the 1-10 change the order of initialization and the latitude is first
class point:
    def __init__(self,latitude,longtitude):
        self.lat = latitude
        self.lng = longtitude


def rad(x):
    from math import pi
    return x * pi / 180

#the distnace returned is in meter checked on the 1-10 and worked
def getDistance(p1, p2):
    from math import sin,atan2,cos,sqrt
    R = 6378137
    dLat = rad(p2.lat - p1.lat)
    dLong = rad(p2.lng - p1.lng)
    a = sin(dLat / 2) * sin(dLat / 2) + cos(rad(p1.lat)) * cos(rad(p2.lat)) * sin(dLong / 2) * sin(dLong / 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    d = R * c
    return d # returns the distance in meter

def nearestPointinSet(targetset,point,threshold_distance_in_meters):
    targetset=set()
    min_point =None
    for item in targetset:
        temp_distance = getDistance(item,point)
        if temp_distance<threshold_distance_in_meters:
            if min_point is not None:
                if temp_distance<min_point:
                    min_point=temp_distance
            else:
                min_point = temp_distancee
    return min_point

"""
the next function the create_id_with_map will extract the graph map like a cost graph with the id of the related edges in it 
"""
def create_id_with_map(decoded_paths):
    # decoded_paths = []
    costs={}
    ids={}
    for id,path in enumerate(decoded_paths):
        for cord_id,cord in enumerate(path):
            if cord_id+1<len(path):
                if (cord,path[cord_id+1]) in ids:
                    ids[(cord, path[cord_id + 1])].append(id)
                else:
                    ids[(cord,path[cord_id+1])]=[id]
#todo need to see if the next section is necessary 30-10 it was commented becaues there is another funcion that extract the cost
                # if (cord,path[cord_id+1]) not in costs:
                #     costs[(cord,path[cord_id+1])]=0#todo need to add here the cost of this specific edge
    return ids
            #     if (cord,cord[cord_id+1]) in ids:
def get_points_with_same_path_id(ids,id):
    new_list = []#list of edges
    # for k, v in ids.iteritems():
    for k, v in ids.iteritems():
        # v=[]
        if id in v:
            new_list.append(k)
    return new_list

def get_points_with_same_path_id_new(ids, id,s,t):
    c = t
    path=[]
    # if id in ids[c][1:]:
    #     path.insert(0, ids[c])
    # path = [c]
    # lat = []
    # long = []
    # print 'min cost:', min_cost
    while ids.get(c):
        if id in ids[c][1:]:
            path.insert(0, ids[c])
        # lat.append(predecessors[c][0])
        # long.append(predecessors[c][1])
        c = ids[c][0]
    return path
    # new_list = []  # list of edges
    # # for k, v in ids.iteritems():
    # for k, v in ids.iteritems():
    #     # v=[]
    #     if id in v:
    #         new_list.append(k)
    # return new_list

 # while predecessors.get(c):
 #            path.insert(0, predecessors[c])
 #            lat.append(predecessors[c][0])
 #            long.append(predecessors[c][1])
 #            c = predecessors[c]







if __name__=='__main__':
    if 0:
        decodedr1 = [(2, 1), (3, 1), (3, 2), (3, 3), (4, 3), (4, 4)]
        cost1 = {((2, 1), (3, 1)): 12, ((3, 1), (3, 2)): 10, ((3, 2), (3, 3)): 4, ((3, 3), (4, 3)): 150,
                 ((4, 3), (4, 4)): 210}
        # decodedr2 = {(1, 2): [(2, 2)], (2, 2): [(2, 3)], (2, 3): [(3, 3)], (3, 3): [(3, 4)], (3, 4): [(4, 4)]}
        decodedr2 = [(1, 2), (2, 2), (2, 3), (3, 3), (3, 4), (4, 4)]
        cost2 = {((1, 2), (2, 2)): 12, ((2, 2), (2, 3)): 10, ((2, 3), (3, 3)): 4, ((3, 3), (3, 4)): 150,
                 ((3, 4), (4, 4)): 210}
        # decodedr3 = {(1, 4): [(1, 3)], (1, 3): [(2, 3)], (2, 3): [(2, 2)], (2, 2): [(3, 2)], (3, 2): [(3, 1)],(3,1):[(4,1)]}
        decodedr3 = [(1, 4), (1, 3), (2, 3), (2, 2), (3, 2), (3, 1), (4, 1)]
        cost3 = {((1, 4), (1, 3)): 12, ((1, 3), (2, 3)): 10, ((2, 3), (2, 2)): 4, ((2, 2), (3, 2)): 150,
                 ((3, 2), (3, 1)): 210, ((3, 1), (4, 1)): 4}
        decoded_paths_decoder = [decodedr1, decodedr2, decodedr3]
        costs_array = [cost1, cost2, cost3]
        adjacent, costs = create_map_graph(decoded_paths_decoder, costs_array)

    if 0:
        cord_list = [(32.114, 34.81675), (32.11384, 34.81673), (32.11366, 34.81671), (32.11366, 34.81669), (32.11369, 34.81631), (32.1137, 34.81617), (32.11375, 34.81534), (32.11375, 34.81506), (32.11375, 34.8149), (32.11375, 34.81406), (32.11375, 34.81351), (32.11426, 34.8135), (32.11471, 34.81344), (32.11486, 34.81401), (32.11505, 34.81456), (32.11524, 34.81494), (32.11528, 34.81502), (32.11564, 34.81554), (32.11594, 34.81589), (32.11601, 34.81577), (32.11637, 34.81538)]
        # encoded = '_p~iF~ps|U_ulLnnqC_mqNvxq`@'
        # decoded_path = decode(encoded)
        encoded_path= encode(cord_list)
        decoded = decode(encoded_path)
        route1=extractRouteSetFile('poly1')
        decoded1,cost1 = extract_cost_and_route_from_route(route1)
        from datetime import time,datetime
        starting_time = datetime.now()
        decoded1_test, cost1_with_time = extract_cost_and_time_and_route_from_route(route1,starting_time)

        # decoded1=getRoutesSet(route1)
        route2 = extractRouteSetFile('poly2')
        decoded2, cost2 = extract_cost_and_route_from_route(route2)
        # decoded2 = getRoutesSet(route2)
        route3 = extractRouteSetFile('poly3')
        decoded3, cost3 = extract_cost_and_route_from_route(route3)
        # decoded3 = getRoutesSet(route3)
        decoded_paths = [decoded1,decoded2,decoded3]
        decoded_costs = [cost1,cost2,cost3]
        adjacent,costs = create_map_graph(decoded_paths,decoded_costs)
        ids = create_id_with_map(decoded_paths)
        from MapAlgo import initializecost
        # cost = initializecost(adjacent,1)
        from MapAlgo import dijkstra
        s = (32.114, 34.81675)
        t = (32.11637, 34.81538)
        predecessors, min_cost,dijkstra_ids=dijkstra(adjacent,costs,s,t,ids)
        import matplotlib.pyplot as plt
        # x = predecessors
        # plt.plot([1, 2, 3, 4], [1, 4, 9, 16])
        print adjacent
        encoded_path = encode(cord_list)
        # ids = create_id_with_map(decoded_paths)
        path_of_id_0=get_points_with_same_path_id(dijkstra_ids,0)
        path_of_id_0_new = get_points_with_same_path_id_new(dijkstra_ids, 0,s,t)
        # get_points_with_same_path_id_new
        path_of_id_1 = get_points_with_same_path_id(dijkstra_ids, 1)
        path_of_id_1_new = get_points_with_same_path_id_new(dijkstra_ids, 1, s, t)
        path_of_id_2 = get_points_with_same_path_id(dijkstra_ids, 2)
        path_of_id_2_new = get_points_with_same_path_id_new(dijkstra_ids, 2, s, t)
        id_path=[]
        id_path_x=[]
        id_path_y=[]
        for v in path_of_id_0_new:
            id_path.append(v[0])
            id_path_x.append(v[0][0])
            id_path_y.append(v[0][1])
        plt.figure()
        plt.plot(id_path_y, id_path_x,'bo')
        id_path = []
        id_path_x = []
        id_path_y = []
        for v in path_of_id_1_new:
            id_path.append(v[0])
            id_path_x.append(v[0][0])
            id_path_y.append(v[0][1])
        plt.figure()
        plt.plot(id_path_y, id_path_x, 'bo')

        id_path = []
        id_path_x = []
        id_path_y = []
        # for v in path_of_id_2:
        #     id_path.append(v)
        #     id_path_x.append(v[0])
        #     id_path_y.append(v[1])
        for v in path_of_id_2_new:
            id_path.append(v[0])
            id_path_x.append(v[0][0])
            id_path_y.append(v[0][1])
        plt.figure()
        plt.plot(id_path_y, id_path_x, 'bo')


        # predecessors, min_cost = dijkstra(adj, cost, s, t)
        c = t
        path = [c]
        lat=[]
        long=[]
        print 'min cost:', min_cost
        while predecessors.get(c):
            path.insert(0, predecessors[c])
            lat.append(predecessors[c][0])
            long.append(predecessors[c][1])
            c = predecessors[c]
        plt.figure()
        # plt.plot(long, lat)
        plt.plot(long,lat, 'bo')
        print 'shortest path:', path
    if 1:
        raw_route_osm = [(32.11147792891708,34.815475459275845),(32.111468262995665,34.816038608551025),(32.11145008801211,34.81644630432129),(32.11095027454657,34.81638193130493),(32.11006877867545,34.81616735458374),(32.10790589707716,34.81492280960083),(32.10775140357391,34.814794063568115),(32.107660524920476,34.81490135192871),(32.10758782193261,34.81498718261719),(32.10740606420968,34.8152232170105),(32.106951668319496,34.815802574157715),(32.106915316550605,34.81584548950195),(32.106415478260736,34.81642484664917),(32.10606104581547,34.81739044189453),(32.10593381332007,34.81848478317261),(32.1059247252779,34.819514751434326),(32.1059247252779,34.820287227630615),(32.10593381332007,34.820566177368164),(32.10602469369202,34.82136011123657),(32.106206454164564,34.82266902923584),(32.10660632593067,34.82494354248047),(32.106651765793295,34.8252010345459),(32.10666994173201,34.825286865234375),(32.10698802007393,34.827046394348145),(32.10696075625947,34.827260971069336),(32.10694258037863,34.827303886413574),(32.10688805271443,34.82738971710205),(32.106806261157075,34.82745409011841),(32.10667902970002,34.82747554779053),(32.106569974024296,34.82749700546265),(32.10642456625406,34.82749700546265),(32.10626098223578,34.827518463134766),(32.10587019700589,34.827561378479004),(32.105461233927976,34.82760429382324),(32.105379441092595,34.82762575149536),(32.10504318088861,34.82764720916748),(32.10485232995175,34.82764720916748),(32.10469783128215,34.82764720916748),(32.104207069068444,34.82764720916748),(32.10343456764833,34.82769012451172),(32.1032164484173,34.82769012451172),(32.10224399384034,34.827754497528076),(32.102116756026845,34.827775955200195),(32.101862279868065,34.827797412872314),(32.1017804838093,34.827797412872314),(32.1015623606278,34.82784032821655),(32.10124426338764,34.82788324356079),(32.100789836837,34.82796907424927),(32.100535356981055,34.828011989593506),(32.10043538255795,34.828033447265625),(32.10019907894103,34.82809782028198),(32.09920841481621,34.82833385467529),(32.09906299532471,34.828290939331055),(32.0990084629557,34.8282265663147),(32.09893575307973,34.82816219329834),(32.098826688157246,34.827754497528076),(32.09848131504356,34.826531410217285),(32.098399515956935,34.82623100280762),(32.09837224957845,34.82614517211914),(32.09828136159138,34.825801849365234),(32.09763605428278,34.823527336120605),(32.0974997211695,34.823055267333984),(32.097454276753176,34.82290506362915),(32.09721796542391,34.82208967208862),(32.09717252086742,34.82193946838379),(32.0971179873698,34.821810722351074),(32.09704527598898,34.82168197631836),(32.0969725645503,34.821574687957764),(32.09681805255093,34.82140302658081),(32.09668171821679,34.82146739959717),(32.09647267184259,34.821574687957764),(32.09635451498469,34.82159614562988),(32.09627271399355,34.821617603302),(32.09587279698238,34.82166051864624),(32.095427432795844,34.8217248916626),(32.09520929444291,34.821789264678955),(32.09414586251328,34.822025299072266),(32.093773203950825,34.82208967208862),(32.09367322212666,34.82211112976074),(32.09265521914292,34.822304248809814),(32.09237344845561,34.82236862182617),(32.091946246078635,34.82245445251465),(32.09095549245261,34.8226261138916),(32.09081914936955,34.8226261138916),(32.0906646269628,34.8226261138916),(32.09050101471787,34.8226261138916),(32.09013743090247,34.82260465621948),(32.08868308117335,34.822518825531006),(32.088455836937186,34.822518825531006),(32.08785590943842,34.822518825531006),(32.0867651220772,34.82236862182617),(32.08615609346964,34.822261333465576),(32.085483430290545,34.82215404510498),(32.085310718675466,34.82213258743286),(32.08489257341356,34.822068214416504),(32.084838032586184,34.822046756744385),(32.084601688624744,34.82189655303955),(32.084283532326864,34.8217248916626),(32.08376538969968,34.82140302658081),(32.08362903588851,34.82133865356445),(32.082274576984304,34.8206090927124),(32.08173824207194,34.82041597366333),(32.08033830221842,34.81990098953247),(32.080292849266414,34.820008277893066),(32.08068374391515,34.8201584815979),(32.08117463250158,34.82033014297485)]
        encoded_path = encode(raw_route_osm)



    decodedr1 = [(2, 1), (3, 1), (3, 2), (3, 3), (4, 3), (4, 4)]
    # decodedr2 = {(1, 2): [(2, 2)], (2, 2): [(2, 3)], (2, 3): [(3, 3)], (3, 3): [(3, 4)], (3, 4): [(4, 4)]}
    decodedr2 = [(1, 2), (2, 2), (2, 3), (3, 3), (3, 4), (4, 4)]
    # decodedr3 = {(1, 4): [(1, 3)], (1, 3): [(2, 3)], (2, 3): [(2, 2)], (2, 2): [(3, 2)], (3, 2): [(3, 1)],(3,1):[(4,1)]}
    decodedr3 = [(1, 4), (1, 3), (2, 3), (2, 2), (3, 2), (3, 1), (4, 1)]
    decoder4 = [(2, 1), (3, 1), (3, 2), (3, 3), (4, 3), (3, 3)]
    decoded_paths_decoder = [decodedr1, decodedr2, decodedr3,decoder4]
    ids = create_id_with_map(decoded_paths_decoder)
    adjacent= create_map_graph(decoded_paths_decoder)
    # adjacent list
    adj =   adjacent

    # edge costs
    cost = {(1, 2): 7,
            (1, 3): 9,
            (1, 6): 14,
            (2, 3): 10,
            (2, 4): 15,
            (3, 4): 11,
            (3, 6): 2,
            (4, 5): 6,
            (5, 6): 9,
            (4, 7): 2,
            (5, 7): 1,
            (6, 7): 12}

    # cost = make_undirected(cost)

    s, t = 1, 7  # start and stop
    predecessors, min_cost = dijkstra(adj, cost, s, t)
    c = t
    path = [c]
    print 'min cost:', min_cost
    while predecessors.get(c):
        path.insert(0, predecessors[c])
        c = predecessors[c]

    print 'shortest path:', path






    # decoded_paths.append(decoded1)
    # decoded_paths.append(decoded2)
    # decoded_paths.append(decoded3)


    # decoded_paths_decoder = [decoded1, decoded2, decoded3]
    # getRoutesSet(wholejson)
    encoded = '_p~iF~ps|U_ulLnnqC_mqNvxq`@'
    decoded=decode(encoded)
    #[(38.5, -120.2), (40.7, -120.95), (43.252, -126.453)]
    # decodedr1={(2,1):[(3,1)],(3,1):[(3,2)],(3,2):[(3,3)],(3,3):[(4,3)],(4,3):[(4,4)]}


    r1='snbcEosqsEHuAPcAfBvA|AnA`B|ArBrBnJrJpJlJ@FTZz@x@dCzAxAv@h@^XZb@l@zAtC`@|@~CpGbAbB`@d@`BvA`BhAxA~@nCzAXLfCfAh@PzFdBbBXpFt@`DVrGXvETdBRnAT`AVjC`AxC`B~B~AjA`AxAlAt@`Ad@j@|AjBnBnCt@lAp@pAlBbEnAlDh@zA~AlF~@rDFVG|B_@HqARSq@JEFEBI?ME]GY^W'
    r2='us`cEqhlsEQCKOKYAs@pA}LHa@x@gE|ByI|A{Gn@uCLaALaALoBJsCD_BVcRFeEN}BPuALk@FCNk@DGFCr@AnAh@h@PzFdBdC`@nEl@`DVrJb@xDVb@FnAT`AVjC`A^PxBnAnBnANNRU~BkCv@{@x@iAVe@Vs@`@uBZkDH{ADeDAkCAkAi@gI[mD_AyIGiAAMOiA}@uIGm@?{@'
    #Raw east to west maoz aviv 22-10
    poly1 = '}e`cEknnsEPgLFeEN}BPuALk@FCNk@DGFCr@AnAh@h@PzFdBdC`@nEl@`DVrJb@xDVBg@Xy@`@aAdAmBb@iA'
    #Raw 22-10 14:21 south to north on bnei efraim
    poly2 = 'um`cEghmsEtBuIt@eDPkAPqAHiANiCBsAXsSN_HPkBLs@v@{CoBcA}Ay@'

    #22-10 14:22 routes generall sout to north
    poly3= 'iq}bEognsEcCsDaAmAoAyAsBiBc@kAAWFo@`AoAj@m@n@w@n@eAh@iB^sCN{BJ_EAkEGoBq@}ImAkLk@qEI]e@yDMgAIm@Ag@B}@RwDd@qDh@iHDkAAwAEkAIu@Mm@Sy@_@cA'
    #poly4 is only for testing

    # poly4 = '_p~iF~ps|U_ulLnnqC_mqNvxq`@'
    poly_paths = [poly1,poly2,poly3]
    decoded_paths=[]
    for pol in poly_paths:
        decoded_paths.append(decode(pol))
    # adjacent= createMapGraph(decoded_paths)
    adjacent = create_map_graph(decoded_paths)
    print adjacent
#todo  11-10 seems like the overview polyline is includes the route but not sure yet if it includes all points. seems interesting, that we have also a 'overview_polyline' json enty
#to1do-finished need to check if the getDistnace function works it works
#to1do the creation of a set from a list of points seems to work. need to create 2 routes that have intesection from android debug window and parse the json see if it is fine.
#to1do (created 2 files for that, on N2S north to south and W2E)then create 2 sets one with the first route and a second with the second route and check of intersection between them see if it workds 2-10 works see the doc with the running lines
#todo then try to also add the predicate by distance and see if it works and the intersection now has more options
#todo added the nearestPointinSet given a point a set and a threshold will give you the minimal point in the set near the point
#todo just checked the cpython code itself for the set class it uses key and hash for intersection so it mean need to overide the interecation
#todo continue if we want to use the near point function use the dir() python function to check the names of the class being inported
#todo 2-10 the intersection worked and got 3 points intersect. need to see if a distance predicate can help here and how if i can implemet it in a set

# #js[0]['legs'][0]['steps'][0]['polyline']['points']
#1-10 old FindRoute.decode('gd`cEy~ssEAA?A?AA??AA??AAAA??AA?A?A??AA?A@A?A?A@A??@A??@A??@A@?@?@?@A??@?@?@?@@@?@?@@@?@@??@@??@@??@@?@?@??@@?Q^MXELCJETATAR?P@F@ZDt@DZBVBPF^b@tAN`@NVTVNLNJVRBBDDBFBFLh@BNPt@FN')
#todo seems that trunckating the steps worked now need to check if the start and stop points are inside the route. found a good function and wrote it to find a distance between 2 points with latitude and longtitude

#[(69.68314, 64.2372), (69.68316, 64.23722), (69.68318, 64.23722), (69.6832, 64.23722), (69.6832, 64.23724), (69.68322, 64.23724), (69.68322, 64.23726), (69.68324, 64.23726), (69.68326, 64.23728), (69.68326, 64.2373), (69.68328, 64.2373), (69.68328, 64.23732), (69.68328, 64.23734), (69.68328, 64.23736), (69.6833, 64.23736), (69.6833, 64.23738), (69.68331, 64.2374), (69.68331, 64.23742), (69.68331, 64.23744), (69.68332, 64.23746), (69.68332, 64.23748), (69.68333, 64.23748), (69.68333, 64.2375), (69.68334, 64.2375), (69.68334, 64.23752), (69.68335, 64.23752), (69.68336, 64.23754), (69.68337, 64.23754), (69.68338, 64.23754), (69.68339, 64.23754), (69.68339, 64.23756), (69.6834, 64.23756), (69.68341, 64.23756), (69.68342, 64.23756), (69.68343, 64.23756), (69.68344, 64.23757), (69.68345, 64.23757), (69.68346, 64.23757), (69.68347, 64.23758), (69.68348, 64.23758), (69.68348, 64.23759), (69.68349, 64.23759), (69.68349, 64.2376), (69.6835, 64.2376), (69.6835, 64.23761), (69.68351, 64.23761), (69.68351, 64.23762), (69.68351, 64.23763), (69.68351, 64.23764), (69.68352, 64.23764), (69.68352, 64.23765), (69.68383, 64.23783), (69.68408, 64.23797), (69.68421, 64.23803), (69.68432, 64.23807), (69.68453, 64.23813), (69.68474, 64.23815), (69.68493, 64.23817), (69.6851, 64.23817), (69.68517, 64.23818), (69.68544, 64.23819), (69.68597, 64.23824), (69.68624, 64.23829), (69.68647, 64.23832), (69.68664, 64.23835), (69.68695, 64.23842), (69.6878, 64.23877), (69.68813, 64.23892), (69.68836, 64.23907), (69.68859, 64.23928), (69.68872, 64.23943), (69.68883, 64.23958), (69.68902, 64.23981), (69.68905, 64.23984), (69.6891, 64.23989), (69.68917, 64.23992), (69.68924, 64.23995), (69.68965, 64.24008), (69.6898, 64.24011), (69.69033, 64.24028), (69.69048, 64.24035)]
#todo need check for routes with start and stop that answers the query and after that merge the routes after decoding and find suitable combined route
# possilbe algorithm. first check routes that contain start or/end points
# the start and end point should be
#http://tainguyen.me/blog/python-encode-and-decode-polylines-from-google-direction-api/
#fp=open('routejson')
#fp
#<open file 'routejson', mode 'r' at 0x7f41605ede40>
#import json
#json_route=json.load(fp)
#FindRoute.getRoutesSet(json_route)
#[(69.6831, 64.23482), (69.68322, 64.23504), (69.68334, 64.23532), (69.6834, 64.23564), (69.6834, 64.23604), (69.68343, 64.2363), (69.68346, 64.23646), (69.68349, 64.23666), (69.68362, 64.2372), (69.68314, 64.2372), (69.68316, 64.23722), (69.68318, 64.23722), (69.6832, 64.23722), (69.6832, 64.23724), (69.68322, 64.23724), (69.68322, 64.23726), (69.68324, 64.23726), (69.68326, 64.23728), (69.68326, 64.2373), (69.68328, 64.2373), (69.68328, 64.23732), (69.68328, 64.23734), (69.68328, 64.23736), (69.6833, 64.23736), (69.6833, 64.23738), (69.68331, 64.2374), (69.68331, 64.23742), (69.68331, 64.23744), (69.68332, 64.23746), (69.68332, 64.23748), (69.68333, 64.23748), (69.68333, 64.2375), (69.68334, 64.2375), (69.68334, 64.23752), (69.68335, 64.23752), (69.68336, 64.23754), (69.68337, 64.23754), (69.68338, 64.23754), (69.68339, 64.23754), (69.68339, 64.23756), (69.6834, 64.23756), (69.68341, 64.23756), (69.68342, 64.23756), (69.68343, 64.23756), (69.68344, 64.23757), (69.68345, 64.23757), (69.68346, 64.23757), (69.68347, 64.23758), (69.68348, 64.23758), (69.68348, 64.23759), (69.68349, 64.23759), (69.68349, 64.2376), (69.6835, 64.2376), (69.6835, 64.23761), (69.68351, 64.23761), (69.68351, 64.23762), (69.68351, 64.23763), (69.68351, 64.23764), (69.68352, 64.23764), (69.68352, 64.23765), (69.68383, 64.23783), (69.68408, 64.23797), (69.68421, 64.23803), (69.68432, 64.23807), (69.68453, 64.23813), (69.68474, 64.23815), (69.68493, 64.23817), (69.6851, 64.23817), (69.68517, 64.23818), (69.68544, 64.23819), (69.68597, 64.23824), (69.68624, 64.23829), (69.68647, 64.23832), (69.68664, 64.23835), (69.68695, 64.23842), (69.6878, 64.23877), (69.68813, 64.23892), (69.68836, 64.23907), (69.68859, 64.23928), (69.68872, 64.23943), (69.68883, 64.23958), (69.68902, 64.23981), (69.68905, 64.23984), (69.6891, 64.23989), (69.68917, 64.23992), (69.68924, 64.23995), (69.68965, 64.24008), (69.6898, 64.24011), (69.69033, 64.24028), (69.69048, 64.24035), (69.6756, 64.2355), (69.67609, 64.23608), (69.67656, 64.23652), (69.67669, 64.23664), (69.67704, 64.23696), (69.67755, 64.2373), (69.67768, 64.23738), (69.67805, 64.23754), (69.67848, 64.2377), (69.67889, 64.23782), (69.67906, 64.23784), (69.67925, 64.23788), (69.67952, 64.23792), (69.68105, 64.23797), (69.68138, 64.23798), (69.68209, 64.23801), (69.66896, 64.2378), (69.66897, 64.23797), (69.66897, 64.23818), (69.66898, 64.23819), (69.66898, 64.23826), (69.66899, 64.23837), (69.66902, 64.23846), (69.66905, 64.23861), (69.6699, 64.24096), (69.67015, 64.24159), (69.6702, 64.24174), (69.67047, 64.24241), (69.67054, 64.24264), (69.67095, 64.24377), (69.67134, 64.24458), (69.67141, 64.24473), (69.67164, 64.2453), (69.67173, 64.24561), (69.66604, 64.22982), (69.66609, 64.22982), (69.6661, 64.22982), (69.66613, 64.22982), (69.66614, 64.22982), (69.66615, 64.22983), (69.66616, 64.22984), (69.66617, 64.22985), (69.66618, 64.22986), (69.66618, 64.22987), (69.66618, 64.22988), (69.66625, 64.23055), (69.66627, 64.23056), (69.66629, 64.23057), (69.66633, 64.23058), (69.66647, 64.23059), (69.66649, 64.2306), (69.66653, 64.2306), (69.66655, 64.23062), (69.66657, 64.23064), (69.66661, 64.2307), (69.66729, 64.23083), (69.66678, 64.22888), (69.66704, 64.22946)]
#p1 = FindRoute.point(64.23482,69.6831)
#p2 = FindRoute.point(64.23504,69.68322)
#FindRoute.getDistance(p1,p2)
#15.83513205271976

#import sets
#route_list = [(69.6831, 64.23482), (69.68322, 64.23504), (69.68334, 64.23532), (69.6834, 64.23564), (69.6834, 64.23604), (69.68343, 64.2363), (69.68346, 64.23646), (69.68349, 64.23666), (69.68362, 64.2372), (69.68314, 64.2372), (69.68316, 64.23722), (69.68318, 64.23722), (69.6832, 64.23722), (69.6832, 64.23724), (69.68322, 64.23724), (69.68322, 64.23726), (69.68324, 64.23726), (69.68326, 64.23728), (69.68326, 64.2373), (69.68328, 64.2373), (69.68328, 64.23732), (69.68328, 64.23734), (69.68328, 64.23736), (69.6833, 64.23736), (69.6833, 64.23738), (69.68331, 64.2374), (69.68331, 64.23742), (69.68331, 64.23744), (69.68332, 64.23746), (69.68332, 64.23748), (69.68333, 64.23748), (69.68333, 64.2375), (69.68334, 64.2375), (69.68334, 64.23752), (69.68335, 64.23752), (69.68336, 64.23754), (69.68337, 64.23754), (69.68338, 64.23754), (69.68339, 64.23754), (69.68339, 64.23756), (69.6834, 64.23756), (69.68341, 64.23756), (69.68342, 64.23756), (69.68343, 64.23756), (69.68344, 64.23757), (69.68345, 64.23757), (69.68346, 64.23757), (69.68347, 64.23758), (69.68348, 64.23758), (69.68348, 64.23759), (69.68349, 64.23759), (69.68349, 64.2376), (69.6835, 64.2376), (69.6835, 64.23761), (69.68351, 64.23761), (69.68351, 64.23762), (69.68351, 64.23763), (69.68351, 64.23764), (69.68352, 64.23764), (69.68352, 64.23765), (69.68383, 64.23783), (69.68408, 64.23797), (69.68421, 64.23803), (69.68432, 64.23807), (69.68453, 64.23813), (69.68474, 64.23815), (69.68493, 64.23817), (69.6851, 64.23817), (69.68517, 64.23818), (69.68544, 64.23819), (69.68597, 64.23824), (69.68624, 64.23829), (69.68647, 64.23832), (69.68664, 64.23835), (69.68695, 64.23842), (69.6878, 64.23877), (69.68813, 64.23892), (69.68836, 64.23907), (69.68859, 64.23928), (69.68872, 64.23943), (69.68883, 64.23958), (69.68902, 64.23981), (69.68905, 64.23984), (69.6891, 64.23989), (69.68917, 64.23992), (69.68924, 64.23995), (69.68965, 64.24008), (69.6898, 64.24011), (69.69033, 64.24028), (69.69048, 64.24035), (69.6756, 64.2355), (69.67609, 64.23608), (69.67656, 64.23652), (69.67669, 64.23664), (69.67704, 64.23696), (69.67755, 64.2373), (69.67768, 64.23738), (69.67805, 64.23754), (69.67848, 64.2377), (69.67889, 64.23782), (69.67906, 64.23784), (69.67925, 64.23788), (69.67952, 64.23792), (69.68105, 64.23797), (69.68138, 64.23798), (69.68209, 64.23801), (69.66896, 64.2378), (69.66897, 64.23797), (69.66897, 64.23818), (69.66898, 64.23819), (69.66898, 64.23826), (69.66899, 64.23837), (69.66902, 64.23846), (69.66905, 64.23861), (69.6699, 64.24096), (69.67015, 64.24159), (69.6702, 64.24174), (69.67047, 64.24241), (69.67054, 64.24264), (69.67095, 64.24377), (69.67134, 64.24458), (69.67141, 64.24473), (69.67164, 64.2453), (69.67173, 64.24561), (69.66604, 64.22982), (69.66609, 64.22982), (69.6661, 64.22982), (69.66613, 64.22982), (69.66614, 64.22982), (69.66615, 64.22983), (69.66616, 64.22984), (69.66617, 64.22985), (69.66618, 64.22986), (69.66618, 64.22987), (69.66618, 64.22988), (69.66625, 64.23055), (69.66627, 64.23056), (69.66629, 64.23057), (69.66633, 64.23058), (69.66647, 64.23059), (69.66649, 64.2306), (69.66653, 64.2306), (69.66655, 64.23062), (69.66657, 64.23064), (69.66661, 64.2307), (69.66729, 64.23083), (69.66678, 64.22888), (69.66704, 64.22946)]
#route_list
#[(69.6831, 64.23482), (69.68322, 64.23504), (69.68334, 64.23532), (69.6834, 64.23564), (69.6834, 64.23604), (69.68343, 64.2363), (69.68346, 64.23646), (69.68349, 64.23666), (69.68362, 64.2372), (69.68314, 64.2372), (69.68316, 64.23722), (69.68318, 64.23722), (69.6832, 64.23722), (69.6832, 64.23724), (69.68322, 64.23724), (69.68322, 64.23726), (69.68324, 64.23726), (69.68326, 64.23728), (69.68326, 64.2373), (69.68328, 64.2373), (69.68328, 64.23732), (69.68328, 64.23734), (69.68328, 64.23736), (69.6833, 64.23736), (69.6833, 64.23738), (69.68331, 64.2374), (69.68331, 64.23742), (69.68331, 64.23744), (69.68332, 64.23746), (69.68332, 64.23748), (69.68333, 64.23748), (69.68333, 64.2375), (69.68334, 64.2375), (69.68334, 64.23752), (69.68335, 64.23752), (69.68336, 64.23754), (69.68337, 64.23754), (69.68338, 64.23754), (69.68339, 64.23754), (69.68339, 64.23756), (69.6834, 64.23756), (69.68341, 64.23756), (69.68342, 64.23756), (69.68343, 64.23756), (69.68344, 64.23757), (69.68345, 64.23757), (69.68346, 64.23757), (69.68347, 64.23758), (69.68348, 64.23758), (69.68348, 64.23759), (69.68349, 64.23759), (69.68349, 64.2376), (69.6835, 64.2376), (69.6835, 64.23761), (69.68351, 64.23761), (69.68351, 64.23762), (69.68351, 64.23763), (69.68351, 64.23764), (69.68352, 64.23764), (69.68352, 64.23765), (69.68383, 64.23783), (69.68408, 64.23797), (69.68421, 64.23803), (69.68432, 64.23807), (69.68453, 64.23813), (69.68474, 64.23815), (69.68493, 64.23817), (69.6851, 64.23817), (69.68517, 64.23818), (69.68544, 64.23819), (69.68597, 64.23824), (69.68624, 64.23829), (69.68647, 64.23832), (69.68664, 64.23835), (69.68695, 64.23842), (69.6878, 64.23877), (69.68813, 64.23892), (69.68836, 64.23907), (69.68859, 64.23928), (69.68872, 64.23943), (69.68883, 64.23958), (69.68902, 64.23981), (69.68905, 64.23984), (69.6891, 64.23989), (69.68917, 64.23992), (69.68924, 64.23995), (69.68965, 64.24008), (69.6898, 64.24011), (69.69033, 64.24028), (69.69048, 64.24035), (69.6756, 64.2355), (69.67609, 64.23608), (69.67656, 64.23652), (69.67669, 64.23664), (69.67704, 64.23696), (69.67755, 64.2373), (69.67768, 64.23738), (69.67805, 64.23754), (69.67848, 64.2377), (69.67889, 64.23782), (69.67906, 64.23784), (69.67925, 64.23788), (69.67952, 64.23792), (69.68105, 64.23797), (69.68138, 64.23798), (69.68209, 64.23801), (69.66896, 64.2378), (69.66897, 64.23797), (69.66897, 64.23818), (69.66898, 64.23819), (69.66898, 64.23826), (69.66899, 64.23837), (69.66902, 64.23846), (69.66905, 64.23861), (69.6699, 64.24096), (69.67015, 64.24159), (69.6702, 64.24174), (69.67047, 64.24241), (69.67054, 64.24264), (69.67095, 64.24377), (69.67134, 64.24458), (69.67141, 64.24473), (69.67164, 64.2453), (69.67173, 64.24561), (69.66604, 64.22982), (69.66609, 64.22982), (69.6661, 64.22982), (69.66613, 64.22982), (69.66614, 64.22982), (69.66615, 64.22983), (69.66616, 64.22984), (69.66617, 64.22985), (69.66618, 64.22986), (69.66618, 64.22987), (69.66618, 64.22988), (69.66625, 64.23055), (69.66627, 64.23056), (69.66629, 64.23057), (69.66633, 64.23058), (69.66647, 64.23059), (69.66649, 64.2306), (69.66653, 64.2306), (69.66655, 64.23062), (69.66657, 64.23064), (69.66661, 64.2307), (69.66729, 64.23083), (69.66678, 64.22888), (69.66704, 64.22946)]
#set(route_list)
#set([(69.67889, 64.23782), (69.6835, 64.2376), (69.66897, 64.23797), (69.6891, 64.23989), (69.68917, 64.23992), (69.68335, 64.23752), (69.6832, 64.23724), (69.68342, 64.23756), (69.66629, 64.23057), (69.67669, 64.23664), (69.68348, 64.23758), (69.68328, 64.23734), (69.68343, 64.23756), (69.68351, 64.23762), (69.68883, 64.23958), (69.68905, 64.23984), (69.68316, 64.23722), (69.68339, 64.23756), (69.68493, 64.23817), (69.67925, 64.23788), (69.68647, 64.23832), (69.68517, 64.23818), (69.67054, 64.24264), (69.68336, 64.23754), (69.66649, 64.2306), (69.68339, 64.23754), (69.68859, 64.23928), (69.6833, 64.23738), (69.6834, 64.23756), (69.68924, 64.23995), (69.6834, 64.23604), (69.66653, 64.2306), (69.68348, 64.23759), (69.6878, 64.23877), (69.68326, 64.23728), (69.68349, 64.23666), (69.68318, 64.23722), (69.66898, 64.23819), (69.68341, 64.23756), (69.6661, 64.22982), (69.6756, 64.2355), (69.68326, 64.2373), (69.68209, 64.23801), (69.68344, 64.23757), (69.68337, 64.23754), (69.6699, 64.24096), (69.67768, 64.23738), (69.67047, 64.24241), (69.68836, 64.23907), (69.68544, 64.23819), (69.68664, 64.23835), (69.68383, 64.23783), (69.66627, 64.23056), (69.67015, 64.24159), (69.6833, 64.23736), (69.66625, 64.23055), (69.68328, 64.2373), (69.66655, 64.23062), (69.68349, 64.23759), (69.67755, 64.2373), (69.68343, 64.2363), (69.67164, 64.2453), (69.68695, 64.23842), (69.68346, 64.23646), (69.66618, 64.22988), (69.68432, 64.23807), (69.69033, 64.24028), (69.66899, 64.23837), (69.66616, 64.22984), (69.66618, 64.22986), (69.68331, 64.2374), (69.68474, 64.23815), (69.6835, 64.23761), (69.68965, 64.24008), (69.67609, 64.23608), (69.68322, 64.23724), (69.67134, 64.24458), (69.68332, 64.23746), (69.66657, 64.23064), (69.68421, 64.23803), (69.67704, 64.23696), (69.66617, 64.22985), (69.68322, 64.23726), (69.68334, 64.23752), (69.68332, 64.23748), (69.68333, 64.2375), (69.66704, 64.22946), (69.67656, 64.23652), (69.68408, 64.23797), (69.68334, 64.2375), (69.68352, 64.23765), (69.67848, 64.2377), (69.68347, 64.23758), (69.68333, 64.23748), (69.66897, 64.23818), (69.68813, 64.23892), (69.66615, 64.22983), (69.67906, 64.23784), (69.68322, 64.23504), (69.68324, 64.23726), (69.68331, 64.23742), (69.66905, 64.23861), (69.68334, 64.23532), (69.69048, 64.24035), (69.66633, 64.23058), (69.66896, 64.2378), (69.66729, 64.23083), (69.66613, 64.22982), (69.6898, 64.24011), (69.68314, 64.2372), (69.66604, 64.22982), (69.68351, 64.23761), (69.68346, 64.23757), (69.66609, 64.22982), (69.6834, 64.23564), (69.66678, 64.22888), (69.68345, 64.23757), (69.68597, 64.23824), (69.6851, 64.23817), (69.66661, 64.2307), (69.6702, 64.24174), (69.66647, 64.23059), (69.68338, 64.23754), (69.68331, 64.23744), (69.68351, 64.23764), (69.68351, 64.23763), (69.66898, 64.23826), (69.67095, 64.24377), (69.67141, 64.24473), (69.68352, 64.23764), (69.68624, 64.23829), (69.68362, 64.2372), (69.68328, 64.23732), (69.68453, 64.23813), (69.68872, 64.23943), (69.66902, 64.23846), (69.68902, 64.23981), (69.6832, 64.23722), (69.68328, 64.23736), (69.67952, 64.23792), (69.67173, 64.24561), (69.67805, 64.23754), (69.68105, 64.23797), (69.6831, 64.23482), (69.66618, 64.22987), (69.68349, 64.2376), (69.68138, 64.23798), (69.66614, 64.22982)])

