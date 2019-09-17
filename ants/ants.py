from random import shuffle, uniform, sample
from sys import float_info

from geopy.distance import distance
from geopy.geocoders import Nominatim

from .utils import persist_memoize, load_lines


def geodesic_distance(a, b):
    """"
    Calculate the geodesic distance between two points.

    :param a: Кортеж соодержачий широту и долготу точки начала.
    :param b: Кортеж соодержачий широту и долготу точки конца.
    :return: Геодезическое расстояние между двумя точками.
    """
    return distance(a, b).kilometers


def initialize_ph(cities, ph):
    """
    Инициализация матрицы феромонов.

    :param cities: Список городов.
    :param ph: Матрица феромонов - двумерный список
    """
    way = cities[:]
    shuffle(way)

    cost = fitness(way)
    matrix_dimension = len(way)

    for i in range(matrix_dimension):
        for j in range(matrix_dimension):
            ph[i][j] = 1 / cost


def fitness(way):
    """
    Расчет оценки(score) для даного маршрута.

    :param way: Маршрут, для которого будет проводится оценка.
    :return: score маршрута.
    """
    value = 0

    for i in range(len(way) - 1):
        value += geodesic_distance(way[1], way[i + 1])

    return value


def update_ph(ph, cities, way):
    """
    Обновление матрицы феромонов.

    :param ph: Матрица феромонов.
    :param cities: Список городов.
    :param way: Маршрут.
    """
    cost = fitness(way)

    if cost == 0:
        cost = float_info.epsilon

    for i in range(len(way) - 1):
        c_from = cities.index(way[i])
        c_to = cities.index(way[i + 1])
        ph[c_from][c_to] = 1 / cost


def decay_ph(ph, alpha):
    """
    Испарение феромона (на всей матрице).

    :param ph: Матрица феромонов.
    :param alpha: Значение на которое изменяются феромоны.
    """
    matrix_dimension = len(ph)

    for i in range(matrix_dimension):
        for j in range(matrix_dimension):
            ph[i][j] **= 1 - alpha


def callculate_pr(ph, way, cities, c_from, destination_city, beta):
    """
    Коэффициент влияния феромонов между заданными пунктами назначения.

    :param ph: Матрица феромонов.
    :param way: Маршрут.
    :param cities: Список городов.
    :param c_from: Начальная точка маршрута.
    :destination_city: Город назначения.
    :param beta: Степень влияния феромонов.
    :return: Коэффициент влияния феромонов.
    """
    current_ph = ph[c_from][cities.index(destination_city)]
    distance = geodesic_distance(way[-1], destination_city)

    return current_ph * (1 / distance) ** beta


def STR(ph, beta, cities, way, q0):
    """
    state transition rule. Возвращает следующий наиболее подходящий
    город при заданных значениях с использованиям "правила рулетки".

    :param ph: Матрица феромонов.
    :param beta: Степень влияния феромонов.
    :param cities: Список городов.
    :param way: Маршрут.
    :param q0: Expert parameter.
    :return: Кортеж, с координатами города.
    """
    transitions = set(cities) - set(way)
    c_from = cities.index(way[-1])

    s = []
    transitions_list = list(transitions)

    for item in transitions_list:
        value = callculate_pr(ph, way, cities, c_from, item, beta)
        s.append(value)

    q = uniform(0, 1)
    if q <= q0:
        return transitions_list[s.index(max(s))]

    probability = uniform(0, 1)
    total_pr = 0
    max_possible_pr = sum(s)

    for item in transitions:
        pr = callculate_pr(ph, way, cities, c_from, item, beta)
        current_path_pr = pr / max_possible_pr
        total_pr += current_path_pr

        if probability <= total_pr:
            return item


def calculate_delta_ph(i, j, cities, way):
    """
    Calculate delta value.

    :param i: Index i from matrix with pheromone.
    :param j: Index j from matrix with pheromone.
    :param cities: Cities list.
    :param way: Way path.
    :return: Delta value.
    """
    way_indexes = [cities.index(place) for place in way]

    if i in way_indexes and way_indexes.index(i) + 1 == j:
        return way_distance(way) ** -1

    return 0


def ph_updating_rule(ph, cities, way, decay):
    """
    ACS pheromone updating rule.
    For local updating rule use for argument decay 'p'.
    For global updating rule use for argument decay 'alpha'.

    :param ph: Pheromone matrix.
    :param cities: List of cities.
    :param way: Path way.
    :param decay: Pheromone decay parameter.
    """
    matrix_dimension = len(ph)

    for i in range(matrix_dimension):
        for j in range(matrix_dimension):
            delta = calculate_delta_ph(i, j, cities, way)
            ph[i][j] = (1 - decay) * ph[i][j] + decay * delta


def search(cities, alpha, betta, m, num_anthill, q0, p, first_pos=0):
    """
    Поиск маршрута, проходящего через задание города(`cities`).

    :param cities: Список городов.
    :param alpha: Зн. на которое изменяются феромоны на итерации.
    :param betta: Степень влияния феромонов..
    :param m: Колличество муравьем.
    :param num_anthill: Количество муравейников.
    :param q0: Первоначальная оценка експерта.
    :param p: User expert parameter.
    :param first_pos: Индекс города с которого начинаем маршрут
    :return: Список городов в порядке посещения.
    """
    ph = [[0] * len(cities) for _ in range(len(cities))]
    initialize_ph(cities, ph)

    best_way = None
    best_cost = 0

    for anthill in range(num_anthill):
        for k in range(m):
            current, way = first_pos, []
            way.append(cities[current])

            while len(way) < len(cities):
                current = STR(ph, betta, cities, way, q0)
                way.append(current)

            decay_ph(ph, alpha)
            cost = fitness(way)
            ph_updating_rule(ph, cities, way, p)

            if best_cost == 0 or cost < best_cost:
                best_cost = cost
                best_way = way

        ph_updating_rule(ph, cities, best_way, alpha)

    return best_way


def way_distance(way):
    """
    Calculate way distance in km.

    :param way: Iterable object with distance.
    :return: Total distance in km, 0 for empty and one point path.
    """
    total_length = 0
    
    for i in range(len(way) - 1):
        total_length += geodesic_distance(way[i], way[i + 1])

    return total_length


@persist_memoize('locations.shelve')
def load_locations(place_names):
    """
    Load geo location coordinates and their address to dict.
    Function cache result with shelve module.

    :param place_names: List with the place names to fetch a geo inf.
    :return: Dictionary with coordinates tuple(latitude, longitude) as
             key, and tuple(place:string, address:string) as value.
    """
    geo_locator = Nominatim()
    locations = {}

    for place in place_names:
        location = geo_locator.geocode(place)

        if not location:
            continue

        coordinates = (location.latitude, location.longitude)
        address = str(location.address)
        locations[coordinates] = place, address

    return locations


def get_cities_sample(locations, n, randomize=True):
    """
    Return a n-length list of elements chosen from locations.
    If randomize is True result list will be contain random values
    from locations, otherwise first n-elements.
    For n larger than the locations size, a result list will be
    equivalent to the locations size.

    :param locations: Dictionary with locations.
    :param n: Number of cities.
    :param randomize: Boolean flag, define sample behavior.
    :return: List with city coordinates as tuple(latitude, longitude).
    """
    n = n if n < len(locations) else len(locations)

    if randomize:
        return sample(locations.keys(), n)

    return list(locations.keys())[:n]


def show_path(path, locations):
    """
    Виводит на стандартный поток вивода маршрут, вместе с названиям
    городов.

    :param path: Маршрут.
    :param locations: Словарь доступних мест.
    """
    for i, city in enumerate(path):
        place, address = locations.get(city, ('N/A', 'Unknown'))
        print('{}: {}, {} - {}'.format(i, city, place[:-4], address))


def demonstrate():
    """
    Демонстрация примера использования.
    """
    cities_file = 'data/ua_cities.txt'
    num_of_cities = 10
    num_of_ants = 32
    alpha = 0.2
    betta = 2
    q0 = 0.4
    p = 0.7
    num_of_anthill = 3

    place_names = load_lines(cities_file)
    place_names = ['{}, UA'.format(i) for i in place_names]
    locations = load_locations(place_names)
    cities = get_cities_sample(locations, num_of_cities, randomize=False)

    path = search(cities, alpha, betta, num_of_ants, num_of_anthill, q0, p)
    total_path_distance = way_distance(path)

    print('Num of anthill: {}'.format(num_of_anthill))
    print('Num of cities: {}'.format(num_of_cities))
    print('Num of ants: {}'.format(num_of_ants))
    print('\n[Path, distance: {}]'.format(total_path_distance))

    show_path(path, locations)
