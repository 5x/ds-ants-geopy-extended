import os
import shelve
from functools import wraps
from itertools import chain


def load_lines(file_path):
    """
    Return list with loaded lines from file, separated with newline.
    Leading and trailing whitespace removed from each line.

    :param file_path: Absolute or relative file path.
    :return: List of lines.
    """
    lines = []
    with open(file_path, 'r') as file:
        for line in file:
            lines.append(line.strip())

    return lines


def __build_memoize_path(path, store_name):
    if not path:
        path = os.getcwd()

    dir_name = '.memoize_store'
    store_dir = os.path.join(path, dir_name, store_name)

    if not os.path.exists(store_dir):
        os.makedirs(store_dir)

    return os.path.join(store_dir, store_name)


def persist_memoize(store_name, path=None):
    store_path = __build_memoize_path(path, store_name)

    def real_decorator(fn):
        @wraps(fn)
        def wrapper(*args, **kwargs):
            arguments = [str(i) for i in chain(args, kwargs.items())]
            arguments = ','.join(arguments)
            key = '{}:{}({})'.format(fn.__module__, fn.__name__, arguments)

            with shelve.open(store_path) as store:
                if key not in store:
                    store[key] = fn(*args, **kwargs)
                    store.sync()

                result = store[key]

            return result

        return wrapper

    return real_decorator
