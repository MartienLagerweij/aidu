__author__ = 'rolf'


class Location():
    def __init__(self, id, name):
        self.id = id
        self.name = name

    def __str__(self):
        return self.name

    @staticmethod
    def get_locations():
        if 'locations' not in dir(Location):
            Location.locations = [Location('IO-02', 'Demo locatie 1'),
                                  Location('IO-01', 'Demo locatie 2')]
        return Location.locations
