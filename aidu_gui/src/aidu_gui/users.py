__author__ = 'rolf'


class User():
    def __init__(self, id, name):
        self.id = id
        self.name = name

    def __str__(self):
        return self.name

    @staticmethod
    def get_users():
        if 'users' not in dir(User):
            User.users = [User('670056546044310CA932A80', 'Robert'),
                          User('670047480047453BA202A80', 'Dylan')]
        return User.users
