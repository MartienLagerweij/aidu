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
            User.users = [User('campus-card-code01', 'Name1'),
                          User('campus-card-code02', 'Name2'),
                          User('campus-card-code03', 'Name3')]
        return User.users
