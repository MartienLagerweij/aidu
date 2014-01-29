__author__ = 'rolf'


class Drawers:

    @staticmethod
    def setup():
        Drawers.drawers = {'1': None, '2': None, '3': None}

    @staticmethod
    def add_delivery(drawer_id, recipient_id, location_id):
        Drawers.drawers[drawer_id] = {'recipient': recipient_id, 'location': location_id}

    @staticmethod
    def remove_delivery(drawer_id):
        Drawers.drawers[drawer_id] = None

    @staticmethod
    def receive_drawers(recipient_id):
        return [k for k, v in Drawers.drawers.iteritems() if v is not None and v['recipient'] == recipient_id]

    @staticmethod
    def available_drawers():
        arr = [k for k, v in Drawers.drawers.iteritems() if v is None]
        arr.sort()
        return arr
