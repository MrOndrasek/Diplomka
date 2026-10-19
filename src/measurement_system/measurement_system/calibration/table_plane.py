import zmq
import struct
import psycopg2

class table_plane():

    def __init__(self,measurement_client):

        self.measurement_client = measurement_client

    def measure_point(self, point_id):
        return self.measurement_client.measure(
            point_type="table",
            point_id=point_id
        )



