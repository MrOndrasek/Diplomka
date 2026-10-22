import psycopg2

from measurement_system.measurement_system.drivers.leika_driver.leika_ZMQ_driver import Leika_ZMQ_driver

class table_plane():

    def __init__(self):
        self.points={}
        self.leika=Leika_ZMQ_driver()
        self.leika.connection()
        self.conn = self.get_connection()

        if self.conn:
                    self.cur = self.conn.cursor()
                    self.get_logger().info("Connected to PostgreSQL")
        else:
                    self.cur = None
                    self.get_logger().error(
                        "Could not connect to PostgreSQL"
                    )

    def measure_table_point(self,point_ID):
        self.points[point_ID]=self.leika.measure_point()

    def send_to_database(self):
        
        try:
            for point_ID, measurement in self.points.items():
                timestamp,distance,angle_hz,angle_vt=measurement
                self.cur.execute(
                            """
                            INSERT INTO table_plane_points
                            (
                                point,
                                distance,
                                angleHz,
                                angleVt

                            )
                            VALUES (%s, %s, %s, %s)
                            """,
                            (   
                                point_ID,
                                distance,
                                angle_hz,
                                angle_vt
                            ),
                        )
            
                self.conn.commit()
            return True
                    
        except Exception as e:
            self.conn.rollback()
            self.logger.error(
                f"Database error: {e}"
            )
            return False
        
    def get_connection(self):
        
                try:
        
                    return psycopg2.connect(
                        database="laser_tracker_data",
                        user="dohnaon1",
                        password="1234",
                        host="127.0.0.1",
                        port=5432,
                    )
        
                except Exception as e:
        
                    self.get_logger().error(
                        f"PostgreSQL connection failed: {e}"
                    )
        
                    return None

    def close(self):
        self.leika.end_connection()

        if self.cur is not None:
            self.cur.close()

        if self.conn is not None:
            self.conn.close()

