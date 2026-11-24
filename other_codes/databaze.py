import psycopg2


def main():
    if conn:
        print("Connection to DB succesfull")
    else:
        print("Connection to DB failed")
    create_table()


def create_table():
    print("created")
    cur = conn.cursor()
    cur.execute("""CREATE TABLE IF NOT EXISTS robots (
        id SERIAL PRIMARY KEY,
        robot_name TEXT NOT NULL
        )""")
    cur.execute("""CREATE TABLE IF NOT EXISTS measurements (
        id SERIAL PRIMARY KEY,
        robot_id INTEGER REFERENCES robots(id),
        time_of_measuring TIMESTAMP DEFAULT NOW(),
        name TEXT NOT NULL
        )""")
    cur.execute("""CREATE TABLE IF NOT EXISTS measurement_data (
        id SERIAL PRIMARY KEY,
        measurement_id INTEGER REFERENCES measurements(id),
        time_of_meas BIGINT,
        distance DOUBLE PRECISION,
        angleHz DOUBLE PRECISION,
        angleVt DOUBLE PRECISION
        )""")
    
    cur.close()
    conn.commit()


def get_connection():
    try:
        return psycopg2.connect(
            database="laser_tracker_data",
            user="dohnaon1",
            password="1234",
            host="127.0.0.1",
            port=5432,
        )
    except:
        return False


if __name__ == "__main__":
    conn = get_connection()
    main()
