from felix.scripts.settings import settings
import sqlite3
from sqlite3 import Error
import atexit
import time
from typing import Optional
import pandas as pd

MOTION_TABLE = "motion"
MOTION_DDL = f"""CREATE TABLE IF NOT EXISTS {MOTION_TABLE}(
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    vx FLOAT NOT NULL,
    vy FLOAT NOT NULL,
    vz FLOAT NOT NULL,
    rx FLOAT NOT NULL,
    ry FLOAT NOT NULL,
    rz FLOAT NOT NULL,
    image BLOB,
    ts TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL
);"""

class Motion:
    def __init__(self, vx: float, vy: float, vz: float, rx: float, ry: float, rz: float, image, ts = time.time()):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.image = image
        self.ts = ts if ts else time.time()
    

    def for_insert(self):
        return (self.vx, self.vy, self.vz, self.rx, self.ry, self.rz, sqlite3.Binary(self.image))

    def __repr__(self):
        return f"{self.vx}, {self.vy}, {self.vz}, {self.rx}, {self.ry}, {self.rz}, {self.ts}"
        

class Database:
    def __init__(self):
        self.conn = self.create_connection()
        self.create_table()
        atexit.register(self.close)

    def cursor(self):
        return self.conn.cursor()

    def close(self):
        if self.conn:
            self.conn.close()

    def create_connection(self):
        """ create a database connection to a SQLite database """
        conn = None
        try:
            conn = sqlite3.connect(settings.Db.path)
            print(sqlite3.version)
            return conn
        except Error as e:
            print(e)
        

    def create_table(self):
        self.cursor().execute(MOTION_DDL)

    def insert_motion(self, m: Motion):
        # https://www.geeksforgeeks.org/storing-opencv-image-in-sqlite3-with-python/
        try:
            self.cursor().execute(
                "insert into motion (vx,vy,vz,rx,ry,rz,image) VALUES (?,?,?,?,?,?,?);",
                m.for_insert()
            )
            self.conn.commit()
            return True
        except Exception as ex:
            print(ex)
            raise ex
        

    def parse_motion(self, row):
        return Motion(
            vx=row[0],
            vy=row[1],
            vz=row[2],
            rx=row[3],
            ry=row[4],
            rz=row[5],
            image=row[6],
            ts=row[7]
        )

    def get_motion(self, limit: int =100, offset:int = 0):
        
        cursor = self.conn.cursor()
        cursor.execute(f"""select vx, vy, vz, rx, ry, rz, image, ts from {MOTION_TABLE} order by ts limit {limit} offset {offset};""")
        results = cursor.fetchall()
        return [self.parse_motion(m) for m in results]




