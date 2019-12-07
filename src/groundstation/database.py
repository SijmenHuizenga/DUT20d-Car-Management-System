import sqlite3
import threading


class Database:

    def __init__(self):
        self.database = sqlite3.connect('development.db', check_same_thread=False)
        self.database.row_factory = dict_factory
        self.databaselock = threading.Lock()
        self.create_schema()

    def create_schema(self):
        self.databaselock.acquire()
        with self.database:
            self.database.execute('CREATE TABLE IF NOT EXISTS pings (timestamp FLOAT, host VARCHAR(10), success BOOLEAN)')
            self.database.execute('CREATE TABLE IF NOT EXISTS rosnodehealth (timestamp FLOAT, up BOOLEAN)')
            self.database.execute('CREATE TABLE IF NOT EXISTS logbook (timestamp FLOAT, text TEXT, source VARCHAR(255))')
            self.database.execute('CREATE TABLE IF NOT EXISTS nodes (timestamp FLOAT, name VARCHAR(255))')
            self.database.execute('CREATE TABLE IF NOT EXISTS topics (timestamp FLOAT, name VARCHAR(255), type VARCHAR(255))')
            self.database.execute('CREATE TABLE IF NOT EXISTS node_topic_subscription (timestamp FLOAT, node VARCHAR(255), topic VARCHAR(255))')
            self.database.execute('CREATE TABLE IF NOT EXISTS node_topic_publication (timestamp FLOAT, node VARCHAR(255), topic VARCHAR(255))')
        self.databaselock.release()

    def insert(self, query, args):
        self.databaselock.acquire()
        cursor = self.database.cursor()
        cursor.execute(query, args)
        cursor.close()
        self.database.commit()
        self.databaselock.release()
        return cursor.lastrowid

    def select_one(self, query, args):
        self.databaselock.acquire()
        cursor = self.database.cursor()
        cursor.execute(query, args)
        out = cursor.fetchone()
        cursor.close()
        self.databaselock.release()
        return out

    def select_all(self, query, args):
        self.databaselock.acquire()
        cursor = self.database.cursor()
        cursor.execute(query, args)
        out = cursor.fetchall()
        cursor.close()
        self.databaselock.release()
        return out

    def query(self, query, args):
        self.databaselock.acquire()
        cursor = self.database.cursor()
        cursor.execute(query, args)
        cursor.close()
        self.database.commit()
        self.databaselock.release()


def dict_factory(cursor, row):
    d = {}
    for idx, col in enumerate(cursor.description):
        d[col[0]] = row[idx]
    return d
