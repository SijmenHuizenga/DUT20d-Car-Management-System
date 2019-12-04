from database import database as db
from state import statemanager as state


class Logbook:

    def __init__(self):
        pass

    @staticmethod
    def get_line(rowid):
        for line in state.state['logbook']:
            if line['rowid'] == rowid:
                return line
        return None

    @staticmethod
    def add_line(timestamp, text, source):
        line = {'timestamp': timestamp, 'text': text, 'source': source}
        line['rowid'] = db.insert('INSERT INTO logbook VALUES (:timestamp, :text, :source)', line)
        state.state['logbook'].append(line)
        state.emit_state()

    def update_line_text(self, rowid, text):
        line = self.get_line(rowid)
        if line is None:
            raise Exception("Line does not exit")
        line['text'] = text
        db.query("UPDATE logbook SET text=:text WHERE rowid=:rowid", line)
        state.emit_state()

    def update_line_timestamp(self, rowid, timestmap):
        line = self.get_line(rowid)
        if line is None:
            raise Exception("Line does not exit")
        line['timestamp'] = timestmap
        db.query("UPDATE logbook SET text=:text WHERE timestamp=:timestamp", line)
        state.emit_state()


logbook = Logbook()
