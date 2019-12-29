from .database import Database
from .state import LogbookLine, State


class Logbook:

    def __init__(self,
                 db,  # type: Database
                 state  # type: State
                 ):
        self.db = db
        self.state = state

    def get_line(self, rowid):
        for line in self.state.logbook:
            if line.rowid == rowid:
                return line
        return None

    def add_line(self, timestamp, text, source):
        line = LogbookLine(timestamp=timestamp, text=text, source=source)
        line.rowid = self.db.insert('INSERT INTO logbook VALUES (:timestamp, :text, :source)', {
            'timestamp': line.timestamp,
            'text': line.text,
            'source': line.source
        })
        self.state.logbook.append(line)
        self.state.emit_state()

    def update_line_text(self, rowid, text):
        line = self.get_line(rowid)
        if line is None:
            raise Exception("Line does not exit")
        line.text = text
        self.db.query("UPDATE logbook SET text=:text WHERE rowid=:rowid", {
            'text': line.text,
            'rowid': line.rowid
        })
        self.state.emit_state()

    def update_line_timestamp(self, rowid, timestmap):
        line = self.get_line(rowid)
        if line is None:
            raise Exception("Line does not exit")
        line.timestamp = timestmap
        self.db.query("UPDATE logbook SET timestmap=:timestmap WHERE rowid=:rowid", {
            'rowid': line.rowid,
            'timestmap': line.timestamp
        })
        self.state.emit_state()
