import time

from groundstation.utils import sendstate


class TransformCapture:

    def __init__(self):
        self.last_update = time.time()

        self.state = {
            'map->base_link': {'lastseen': 0.0},
            'base_link->velodyne': {'lastseen': 0.0},
            'velodyne->quad2': {'lastseen': 0.0},
            'velodyne->quad1': {'lastseen': 0.0},
            'quad1->quad3': {'lastseen': 0.0},
            'quad2->quad4': {'lastseen': 0.0},
            'imu->vio': {'lastseen': 0.0},
            'vio->quad1': {'lastseen': 0.0},
        }

    def transform_callback(self, data):
        for transform in data.transforms:
            key = transform.header.frame_id+'->'+transform.child_frame_id
            if key not in self.state:
                self.state[key] = {'lastseen': 0}
            self.state[key]['lastseen'] = time.time()

        if time.time() - self.last_update > 0.5:
            sendstate({'transforms': self.state})
            self.last_update = time.time()
