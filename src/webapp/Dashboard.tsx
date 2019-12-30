import React from 'react';
import io from 'socket.io-client';
import {toast, ToastContainer} from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';
import './style/app.sass';
import HealthBlock from './blocks/HealthBlock'
import NodesBlock from "./blocks/NodesBlock";
import TopicsBlock from "./blocks/TopicsBlock";
import ComputeboxBlock from "./blocks/ComputeboxBlock";
import ServicesBlock from "./blocks/ServicesBlock";
import GitBlock from "./blocks/GitBlock";
import LogbookBlock from "./blocks/LogbookBlock";
import RecordingBlock from "./blocks/RecordingBlock";
import {Dashboard, SystemdServiceEnabled, SystemdServiceRunning} from "./statetypes";
import {TooltipContainer} from "./util/Tooltip";

const devmode = false;
const fakeDashboard: Dashboard = {
    rosnode: {
        up: false,
    },
    ping: {
        timestamp: 1234,
        success: true
    },
    logbook: [
        {rowid: 111, timestamp: 1, text: "Example line 111", source: "example"},
        {rowid: 222, timestamp: 2, text: "Example line 222", source: "example"},
        {rowid: 333, timestamp: 3, text: "Example line 333", source: "example"},
        {rowid: 444, timestamp: 4, text: "Example line 444", source: "example"},
        {rowid: 555, timestamp: 5, text: "Example line 555", source: "example"},
        {rowid: 666, timestamp: 66, text: "Example line 666", source: "example"},
    ],
    topics: [
        {name: "/world_state", lastseen: 1577545897},
        {name: "/planning_ReferencePath", lastseen: 1577545897},
        {name: "/visualization_markers/world_state", lastseen: 1577545897},
        {name: "/planning_BoundaryMarkers", lastseen: 1577545897},
        {name: "/mavros/local_position/velocity_body", lastseen: 1577545897},
        {name: "/visualization_markers/world_evidence", lastseen: 1577545897},
        {name: "/mavros/lo4cal_position/velocity_local", lastseen: 1577545897},
        {name: "/world_st5ate", lastseen: 1577545897},
        {name: "/mavros/lhgocal_position/pose", lastseen: 1577545897},
        {name: "/planning_sReferencePath1", lastseen: 1577545897},
        {name: "/visualizatio2n_markers/world_state", lastseen: 1577545897},
        {name: "/mavros/lofcal3_position/velocity_local", lastseen: 1577545897},
        {name: "/world_stab4te", lastseen: 1577545897},
        {name: "/ma5vros/logcal_position/pose", lastseen: 1577545897},
        {name: "/world_astate", lastseen: 1577545897},
        {name: "/planninag_ReferencePath", lastseen: 1577545897},
        {name: "/visuaaliazation_markers/world_state", lastseen: 1577545897},
        {name: "/planning_aBoundaryMarkers", lastseen: 1577545897},
        {name: "/mavroas/loacal_position/velocity_body", lastseen: 1577545897},
        {name: "/visualizatiaon_markers/world_evidence", lastseen: 1577545897},
        {name: "/mavros/lo4caal_position/velocity_local", lastseen: 1577545897},
        {name: "/worldb_st5ate", lastseen: 1577545897},
        {name: "/mavrosb/lhgocal_position/pose", lastseen: 1577545897},
        {name: "/planninbg_sReferencePath1", lastseen: 1577545897},
        {name: "/visualizbatio2n_markers/world_state", lastseen: 1577545897},
        {name: "/mavros/lobfcal3_position/velocity_local", lastseen: 1577545897},
        {name: "/world_stabb4te", lastseen: 1577545897},
        {name: "/ma5vros/logdcal_position/pose", lastseen: 1577545897},
    ],
    nodes: [
        {name: "/mavros", lastseen: 1577545897}
    ],
    publications: [],
    subscriptions: [],
    topicstatistics: [],
    ssh: {
        connected: true,
        lastping: 1577545897,
        uptime: "up for -1 day"
    },
    systemdservices: [{
            name: "recording.service",
            statustext: "Lorem ipsum dolor sit amet, consectetur adipiscing elit, \nsed do eiusmod tempor \nincididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum",
            running: SystemdServiceRunning.running,
            lastupdate: 1577545897,
            enabled: SystemdServiceEnabled.enabled,
        }
    ],
    recording: {
        lastrefresh: 0,
        is_recording: false,
        filename: "sysid_corner1_vioenabled",
        bagfilename: "sysid_corner1_vioenabled_20190804172203.bag.active",
        recordingduration: "03:42",
        selected_topics: [
            "/planning_ReferencePath",
            "/visualization_markers/world_state",
            "/mavros/local_position/velocity_local",
            "/world_state",
            "/mavros/local_position/pose",
            "/planning_ReferencePath1",
            "/visualizatio2n_markers/world_state",
            "/mavros/local3_position/velocity_local",
            "/world_sta4te",
            "/ma5vros/local_position/pose",
            "/planning_Reference2Path",
            "/visualization_3markers/world_state",
        ],
    }
};

interface State {
    groundStationState: Dashboard | null
    connectionerror: string | null
}


export default class DashboardStateLoader extends React.PureComponent<{}, State> {
    private socket: SocketIOClient.Socket;

    constructor(props: {}) {
        super(props);
        this.state = {
            groundStationState: devmode ? fakeDashboard : null,
            connectionerror: devmode ? null : "groundstation offline"
        };
        this.socket = io(window.location.hostname + ':5000');
    }

    render() {
        return <div className="container-fluid">
            {this.state.groundStationState === null || this.state.connectionerror !== null
                ? <div className="erroroverlay  text-center">{this.state.connectionerror}</div>
                : <DashboardComponent {...this.state.groundStationState} />}
            <ToastContainer position={toast.POSITION.BOTTOM_RIGHT} autoClose={false}/>
        </div>
    }

    componentDidMount() {
        if (devmode) {
            return;
        }
        this.socket.on('state', (data: any) => {
            this.setState({
                groundStationState: JSON.parse(data),
                connectionerror: null,
            })
        });
        this.socket.on('disconnect', () => {
            this.setState({
                connectionerror: "groundstation disconnected",
            })
        });
    }
}


class DashboardComponent extends React.Component<Dashboard, {}> {

    render() {
        console.log("render", this.props);

        let {rosnode, ping, logbook, recording, topics, nodes, ssh,
            systemdservices, subscriptions, publications, topicstatistics} = this.props;

        return <main>
            <TooltipContainer>
            <div className="row">
                <div className="col-xl-2 col-lg-4 col-sm-12 col-xs-12 gutter-small">
                    <HealthBlock/>
                </div>
                <div className="col-xl-2 col-lg-4 col-sm-6 col-xs-12 gutter-small">
                    <NodesBlock nodes={nodes} subscriptions={subscriptions} publications={publications}
                                topicstatistics={topicstatistics}/>
                </div>
                <div className="col-xl-2 col-lg-4 col-sm-6 col-xs-12 gutter-small">
                    <TopicsBlock topics={topics} subscriptions={subscriptions} publications={publications}/>
                </div>
                <div className="col-xs-12 col-xl-6 gutter-small ">
                    <ComputeboxBlock rosnode_up={rosnode.up} ping={ping} ssh={ssh}/>
                    <ServicesBlock systemdservices={systemdservices}/>
                    <GitBlock/>
                </div>

            </div>
            <div className="row">
                <div className="col-xl-6 col-xs-12 gutter-small">
                    <LogbookBlock lines={logbook}/>
                </div>
                <div className="col-xl-6 col-xs-12 gutter-small">
                    <RecordingBlock {...recording} topics={topics}/>
                </div>
            </div>
            </TooltipContainer>
        </main>
    }
}
