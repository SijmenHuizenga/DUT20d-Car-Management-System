import React, {MouseEventHandler} from 'react';
import io from 'socket.io-client';
import {toast, ToastContainer} from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';
import './style/app.sass';
import NodesBlock from "./blocks/NodesBlock";
import TopicsBlock from "./blocks/TopicsBlock";
import ComputeboxBlock from "./blocks/ComputeboxBlock";
import ServicesBlock from "./blocks/ServicesBlock";
import GitBlock from "./blocks/GitBlock";
import LogbookBlock from "./blocks/LogbookBlock";
import RecordingBlock from "./blocks/RecordingBlock";
import {Dashboard, SystemdServiceEnabled, SystemdServiceRunning} from "./statetypes";
import {TooltipContainer} from "./util/Tooltip";

export const devmode = false;
const fakeDashboard: Dashboard = {
    rosnode_up: false,
    ping: {
        timestamp: 1234,
        success: true
    },
    topics: [
        {name: "/mavros/exampletopic/1", lastseen: 1577545897, statistics: {traffic: 244, lastseen: 123456}},
        {name: "/world_state", lastseen: 1577545897, statistics: null},
        {name: "/planning_ReferencePath", lastseen: 1577545897, statistics: null},
        {name: "/visualization_markers/world_state", lastseen: 1577545897, statistics: null},
        {name: "/planning_BoundaryMarkers", lastseen: 1577545897, statistics: null},
        {name: "/mavros/local_position/velocity_body", lastseen: 1577545897, statistics: null},
        {name: "/visualization_markers/world_evidence", lastseen: 1577545897, statistics: null},
        {name: "/mavros/lo4cal_position/velocity_local", lastseen: 1577545897, statistics: null},
        {name: "/world_st5ate", lastseen: 1577545897, statistics: null},
        {name: "/mavros/lhgocal_position/pose", lastseen: 1577545897, statistics: null},
        {name: "/planning_sReferencePath1", lastseen: 1577545897, statistics: null},
        {name: "/visualizatio2n_markers/world_state", lastseen: 1577545897, statistics: null},
        {name: "/mavros/lofcal3_position/velocity_local", lastseen: 1577545897, statistics: null},
        {name: "/world_stab4te", lastseen: 1577545897, statistics: null},
        {name: "/ma5vros/logcal_position/pose", lastseen: 1577545897, statistics: null},
        {name: "/world_astate", lastseen: 1577545897, statistics: null},
        {name: "/planninag_ReferencePath", lastseen: 1577545897, statistics: null},
        {name: "/visuaaliazation_markers/world_state", lastseen: 1577545897, statistics: null},
        {name: "/planning_aBoundaryMarkers", lastseen: 1577545897, statistics: null},
        {name: "/mavroas/loacal_position/velocity_body", lastseen: 1577545897, statistics: null},
        {name: "/visualizatiaon_markers/world_evidence", lastseen: 1577545897, statistics: null},
        {name: "/mavros/lo4caal_position/velocity_local", lastseen: 1577545897, statistics: null},
        {name: "/worldb_st5ate", lastseen: 1577545897, statistics: null},
        {name: "/mavrosb/lhgocal_position/pose", lastseen: 1577545897, statistics: null},
        {name: "/planninbg_sReferencePath1", lastseen: 1577545897, statistics: null},
        {name: "/visualizbatio2n_markers/world_state", lastseen: 1577545897, statistics: null},
        {name: "/mavros/lobfcal3_position/velocity_local", lastseen: 1577545897, statistics: null},
        {name: "/world_stabb4te", lastseen: 1577545897, statistics: null},
        {name: "/ma5vros/logdcal_position/pose", lastseen: 1577545897, statistics: null},
    ],
    nodes: [
        {name: "/mavros", lastseen: 1577545897}
    ],
    publications: [
        {topicname: "/mavros/exampletopic/1", lastseen: 1577545897, nodename: "/mavros"},
    ],
    subscriptions: [
        {topicname: "/mavros/exampletopic/2", lastseen: 1577545897, nodename: "/mavros"},
    ],
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
        is_recording: false,
        config_filename: "sysid_corner1_vioenabled",
        config_topics: [
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
        recording_file: "/storage/bags/sysid_corner1_vioenabled.bag.active",
        recording_duration: 60,
        recording_topics: [
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
        lastrefresh_config: 0,
        lastrefresh_recording: 0,
        recording_filesize: 1024
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
        this.socket = io(window.location.hostname + ':1097');
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

    private readonly tooltipContainerRef: React.RefObject<TooltipContainer>;

    constructor(props :any) {
        super(props);
        this.tooltipContainerRef = React.createRef();
    }

    render() {
        console.log("render", this.props);

        let {rosnode_up, ping, recording, topics, nodes, ssh,
            systemdservices, subscriptions, publications} = this.props;

        return <main onClick={this.containerClicked}>
            <TooltipContainer ref={this.tooltipContainerRef}/>
            <div className="row">
                {/*<div className="col-xl-2 col-lg-4 col-sm-12 col-xs-12 gutter-small">*/}
                    {/*<HealthBlock/>*/}
                {/*</div>*/}
                <div className="col-xl-3 col-lg-6 col-sm-6 col-xs-12 gutter-small">
                    {
                        // @ts-ignore
                        [nodes, topics, subscriptions, publications].includes(undefined)
                            ? "Undefined"
                            : <NodesBlock nodes={nodes} topics={topics} subscriptions={subscriptions} publications={publications}/>
                    }

                </div>
                <div className="col-xl-3 col-lg-6 col-sm-6 col-xs-12 gutter-small">
                    {
                        // @ts-ignore
                        [topics, subscriptions, publications].includes(undefined)
                            ? "Undefined"
                            : <TopicsBlock topics={topics} subscriptions={subscriptions} publications={publications}/>
                    }

                </div>
                <div className="col-xs-12 col-xl-6 gutter-small ">
                    <ComputeboxBlock rosnode_up={rosnode_up} ping={ping} ssh={ssh}/>
                    <ServicesBlock systemdservices={systemdservices}/>
                    <GitBlock/>
                </div>

            </div>
            <div className="row">
                <div className="col-xl-6 col-xs-12 gutter-small">
                    <LogbookBlock />
                </div>
                <div className="col-xl-6 col-xs-12 gutter-small">
                    {
                        // @ts-ignore
                        [topics, recording, publications].includes(undefined)
                            ? "undefined"
                            : <RecordingBlock {...recording} topics={topics} publications={publications}/>
                    }
                </div>
            </div>

        </main>
    }

    containerClicked :MouseEventHandler = () => {
        this.tooltipContainerRef.current!.hideTip();
    };
}
