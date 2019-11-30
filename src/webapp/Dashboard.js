import React from 'react';
import io from 'socket.io-client';
import './style.css';
import HealthBlock from './blocks/HealthBlock'
import NodesBlock from "./blocks/NodesBlock";
import TopicsBlock from "./blocks/TopicsBlock";
import ComputeboxBlock from "./blocks/ComputeboxBlock";
import ServicesBlock from "./blocks/ServicesBlock";
import GitBlock from "./blocks/GitBlock";
import LogbookBlock from "./blocks/LogbookBlock";
import RecordingBlock from "./blocks/RecordingBlock";

const devmode = true;
const fakeDashboard = {
    rosnode: {
        up: true,
    },
    pinger: {
        computebox: {
            timestamp: 1234,
            success: true
        }
    },
    logbook: [
        {rowid: 111, timestamp: 1, text: "Example line 111"},
        {rowid: 222, timestamp: 2, text: "Example line 222"},
        {rowid: 333, timestamp: 3, text: "Example line 333"},
        {rowid: 444, timestamp: 4, text: "Example line 444"},
        {rowid: 555, timestamp: 5, text: "Example line 555"},
        {rowid: 666, timestamp: 66, text: "Example line 666"},
    ]
};

class Dashboard extends React.Component {

    constructor(props){
        super(props);
        this.state = {
            groundStationState: devmode ? fakeDashboard : null,
            connectionerror: devmode ? null : "groundstation offline"
        };
    }

    render() {
        return <div className="container-fluid">
            {this.state.groundStationState === null || this.state.connectionerror !== null
                ? <div className="overlay error text-center">{this.state.connectionerror}</div>
                : this.renderDashboard()}
        </div>

    }

    renderDashboard() {
        console.log("render", this.state.groundStationState);

        let {rosnode, pinger, logbook} = this.state.groundStationState;

        return <main id="page-main">
            <div className="row">
                <div className="col-xl-2 col-lg-4 col-sm-12 col-xs-12 gutter-small">
                    <HealthBlock/>
                </div>
                <div className="col-xl-2 col-lg-4 col-sm-6 col-xs-12 gutter-small">
                    <NodesBlock/>
                </div>
                <div className="col-xl-2 col-lg-4 col-sm-6 col-xs-12 gutter-small">
                    <TopicsBlock/>
                </div>
                <div className="col-xs-12 col-xl-6 gutter-small ">
                    <ComputeboxBlock rosnode_up={rosnode.up}  pinger={pinger}/>
                    <ServicesBlock/>
                    <GitBlock/>
                </div>

            </div>
            <div className="row">
                <div className="col-xl-6 col-xs-12 gutter-small">
                    <LogbookBlock lines={logbook}/>
                </div>
                <div className="col-xl-6 col-xs-12 gutter-small">
                    <RecordingBlock/>
                </div>
            </div>
        </main>
    }

    componentDidMount() {
        if(devmode) {
            return;
        }
        this.socket = io({
            reconnectionDelayMax: 1000,
        });
        this.socket.on('state', (data) => {
            this.setState({
                groundStationState: data,
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

export default Dashboard;
