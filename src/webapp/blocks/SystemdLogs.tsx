import React from "react";
import Requestor from "../util/Requestor";
import {devmode} from "../Dashboard";
import Modal from "../util/Modal";

interface State {
    linecount :number,
    isLoading :boolean
    logdata :string | null
    error: null
}

interface Props {
    servicename :string
}

export default class SystemdLogsBlock extends React.Component<Props, State> {
    private scroller: HTMLDivElement | null;

    constructor(props :Props) {
        super(props);
        this.scroller = null;
        this.state = {
            linecount: 300,
            isLoading: true,
            logdata: null,
            error: null,
        }
    }

    render() {
        return <React.Fragment>
            <div className="modal-header">
                <button type="button"
                        className="btn btn-outline-primary py-0 m-1"
                        disabled={this.state.isLoading}
                        onClick={this.refresh.bind(this)}>Refresh
                </button>
                <select className="custom-select m-1" disabled={this.state.isLoading}
                        value={this.state.linecount} onChange={
                            (e) =>{
                                console.log(e.target.value);
                                this.setState({linecount: parseInt(e.target.value)}, this.refresh)
                            }
                        }>
                    <option value={300}>300 rows</option>
                    <option value={500}>500 rows</option>
                    <option value={1000}>1000 rows</option>
                </select>
                <h5 className="modal-title ml-1">{this.props.servicename}</h5>
                <button type="button" className="close" onClick={() => Modal(null)}>
                    &times;
                </button>
            </div>
            <div className="modal-body" ref={(el) => this.scroller = el}>
            {this.renderLogs()}
            </div>
        </React.Fragment>
    }

    renderLogs() {
        if(this.state.error !== null) {
            return <span style={{color: "red"}}>{this.state.error}</span>
        }
        if(this.state.isLoading || this.state.logdata == null) {
            return "Loading..."
        }
        return this.state.logdata.split('\n').map((item, key) => {
            return <React.Fragment key={key}>{item}<br/></React.Fragment>
        })
    }

    componentDidMount() {
        this.refresh()
    }

    refresh() {
        this.setState({
            isLoading: true,
            error: null,
        });
        if(devmode) {
            setTimeout(() => this.setState({
                error: null,
                isLoading: false,
                logdata: [...Array(this.state.linecount)].map(_ => "Mar 06 09:24:10 raampje systemd[562]: var-lib-docker-overlay2-8f53fd4f133cc803bc6cca0424909a406f7c07a42e1cdf90657c9cf43074bd2b-merged-opt-pycharm-skeletons.mount: Succeeded.").join("\n"),
            }, () => this.scrollToBottom()), 1000);
            return;
        }
        Requestor.runcommand(`sudo journalctl --no-pager --unit ${this.props.servicename} --lines ${this.state.linecount}`)
            .then((response) => response.json())
            .then((json) => {
                this.setState({
                    error: null,
                    isLoading: false,
                    logdata: json.output,
                }, () => this.scrollToBottom());
            })
            .catch((e) => {
                this.setState({
                    isLoading: false,
                    logdata: null,
                    error: e,
                })
            });
    }

    scrollToBottom() {
        this.scroller!.scrollTop = this.scroller!.scrollHeight - this.scroller!.offsetHeight;
    }

}
