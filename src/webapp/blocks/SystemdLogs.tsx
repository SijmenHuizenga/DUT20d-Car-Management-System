import React from "react";
import Requestor from "../util/Requestor";

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

    constructor(props :Props) {
        super(props);
        this.state = {
            linecount: 300,
            isLoading: true,
            logdata: null,
            error: null
        }
    }

    render() {
        if(this.state.error !== null) {
            return this.state.error
        }
        if(this.state.isLoading || this.state.logdata == null) {
            return "Loading..."
        }
        return this.state.logdata.split('\n').map((item, key) => {
            return <React.Fragment key={key}>{item}<br/></React.Fragment>
        })
    }

    componentDidMount() {
        Requestor.runcommand(`sudo journalctl --no-pager --unit ${this.props.servicename} --lines ${this.state.linecount}`)
            .then((response) => response.json())
            .then((json) => {
                this.setState({
                    error: null,
                    isLoading: false,
                    logdata: json.output
                });
            })
            .catch((e) => {
                this.setState({
                    isLoading: false,
                    logdata: null,
                    error: e,
                })
            });
    }

}
