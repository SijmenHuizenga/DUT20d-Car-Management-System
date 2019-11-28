import React from "react";

class LogbookBlock extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            input: '',
            inputDisabled: false,
            error: null,
        };
    }

    render() {
        let {input, inputDisabled, error} = this.state;
        let {logbook} = this.props.groundStationState;

        return <div className="block y-50 d-flex flex-column">
            {error ? <div className="alert alert-danger" role="alert">{error}</div> : null}
            <div className="flex-row overflow-auto">
                {logbook.map(this.renderLine.bind(this))}
                <div style={{ float:"left", clear: "both" }}
                     ref={(el) => { this.logbookEnd = el; }}>
                </div>
            </div>
            <div className="flex-row flex-grow-1">
                <div className="input-group pt-1 input-group-sm">
                    <input type="text" className="form-control" value={input}
                           onChange={(e) => this.setState({input: e.target.value})}
                           onKeyDown={this.handleKeyDown.bind(this)}
                           disabled={inputDisabled}
                           ref={(el) => { this.inputfield = el; }}/>
                </div>
            </div>
        </div>
    }

    renderLine(line) {
        return <div>{this.renderTimestamp(line.timestamp)}: {line.text}</div>
    }

    renderTimestamp(timestamp) {
        let date = new Date(timestamp*1000);
        let hours = date.getHours();
        let minutes = "0" + date.getMinutes();
        let seconds = "0" + date.getSeconds();
        return hours + ':' + minutes.substr(-2) + ':' + seconds.substr(-2);
    }

    handleKeyDown(e) {
        if (e.key === 'Enter') {
            this.storeNewLine()
        }
        return false;
    }

    scrollToBottom = () => {
        this.logbookEnd.scrollIntoView({ behavior: "smooth" });
    };

    componentDidMount() {
        this.scrollToBottom();
    }

    componentDidUpdate() {
        this.scrollToBottom();
    }

    storeNewLine() {
        this.setState({inputDisabled: true});

        fetch('/logbook', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                text: this.state.input,
                source: 'human'
            })
        }).then((response) => {
            if (response.status !== 201 || !response.ok) {
                console.log("Failed to store logline", response);
                this.setState({inputDisabled: false, error: 'Failed to store logline: ' + response.statusText})
            } else {
                this.setState({inputDisabled: false, input: '', error: null});
                this.inputfield.focus();
            }
        })
    }
}

export default LogbookBlock;

