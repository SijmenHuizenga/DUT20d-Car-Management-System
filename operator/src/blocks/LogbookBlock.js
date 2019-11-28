import React from "react";
import TextareaAutosize from "react-autosize-textarea";


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
                <table className="logtable">
                    <tbody>
                        {logbook.map((line) => <LogbookLine {...line} setError={(e) => this.setState({error: e})} />)}
                    </tbody>
                </table>
            </div>
            <div className="flex-row flex-grow-1">
                <div className="input-group pt-1 input-group-sm">
                    <input type="text" className="form-control" value={input}
                           onChange={(e) => this.setState({input: e.target.value})}
                           onKeyDown={this.handleKeyDown.bind(this)}
                           disabled={inputDisabled}
                           ref={(el) => this.inputfield = el}/>
                </div>
            </div>
        </div>
    }

    handleKeyDown(e) {
        if (e.key === 'Enter') {
            this.storeNewLine()
        }
        return false;
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

class LogbookLine extends React.Component {
    render() {
        return <tr>
            <td className="timestamp">
                {this.renderTimestamp(this.props.timestamp)}
            </td>
            <td>
                <EditableText value={this.props.text} save={this.saveUpdate.bind(this)}>
                    <span className="pl-1">{this.props.text}</span>
                </EditableText>
            </td>
        </tr>
    }

    renderTimestamp(timestamp) {
        let date = new Date(timestamp * 1000);
        let hours = date.getHours();
        let minutes = "0" + date.getMinutes();
        let seconds = "0" + date.getSeconds();
        return hours + ':' + minutes.substr(-2) + ':' + seconds.substr(-2);
    }

    saveUpdate(newText) {
        return fetch('/logbook/' + this.props.rowid, {
            method: 'PUT',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                text: newText
            })
        }).then((response) => {
            if (response.status !== 201 || !response.ok) {
                console.log("Failed to update logline", response);
                this.props.setError('Failed to update logline: ' + response.statusText);
                return false;
            } else {
                this.props.setError(null);
                return true;
            }
        })
    }
}

class EditableText extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            editing: false,
            input: null,
            inputDisabled: false,
        };
        this.inputref = React.createRef()
    }

    render() {
        if (this.state.editing) {
            return <div>
                <TextareaAutosize
                    maxRows={10}
                    className="pl-1 d-flex bd-highlight"
                    value={this.state.input}
                    onKeyDown={this.onKey.bind(this)}
                    onChange={(e) => this.setState({input: e.target.value})}
                    disabled={this.state.inputDisabled}
                    ref={this.inputref}
                />
            </div>
        }
        return <div onClick={this.startEditing.bind(this)}>
            {this.props.children}
        </div>;
    }

    onKey(e) {
        if (e.key === "Escape") {
            this.stopEditing();
            return;
        }
        if(e.key === "Enter" && !e.shiftKey) {
            this.saveEditing();
        }
    }

    startEditing() {
        this.setState({
            editing: true,
            input: this.props.value
        }, () => this.focusEditor());
    }

    stopEditing() {
        this.setState({
            editing: false,
            input: null
        })
    }

    saveEditing() {
        this.setState({
            inputDisabled: true,
        }, () => {
            this.props.save(this.state.input)
                .then((success) => {
                    if(success){
                        this.setState({
                            editing: false,
                            input: null,
                            inputDisabled: false
                        })
                    } else {
                        this.setState({
                            inputDisabled: false
                        }, () => this.focusEditor())
                    }
                })
        })
    }

    focusEditor() {
        this.inputref.current.focus();
    }

}

export default LogbookBlock;

