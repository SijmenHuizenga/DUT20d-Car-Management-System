import React from "react";
import Markdown from 'react-markdown';
import EditableText from "../util/EditableText";

class LogbookBlock extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            input: '',
            inputDisabled: false,
            error: null,
            scrollStickToBottom: true,
            dragSelectedRowid: null,
            dragLastHoveredRowid: null,
            dragCurrentHoverRowid: null,
        };
    }

    render() {
        let {input, inputDisabled, error} = this.state;
        return <div className="block y-50 d-flex flex-column">
            {error ? <div className="alert alert-danger" role="alert">{error}</div> : null}
            <div className="flex-row overflow-auto" ref={(el) => this.scroller = el}
                 onScroll={this.handleUserScroll.bind(this)}>
                <table className="logtable" onMouseLeave={this.resetDragging.bind(this)}>
                    <tbody>
                    {this.renderDragIndicator()}
                    {this.state.dragSelectedRowid != null ? <style>{".logtable .timestamp {user-select: none;}"}</style> : null}
                    {this.props.lines.map((line) =>
                        <LogbookLine {...line}
                                     setError={(e) => this.setState({error: e})}
                                     onMouseDown={this.lineOnMouseDown.bind(this)}
                                     onMouseUp={this.lineOnMouseUp.bind(this)}
                                     onMouseLeave={this.lineOnMouseLeave.bind(this)}
                                     onMouseEnter={this.lineOnMouseEnter.bind(this)}/>
                        )}
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

    renderDragIndicator() {
        if(this.state.dragSelectedRowid == null || this.state.dragLastHoveredRowid == null || this.state.dragCurrentHoverRowid == null){
            return null;
        }
        if(this.state.dragCurrentHoverRowid === this.state.dragSelectedRowid) {
            return null;
        }

        let originalLineIndex = this.props.lines.findIndex((line) => line.rowid === this.state.dragSelectedRowid);
        let currentLineIndex = this.props.lines.findIndex((line) => line.rowid === this.state.dragCurrentHoverRowid);
        let previousLineIndex = this.props.lines.findIndex((line) => line.rowid === this.state.dragLastHoveredRowid);
        let willBePlasedUnderCurrentLine =  currentLineIndex > previousLineIndex;

        return <style>
            {`.logtable tbody tr:nth-of-type(${currentLineIndex + 1}) { border-${willBePlasedUnderCurrentLine ? "bottom" : "top"}: solid 3px #262f38;}
              .logtable tbody tr:nth-of-type(${originalLineIndex + 1}) {background-color: #262f38} 
              `}
        </style>
    }

    lineOnMouseDown(rowid) {
        if(this.state.dragSelectedRowid != null){
            return;
        }
        this.setState({
            dragSelectedRowid: rowid,
            dragLastHoveredRowid: rowid
        })
    }
    lineOnMouseUp() {
        let lastLineIndex = this.findIndexOfRow(this.state.dragLastHoveredRowid);
        let currentLineIndex = this.findIndexOfRow(this.state.dragCurrentHoverRowid);

        let betweenBottom, betweenTop;
        if(lastLineIndex < currentLineIndex) {
            //moving down (positive)
            betweenBottom = lastLineIndex + 1;
            betweenTop = currentLineIndex + 1;
        }
        if(lastLineIndex > currentLineIndex) {
            //moving up (negative)
            betweenBottom = lastLineIndex - 1;
            betweenTop = currentLineIndex - 1;
        }

        let lineBetweenBottom = this.props.lines[betweenBottom];
        let lineBetweenTop = this.props.lines[betweenTop];

        let newTimestamp;
        if(lineBetweenBottom !== undefined && lineBetweenTop !== undefined) {
            newTimestamp = Math.min(lineBetweenBottom.timestamp, lineBetweenTop.timestamp) + Math.abs(lineBetweenBottom.timestamp - lineBetweenTop.timestamp) / 2;
        } else if(lineBetweenBottom !== undefined) {
            newTimestamp = lineBetweenBottom.timestamp - 1
        } else if(lineBetweenTop !== undefined) {
            newTimestamp = lineBetweenTop.timestamp + 1;
        } else {
            this.resetDragging();
            return;
        }

        console.log(`Moving ${this.state.dragSelectedRowid} to timestamp ${newTimestamp}`);

        this.resetDragging();
    }
    lineOnMouseEnter(rowid) {
        this.setState({
            dragCurrentHoverRowid: rowid
        })
    }
    lineOnMouseLeave(rowid) {
        this.setState({
            dragLastHoveredRowid: rowid
        })
    }
    resetDragging() {
        this.setState({
            dragSelectedRowid: null,
            dragLastHoveredRowid: null,
        })
    }

    findIndexOfRow(rowid) {
        return this.props.lines.findIndex((line) => line.rowid === rowid)
    }

    handleUserScroll() {
        let n = this.isScrolledToBottom();
        if (n !== this.state.scrollStickToBottom)
            this.setState({scrollStickToBottom: n});
    }

    componentDidMount() {
        this.scrollToBottom();
    }

    componentDidUpdate() {
        this.scrollToBottom();
    }

    isScrolledToBottom() {
        return this.scroller.scrollTop === (this.scroller.scrollHeight - this.scroller.offsetHeight)
    }

    scrollToBottom() {
        if (this.state.scrollStickToBottom)
            this.scroller.scrollTop = this.scroller.scrollHeight - this.scroller.offsetHeight;
    }

    handleKeyDown(e) {
        if (e.key === 'Enter') {
            this.storeNewLine()
        }
        return false;
    }

    storeNewLine() {
        this.setState({inputDisabled: true, scrollStickToBottom: true});

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
    constructor(props) {
        super(props);
        this.state = {
            moving: false
        };
    }

    render() {
        return <tr onMouseUp={this.onMouseUp.bind(this)}
                   onMouseLeave={this.onMouseLeave.bind(this)}
                   onMouseEnter={this.onMouseEnter.bind(this)}
                   ref={(el) => this.trRef = el}>
            <td className="timestamp" onMouseDown={this.onMouseDown.bind(this)}>
                {this.renderTimestamp(this.props.timestamp)}
            </td>
            <td>
                <EditableText value={this.props.text} save={this.saveUpdate.bind(this)} multiline={true}>
                    <Markdown source={this.props.text} className="pl-1 logline"/>
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

    onMouseDown(e) {
        if (e.button !== 0) return;
        this.props.onMouseDown(this.props.rowid);
    }
    onMouseUp(e) {
        if (e.button !== 0) return;
        this.props.onMouseUp(this.props.rowid);
    }

    onMouseLeave(e) {
        if (e.button !== 0) return;
        this.props.onMouseLeave(this.props.rowid);
    }

    onMouseEnter(e) {
        if (e.button !== 0) return;
        this.props.onMouseEnter(this.props.rowid);
    }
}

export default LogbookBlock;

