import React from "react";
import Markdown from 'react-markdown';
import EditableText from "../util/EditableText";
import Requestor from "../util/Requestor";

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
        let {lines} = this.props;
        lines.sort((a, b) => a.timestamp - b.timestamp);
        return <div className="block y-50 d-flex flex-column">
            {error ? <div className="alert alert-danger" role="alert">{error}</div> : null}
            <div className="flex-row overflow-auto" ref={(el) => this.scroller = el}
                 onScroll={this.handleUserScroll.bind(this)}>
                <table className="logtable" onMouseLeave={this.resetDragging.bind(this)}>
                    <tbody>
                    {this.renderDragIndicator()}
                    {this.state.dragSelectedRowid != null ?
                        <style>{".logtable .timestamp {user-select: none;}"}</style> : null}
                    {lines.map((line) =>
                        <LogbookLine {...line}
                                     key={line.timestamp}
                                     onMouseDown={this.lineOnMouseDown.bind(this)}
                                     onMouseUp={this.lineOnMouseUp.bind(this)}
                                     onMouseLeave={this.lineOnMouseLeave.bind(this)}
                                     onMouseEnter={this.lineOnMouseEnter.bind(this)}
                                     updateLine={this.updateLine.bind(this)}/>
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
        if (this.state.dragSelectedRowid == null || this.state.dragLastHoveredRowid == null || this.state.dragCurrentHoverRowid == null) {
            return null;
        }
        if (this.state.dragCurrentHoverRowid === this.state.dragSelectedRowid) {
            return null;
        }

        let originalLineIndex = this.props.lines.findIndex((line) => line.rowid === this.state.dragSelectedRowid);
        let currentLineIndex = this.props.lines.findIndex((line) => line.rowid === this.state.dragCurrentHoverRowid);
        let previousLineIndex = this.props.lines.findIndex((line) => line.rowid === this.state.dragLastHoveredRowid);
        let willBePlasedUnderCurrentLine = currentLineIndex > previousLineIndex;

        return <style>
            {`.logtable tbody tr:nth-of-type(${currentLineIndex + 1}) { border-${willBePlasedUnderCurrentLine ? "bottom" : "top"}: solid 3px #262f38;}
              .logtable tbody tr:nth-of-type(${originalLineIndex + 1}) {background-color: #262f38} 
              `}
        </style>
    }

    lineOnMouseDown(rowid) {
        if (this.state.dragSelectedRowid != null) {
            return;
        }
        this.setState({
            dragSelectedRowid: rowid,
            dragLastHoveredRowid: rowid
        })
    }

    lineOnMouseUp() {
        if(this.state.dragCurrentHoverRowid === this.state.dragSelectedRowid) {
            //did not move. Mouse released on same row as it was dragged from.
            this.resetDragging();
            return;
        }

        let lastLineIndex = this.findIndexOfRow(this.state.dragLastHoveredRowid);
        let currentLineIndex = this.findIndexOfRow(this.state.dragCurrentHoverRowid);

        let betweenBottomIndex, betweenTopIndex;
        if (lastLineIndex < currentLineIndex) {
            //moving down (positive)
            betweenBottomIndex = lastLineIndex + 1;
            betweenTopIndex = currentLineIndex + 1;
        }
        if (lastLineIndex > currentLineIndex) {
            //moving up (negative)
            betweenBottomIndex = lastLineIndex - 1;
            betweenTopIndex = currentLineIndex - 1;
        }

        let lineBetweenBottom = this.props.lines[betweenBottomIndex];
        let lineBetweenTop = this.props.lines[betweenTopIndex];

        if((lineBetweenBottom !== undefined && lineBetweenBottom.rowid === this.state.dragSelectedRowid) ||
            (lineBetweenTop !== undefined && lineBetweenTop.rowid === this.state.dragSelectedRowid)) {
            //did not move. Placed directly under or directly above current line.
            this.resetDragging();
            return;
        }

        let newTimestamp;
        if (lineBetweenBottom !== undefined && lineBetweenTop !== undefined) {
            newTimestamp = Math.min(lineBetweenBottom.timestamp, lineBetweenTop.timestamp) + Math.abs(lineBetweenBottom.timestamp - lineBetweenTop.timestamp) / 2;
        } else if (lineBetweenBottom !== undefined) {
            newTimestamp = lineBetweenBottom.timestamp - 1
        } else if (lineBetweenTop !== undefined) {
            newTimestamp = lineBetweenTop.timestamp + 1;
        } else {
            this.resetDragging();
            return;
        }

        console.log(`Moving ${this.state.dragSelectedRowid} to timestamp ${newTimestamp}`);
        this.updateLine(this.state.dragSelectedRowid, {timestamp: newTimestamp});
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

        Requestor.execute('/logbook', 'POST', {text: this.state.input, source: 'human'})
            .then(() => this.setState({inputDisabled: false, input: '', error: null}))
            .catch((error) => this.setState({
                inputDisabled: false,
                error: 'Failed to store logline: ' + error
            }));
    }

    updateLine(rowid, changeset) {
        return Requestor.put(`/logbook/${rowid}`, changeset)
            .then(() => {
                this.setState({error: null});
                return true;
            })
            .catch((error) => {
                this.setState({error: 'Failed to update logline: ' + error});
                return false;
            });
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
        let {rowid, timestamp, text, updateLine} = this.props;
        return <tr onMouseLeave={this.onMouseLeave.bind(this)}
                   onMouseEnter={this.onMouseEnter.bind(this)}>
            <td className="timestamp grayyed"
                onMouseDown={this.onMouseDown.bind(this)}
                onMouseUp={this.onMouseUp.bind(this)}>
                {this.renderTimestamp(timestamp)}
            </td>
            <td>
                <EditableText value={text} save={(newText) => updateLine(rowid, {text: newText})} multiline={true}>
                    <Markdown source={text} className="pl-1 logline"/>
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

