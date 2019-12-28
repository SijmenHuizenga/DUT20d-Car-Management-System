import React from "react";
import Markdown from 'react-markdown';
import EditableText from "../util/EditableText";
import Requestor from "../util/Requestor";
import {LogbookLine} from "../statetypes";

interface Line {
    rowid :number
    timestamp :number
    text :string
}

interface Props {
    lines :LogbookLine[]
}

interface State {
    input :string
    inputDisabled :boolean
    error :string | null
    scrollStickToBottom :boolean
    dragSelectedRowid :number | null
    dragLastHoveredRowid :number | null
    dragCurrentHoverRowid :number | null
}

class LogbookBlock extends React.Component<Props, State> {
    private scroller: HTMLDivElement | null;
    private inputfield: HTMLInputElement | null;

    constructor(props :Props) {
        super(props);
        this.inputfield = null;
        this.scroller = null;
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
                        <LogbookLineComponent {...line}
                                     key={line.rowid}
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

    lineOnMouseDown(rowid :number) {
        if (this.state.dragSelectedRowid != null) {
            return;
        }
        this.setState({
            dragSelectedRowid: rowid,
            dragLastHoveredRowid: rowid
        })
    }

    lineOnMouseUp() {
        const {dragLastHoveredRowid, dragSelectedRowid, dragCurrentHoverRowid} = this.state;
        if(dragCurrentHoverRowid === dragSelectedRowid) {
            //did not move. Mouse released on same row as it was dragged from.
            this.resetDragging();
            return;
        }

        if(dragLastHoveredRowid == null || dragCurrentHoverRowid == null || dragSelectedRowid == null) {
            return
        }

        let lastLineIndex = this.findIndexOfRow(dragLastHoveredRowid);
        let currentLineIndex = this.findIndexOfRow(dragCurrentHoverRowid);

        let betweenBottomIndex, betweenTopIndex;
        if (lastLineIndex < currentLineIndex) {
            //moving down (positive)
            betweenBottomIndex = lastLineIndex + 1;
            betweenTopIndex = currentLineIndex + 1;
        } else if (lastLineIndex > currentLineIndex) {
            //moving up (negative)
            betweenBottomIndex = lastLineIndex - 1;
            betweenTopIndex = currentLineIndex - 1;
        } else {
            //todo: is this correct?
            return
        }

        let lineBetweenBottom = this.props.lines[betweenBottomIndex];
        let lineBetweenTop = this.props.lines[betweenTopIndex];

        if((lineBetweenBottom !== undefined && lineBetweenBottom.rowid === dragSelectedRowid) ||
            (lineBetweenTop !== undefined && lineBetweenTop.rowid === dragSelectedRowid)) {
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

        console.log(`Moving ${dragSelectedRowid} to timestamp ${newTimestamp}`);
        this.updateLine(dragSelectedRowid, {timestamp: newTimestamp});
        this.resetDragging();
    }

    lineOnMouseEnter(rowid :number) {
        this.setState({
            dragCurrentHoverRowid: rowid
        })
    }

    lineOnMouseLeave(rowid :number) {
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

    findIndexOfRow(rowid :number) {
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
        return this.scroller!.scrollTop === (this.scroller!.scrollHeight - this.scroller!.offsetHeight)
    }

    scrollToBottom() {
        if (this.state.scrollStickToBottom)
            this.scroller!.scrollTop = this.scroller!.scrollHeight - this.scroller!.offsetHeight;
    }

    handleKeyDown(e :React.KeyboardEvent<HTMLInputElement>) {
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

    updateLine(rowid :number, changeset :any) {
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

interface LineProps extends Line {
    updateLine(rowid :number, changeset :any) :Promise<boolean>
    onMouseDown(rowid :number) :void
    onMouseUp(rowid :number) :void
    onMouseLeave(rowid :number) :void
    onMouseEnter(rowid :number) :void
}

class LogbookLineComponent extends React.Component<LineProps, {moving :boolean}> {
    constructor(props :LineProps) {
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

    renderTimestamp(timestamp :number) {
        let date = new Date(timestamp * 1000);
        let hours = date.getHours();
        let minutes = "0" + date.getMinutes();
        let seconds = "0" + date.getSeconds();
        return hours + ':' + minutes.substr(-2) + ':' + seconds.substr(-2);
    }

    onMouseDown(e :React.MouseEvent<HTMLTableDataCellElement>) {
        if (e.button !== 0) return;
        this.props.onMouseDown(this.props.rowid);
    }

    onMouseUp(e :React.MouseEvent<HTMLTableDataCellElement>) {
        if (e.button !== 0) return;
        this.props.onMouseUp(this.props.rowid);
    }

    onMouseLeave(e :React.MouseEvent<HTMLTableRowElement>) {
        if (e.button !== 0) return;
        this.props.onMouseLeave(this.props.rowid);
    }

    onMouseEnter(e :React.MouseEvent<HTMLTableRowElement>) {
        if (e.button !== 0) return;
        this.props.onMouseEnter(this.props.rowid);
    }
}

export default LogbookBlock;

