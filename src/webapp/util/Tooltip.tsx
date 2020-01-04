import React, {MouseEventHandler} from 'react';
import ReactDOM from 'react-dom';

const TooltipContext = React.createContext<TooltipContextType | null>(null);
interface TooltipContextType {
    showTip: (content :TooltipContent, location :DOMRect) => void
    hideTip: () => void
}

type TooltipContent = string | JSX.Element

export default class Tooltip extends React.PureComponent<{tooltip: TooltipContent}> {

    static contextType = TooltipContext;
    private hideOnLeave: boolean = true;

    render() {
        return (
            <div className="tooltip-initiator"
                 onClick={this.onClick}
                 onMouseEnter={this.onMouseEnter}
                 onMouseLeave={this.onMouseLeave}>
                {this.props.children}
            </div>
        )
    }

    onClick = (e :React.MouseEvent) => {
        this.context.showTip(this.props.tooltip, this.getPosition());
        this.hideOnLeave = false;
        // When a user clicks the root app, a onclick is triggered and all tooltips are hidden. 
        // When someone clicks on a tooltip trigger that click event should be exclusively by the tooltip trigger. 
        // If the event was propagated to the root listener the tooltip would be hidden directly.
        // This was a bug which is fixed using stopPropegation()
        e.stopPropagation()
    };

    onMouseEnter = () => {
        this.context.showTip(this.props.tooltip, this.getPosition());
        this.hideOnLeave = true
    };

    onMouseLeave =() => {
        if(this.hideOnLeave) {
            this.context.hideTip()
        }
    };

    getPosition() {
        let element = ReactDOM.findDOMNode(this) as HTMLDivElement;
        if(element == null){
            return
        }
        return element.getBoundingClientRect();
    }
}

interface TT {
    content: TooltipContent | null
    position: DOMRect | null
}

export class TooltipContainer extends React.Component<{}, TT> {

    constructor(props: {}, context: any) {
        super(props, context);
        this.state = {content: null, position: null}
    }

    render() {
        return <TooltipContext.Provider value={{
            showTip: this.showTip,
            hideTip: this.hideTip,
        }}>
            <div onClick={this.containerClicked}>
                {this.props.children}
            </div>
            {this.state.content === null ? null : <TooltipBox {...this.state} />}
        </TooltipContext.Provider>
    }

    showTip = (content :TooltipContent, position :DOMRect) => {
        this.setState({content, position})
    };

    hideTip = () => {
        this.setState({content: null})
    };

    containerClicked :MouseEventHandler = () => {
        this.hideTip()
    };
}

class TooltipBox extends React.PureComponent<TT> {
    render() {
        return <div className="tooltip-body" style={{
            top: this.props.position!.top + 25 + window.scrollY,
            left: this.props.position!.left + 5,
        }}>
            {this.renderContent()}
        </div>

    }
    renderContent() {
        if(typeof this.props.content == "string") {
            return this.props.content.split('\n').map((item, key) => {
                return <React.Fragment key={key}>{item}<br/></React.Fragment>
            })
        }
        return this.props.content;
    }
}

