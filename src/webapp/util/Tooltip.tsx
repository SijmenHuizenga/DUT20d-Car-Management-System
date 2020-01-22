import React from 'react';
import ReactDOM from 'react-dom';

var showTip: undefined | ((content :TooltipContent, location :DOMRect) => void) = undefined;
var hideTip: undefined | (() => void) = undefined;

export type TooltipContent = string | JSX.Element
export type TooltipCreator = () => TooltipContent

interface Props {
    tooltip: TooltipCreator
}

export default class Tooltip extends React.Component<Props, {}> {

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

    shouldComponentUpdate(nextProps: any, nextState: { outdated: boolean }) {
        return this.props.children !== nextProps.children;
    }

    onClick = (e :React.MouseEvent) => {
        if(showTip !== undefined)
            showTip(this.props.tooltip(), this.getPosition());
        this.hideOnLeave = false;
        // When a user clicks the root app, a onclick is triggered and all tooltips are hidden. 
        // When someone clicks on a tooltip trigger that click event should be exclusively by the tooltip trigger. 
        // If the event was propagated to the root listener the tooltip would be hidden directly.
        // This was a bug which is fixed using stopPropegation()
        e.stopPropagation()
    };

    onMouseEnter = () => {
        if(showTip !== undefined)
            showTip(this.props.tooltip(), this.getPosition());
        this.hideOnLeave = true
    };

    onMouseLeave =() => {
        if(this.hideOnLeave) {
            if(hideTip !== undefined)
                hideTip()
        }
    };

    getPosition() {
        let element = ReactDOM.findDOMNode(this) as HTMLDivElement;
        if(element == null){
            throw new DOMException("Invalid state")
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
        this.state = {content: null, position: null};
        showTip = this.showTip;
        hideTip = this.hideTip;
    }

    render() {
        return this.state.content === null ? null : <TooltipBox {...this.state} />
    }

    shouldComponentUpdate(nextProps: Readonly<{}>, nextState: Readonly<TT>, nextContext: any): boolean {
        return nextState.position !== this.state.position || nextState.content !== this.state.content;
    }

    showTip = (content :TooltipContent, position :DOMRect) => {
        this.setState({content, position})
    };

    hideTip = () => {
        this.setState({content: null})
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

