import React from 'react';

class Tooltip extends React.Component {

    render() {
        return (
            <div className="tooltp">
                {this.props.children}
                <span className="tooltptext">{this.props.tooltip}</span>
            </div>
        )
    }
}

export default Tooltip