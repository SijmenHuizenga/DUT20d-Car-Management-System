import React from 'react';

class Tooltip extends React.Component {

    render() {
        return (
            <div className="tooltp">
                {this.props.children}
                <div className="tooltptext">
                    {this.props.tooltip.split('\n').map((item, key) => {
                        return <React.Fragment key={key}>{item}<br/></React.Fragment>
                    })}
                </div>
            </div>
        )
    }
}

export default Tooltip