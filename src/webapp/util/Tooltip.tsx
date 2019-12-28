import React from 'react';

interface Props {
    tooltip: string
}

class Tooltip extends React.Component<Props, {}> {

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