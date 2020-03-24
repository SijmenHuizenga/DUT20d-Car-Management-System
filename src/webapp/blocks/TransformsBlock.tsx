import React from 'react';
import {Transforms} from "../statetypes";
import {Indicator, IndicatorColor} from "../util/Indicator";

interface Props {
    transforms : Transforms
}

class TransformsBlock extends React.Component<Props, {}> {

    render() {
        return <div className="block">
            <h2>Transformations</h2>
            {this.props.transforms == null ? null : this.renderTransforms()}
        </div>
    }

    renderTransforms() {
        return Object.keys(this.props.transforms).map((transformKey) =>
            <div key={transformKey}>
                <Indicator color={IndicatorColor.active} dataTimestamp={this.props.transforms[transformKey].lastseen}/>
                {transformKey}
            </div>
        )
    }

}

export default TransformsBlock;