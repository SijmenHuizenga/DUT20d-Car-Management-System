import React from "react";
import deepEqual from "deep-equal";

export default class RecursivePureComponent<P, S> extends React.Component<P, S>{
    shouldComponentUpdate(nextProps: Readonly<P>, nextState: Readonly<S>, nextContext: any): boolean {
        return !deepEqual(nextProps, this.props) || !deepEqual(this.state, nextState)
    }
}