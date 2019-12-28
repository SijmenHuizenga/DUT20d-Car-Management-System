import {OUTDATED_AFTER_SECONDS} from "../statetypes";
import {IndicatorColor} from "./Indicator";

function isRecent(timestamp :number, secondsRecent = OUTDATED_AFTER_SECONDS) {
    return new Date().getTime() / 1000 - timestamp < secondsRecent
}

function nicenumber(n :number) {
    n = Math.round(n * 10) / 10;
    if(n % 1 === 0)
        return n + '.0';
    return n + '';
}

function indicatorColorBasedOnTime(lastSeen :number, wasSuccessfull = true) :IndicatorColor{
    if(!isRecent(lastSeen)) {
        return IndicatorColor.fault;
    }
    return wasSuccessfull ? IndicatorColor.active : IndicatorColor.idle
}

function formatUnicorn(str :string, params :any) {
    if(!str){
        return ""
    }
    for (let key in params) {
        if(key !== undefined && params[key] !== undefined)
            str = str.replace(new RegExp("\\{" + key + "\\}", "gi"), params[key]);
    }
    return str;
}

export {indicatorColorBasedOnTime, nicenumber, isRecent, formatUnicorn}