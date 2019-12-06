function lastPingWasRecent(timestamp, secondsRecent = 4) {
    return new Date().getTime() / 1000 - timestamp < secondsRecent
}

function nicenumber(n) {
    n = Math.round(n * 10) / 10;
    if(n % 1 === 0)
        return n + '.0';
    return n + '';
}

function indicatorColorBasedOnTime(lastSeen, wasSuccessfull = true) {
    if(!lastPingWasRecent(lastSeen)) {
        return "warning";
    }
    return wasSuccessfull ? "success" : "danger"

}

function formatUnicorn(str, params) {
    if(!str){
        return ""
    }
    for (let key in params) {
        if(key !== undefined && params[key] !== undefined)
            str = str.replace(new RegExp("\\{" + key + "\\}", "gi"), params[key]);
    }
    return str;
}

export {indicatorColorBasedOnTime, nicenumber, lastPingWasRecent, formatUnicorn}