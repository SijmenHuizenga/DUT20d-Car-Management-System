function put(uri, body) {
    return execute(uri, "PUT", body)
}

function execute(uri, method, body) {
    return new Promise((resolve, reject) => {
        fetch(uri, {
            method: method,
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(body)
        }).then((response) => {
            if ((response.status === 201 || response.status === 200) && response.ok) {
                resolve(response.body);
            } else {
                console.log("request failed", response);
                response.text().then((body) => {
                    let errString = `Request failed with code ${response.status}: ${response.statusText}`;
                    if(body !== ""){
                        errString += '\n' + body;
                    }
                    reject(errString);
                });
            }
        })
    })
}

export default {
    put, execute
}