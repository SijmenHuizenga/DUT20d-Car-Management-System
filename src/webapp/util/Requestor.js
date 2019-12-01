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
            if (response.status !== 201 || !response.ok) {
                console.log("Put failed", response);
                reject(`Request failed with code ${response.status}: ${response.statusText}`);
            } else {
                resolve();
            }
        })
    })
}

export default {
    put, execute
}